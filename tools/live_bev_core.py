"""ROS-independent rolling BEV fusion and live-pose initialization geometry."""
from collections import deque
import hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


OBSTACLE_MODES = ('dynamic', 'static_test')
STATIC_TEST_WARNING = 'STATIC TEST - REALTIME OBSTACLES IGNORED'


def obstacle_map_topic(mode):
    """Startup-only source selection; unknown modes must never silently fall back."""
    if mode not in OBSTACLE_MODES:
        raise ValueError('Unknown obstacle mode: %r' % mode)
    return '/g1/bev/task' if mode == 'dynamic' else '/g1/bev/static_test'


def room_no_go(shape, origin, res, bounds):
    """Inherit Go2 room policy exactly, including its grid-anchor convention."""
    bounds = np.asarray(bounds, dtype=float)
    if (bounds.shape != (4,) or not np.isfinite(bounds).all() or
            bounds[0] >= bounds[1] or bounds[2] >= bounds[3]):
        raise ValueError('Invalid explicit room_bounds for fixed no-go policy')
    x = origin[0] + np.arange(shape[1]) * res
    y = origin[1] + np.arange(shape[0]) * res
    return ~((x[None, :] >= bounds[0]) & (x[None, :] <= bounds[1]) &
             (y[:, None] >= bounds[2]) & (y[:, None] <= bounds[3]))


def load_fixed_no_go(map_dir, config, shape, origin, res):
    """Legacy maps have no policy; declared policy must load and validate loudly."""
    specification = config.get('fixed_no_go')
    if specification is None:
        return None
    path = Path(specification['file'])
    if path.is_absolute() or len(path.parts) != 1:
        raise ValueError('Fixed no-go file must be inside the map package')
    path = Path(map_dir) / path
    if hashlib.sha256(path.read_bytes()).hexdigest() != specification['sha256']:
        raise ValueError('Fixed no-go checksum mismatch')
    with np.load(path, allow_pickle=False) as data:
        mask = data['no_go'].copy()
        if (mask.dtype != np.bool_ or mask.shape != tuple(shape) or
                not np.array_equal(data['origin'], origin) or float(data['res']) != res):
            raise ValueError('Fixed no-go grid geometry/dtype mismatch')
    if specification.get('source') == 'go2_room_bounds_grid_anchor_inclusive':
        expected = room_no_go(shape, origin, res, specification['room_bounds_xmin_xmax_ymin_ymax_m'])
        if not np.array_equal(mask, expected):
            raise ValueError('Fixed no-go differs from declared room policy')
    else:
        raise ValueError('Unknown fixed no-go source')
    return mask


def grid_start_status(free, origin, res, xy, nearest=True):
    """Inspect the CURRENT position, never snap it or mutate the map."""
    result = dict(start_free=None, start_rejection_reason='start_unknown',
                  nearest_free_cell_distance_m=None, nearest_free_xy_m=None)
    if free is None or xy is None:
        return result
    xy = np.asarray(xy, dtype=float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        return result
    col, row = np.floor((xy - origin) / res).astype(int)
    inside = 0 <= row < free.shape[0] and 0 <= col < free.shape[1]
    good = bool(inside and free[row, col])
    result.update(start_free=good, start_rejection_reason=(
        '' if good else 'start_in_obstacle' if inside else 'start_outside_map'))
    if good:
        result['nearest_free_cell_distance_m'] = 0.
    elif nearest:
        rows, cols = np.nonzero(free)
        if len(rows):
            points = np.column_stack([cols + .5, rows + .5]) * res + origin
            distances = np.sum((points - xy) ** 2, axis=1)
            index = int(np.argmin(distances))
            result.update(nearest_free_cell_distance_m=float(np.sqrt(distances[index])),
                          nearest_free_xy_m=points[index].tolist())
    return result


def checked_transform(value):
    t = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Invalid rigid transform')
    return t


def initial_map_from_odom(x, y, yaw, z, imu_samples, odom_pose):
    """Samples are [stamp, ax, ay, az, gx, gy, gz] from a quiet live window."""
    v = np.asarray(imu_samples, dtype=float)
    if v.ndim != 2 or v.shape[1] != 7 or len(v) < 100 or not np.isfinite(v).all():
        raise ValueError('Need at least 100 finite IMU samples')
    if v[-1, 0] - v[0, 0] < 1.8 or np.any(np.diff(v[:, 0]) <= 0):
        raise ValueError('Need two seconds of increasing IMU timestamps')
    up = v[:, 1:4].mean(axis=0)
    norm = np.linalg.norm(up)
    if norm < .1:
        raise ValueError('Invalid acceleration mean')
    variation = np.linalg.norm(v[:, 1:4].std(axis=0)) / norm
    gyro = np.quantile(np.linalg.norm(v[:, 4:7], axis=1), .95)
    if variation > .15 or gyro > .15:
        raise ValueError('Keep G1 stationary: IMU window is not quiet')
    up /= norm
    forward = np.array([1., 0, 0]) - up * up[0]
    if np.linalg.norm(forward) < .1:
        raise ValueError('IMU forward axis is near vertical')
    forward /= np.linalg.norm(forward)
    level = np.stack([forward, np.cross(up, forward), up])
    desired = np.eye(4)
    desired[:3, :3] = Rotation.from_euler('z', yaw).as_matrix() @ level
    desired[:3, 3] = [x, y, z]
    return checked_transform(desired @ np.linalg.inv(checked_transform(odom_pose)))


class RollingBEV:
    def __init__(self, static, origin, res, window=10, min_points=2,
                 z_min=.05, z_max=1.3, radius=2.,
                 max_window_age=2., inner_radius=.3, no_go=None):
        self.static = np.asarray(static, dtype=bool).copy()
        self.origin = np.asarray(origin, dtype=float)
        self.res = float(res)
        if self.static.ndim != 2 or self.res <= 0 or window < 1 or min_points < 1:
            raise ValueError('Invalid BEV configuration')
        self.window, self.min_points = window, min_points
        self.z_min, self.z_max = z_min, z_max
        self.radius = float(radius)
        if not np.isfinite(self.radius) or self.radius <= 0:
            raise ValueError('Update radius must be finite and positive')
        self.inner_radius = float(inner_radius)
        if not np.isfinite(self.inner_radius) or not 0 <= self.inner_radius < self.radius:
            raise ValueError('Inner radius must be finite, nonnegative and smaller than update radius')
        self.max_window_age = max_window_age
        self.no_go = np.zeros_like(self.static) if no_go is None else np.asarray(no_go).copy()
        if self.no_go.dtype != np.bool_ or self.no_go.shape != self.static.shape:
            raise ValueError('Fixed no-go must be a same-shape boolean mask')
        self.no_go.setflags(write=False)
        self.no_go_cells = int(self.no_go.sum())
        self.last_no_go_protected_cells = 0
        rows, cols = np.indices(self.static.shape)
        self.x = self.origin[0] + (cols + .5) * self.res
        self.y = self.origin[1] + (rows + .5) * self.res
        self.frames = deque(maxlen=window)
        self.task = self.static.copy()
        self.task |= self.no_go
        self.local = np.zeros_like(self.static)
        self.mask = np.zeros_like(self.static)
        self.last_stamp = None

    def pause(self):
        self.frames.clear()
        self.local.fill(False)
        self.mask.fill(False)
        # Keep last_stamp so repeated observations cannot refill the window.

    def reset(self):
        self.pause()
        self.task[:] = self.static
        self.task |= self.no_go
        self.last_no_go_protected_cells = 0
        self.last_stamp = None

    def region(self, x, y):
        dx, dy = self.x - x, self.y - y
        radius = np.hypot(dx, dy)
        return (radius >= self.inner_radius) & (radius <= self.radius)

    def clear_initial_region(self, x, y):
        """Explicit legacy opt-in only. Fixed no-go is never cleared."""
        if not np.isfinite([x, y]).all():
            raise ValueError('Initial clearing position must be finite')
        disk = np.hypot(self.x - x, self.y - y) <= self.inner_radius
        self.last_no_go_protected_cells = int((disk & self.no_go).sum())
        clear = disk & ~self.no_go
        cleared = int((self.task & clear).sum())
        self.task[clear] = False
        self.task |= self.no_go
        return cleared

    def add(self, stamp, points_map, pose_xy_yaw):
        """Return True only after a new, nonempty, valid frame commits a fusion."""
        points = np.asarray(points_map, dtype=float)
        if not np.isfinite(stamp) or not np.isfinite(pose_xy_yaw).all():
            self.pause()
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp:  # clock reset: discard corrupted history
                self.reset()
            return False
        if points.ndim != 2 or points.shape[1] != 3:
            self.pause()
            return False
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:  # Missing cloud must never clear the map.
            self.pause()
            return False
        if self.last_stamp is not None and stamp - self.last_stamp > self.max_window_age:
            self.pause()
        self.last_stamp = stamp
        q = points[(points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)]
        indices = np.floor((q[:, :2] - self.origin) / self.res).astype(np.int64)
        height, width = self.static.shape
        valid = ((indices[:, 0] >= 0) & (indices[:, 0] < width) &
                 (indices[:, 1] >= 0) & (indices[:, 1] < height))
        indices = indices[valid]
        counts = np.zeros(self.static.shape, dtype=np.int32)
        np.add.at(counts, (indices[:, 1], indices[:, 0]), 1)
        self.frames.append((stamp, counts))
        while self.frames and stamp - self.frames[0][0] > self.max_window_age:
            self.frames.popleft()
        if len(self.frames) < self.window:
            return False
        self.local = sum(c for _, c in self.frames) >= self.min_points
        self.mask = self.region(*pose_xy_yaw[:2])
        self.task[self.mask] = self.local[self.mask]
        self.last_no_go_protected_cells = int((self.mask & self.no_go & ~self.local).sum())
        self.task |= self.no_go
        return True
