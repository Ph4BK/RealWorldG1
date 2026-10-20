"""ROS-independent rolling BEV fusion and live-pose initialization geometry."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation


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
                 max_window_age=2.):
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
        self.max_window_age = max_window_age
        rows, cols = np.indices(self.static.shape)
        self.x = self.origin[0] + (cols + .5) * self.res
        self.y = self.origin[1] + (rows + .5) * self.res
        self.frames = deque(maxlen=window)
        self.task = self.static.copy()
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
        self.last_stamp = None

    def region(self, x, y):
        dx, dy = self.x - x, self.y - y
        radius = np.hypot(dx, dy)
        return radius <= self.radius

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
        return True
