"""Opt-in map-space ground/speckle filters. No ROS, ICP or height calibration IO."""
from collections import deque
from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy.spatial import ConvexHull, QhullError


@dataclass(frozen=True)
class ObservationFilterConfig:
    ground: bool = False
    speckles: bool = False
    cell_m: float = .10            # 11-24 diagnostic used 10cm cell medians; one pose only.
    min_cells: int = 80            # Candidate floor coverage gate; not G1-calibrated.
    min_span_m: float = 1.2        # Candidate coverage in BOTH map axes; not object size.
    min_quadrants: int = 3         # Candidate spatial coverage; cannot certify ground truth.
    min_quadrant_cells: int = 5    # Candidate per-quadrant evidence gate, unvalidated.
    min_inlier_fraction: float = .70  # Candidate rejects clutter-dominated fits.
    fit_residual_m: float = .012   # 11-24 floor residual p95 ~5mm; candidate margin.
    ground_band_m: float = .01     # 11-24 offline +/-1cm floor removal preserved object ROIs.
    search_min_z: float = -.15     # Candidate permits small height-reference offset, no calibration change.
    search_max_z: float = .20      # Candidate; prevents fitting the recorded 30cm object top.
    max_slope_deg: float = 5.      # Candidate indoor-floor gate, not a measured site slope bound.
    confirmation_frames: int = 3   # Candidate independent-frame confirmation, ~0.3s at recorded rate.
    max_jump_m: float = .025       # Candidate plane change bound; recorded drift was a few mm.
    near_exclusion_m: float = .35  # Candidate excludes robot/cable attachment area while fitting.
    radius_extra_m: float = .20    # Candidate fit coverage beyond 2m update disc.
    hull_padding_m: float = .05    # One existing 5cm grid cell; bounded fit extrapolation.
    neighbor_points: int = 4       # Candidate 3x3 FINE-cell support; not Go2's 0.75m neighborhood.
    independent_frames: int = 2    # Candidate alternate evidence; counts frames, never raw points.

    def __post_init__(self):
        for key in ('ground', 'speckles'):
            if not isinstance(getattr(self, key), bool):
                raise ValueError('Observation switches must be booleans')
        for key, value in asdict(self).items():
            if key in ('ground', 'speckles', 'search_min_z'):
                continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid observation parameter: '+key)
        if (not math.isfinite(self.search_min_z) or self.search_min_z >= self.search_max_z or
                self.min_inlier_fraction > 1 or not 0 < self.max_slope_deg < 45):
            raise ValueError('Invalid ground search/confidence bounds')
        for key in ('min_cells', 'min_quadrants', 'min_quadrant_cells', 'confirmation_frames',
                    'neighbor_points', 'independent_frames'):
            if not isinstance(getattr(self, key), int):
                raise ValueError('Observation counts must be integers')
        if not 1 <= self.min_quadrants <= 4 or self.min_cells < 3:
            raise ValueError('Invalid ground coverage gates')


def add_observation_options(parser):
    parser.add_argument('--ground-filter', action='store_true',
                        help='Candidate confident local-ground filtering; default off')
    parser.add_argument('--speckle-filter', action='store_true',
                        help='Candidate spatial/independent-frame support for new isolated cells; default off')


def observation_options(args):
    config = ObservationFilterConfig(ground=getattr(args, 'ground_filter', False),
                                     speckles=getattr(args, 'speckle_filter', False))
    return config if config.ground or config.speckles else None


def cell_medians(points, cell_m):
    cells, inverse, counts = np.unique(np.floor(points[:, :2]/cell_m).astype(np.int64),
                                     axis=0, return_inverse=True, return_counts=True)
    ordered = points[np.lexsort((points[:, 2], inverse)), 2]
    starts = np.cumsum(counts)-counts
    lo, hi = (counts-1)//2, counts//2
    z = (ordered[starts+lo]+ordered[starts+hi])*.5
    return cells*cell_m+cell_m*.5, z


class ObservationFilter:
    def __init__(self, config):
        self.config = config
        self.fits = deque(maxlen=config.confirmation_frames)
        self.pause()

    def pause(self):
        self.fits.clear()
        self.active = False
        self.diagnostics = dict(ground_state='collecting' if self.config.ground else 'off',
                                ground_removed_points=0, speckle_removed_cells=0)

    def unavailable(self, reason):
        self.fits.clear()
        self.active = False
        self.diagnostics.update(ground_state=reason, ground_removed_points=0,
                                plane_coefficients=None, plane_floor_z_at_robot_m=None)

    def _fit(self, points, pose, protected, origin, resolution, radius):
        cfg = self.config
        cells = np.floor((points[:, :2]-origin)/resolution).astype(np.int64)
        height, width = protected.shape
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < width) &
                 (cells[:, 1] >= 0) & (cells[:, 1] < height))
        locked = np.ones(len(points), bool)
        locked[valid] = protected[cells[valid, 1], cells[valid, 0]]
        distance = np.linalg.norm(points[:, :2]-pose[:2], axis=1)
        select = (~locked & (distance >= cfg.near_exclusion_m) &
                  (distance <= radius+cfg.radius_extra_m) &
                  (points[:, 2] >= cfg.search_min_z) & (points[:, 2] <= cfg.search_max_z))
        if int(select.sum()) < cfg.min_cells:
            return None, 'insufficient_floor_points', locked
        xy, z = cell_medians(points[select], cfg.cell_m)
        if len(z) < cfg.min_cells:
            return None, 'insufficient_floor_cells', locked
        design = np.column_stack((xy-pose[:2], np.ones(len(z))))
        fit, _, rank, _ = np.linalg.lstsq(design, z, rcond=None)
        if rank != 3:
            return None, 'degenerate_floor', locked
        for _ in range(5):  # Fixed bounded robust-fit iterations, candidate implementation choice.
            residual = z-design@fit
            keep = abs(residual-np.median(residual)) <= cfg.fit_residual_m
            if int(keep.sum()) < cfg.min_cells:
                return None, 'insufficient_floor_inliers', locked
            fit, _, rank, _ = np.linalg.lstsq(design[keep], z[keep], rcond=None)
            if rank != 3:
                return None, 'degenerate_floor', locked
        residual = z-design@fit
        keep = abs(residual) <= cfg.fit_residual_m
        support = xy[keep]-pose[:2]
        if int(keep.sum()) < cfg.min_cells or keep.mean() < cfg.min_inlier_fraction:
            return None, 'low_floor_inlier_fraction', locked
        if np.min(np.ptp(support, axis=0)) < cfg.min_span_m:
            return None, 'insufficient_floor_span', locked
        quadrants = (support[:, 0] >= 0).astype(int)*2+(support[:, 1] >= 0)
        if int((np.bincount(quadrants, minlength=4) >= cfg.min_quadrant_cells).sum()) < cfg.min_quadrants:
            return None, 'insufficient_floor_quadrants', locked
        slope = math.degrees(math.atan(float(np.linalg.norm(fit[:2]))))
        if slope > cfg.max_slope_deg or not cfg.search_min_z <= fit[2] <= cfg.search_max_z:
            return None, 'implausible_floor', locked
        # Store a plane in fixed map XY; recent fits remain comparable as the robot moves.
        absolute = np.array([fit[0], fit[1], fit[2]-fit[:2]@pose[:2]])
        hull = ConvexHull(xy[keep])
        data = dict(coefficients=absolute, hull=hull.equations,
            floor_z_at_robot_m=float(fit[2]), slope_deg=slope, inlier_cells=int(keep.sum()),
            inlier_fraction=float(keep.mean()), residual_p95_m=float(np.quantile(abs(residual[keep]), .95)))
        return data, '', locked

    def filter_points(self, points, pose, protected, origin, resolution, radius, z_min, z_max):
        """Keep unknown fits unchanged; never reject points on known map obstacles."""
        cfg = self.config
        was_active = self.active
        self.diagnostics['ground_removed_points'] = 0
        if not cfg.ground:
            return points, False
        try:
            fitted, reason, locked = self._fit(points, np.asarray(pose), protected, origin, resolution, radius)
        except (ValueError, np.linalg.LinAlgError, QhullError):
            fitted, reason = None, 'invalid_floor_geometry'
        if fitted is None:
            self.unavailable(reason)
            return points, was_active
        coefficients = fitted['coefficients']
        probes = np.asarray(pose[:2])+np.array([[0,0],[1,0],[-1,0],[0,1],[0,-1]])
        probe_design = np.column_stack((probes, np.ones(len(probes))))
        if self.fits and np.max(abs(probe_design@(coefficients-self.fits[-1]))) > cfg.max_jump_m:
            self.fits.clear()
        self.fits.append(coefficients)
        self.active = len(self.fits) >= cfg.confirmation_frames
        self.diagnostics.update({key: value for key, value in fitted.items()
                                 if key not in ('coefficients', 'hull')})
        self.diagnostics['ground_state'] = 'ready' if self.active else 'confirming_floor'
        if not self.active:
            return points, was_active
        model = np.median(self.fits, axis=0)
        predicted = points[:,0]*model[0]+points[:,1]*model[1]+model[2]
        # Apply only in the current observed floor hull and current fitting radius.
        hull = fitted['hull']
        possible = (~locked & (points[:,2] >= z_min) & (points[:,2] <= z_max) &
                    (abs(points[:,2]-predicted) <= cfg.ground_band_m) &
                    (np.linalg.norm(points[:,:2]-pose[:2],axis=1) <= radius+cfg.radius_extra_m))
        ids = np.flatnonzero(possible)
        remove = np.zeros(len(points), bool)
        remove[ids] = np.all(points[ids,:2]@hull[:,:2].T+hull[:,2] <= cfg.hull_padding_m,axis=1)
        self.diagnostics.update(ground_removed_points=int(remove.sum()),
            plane_coefficients=model.tolist(), plane_floor_z_at_robot_m=float(np.dot([*pose[:2],1],model)))
        return points[~remove], not was_active

    def supported_cells(self, frames, counts, previous_task, protected, min_points):
        cfg = self.config
        # Strong one-frame evidence OR independent frame evidence. Retain an already
        # accepted obstacle while any observed evidence remains; no faster forgetting.
        supported = np.ones(counts.shape, bool)
        rows, cols = np.nonzero((counts >= min_points) & ~previous_task & ~protected)
        if len(rows):
            offsets = np.array([(r,c) for r in (-1,0,1) for c in (-1,0,1)])
            rr, cc = rows[:,None]+offsets[:,0], cols[:,None]+offsets[:,1]
            valid = (rr>=0)&(rr<counts.shape[0])&(cc>=0)&(cc<counts.shape[1])
            rr=np.clip(rr,0,counts.shape[0]-1);cc=np.clip(cc,0,counts.shape[1]-1)
            spatial=(counts[rr,cc]*valid).sum(axis=1)
            independent=sum(np.any((frame[rr,cc]>0)&valid,axis=1).astype(np.int32) for _,frame in frames)
            supported[rows,cols] = (spatial>=cfg.neighbor_points)|(independent>=cfg.independent_frames)
        self.diagnostics['speckle_removed_cells'] = int((~supported).sum())
        return supported
