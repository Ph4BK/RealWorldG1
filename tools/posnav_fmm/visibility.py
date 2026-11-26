"""Copied Go2 supercover and macro target selection, without motion control."""
import math
import numpy as np
def supercover_line(start, end):
    """Return every grid cell touched by the segment between two cell centers."""
    row, col = (int(round(float(value))) for value in start)
    end_row, end_col = (int(round(float(value))) for value in end)
    delta_row = end_row - row
    delta_col = end_col - col
    count_row = abs(delta_row)
    count_col = abs(delta_col)
    step_row = 0 if delta_row == 0 else (1 if delta_row > 0 else -1)
    step_col = 0 if delta_col == 0 else (1 if delta_col > 0 else -1)
    index_row = 0
    index_col = 0
    cells = [(row, col)]

    def append(cell_row, cell_col):
        cell = (cell_row, cell_col)
        if cell != cells[-1]:
            cells.append(cell)

    while index_row < count_row or index_col < count_col:
        decision = (1 + 2 * index_col) * count_row - \
            (1 + 2 * index_row) * count_col
        if decision == 0:
            # At an exact grid corner, conservatively include both adjacent
            # cells as well as the diagonal destination.
            append(row, col + step_col)
            append(row + step_row, col)
            row += step_row
            col += step_col
            index_row += 1
            index_col += 1
            append(row, col)
        elif decision < 0:
            col += step_col
            index_col += 1
            append(row, col)
        else:
            row += step_row
            index_row += 1
            append(row, col)

    return cells


def line_is_collision_free(start, end, traversible):
    """Check a supercover segment against an already-inflated free-space map."""
    height, width = traversible.shape
    for row, col in supercover_line(start, end):
        if row < 0 or row >= height or col < 0 or col >= width:
            return False
        if not bool(traversible[row, col]):
            return False
    return True


def segment_clearance(start_rc, end_rc, occupied, resolution, limit_m):
    """Exact segment-to-occupied-cell distance, capped at limit_m (metres).

    Coordinates are continuous grid coordinates measured from cell corners,
    not rounded indices. Occupied cells are closed squares; map exterior is
    blocked. Only cells within the requested distance of the chord are read.
    """
    start, end = np.asarray(start_rc,dtype=float), np.asarray(end_rc,dtype=float)
    if (start.shape!=(2,) or end.shape!=(2,) or not np.isfinite([start,end]).all() or
            occupied.ndim!=2 or not math.isfinite(resolution) or resolution<=0 or
            not math.isfinite(limit_m) or limit_m<=0):
        raise ValueError('Invalid continuous clearance query')
    shape=np.asarray(occupied.shape)
    boundary=max(0.,float(np.min([start,end,shape-start,shape-end])))*resolution
    if boundary==0.:return 0.
    radius=limit_m/resolution
    lower=np.maximum(0,np.floor(np.minimum(start,end)-radius).astype(int))
    upper=np.minimum(shape,np.floor(np.maximum(start,end)+radius).astype(int)+1)
    cells=np.argwhere(occupied[lower[0]:upper[0],lower[1]:upper[1]])+lower
    if not len(cells):return min(limit_m,boundary)
    lo=cells.astype(float); hi=lo+1.
    delta=end-start; length2=float(delta@delta)
    # Endpoint-to-square and corner-to-segment distances cover all disjoint
    # segment/square cases. Slab intersection supplies zero for crossings.
    d0=np.maximum(np.maximum(lo-start,start-hi),0.)
    d1=np.maximum(np.maximum(lo-end,end-hi),0.)
    distance2=np.minimum(np.sum(d0*d0,axis=1),np.sum(d1*d1,axis=1))
    corners=np.stack((lo,np.column_stack((lo[:,0],hi[:,1])),
                      np.column_stack((hi[:,0],lo[:,1])),hi),axis=1)
    relative=corners-start
    fraction=np.clip(np.sum(relative*delta,axis=2)/length2,0.,1.) if length2 else np.zeros(relative.shape[:2])
    offsets=relative-fraction[:,:,None]*delta
    distance2=np.minimum(distance2,np.min(np.sum(offsets*offsets,axis=2),axis=1))
    enter=np.zeros(len(cells));leave=np.ones(len(cells));overlap=np.ones(len(cells),bool)
    for axis in (0,1):
        if delta[axis]==0.:
            overlap &= (start[axis]>=lo[:,axis]) & (start[axis]<=hi[:,axis])
        else:
            a=(lo[:,axis]-start[axis])/delta[axis];b=(hi[:,axis]-start[axis])/delta[axis]
            enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
    distance2[overlap & (enter<=leave)]=0.
    return min(limit_m,boundary,float(np.sqrt(distance2.min()))*resolution)


def get_stg(path, traversible, *, candidate_filter=None):
    """Select the furthest visible rollout STG, checking even path[1]."""
    if len(path) < 2:
        return None

    for candidate in range(len(path)-1, 0, -1):
        if (line_is_collision_free(path[0], path[candidate], traversible) and
                (candidate_filter is None or candidate_filter(candidate,path[candidate]))):
            return path[candidate]
    return None


def get_cost_stg(path, traversible, cost, field, tolerance_cells=.25, *, candidate_filter=None):
    """Keep the hard supercover check; compare chord + propagated goal cost.

    Furthest-visible selection alone can shortcut across a costly wall margin.
    A quarter cost-cell tie band covers discretization (not a clearance promise).
    """
    candidates = []
    for index, point in enumerate(path[1:], 1):
        if not line_is_collision_free(path[0], point, traversible):
            continue
        if candidate_filter is not None and not candidate_filter(index,point):
            continue
        start, end = np.asarray(path[0]), np.asarray(point)
        length = float(np.linalg.norm(end-start))
        samples = np.linspace(start, end, max(2, int(math.ceil(length*4)))).round().astype(int)
        chord = length*float(cost[samples[:, 0], samples[:, 1]].mean())
        candidates.append((index, point, chord+field[tuple(map(int, point))]))
    if not candidates:
        return None
    best = min(row[2] for row in candidates)
    return max((row for row in candidates if row[2] <= best+tolerance_cells),
               key=lambda row: row[0])[1]


