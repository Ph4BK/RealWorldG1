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


def get_stg(path, traversible):
    """Select the furthest visible rollout STG, checking even path[1]."""
    if len(path) < 2:
        return None

    for candidate in range(len(path)-1, 0, -1):
        if line_is_collision_free(path[0], path[candidate], traversible):
            return path[candidate]
    return None


def get_cost_stg(path, traversible, cost, field, tolerance_cells=.25):
    """Keep the hard supercover check; compare chord + propagated goal cost.

    Furthest-visible selection alone can shortcut across a costly wall margin.
    A quarter cost-cell tie band covers discretization (not a clearance promise).
    """
    candidates = []
    for index, point in enumerate(path[1:], 1):
        if not line_is_collision_free(path[0], point, traversible):
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


