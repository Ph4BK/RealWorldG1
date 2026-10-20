"""Copied Go2 supercover and macro target selection, without motion control."""
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
    """Select the furthest visible rollout STG, with path[1] as baseline fallback."""
    if len(path) < 2:
        return None

    candidate = len(path) - 1
    while candidate > 1:
        if line_is_collision_free(path[0], path[candidate], traversible):
            break
        candidate -= 1
    return path[candidate]


