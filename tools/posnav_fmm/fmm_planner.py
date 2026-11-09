"""Vendored Go2 FMM numerical kernel; see PROVENANCE.md."""
import numpy as np
import skfmm
from numpy import ma
from skimage.draw import circle_perimeter

# @numba.jit(nopython=True)
def get_mask(sx, sy, scale, step_size):
    size = int(step_size // scale) * 2 + 1
    mask = np.zeros((size, size))
    # for i in range(size):
    #     for j in range(size):
    #         if ((i + 0.5) - (size // 2 + sx)) ** 2 + \
    #            ((j + 0.5) - (size // 2 + sy)) ** 2 <= \
    #                 step_size ** 2 \
    #            and ((i + 0.5) - (size // 2 + sx)) ** 2 + \
    #            ((j + 0.5) - (size // 2 + sy)) ** 2 > \
    #                 (step_size - 1) ** 2:
    #             mask[i, j] = 1
    rr, cc = circle_perimeter(size // 2, size // 2, step_size)
    mask[rr, cc] = 1

    mask[size // 2, size // 2] = 1
    return mask

# @numba.jit(nopython=True)
def get_dist(sx, sy, scale, step_size):
    size = int(step_size // scale) * 2 + 1
    mask = np.zeros((size, size)) + 1e-10
    # for i in range(size):
    #     for j in range(size):
    #         if ((i + 0.5) - (size // 2 + sx)) ** 2 + \
    #            ((j + 0.5) - (size // 2 + sy)) ** 2 <= \
    #                 step_size ** 2:
    #             mask[i, j] = max(5,
    #                              (((i + 0.5) - (size // 2 + sx)) ** 2 +
    #                               ((j + 0.5) - (size // 2 + sy)) ** 2) ** 0.5)
    for i in range(step_size):
        rr, cc = circle_perimeter(size // 2, size // 2, i+1)
        mask[rr, cc] = i+1
    return mask


class FMMPlanner():
    def __init__(self, traversible, scale=1, step_size=5):
        self.scale = scale
        self.step_size = step_size
        if scale != 1:
            raise ValueError('Only the deployed scale=1 is supported')
        self.traversible = traversible

        self.du = int(self.step_size / (self.scale * 1.))
        self.fmm_dist = None

    def set_multi_goal(self, goal_map, visited=None):
        traversible_ma = ma.masked_values(self.traversible * 1, 0)
        traversible_ma[goal_map == 1] = 0
        dd = skfmm.distance(traversible_ma, dx=1)
        dd = ma.filled(dd, np.max(dd) + 1)
        if visited is not None:
            dd += visited * 0.5
        self.fmm_dist = dd
        return

    def get_short_term_goal(self, state, step_size=5):
        self.step_size = step_size
        self.du = int(self.step_size / (self.scale * 1.))
        scale = self.scale * 1.
        state = [x / scale for x in state]
        dx, dy = state[0] - int(state[0]), state[1] - int(state[1])
        mask = get_mask(dx, dy, scale, self.step_size)
        dist_mask = get_dist(dx, dy, scale, self.step_size)

        state = [int(x) for x in state]

        dist = np.pad(self.fmm_dist, self.du,
                      'constant', constant_values=self.fmm_dist.shape[0] ** 2)
        subset = dist[state[0]:state[0] + 2 * self.du + 1,
                      state[1]:state[1] + 2 * self.du + 1]

        assert subset.shape[0] == 2 * self.du + 1 and \
            subset.shape[1] == 2 * self.du + 1, \
            "Planning error: unexpected subset shape {}".format(subset.shape)

        subset *= mask
        subset += (1 - mask) * self.fmm_dist.shape[0] ** 2

        # Close targets are explicit final STGs in plan_snapshot; distance alone
        # must never terminate planning before the target can be approached.
        stop = False

        subset -= subset[self.du, self.du]
        ratio1 = subset / dist_mask
        subset[ratio1 < -1.5] = 1

        subset[self.du, self.du] = (self.fmm_dist.shape[0] ** 2 - subset[self.du, self.du]) / 5

        (stg_x, stg_y) = np.unravel_index(np.argmin(subset), subset.shape)

        if subset[stg_x, stg_y] > -0.0001:
            replan = True
        else:
            replan = False

        return (stg_x + state[0] - self.du) * scale, \
               (stg_y + state[1] - self.du) * scale, replan, stop

