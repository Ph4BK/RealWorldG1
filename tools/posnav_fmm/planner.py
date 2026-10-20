"""One-shot FMM + Go2 macro rollout in the G1 map XY coordinate system."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import label
from skimage.morphology import binary_dilation, disk
from .fmm_planner import FMMPlanner
from .visibility import get_stg
from .errors import GoalUnreachable


@dataclass
class Plan:
    distance_m: np.ndarray
    traversible: np.ndarray
    start_rc: tuple
    goal_rc: tuple
    stg_xy: tuple | None
    rollout_rc: list
    stopped: bool
    reason: str


def plan_snapshot(occupancy, origin, resolution, start_xy, goal_xy, *,
                  inflation_cells=5, step_cells=10, max_stgs=4, fallback_cells=2):
    """No yaw input, motion commands or mutation of the source occupancy map."""
    occupied=np.asarray(occupancy,dtype=bool)
    if occupied.ndim!=2 or min(occupied.shape)<3:
        raise ValueError('Invalid occupancy grid')
    if not math.isclose(resolution,.05):
        raise ValueError('Go2 stop rules require the configured 5 cm grid')
    if inflation_cells<0 or min(step_cells,max_stgs,fallback_cells)<1:
        raise ValueError('Invalid FMM parameters')
    origin=np.asarray(origin,dtype=float)
    def cell(xy):
        xy=np.asarray(xy,dtype=float)
        if xy.shape!=(2,) or not np.isfinite(xy).all():raise ValueError('Invalid XY coordinates')
        col,row=np.floor((xy-origin)/resolution).astype(int)
        if not (0<=row<occupied.shape[0] and 0<=col<occupied.shape[1]):
            raise ValueError('Robot or goal is outside the map')
        return int(row),int(col)
    start=cell(start_xy);goal=cell(goal_xy)
    free=~binary_dilation(occupied,disk(inflation_cells))
    if not free[goal]:raise ValueError('Goal is occupied after obstacle inflation; redraw the goal')
    # Same 3x3 start clearance and traversible one-cell padding as Go2.
    r,c=start;free[max(0,r-1):r+2,max(0,c-1):c+2]=True
    padded=np.pad(free,1,constant_values=True)
    state=(r+1,c+1); target=(goal[0]+1,goal[1]+1)
    components,_=label(padded)
    reachable=components==components[target]
    if not reachable[state]:raise GoalUnreachable('Goal is unreachable from the current position')
    goals=np.zeros(padded.shape,dtype=np.uint8);goals[target]=1
    planner=FMMPlanner(padded);planner.set_multi_goal(goals)
    distances=planner.fmm_dist[1:-1,1:-1].copy()*resolution
    distances[~reachable[1:-1,1:-1]]=np.inf
    distance=float(planner.fmm_dist[state])*resolution
    euclidean=float(np.linalg.norm(np.asarray(state)-target))*resolution
    def result(stg,path,stopped,reason):
        xy=None if stg is None else (float(origin[0]+(stg[1]-1+.5)*resolution),
                                     float(origin[1]+(stg[0]-1+.5)*resolution))
        return Plan(distances,free,start,goal,xy,[(p[0]-1,p[1]-1) for p in path],stopped,reason)
    if 0<euclidean<1. and distance/euclidean>2.:
        return result(None,[],True,'Go2 global-goal distance ratio stop')
    def next_stg(position):
        row,col,replan,stop=planner.get_short_term_goal(position,step_cells)
        if replan:
            row,col,_,stop=planner.get_short_term_goal(position,fallback_cells)
        return (float(row),float(col)),replan,stop
    first,replan,stop=next_stg(state)
    if stop:return result(None,[],True,'Within Go2 FMM stop distance (25 cm)')
    path=[state,first];current=first
    for _ in range(1,max_stgs):
        nxt,replan,stop=next_stg(current)
        if stop or nxt==current or nxt in path:break
        path.append(nxt);current=nxt
        if replan:break
    selected=get_stg(path,padded)
    if selected is None or selected==state:raise ValueError('No progressing STG found')
    if not (1<=selected[0]<=occupied.shape[0] and 1<=selected[1]<=occupied.shape[1]):
        raise ValueError('Selected STG is outside the original map')
    if not padded[int(selected[0]),int(selected[1])]:raise ValueError('Selected STG is occupied')
    return result(selected,path,False,'STG macro selected from the current snapshot')
