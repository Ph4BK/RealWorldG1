"""One-shot FMM + Go2 macro rollout in the G1 map XY coordinate system."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import label, distance_transform_edt
from skimage.morphology import binary_dilation, disk
from .fmm_planner import FMMPlanner
from .visibility import get_stg, get_cost_stg, line_is_collision_free, segment_clearance, supercover_line
from .errors import GoalUnreachable, NoVisibleSTG, StartInObstacle


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
    final_stg: bool = False
    cost_to_goal: np.ndarray | None = None
    stg_selection: dict | None = None
    inflation_connections: list | None = None


def _connector(free, stop_free, xy, endpoint, origin, resolution, inflation_cells, kind, min_length_cells=0):
    """Open one bounded straight corridor, never an obstacle or the full band.

    The bound is geometric (planning radius plus one cell), not a gait/safety
    calibration. Both exact XY chord and conservative cell-centre supercover
    must stay in the stop mask. A disconnected/narrow environment may reject.
    """
    start=(np.asarray(xy)[::-1]-origin[::-1])/resolution
    radius=inflation_cells+1
    lower=np.maximum(0,np.floor(start-radius).astype(int))
    upper=np.minimum(free.shape,np.ceil(start+radius).astype(int)+1)
    cells=np.argwhere(free[lower[0]:upper[0],lower[1]:upper[1]])+lower
    lengths=np.linalg.norm(cells+.5-start,axis=1)
    for index in np.argsort(lengths,kind='stable'):
        if lengths[index]>radius+1e-12:break
        if lengths[index]<min_length_cells-1e-12:continue
        target=tuple(map(int,cells[index]));end=cells[index]+.5
        if not line_is_collision_free(endpoint,target,stop_free):continue
        if segment_clearance(start,end,~stop_free,resolution,resolution)<=1e-12:continue
        # Include the cells touched by the true sub-cell chord as well as the
        # supercover used by FMM visibility. Slab intersection is exact here.
        lo=np.maximum(0,np.floor(np.minimum(start,end)).astype(int))
        hi=np.minimum(free.shape,np.floor(np.maximum(start,end)).astype(int)+1)
        rr,cc=np.mgrid[lo[0]:hi[0],lo[1]:hi[1]]
        squares=np.column_stack((rr.ravel(),cc.ravel()))
        enter=np.zeros(len(squares));leave=np.ones(len(squares));intersects=np.ones(len(squares),bool)
        delta=end-start
        for axis in (0,1):
            if delta[axis]==0:
                intersects &= (start[axis]>=squares[:,axis]) & (start[axis]<=squares[:,axis]+1)
            else:
                a=(squares[:,axis]-start[axis])/delta[axis]
                b=(squares[:,axis]+1-start[axis])/delta[axis]
                enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        chord={tuple(map(int,p)) for p in squares[intersects & (enter<=leave)]}
        chord.update(supercover_line(endpoint,target))
        if not all(stop_free[p] for p in chord):continue
        result=free.copy()
        opened=[p for p in sorted(chord) if not free[p]]
        for p in opened:result[p]=True
        return result,dict(endpoint=kind,endpoint_xy=list(map(float,xy)),
            anchor_xy=[float(origin[0]+(target[1]+.5)*resolution),float(origin[1]+(target[0]+.5)*resolution)],
            length_m=float(lengths[index]*resolution),max_length_m=radius*resolution,
            policy='bounded_straight_stop_safe',opened_cells_rc=opened)
    raise GoalUnreachable(f'{kind}_inflation_connection_unavailable: no bounded stop-clear connection to planning space')


class _ClearanceGate:
    """Optional hard chord gate; index ONE is the only clearance exception."""
    def __init__(self, occupied, free, padded, state, origin, resolution, start_xy,
                 goal_xy, target, clip_goal, threshold):
        self.occupied,self.free,self.padded=occupied,free,padded
        self.state,self.origin,self.resolution=state,origin,resolution
        self.start_xy=tuple(map(float,start_xy))
        self.start_rc=(np.asarray(start_xy)[::-1]-origin[::-1])/resolution
        self.goal_xy,self.target,self.clip_goal=goal_xy,target,clip_goal
        self.threshold=threshold
        self.cache={}

    def world(self, point):
        if self.clip_goal and tuple(point)==self.target:return tuple(map(float,self.goal_xy))
        return (float(self.origin[0]+(point[1]-1+.5)*self.resolution),
                float(self.origin[1]+(point[0]-1+.5)*self.resolution))

    def check(self, index, point, exact_xy=None):
        xy=self.world(point) if exact_xy is None else tuple(map(float,exact_xy))
        key=(index,*xy)
        if key not in self.cache:
            end_rc=(np.asarray(xy)[::-1]-self.origin[::-1])/self.resolution
            hard=bool(line_is_collision_free(self.state,point,self.padded) and
                segment_clearance(self.start_rc,end_rc,~self.free,self.resolution,self.resolution)>1e-12)
            clearance=segment_clearance(self.start_rc,end_rc,self.occupied,self.resolution,self.threshold)
            wide=clearance>=self.threshold-1e-12
            self.cache[key]=dict(macro_index=index,stg_xy=xy,hard_clear=hard,
                line_clearance_m_capped=clearance,clearance_ok=wide,eligible=hard and (index==1 or wide))
        return self.cache[key]

    def allows(self, index, point):
        return self.check(index,point)['eligible']

    def describe(self, selected, path, kind='macro', exact_xy=None):
        candidates=[self.check(i,p,exact_xy if p==selected else None) for i,p in enumerate(path[1:],1)]
        chosen=next((row for row,p in zip(candidates,path[1:]) if p==selected),None)
        exception=bool(chosen and chosen['macro_index']==1 and not chosen['clearance_ok'])
        return dict(threshold_m=self.threshold,measurement='continuous_segment_to_occupied_cell_squares',
            start_xy=self.start_xy,scope='entire_chord_including_start',kind=kind,
            selected_macro_index=chosen['macro_index'] if chosen else None,
            first_macro_exception=exception,
            fallback_reason='first_stg_only' if exception and not any(c['eligible'] for c in candidates[1:]) else None,
            line_clearance_m_capped=chosen['line_clearance_m_capped'] if chosen else None,
            candidates=candidates)


def plan_snapshot(occupancy, origin, resolution, start_xy, goal_xy, *,
                  inflation_cells=5, step_cells=10, max_stgs=4, min_step_cells=4,
                  direct_goal_range=.50, clearance_weight=0., clearance_safe_distance=.60,
                  clip_stg_goal=False, stg_clearance_m=0., allow_start_in_inflation=False,
                  allow_goal_in_inflation=False, stop_inflation_cells=None):
    """No yaw input, motion commands or mutation of the source occupancy map."""
    occupied=np.asarray(occupancy,dtype=bool)
    if occupied.ndim!=2 or min(occupied.shape)<3:
        raise ValueError('Invalid occupancy grid')
    if not math.isclose(resolution,.05):
        raise ValueError('FMM requires the configured 5 cm grid')
    if not math.isfinite(direct_goal_range) or direct_goal_range < 0:
        raise ValueError('Invalid direct-goal range')
    if isinstance(stg_clearance_m,(bool,np.bool_)) or not math.isfinite(stg_clearance_m) or not 0<=stg_clearance_m<=2.:
        raise ValueError('Invalid STG chord clearance')
    if (not math.isfinite(clearance_weight) or not 0 <= clearance_weight <= 10 or
            not math.isfinite(clearance_safe_distance) or clearance_safe_distance <= 0 or
            not isinstance(clip_stg_goal, (bool, np.bool_))):
        raise ValueError('Invalid clearance planning parameters')
    if (not all(isinstance(v, (int, np.integer)) for v in
                (inflation_cells, step_cells, max_stgs, min_step_cells)) or
            inflation_cells<0 or min(step_cells,max_stgs,min_step_cells)<1 or
            min_step_cells>step_cells):
        raise ValueError('Invalid FMM parameters')
    if not all(isinstance(v,(bool,np.bool_)) for v in (allow_start_in_inflation,allow_goal_in_inflation)):
        raise ValueError('Inflation allowances must be booleans')
    if stop_inflation_cells is None:stop_inflation_cells=inflation_cells
    if (isinstance(stop_inflation_cells,(bool,np.bool_)) or not isinstance(stop_inflation_cells,(int,np.integer)) or
            not 0<=stop_inflation_cells<=inflation_cells):
        raise ValueError('Stop inflation must be an integer from zero to planning inflation')
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
    stop_free=free if stop_inflation_cells==inflation_cells else ~binary_dilation(occupied,disk(stop_inflation_cells))
    if not stop_free[start]:
        raise StartInObstacle('start_in_obstacle: current cell is occupied after stop inflation')
    if not stop_free[goal]:raise ValueError('Goal is occupied after stop inflation; redraw the goal')
    connections=[]
    # Connect both endpoints only to the original planning space, not to each
    # other's newly opened cells. No connected planning space means rejection.
    original_free=free
    if not free[start]:
        if not allow_start_in_inflation:
            raise StartInObstacle('start_in_obstacle: current cell is occupied after obstacle inflation')
        free,connection=_connector(original_free,stop_free,start_xy,start,origin,resolution,inflation_cells,'start',
                                   min_length_cells=min_step_cells)
        connections.append(connection)
    if not original_free[goal]:
        if not allow_goal_in_inflation:
            raise ValueError('Goal is occupied after obstacle inflation; redraw the goal')
        connected,connection=_connector(original_free,stop_free,goal_xy,goal,origin,resolution,inflation_cells,'goal')
        free=free | connected
        connections.append(connection)

    # All original obstacles and the stop band remain blocked. Keep the
    # original padding and numerical kernel beyond the two local connectors.
    r,c=start
    padded=np.pad(free,1,constant_values=True)
    state=(r+1,c+1); target=(goal[0]+1,goal[1]+1)
    components,_=label(padded)
    reachable=components==components[target]
    if not reachable[state]:raise GoalUnreachable('Goal is unreachable from the current position')
    goals=np.zeros(padded.shape,dtype=np.uint8);goals[target]=1
    planner=FMMPlanner(padded)
    cost = None
    if clearance_weight:
        clearance = (distance_transform_edt(~occupied)*resolution if occupied.any()
                     else np.full(occupied.shape, np.inf))
        # Dimensionless density integrated along the route, not a local value
        # added to an already-computed distance field. 0.60 m/weight are uncalibrated.
        density = 1.+clearance_weight*np.maximum(0., (clearance_safe_distance-clearance)/clearance_safe_distance)
        cost = np.pad(density, 1, mode='edge')
        planner.set_multi_goal(goals, cost=cost)
        geometric = FMMPlanner(padded)
        geometric.set_multi_goal(goals)
        distances = geometric.fmm_dist[1:-1,1:-1].copy()*resolution
        cost_to_goal = planner.fmm_dist[1:-1,1:-1].copy()*resolution
        cost_to_goal[~reachable[1:-1,1:-1]] = np.inf
    else:
        planner.set_multi_goal(goals)
        distances=planner.fmm_dist[1:-1,1:-1].copy()*resolution
        cost_to_goal = None
    distances[~reachable[1:-1,1:-1]]=np.inf
    gate=(_ClearanceGate(occupied,free,padded,state,origin,resolution,start_xy,goal_xy,
                        target,clip_stg_goal,stg_clearance_m) if stg_clearance_m else None)
    def result(stg,path,stopped,reason):
        xy=None if stg is None else (float(origin[0]+(stg[1]-1+.5)*resolution),
                                     float(origin[1]+(stg[0]-1+.5)*resolution))
        final = bool(clip_stg_goal and stg is not None and tuple(stg) == target)
        if final:
            xy = tuple(map(float, goal_xy))
        return Plan(distances,free,start,goal,xy,[(p[0]-1,p[1]-1) for p in path],
                    stopped,reason,final_stg=final,cost_to_goal=cost_to_goal,
                    stg_selection=gate.describe(stg,path) if gate else None,inflation_connections=connections or None)
    start_connection=next((c for c in connections if c['endpoint']=='start'),None)
    if start_connection is not None:
        anchor=cell(start_connection['anchor_xy']);anchor=(anchor[0]+1,anchor[1]+1)
        if gate is not None and not gate.check(1,anchor)['eligible']:
            raise NoVisibleSTG('Start inflation connector fails continuous hard corridor check')
        return result(anchor,[state,anchor],False,'Start inflation connector selected as first STG')
    if (np.linalg.norm(np.asarray(start_xy)-goal_xy) <= direct_goal_range+1e-12 and
            line_is_collision_free(start,goal,free) and
            (gate is None or gate.check(1,target,goal_xy)['eligible'])):
        return Plan(distances,free,start,goal,tuple(map(float,goal_xy)),[start,goal],False,
                    'Exact goal selected as final STG',final_stg=True,cost_to_goal=cost_to_goal,
                    stg_selection=gate.describe(target,[state,target],'direct_goal',goal_xy) if gate else None,
                    inflation_connections=connections or None)
    def next_stg(position, radius):
        if (clip_stg_goal and np.linalg.norm(np.asarray(position)-target) <= radius+1e-12 and
                line_is_collision_free(position, target, padded)):
            return target, False, False
        row,col,replan,stop=planner.get_short_term_goal(position,radius)
        return (float(row),float(col)),replan,stop

    def progressing(position, previous):
        row,col=position
        if not (math.isfinite(row) and math.isfinite(col) and
                1<=row<=occupied.shape[0] and 1<=col<=occupied.shape[1]):
            return False
        cell=(int(row),int(col))
        return (padded[cell] and reachable[cell] and
                planner.fmm_dist[cell] < planner.fmm_dist[tuple(map(int,previous))])

    first,replan,stop=next_stg(state,step_cells)
    if stop:return result(None,[],True,'Within Go2 FMM stop distance (25 cm)')
    path=[state]
    if not replan and progressing(first,state):
        path.append(first);current=first
        for _ in range(1,max_stgs):
            if clip_stg_goal and tuple(current) == target:
                break
            nxt,replan,stop=next_stg(current,step_cells)
            if stop or replan or nxt in path or not progressing(nxt,current):break
            path.append(nxt);current=nxt
    filters={'candidate_filter':gate.allows} if gate else {}
    selected = (get_stg(path,padded,**filters) if cost is None else
                get_cost_stg(path,padded,cost,planner.fmm_dist,**filters))
    if selected is not None:
        return result(selected,path,False,'STG macro selected from the current snapshot')

    # Retry only the best candidate at each smaller radius, from the same start
    # and distance field. No extra rollout, sleep or minimum metric-distance test.
    for radius in range(step_cells-1,min_step_cells-1,-1):
        candidate,replan,stop=next_stg(state,radius)
        if stop:return result(None,[],True,'Within Go2 FMM stop distance (25 cm)')
        if (not replan and progressing(candidate,state) and
                line_is_collision_free(state,candidate,padded) and
                (gate is None or gate.allows(1,candidate))):
            return result(candidate,[state,candidate],False,
                          f'STG selected with search radius {radius*resolution:.2f} m')
    raise NoVisibleSTG(
        f'No visible progressing STG at search radii '
        f'{step_cells*resolution:.2f} to {min_step_cells*resolution:.2f} m')
