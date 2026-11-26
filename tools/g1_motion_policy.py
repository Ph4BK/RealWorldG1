"""Round-one motion candidates and runtime identity; no ROS or motion IO."""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class MotionPolicy:
    near_stg_no_realign: bool = False
    near_stg_distance: float = .30       # User requirement: signed projection, not an arrival radius.
    coarse_turn_profile: bool = False
    coarse_turn_kp: float = 1.5          # Existing fine/guard gain, reused; not G1 calibration.
    coarse_turn_max_wz: float = .5      # 11-25 observed coarse platform; unpaired field evidence.
    coarse_turn_near_wz: float = .3     # 11-25 observed guard platform, not a precision guarantee.
    coarse_turn_min_wz: float = .15     # Existing minimum; not a measured walking/turning dead zone.
    coarse_turn_slow_deg: float = 20.   # Proposed transition, not field calibrated.
    coarse_turn_stop_deg: float = 3.    # Proposed first-round stop threshold.
    coarse_turn_release_deg: float = 5. # Proposed post-settle threshold, same angular reference.
    coarse_turn_timeout_s: float = 12.  # Shared effective-input budget per STG, including settling.
    coarse_turn_attempts: int = 3       # Initial turn + corrections, proposed bounded candidate.

    def __post_init__(self):
        for name,value in asdict(self).items():
            if name in ('near_stg_no_realign','coarse_turn_profile'):
                if not isinstance(value,bool):raise ValueError('Expected boolean: '+name)
            elif isinstance(value,bool) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid motion policy parameter: '+name)
        if not 0 < self.coarse_turn_min_wz <= self.coarse_turn_near_wz <= self.coarse_turn_max_wz <= 1.:
            raise ValueError('Invalid coarse turn rate bounds')
        if not 0 < self.coarse_turn_stop_deg < self.coarse_turn_release_deg < self.coarse_turn_slow_deg < 180.:
            raise ValueError('Invalid coarse turn angular bounds')
        if (not isinstance(self.coarse_turn_attempts,int) or not 1<=self.coarse_turn_attempts<=8 or
                not .15 < self.near_stg_distance <= 1.):
            raise ValueError('Invalid coarse turn attempts or near-STG projection')


def effective_planning_config(config, options):
    result=dict(config)
    for name in ('fmm_inflation_cells','fmm_step_cells'):
        if options.get(name) is not None:result[name]=options[name]
    for name in ('fmm_inflation_cells','fmm_step_cells','fmm_min_step_cells','fmm_max_stgs'):
        value=result[name]
        minimum=0 if name=='fmm_inflation_cells' else 1
        if isinstance(value,bool) or not isinstance(value,int) or not minimum<=value<=1000:
            raise ValueError('Invalid planning configuration: '+name)
    if result['fmm_min_step_cells']>result['fmm_step_cells']:
        raise ValueError('FMM step is below its minimum search radius')
    stop=options.get('fmm_stop_inflation_cells')
    if stop is None:stop=result.get('fmm_stop_inflation_cells')
    if stop is None:stop=result['fmm_inflation_cells']
    if isinstance(stop,bool) or not isinstance(stop,int) or not 0<=stop<=result['fmm_inflation_cells']:
        raise ValueError('Stop inflation must be an integer from zero to planning inflation')
    result['fmm_stop_inflation_cells']=stop
    return result


def inflation_masks(occupied, config):
    """Planning preference and execution stop band, on the same immutable grid."""
    from skimage.morphology import binary_dilation, disk
    planning=~binary_dilation(occupied,disk(config['fmm_inflation_cells']))
    stop_cells=config['fmm_stop_inflation_cells']
    stop=planning if stop_cells==config['fmm_inflation_cells'] else ~binary_dilation(occupied,disk(stop_cells))
    return planning,stop


def band_planning_options(config, options):
    """Both preview and navigation use the planner's bounded endpoint connectors."""
    if config['fmm_stop_inflation_cells']==config['fmm_inflation_cells'] and not options['allow_goal_in_inflation']:
        return {}
    return dict(stop_inflation_cells=config['fmm_stop_inflation_cells'],
        allow_start_in_inflation=config['fmm_stop_inflation_cells']<config['fmm_inflation_cells'],
        allow_goal_in_inflation=options['allow_goal_in_inflation'])


def inflation_start_status(planning, stop, origin, resolution, xy, *, nearest=True):
    from live_bev_core import grid_start_status
    result=grid_start_status(stop,origin,resolution,xy,nearest=nearest)
    plan=grid_start_status(planning,origin,resolution,xy,nearest=False)
    return dict(result,planning_start_free=plan['start_free'],stop_start_free=result['start_free'],
        planning_start_rejection_reason=plan['start_rejection_reason'],
        start_requires_connector=result['start_free'] is True and plan['start_free'] is False,
        start_admission_basis='stop_inflation')


def inflation_parameters(config, options, resolution=.05):
    return dict(planning_check_inflation_m=config['fmm_inflation_cells']*resolution,
        stop_check_inflation_m=config['fmm_stop_inflation_cells']*resolution,
        allow_goal_in_inflation=options['allow_goal_in_inflation'],
        stop_check_scope='entire_stg_segment',fine_check_scope='entire_goal_segment',
        endpoint_connection_policy='bounded_straight_stop_safe',
        endpoint_connection_max_length_m=(config['fmm_inflation_cells']+1)*resolution)


def source_identity(directory):
    """Capture startup disk-source hashes once, never in the control loop."""
    names=('g1_motion_policy.py','g1_navigation_core.py','g1_walk_guard.py','g1_final_approach.py',
           'g1_navigation_options.py','g1_navigation_node.py','live_bev_node.py','g1_action_server.py',
           'posnav_fmm/planner.py','posnav_fmm/visibility.py','posnav_fmm/fmm_planner.py')
    return {name:hashlib.sha256((Path(directory)/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
            for name in names}


def grid_identity(occupied, origin, resolution):
    if occupied is None:return None
    metadata=dict(shape=list(occupied.shape),origin_xy_m=list(map(float,origin)),resolution_m=float(resolution))
    data=json.dumps(metadata,sort_keys=True,separators=(',',':')).encode()+occupied.astype('uint8').tobytes()
    return dict(metadata,sha256=hashlib.sha256(data).hexdigest())


def runtime_parameters(control, config, options, sources):
    values={key:getattr(control,key) for key in ('speed','omega','epsilon','turn_out_enabled',
        'turn_settle','turn_stop_window','turn_stop_yaw_rate','turn_stop_extra','walk_settle')}
    values.update(coarse_turn_mode='profile' if control.motion.coarse_turn_profile else 'legacy',
        effective_coarse_turn_max_wz=control.motion.coarse_turn_max_wz if control.motion.coarse_turn_profile else control.omega,
        effective_guard_realign_max_wz=(control.motion.coarse_turn_near_wz if control.motion.coarse_turn_profile else
            control.walk_guard.config.align_max_wz if control.walk_guard else None),
        small_angle_excursion_effective=control.turn_out_enabled and not control.motion.coarse_turn_profile)
    return dict(schema=1,role='navigation',
        control=values,
        motion_policy=asdict(control.motion),
        walk_guard=asdict(control.walk_guard.config) if control.walk_guard else None,
        fine=asdict(control.fine),
        planning={key:config[key] for key in ('fmm_inflation_cells','fmm_step_cells','fmm_max_stgs','fmm_min_step_cells')},
        clearance={key:options[key] for key in ('clearance_weight','clearance_safe_distance','clip_stg_goal','stg_clearance_m')},
        **inflation_parameters(config,options),
        startup_source_sha256_lf=sources)
