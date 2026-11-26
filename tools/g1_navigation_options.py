"""Shared opt-in options for LiveBEV preview and its navigation child."""
import math
from dataclasses import asdict
from g1_final_approach import FineApproachConfig
from g1_walk_guard import WalkGuardConfig
from g1_motion_policy import MotionPolicy

FINE_DEFAULTS = asdict(FineApproachConfig())
MOTION_DEFAULTS = asdict(MotionPolicy())
GUARD_DEFAULTS = {'guard_'+key:value for key,value in asdict(WalkGuardConfig()).items()}
RUNTIME_DEFAULTS = dict(fmm_inflation_cells=None,fmm_step_cells=None,coarse_omega=None,no_turn_out=False,
                        stg_clearance_m=0.,fmm_stop_inflation_cells=None,allow_goal_in_inflation=False)


def add_navigation_options(parser):
    parser.add_argument('--walk-guard', action='store_true',
                        help='Candidate fresh-sample turn/walk guards; G1 thresholds not calibrated')
    parser.add_argument('--clearance-weight', type=float, default=0.,
                        help='Candidate integrated clearance cost; 0 off, start field trials at 1, '
                             'compare 0.5/2; valid 0..10, weights above 2 not validated')
    parser.add_argument('--clearance-safe-distance', type=float, default=.60,
                        help='Soft clearance reference in metres, not an inflation radius')
    parser.add_argument('--clip-stg-goal', action='store_true',
                        help='Clip macro rollout at its current planning goal/pre')
    parser.add_argument('--allow-goal-in-inflation',action='store_true',
                        help='Allow a goal cell inside the planning inflation when it is outside the stop inflation')
    parser.add_argument('--fmm-stop-inflation-cells',type=int,default=None,
                        help='Stop-side inflation in cells (5cm); default follows fmm-inflation-cells')
    for key in ('fmm-inflation-cells','fmm-step-cells'):
        parser.add_argument('--'+key,type=int,default=None,help='Shared preview/navigation override; 5cm cells')
    parser.add_argument('--coarse-omega',type=float,default=None,help='Explicit legacy coarse rate override')
    parser.add_argument('--no-turn-out',action='store_true',help='Disable legacy small-angle excursion')
    parser.add_argument('--stg-clearance-m',type=float,default=0.,
                        help='Hard clearance for later macro chords; 0 off, user candidate 0.40m; first macro keeps hard collision check')
    for key, value in {**FINE_DEFAULTS,**MOTION_DEFAULTS,**GUARD_DEFAULTS}.items():
        flag = '--'+key.replace('_', '-')
        if isinstance(value, bool):
            parser.add_argument(flag, action='store_true', help=(
                'Round-one motion candidate; requires --walk-guard; default off' if key in MOTION_DEFAULTS
                else 'Independent opt-in fine candidate; default off'))
        else:
            parser.add_argument(flag, type=type(value), default=value,
                help='Recorded navigation parameter; G1 calibration pending (default: %(default)s)')


def checked_options(walk_guard=False, clearance_weight=0., clearance_safe_distance=.60,
                    clip_stg_goal=False, **fine):
    if not isinstance(walk_guard, bool) or not isinstance(clip_stg_goal, bool):
        raise ValueError('Navigation switches must be booleans')
    if (not math.isfinite(clearance_weight) or not 0 <= clearance_weight <= 10 or
            not math.isfinite(clearance_safe_distance) or clearance_safe_distance <= 0):
        raise ValueError('Invalid clearance planning parameters')
    motion={key:fine.pop(key,value) for key,value in MOTION_DEFAULTS.items()}
    guard={key:fine.pop(key,value) for key,value in GUARD_DEFAULTS.items()}
    runtime={key:fine.pop(key,value) for key,value in RUNTIME_DEFAULTS.items()}
    MotionPolicy(**motion)
    WalkGuardConfig(**{key[6:]:value for key,value in guard.items()})
    if (motion['near_stg_no_realign'] or motion['coarse_turn_profile']) and not walk_guard:
        raise ValueError('Round-one motion candidates require --walk-guard')
    for key in ('no_turn_out','allow_goal_in_inflation'):
        if not isinstance(runtime[key],bool):raise ValueError('Expected boolean: '+key)
    value=runtime['stg_clearance_m']
    if isinstance(value,bool) or not math.isfinite(value) or not 0<=value<=2.:
        raise ValueError('Invalid STG chord clearance')
    for key in ('fmm_inflation_cells','fmm_step_cells','fmm_stop_inflation_cells'):
        value=runtime[key];minimum=0 if key=='fmm_inflation_cells' else 1
        if key=='fmm_stop_inflation_cells':minimum=0
        if value is not None and (isinstance(value,bool) or not isinstance(value,int) or not minimum<=value<=1000):
            raise ValueError('Invalid FMM override: '+key)
    if (runtime['fmm_stop_inflation_cells'] is not None and runtime['fmm_inflation_cells'] is not None and
            runtime['fmm_stop_inflation_cells']>runtime['fmm_inflation_cells']):
        raise ValueError('Stop inflation must not exceed planning inflation')
    omega=runtime['coarse_omega']
    if omega is not None and (isinstance(omega,bool) or not math.isfinite(omega) or not 0<omega<=1.):
        raise ValueError('Invalid coarse omega override')
    try:
        fine = asdict(FineApproachConfig(**fine))
    except TypeError as exc:
        raise ValueError('Unknown fine option: '+str(exc)) from exc
    return dict(walk_guard=walk_guard, clearance_weight=float(clearance_weight),
                clearance_safe_distance=float(clearance_safe_distance), clip_stg_goal=clip_stg_goal,
                **fine,**motion,**guard,**runtime)


def guard_config(options):
    return WalkGuardConfig(**{key[6:]:options[key] for key in GUARD_DEFAULTS})


def motion_config(options):
    return MotionPolicy(**{key:options[key] for key in MOTION_DEFAULTS})


def options_from_args(args):
    return checked_options(**{key: getattr(args, key, default)
                              for key, default in checked_options().items()})


def planning_options(options):
    if not options['clearance_weight'] and not options['clip_stg_goal'] and not options['stg_clearance_m']:
        return {}  # Preserve the legacy call and numerical path exactly.
    values={key: options[key] for key in ('clearance_weight', 'clearance_safe_distance', 'clip_stg_goal')}
    if options['stg_clearance_m']:values['stg_clearance_m']=options['stg_clearance_m']
    return values


def forward_navigation_options(args):
    options = options_from_args(args)
    command = ['--walk-guard'] if options['walk_guard'] else []
    if options['clearance_weight']:
        command += ['--clearance-weight', str(options['clearance_weight']),
                    '--clearance-safe-distance', str(options['clearance_safe_distance'])]
    if options['clip_stg_goal']:
        command += ['--clip-stg-goal']
    for key, default in {**FINE_DEFAULTS,**MOTION_DEFAULTS,**GUARD_DEFAULTS,**RUNTIME_DEFAULTS}.items():
        value = options[key]
        if isinstance(default, bool):
            if value:
                command += ['--'+key.replace('_', '-')]
        elif value != default:
            command += ['--'+key.replace('_', '-'), str(value)]
    return command
