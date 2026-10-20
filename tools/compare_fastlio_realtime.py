"""Compare single 1x replay with the complete 0.25x voxel-grid baseline."""
from pathlib import Path
import json
import argparse
import numpy as np
r=Path(__file__).resolve().parents[1]/'diagnostics'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',type=Path,default=r/'realtime_1x_voxel010_20260927')
parser.add_argument('--baseline',type=Path,default=r/'voxel_grid_20260927/surf_0.10_map_0.10')
args=parser.parse_args()
out=args.run;baseline=args.baseline
a=np.loadtxt(out/'odometry.csv',delimiter=',',skiprows=1);b=np.loadtxt(baseline/'odometry.csv',delimiter=',',skiprows=1)
timing=np.genfromtxt(out/'output_timing.csv',delimiter=',',skip_header=1,usecols=range(5))
audit=json.loads((r/'voxel_grid_20260927/bag_audit.json').read_text());t0=audit['/utlidar/cloud_livox_mid360']['stamp_start_ns']/1e9
same=len(a)==len(b) and np.allclose(a[:,0],b[:,0],rtol=0,atol=1e-6)
p=a[(a[:,0]-t0>=0)&(a[:,0]-t0<=3),1:4];n=np.linalg.norm(p,axis=1)*100
# Estimate playback-now by advancing the last received clock sample at playback rate 1.
lag=timing[:,4]+timing[:,3]
valid=np.isfinite(lag)&(timing[:,3]<.1)
result=dict(baseline=str(baseline),same_output_timestamps=same,poses=len(a),static_end_cm=n[-1],static_max_cm=n.max(),static_rms_cm=np.sqrt(np.mean(n*n)),
 lag_definition='Latest received /clock extrapolated to odometry callback at rate 1 minus pose stamp. Includes transport/capture scheduling; not pure FAST-LIO processing time.',
 valid_lag_samples=int(valid.sum()),lag_ms_p50_p95_max=(np.percentile(lag[valid],[50,95,100])*1000).tolist(),
 first_5s_lag_median_ms=float(np.median(lag[valid&(a[:,0]-t0<5)])*1000),
 last_5s_lag_median_ms=float(np.median(lag[valid&(a[:,0]-t0>25)])*1000))
if same:
 d=np.linalg.norm(a[:,1:4]-b[:,1:4],axis=1)
 result.update(position_difference_from_baseline_max_m=float(d.max()),position_difference_from_baseline_rms_m=float(np.sqrt(np.mean(d*d))))
(out/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
