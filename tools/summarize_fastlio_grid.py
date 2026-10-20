"""Summarize the voxel grid with identical gravity frame, plot scales and static interval."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
os.environ.setdefault('MPLCONFIGDIR','/tmp/g1_mpl_cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parents[1]
out=root/'diagnostics/voxel_grid_20260927'
manifest=json.loads((out/'manifest.json').read_text())
audit=json.loads((out/'bag_audit.json').read_text())
t0=audit['/utlidar/cloud_livox_mid360']['stamp_start_ns']/1e9
up=np.array(audit['/utlidar/imu_livox_mid360']['first_2s_mean'][:3]);up/=np.linalg.norm(up)
x=np.array([1.,0,0]);x-=up*np.dot(x,up);x/=np.linalg.norm(x)
R=np.stack([x,np.cross(up,x),up]);T=np.eye(4);T[:3,:3]=R
np.savetxt(out/'T_level_from_camera_init.txt',T)
rows=[];data=[];reference_stamps=None
for job in manifest['jobs']:
 run=out/job['name'];summary=json.loads((run/'summary.json').read_text())
 if not summary['success']:raise RuntimeError(f'Failed run {run}')
 subprocess.run([sys.executable,str(root/'tools/plot_fastlio_bev.py'),str(run),'--audit',str(out/'bag_audit.json')],check=True,stdout=subprocess.DEVNULL)
 a=np.loadtxt(run/'odometry.csv',delimiter=',',skiprows=1)
 if len(a)!=298 or summary['registered_cloud_frames']!=298:
  raise RuntimeError(f'Incomplete capture in {run}: expected 298 poses/clouds')
 if reference_stamps is None:reference_stamps=a[:,0].copy()
 elif not np.allclose(a[:,0],reference_stamps,rtol=0,atol=1e-6):
  raise RuntimeError('Output timestamps differ across runs')
 t=a[:,0]-t0;xyz=a[:,1:4]@R.T
 static=(t>=0)&(t<=3);p=xyz[static];ts=t[static]
 if len(p)<20:raise RuntimeError('Too few stationary poses')
 norms=np.linalg.norm(p,axis=1);relative=np.linalg.norm(p-p[0],axis=1)
 q=a[static,4:8];q=q/np.linalg.norm(q,axis=1)[:,None]
 angle=2*np.arccos(np.clip(np.abs(q@q[0]),0,1))*180/np.pi
 row=dict(surf_m=job['surf'],map_m=job['map'],poses=len(a),cloud_frames=summary['registered_cloud_frames'],
  static_poses=len(p),static_first_s=ts[0],static_last_s=ts[-1],
  static_end_origin_cm=norms[-1]*100,static_max_origin_cm=norms.max()*100,
  static_rms_origin_cm=np.sqrt(np.mean(norms**2))*100,
  static_end_first_pose_cm=relative[-1]*100,static_max_first_pose_cm=relative.max()*100,
  static_xy_max_origin_cm=np.linalg.norm(p[:,:2],axis=1).max()*100,
  static_z_span_cm=np.ptp(p[:,2])*100,
  static_rotation_max_from_first_deg=angle.max(),
  full_x_span_m=np.ptp(xyz[:,0]),full_y_span_m=np.ptp(xyz[:,1]),full_z_span_m=np.ptp(xyz[:,2]))
 (run/'static_metrics.json').write_text(json.dumps(row,indent=2))
 rows.append(row)
 points=np.load(run/'registered_xyz.npy')@R.T
 data.append((job,points,xyz,ts,norms))
with (out/'metrics.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(out/'metrics.json').write_text(json.dumps(rows,indent=2))
# Shared bounds, 5 cm cells and color scale: no per-panel crop or contrast adjustment.
lo=np.min([p.min(axis=0)[:2] for _,p,*_ in data],axis=0)-.5
hi=np.max([p.max(axis=0)[:2] for _,p,*_ in data],axis=0)+.5
bins=np.ceil((hi-lo)/.05).astype(int)
hist=[]
for _,p,*_ in data:
 h,xs,ys=np.histogram2d(p[:,0],p[:,1],bins=bins,range=list(zip(lo,hi)));hist.append(np.log1p(h))
vmax=np.percentile(np.concatenate([h[h>0] for h in hist]),98)
fig,axs=plt.subplots(3,3,figsize=(17,17),sharex=True,sharey=True,layout='constrained')
for ax,(job,p,xyz,ts,norms),h in zip(axs.flat,data,hist):
 ax.imshow(h.T,origin='lower',extent=[xs[0],xs[-1],ys[0],ys[-1]],cmap='Greys',vmin=0,vmax=vmax,interpolation='nearest')
 ax.plot(xyz[:,0],xyz[:,1],color='#e24a33',lw=1)
 ax.scatter(*xyz[0,:2],c='green',marker='s',s=18,zorder=4);ax.scatter(*xyz[-1,:2],c='red',marker='^',s=20,zorder=4)
 ax.set_title(f"surf={job['surf']:.2f}, map={job['map']:.2f} m | static end={norms[-1]*100:.2f} cm")
 ax.set_aspect('equal');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
fig.suptitle('Gravity-aligned BEV + IMU trajectory | shared bounds and density scale | all heights, 5 cm cells')
fig.savefig(out/'bev_grid.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(3,3,figsize=(12,11),sharex=True,sharey=True,layout='constrained')
trajlo=np.min([v[2][:,:2].min(axis=0) for v in data],axis=0)-.05
trajhi=np.max([v[2][:,:2].max(axis=0) for v in data],axis=0)+.05
for ax,(job,p,xyz,ts,norms) in zip(axs.flat,data):
 ax.plot(xyz[:,0],xyz[:,1],lw=1);ax.scatter(*xyz[0,:2],c='green',marker='s');ax.scatter(*xyz[-1,:2],c='red',marker='^')
 ax.set(xlim=(trajlo[0],trajhi[0]),ylim=(trajlo[1],trajhi[1]),xlabel='X (m)',ylabel='Y (m)',title=f"surf={job['surf']:.2f}, map={job['map']:.2f} m")
 ax.set_aspect('equal');ax.grid(alpha=.2)
fig.suptitle('Full-bag trajectory detail | identical gravity-aligned coordinates and scale')
fig.savefig(out/'trajectory_grid.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,6),layout='constrained')
for job,p,xyz,ts,norms in data:ax.plot(ts,norms*100,label=f"{job['surf']:.2f} / {job['map']:.2f}")
ax.set(xlabel='Time since first cloud header (s)',ylabel='3D distance from initialization origin (cm)',title='Opening stationary interval [0, 3 s] | surf / map voxel sizes (m)');ax.legend(ncol=3);ax.grid(alpha=.25)
fig.savefig(out/'static_drift.png',dpi=160);plt.close(fig)
print(json.dumps(rows,indent=2))
ranked=sorted(rows,key=lambda r:r['static_rms_origin_cm'])
lines=['# 网格实验结果','',
 '九组均通过 298 帧、298 位姿及相同位姿时间戳检查。',
 '静止窗口为首帧点云 header 起 0–3 秒；所有位置误差单位为厘米，参考初始化原点。',
 'RMS 是静止样本距原点的距离均方根，不是有外部真值的绝对精度。','',
 '| surf (m) | map (m) | 末尾偏移 (cm) | 最大偏移 (cm) | RMS (cm) |',
 '|---:|---:|---:|---:|---:|']
for r in rows:
 lines.append(f"| {r['surf_m']:.2f} | {r['map_m']:.2f} | {r['static_end_origin_cm']:.3f} | {r['static_max_origin_cm']:.3f} | {r['static_rms_origin_cm']:.3f} |")
lines+=['','按静止 RMS 排序：'+', '.join(f"{r['surf_m']:.2f}/{r['map_m']:.2f}" for r in ranked),
 '',f"各组静止输出范围：{rows[0]['static_first_s']:.6f}–{rows[0]['static_last_s']:.6f} 秒，共 {rows[0]['static_poses']} 个位姿。",
 '', '[BEV 九宫格](bev_grid.png) · [轨迹九宫格](trajectory_grid.png) · [静止漂移曲线](static_drift.png) · [全部指标](metrics.csv)',
 '', '逐组可视化：']
for j in manifest['jobs']:
 lines.append(f"- [{j['name']}]({j['name']}/bev_xy_leveled.png)")
lines+=['','初次启动中缺少开头消息的结果位于 rejected_startup，仅保留日志与位姿供审计，不参与统计。',
 '补跑增加 rosbag --delay 3 等待发现。回放速度仍为 0.25；其他算法参数保持一致。',
 '不同组并发运行，wall time 不用于机载性能排名。用户尚未根据九宫格完成目视选择，主配置保持 0.15/0.15。']
(out/'RESULTS.md').write_text('\n'.join(lines)+'\n')
