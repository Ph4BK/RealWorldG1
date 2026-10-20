"""Plot accumulated registered points and the matching IMU trajectory, without height filtering."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/g1_mpl_cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def draw(points, poses, output, title):
    resolution = 0.05
    lo = np.minimum(points[:,:2].min(axis=0),poses[:,:2].min(axis=0)) - 0.5
    hi = np.maximum(points[:,:2].max(axis=0),poses[:,:2].max(axis=0)) + 0.5
    bins = np.maximum(1, np.ceil((hi-lo)/resolution).astype(int))
    if np.prod(bins)>25_000_000:
        raise ValueError('Map extent is too large; inspect trajectory divergence before plotting')
    hits, xs, ys = np.histogram2d(points[:,0],points[:,1], bins=bins,range=list(zip(lo,hi)))
    fig, ax = plt.subplots(figsize=(10,9), layout='constrained')
    ax.imshow(np.log1p(hits.T), origin='lower', extent=[xs[0],xs[-1],ys[0],ys[-1]],
              cmap='Greys', interpolation='nearest',
              vmax=max(1,np.percentile(np.log1p(hits[hits>0]),98)))
    ax.plot(poses[:,0],poses[:,1],color='#e24a33',lw=1.8,label='FAST-LIO IMU/body trajectory')
    ax.scatter(*poses[0,:2],marker='s',s=60,color='#20854e',edgecolors='white',zorder=5,label='Start')
    ax.scatter(*poses[-1,:2],marker='^',s=75,color='#a00020',edgecolors='white',zorder=5,label='End')
    ax.set(xlabel='X (m)',ylabel='Y (m)',title=title)
    ax.set_aspect('equal')
    ax.legend(loc='best',fontsize=9)
    ax.grid(alpha=.12)
    fig.text(.01,.005,f'{len(points):,} registered points | all heights | 5 cm density grid | unvalidated trajectory',fontsize=8)
    fig.savefig(output,dpi=170)
    plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--audit',type=Path,required=True)
    args=p.parse_args()
    points=np.load(args.run/'registered_xyz.npy')
    trajectory=np.genfromtxt(args.run/'odometry.csv',delimiter=',',skip_header=1)
    xyz=trajectory[:,1:4]
    if len(xyz)<2 or not np.isfinite(xyz).all():
        raise ValueError('No finite trajectory')
    draw(points,xyz,args.run/'bev_xy_raw.png','Registered point cloud + trajectory (FAST-LIO camera_init XY)')
    audit=json.loads(args.audit.read_text())
    up=np.array(audit['/utlidar/imu_livox_mid360']['first_2s_mean'][:3]);up/=np.linalg.norm(up)
    forward=np.array([1.,0.,0.]);forward-=up*np.dot(forward,up);forward/=np.linalg.norm(forward)
    rotation=np.stack([forward,np.cross(up,forward),up])
    leveled_points=points@rotation.T
    leveled_xyz=xyz@rotation.T
    draw(leveled_points,leveled_xyz,args.run/'bev_xy_leveled.png','Registered point cloud + trajectory (gravity-aligned XY)')
    T=np.eye(4);T[:3,:3]=rotation
    np.savetxt(args.run/'T_level_from_camera_init.txt',T)
    result=dict(points=len(points),poses=len(xyz),height_filter='none',
                trajectory_reference='FAST-LIO IMU/body, not calibrated G1 base center',
                raw_xyz_span_m=np.ptp(xyz,axis=0).tolist(),
                leveled_xyz_span_m=np.ptp(leveled_xyz,axis=0).tolist(),
                path_length_3d_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
                start_to_end_3d_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
                first_2s_displacement_m=float(np.linalg.norm(xyz[np.searchsorted(trajectory[:,0],trajectory[0,0]+2)]-xyz[0])),
                quality_validated=False,
                leveling='Initial 2 s mean accelerometer defines up; original x projected horizontally; no scale change')
    (args.run/'visualization_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
