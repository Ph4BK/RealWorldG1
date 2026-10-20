#!/usr/bin/env python3
"""Overlay map-frame localization.csv on the fixed 3D map's 5 cm BEV."""
import argparse,csv,os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/g1_mpl_cache')
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(map_path,trajectory_path,output,dataset):
    with trajectory_path.open() as f:
        records=list(csv.DictReader(f))
    if not records:raise ValueError('No localization poses to plot')
    poses=np.array([[float(r[k]) for k in ['stamp','x','y','z']] for r in records])
    if not np.isfinite(poses).all():raise ValueError('Nonfinite localization poses')
    # Both inputs already use map coordinates. Do not apply leveling again.
    cloud=np.asarray(o3d.io.read_point_cloud(str(map_path)).points)
    if not len(cloud) or not np.isfinite(cloud).all():raise ValueError('Invalid reference point cloud')
    # Same grid origin, cell size and base bounds as full_lab_v1/bev_preview.png.
    lo=np.array([-1.7,-7.1]);hi=np.array([8.5,3.1]);resolution=.05
    lo=np.minimum(lo,np.floor((poses[:,1:3].min(axis=0)-.3)/resolution)*resolution)
    hi=np.maximum(hi,np.ceil((poses[:,1:3].max(axis=0)+.3)/resolution)*resolution)
    bins=np.rint((hi-lo)/resolution).astype(int)
    if np.prod(bins)>25_000_000:raise ValueError('Trajectory extent is unexpectedly large')
    edges=[lo[i]+np.arange(bins[i]+1)*resolution for i in range(2)]
    h,xe,ye=np.histogram2d(cloud[:,0],cloud[:,1],bins=edges);v=np.log1p(h)
    fig,ax=plt.subplots(figsize=(10,10),layout='constrained')
    ax.imshow(v.T,origin='lower',extent=[xe[0],xe[-1],ye[0],ye[-1]],cmap='Greys',vmax=max(1.,np.percentile(v[v>0],98)),interpolation='nearest')
    ax.plot(poses[:,1],poses[:,2],color='#e24a33',lw=1.6,label='Localized IMU trajectory',zorder=3)
    ax.scatter(*poses[0,1:3],c='green',marker='s',s=65,edgecolors='white',label='First localized pose',zorder=4)
    ax.scatter(*poses[-1,1:3],c='red',marker='^',s=70,edgecolors='white',label='Last localized pose',zorder=5)
    ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title=f'{dataset} | {map_path.parent.name} | localization trajectory')
    ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='upper right')
    fig.text(.01,.004,f'{len(poses):,} poses | {poses[-1,0]-poses[0,0]:.2f} s of localized trajectory | all heights | 5 cm grid',fontsize=9)
    fig.savefig(output,dpi=180);plt.close(fig)
    print(f'Saved {output} ({len(poses)} poses)')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map',type=Path,required=True);p.add_argument('--trajectory',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--dataset',required=True);a=p.parse_args()
    plot(a.map,a.trajectory,a.output,a.dataset)
if __name__=='__main__':main()
