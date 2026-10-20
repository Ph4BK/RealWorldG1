"""Plot all extracted points in one XY projection, without pose transforms."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    p=np.load(args.directory/'points_xyz.npy',mmap_mode='r')
    s=json.loads((args.directory/'summary.json').read_text())
    resolution=.01
    lower=np.floor(p[:,:2].min(axis=0)/resolution)*resolution
    upper=np.ceil(p[:,:2].max(axis=0)/resolution)*resolution+resolution
    edges=[np.arange(lower[i],upper[i]+resolution*.5,resolution) for i in range(2)]
    counts,_,_=np.histogram2d(p[:,0],p[:,1],bins=edges)
    assert int(counts.sum())==len(p), 'Some points fell outside plot bounds'
    fig,ax=plt.subplots(figsize=(11,10),layout='constrained')
    im=ax.imshow(np.ma.masked_equal(counts.T,0),origin='lower',extent=[edges[0][0],edges[0][-1],edges[1][0],edges[1][-1]],cmap='Greys',norm=LogNorm(vmin=1,vmax=max(2,int(counts.max()))),interpolation='nearest')
    ax.scatter([0],[0],marker='+',s=120,color='red',linewidths=1.5,label='livox_frame origin')
    ax.set_aspect('equal'); ax.set_xlabel('Raw X in livox_frame (m)'); ax.set_ylabel('Raw Y in livox_frame (m)')
    ax.set_title(f"G1 MID360: all {s['frames']} frames accumulated in raw XY\n{len(p):,} points | no odom / extrinsic transform | no height filter")
    ax.grid(alpha=.15); ax.legend(loc='upper right')
    fig.colorbar(im,ax=ax,shrink=.8,label='Point count per 1 cm display cell (log scale; not time)')
    path=args.directory/'xy_accumulated.png'
    fig.savefig(path,dpi=180); plt.close(fig)
    print(path)

if __name__=='__main__': main()
