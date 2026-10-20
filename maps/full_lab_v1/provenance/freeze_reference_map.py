#!/usr/bin/env python3
"""Freeze a versioned full_lab reference; refuses to overwrite an existing version."""
import argparse, hashlib, json, os, shutil
from scipy.spatial import cKDTree
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/g1_mpl_cache')
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--replace',action='store_true',help='Replace an existing generated map package after successful generation');a=ap.parse_args()
    s=a.source.resolve();d=a.output.resolve()
    target=d
    if target.exists() and not a.replace: raise SystemExit('Existing map: use --replace explicitly')
    if target.exists() and not (target/'metadata.yaml').is_file(): raise SystemExit('Refusing to replace unrecognized directory')
    d=target.with_name('.'+target.name+'.building')
    if d.exists(): raise SystemExit('Staging directory already exists')
    raw=np.load(s/'registered_xyz.npy').astype(np.float64)
    valid=np.isfinite(raw).all(axis=1);raw=raw[valid]
    L=np.loadtxt(s/'T_level_from_camera_init.txt');level=raw@L[:3,:3].T+L[:3,3]
    # Explicit lab ROI in the previously validated gravity-aligned frame.
    low=np.array([-1.6,-7.0,-2.5]);high=np.array([8.3,3.0,2.5])
    keep=((level>=low)&(level<=high)).all(axis=1)
    fit=level[(level[:,0]>0)&(level[:,0]<7.5)&(level[:,1]>-6)&(level[:,1]<2)&(level[:,2]>-1.45)&(level[:,2]<-0.95)]
    o3d.utility.random.seed(42)
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(fit))
    plane,idx=cloud.segment_plane(distance_threshold=.025,ransac_n=3,num_iterations=3000)
    plane=np.asarray(plane);plane*=1 if plane[2]>0 else -1
    normal=plane[:3];tilt=np.degrees(np.arccos(normal[2]))
    if tilt>3 or len(idx)<1000: raise RuntimeError('Ambiguous floor fit')
    # Preserve +X heading by projecting it onto the fitted floor.
    x=np.array([1.,0,0]);x-=normal*np.dot(x,normal);x/=np.linalg.norm(x)
    F=np.eye(4);F[:3,:3]=np.stack([x,np.cross(normal,x),normal]);F[2,3]=plane[3]
    T=F@L
    p=level[keep]@F[:3,:3].T+F[:3,3]
    # Conservative isolated-point removal; retain dense small structures.
    # Query only three nearest neighbours to avoid enumerating huge dense radius neighbourhoods.
    distances,_=cKDTree(p).query(p,k=3,workers=4)
    supported=distances[:,-1]<=.25
    c=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p[supported]))
    before_voxel=len(c.points);c=c.voxel_down_sample(.05);p=np.asarray(c.points)
    assert len(p)>10000 and np.isfinite(p).all()
    assert np.allclose(T[:3,:3]@T[:3,:3].T,np.eye(3),atol=1e-10) and np.linalg.det(T[:3,:3])>0
    od=np.loadtxt(s/'odometry.csv',delimiter=',',skiprows=1)
    xyz=od[:,1:4]@T[:3,:3].T+T[:3,3]
    quat=(Rotation.from_matrix(T[:3,:3])*Rotation.from_quat(od[:,4:8])).as_quat()
    d.mkdir(parents=True)
    (d/'source').mkdir();(d/'provenance').mkdir()
    for name in ['registered_xyz.npy','T_level_from_camera_init.txt','odometry.csv']:
        shutil.copy2(s/name,d/'source'/name)
    shutil.copy2(s.parent/'config.yaml',d/'provenance'/'mapping_config.yaml')
    shutil.copy2(__file__,d/'provenance'/'freeze_reference_map.py')
    if not o3d.io.write_point_cloud(str(d/'map.pcd'),c,write_ascii=False,compressed=True): raise RuntimeError('PCD write failed')
    np.savetxt(d/'T_map_from_camera_init.txt',T)
    np.savetxt(d/'T_map_from_gravity_level.txt',F)
    np.savetxt(d/'trajectory_map.csv',np.column_stack([od[:,0],xyz,quat]),delimiter=',',header='stamp,x,y,z,qx,qy,qz,qw',comments='')
    floor_map=fit[idx]@F[:3,:3].T+F[:3,3]
    # Display dense registered observations for comparison; saved map has its own preview.
    dense=np.load(s/'registered_xyz.npy');dense=dense@T[:3,:3].T+T[:3,3]
    def bev(points,name,traj=False):
        edges=[np.arange(-1.7,8.51,.05),np.arange(-7.1,3.11,.05)]
        h,xe,ye=np.histogram2d(points[:,0],points[:,1],bins=edges)
        fig,ax=plt.subplots(figsize=(10,10),layout='constrained');v=np.log1p(h)
        ax.imshow(v.T,origin='lower',extent=[xe[0],xe[-1],ye[0],ye[-1]],cmap='Greys',vmax=np.percentile(v[v>0],98))
        if traj:
            ax.plot(xyz[:,0],xyz[:,1],color='orangered',lw=1.3,label='Mapping IMU trajectory');ax.scatter(*xyz[0,:2],c='green',marker='s',label='Start');ax.scatter(*xyz[-1,:2],c='red',marker='^',label='End');ax.legend()
        ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title='full_lab_v1 | floor-aligned | '+('dense observations' if name.startswith('dense') else 'frozen 3D map'));ax.set_aspect('equal');ax.grid(alpha=.15)
        fig.savefig(d/name,dpi=180);plt.close(fig)
    bev(p,'bev_preview.png');bev(p,'trajectory_preview.png',True);bev(dense,'dense_reference_preview.png',True)
    fig,axs=plt.subplots(1,3,figsize=(16,5),layout='constrained')
    for ax,j in zip(axs[:2],[0,1]):
        ax.scatter(p[:,j],p[:,2],s=.15,alpha=.3);ax.axhline(0,c='red',lw=1);ax.set(xlabel=['X (m)','Y (m)'][j],ylabel='Map Z (m)',title='Frozen map side view');ax.set_aspect('equal')
    axs[2].hist(floor_map[:,2],bins=60);axs[2].set(xlabel='Fitted floor residual Z (m)',ylabel='Points',title='Floor-fit inliers (2.5 cm threshold)')
    fig.savefig(d/'floor_check.png',dpi=160);plt.close(fig)
    back=np.asarray(o3d.io.read_point_cloud(str(d/'map.pcd')).points)
    assert len(back)==len(p) and np.allclose(back,p,atol=2e-6)
    meta={'version':target.name,'source_cloud':'source/registered_xyz.npy','voxel_downsample_passes':1,'frame':'map','units':'metres','source_run':str(s),'source_bag':'RealWorldG1/datasets/full_lab_walk/bag','navigation_reference':'IMU origin; positive Z above fitted floor','floor_method':'RANSAC near-horizontal floor on gravity-leveled map; project original +X onto plane','floor_fit_roi_gravity':{'x':[0,7.5],'y':[-6,2],'z':[-1.45,-.95]},'floor_ransac':{'seed':42,'distance_threshold_m':.025,'iterations':3000},'floor_plane_in_gravity_frame':plane.tolist(),'floor_refinement_degrees':float(tilt),'floor_inlier_count':len(idx),'floor_inlier_rmse_m':float(np.sqrt(np.mean(floor_map[:,2]**2))),'crop_frame':'gravity-aligned before floor refinement','crop_min':low.tolist(),'crop_max':high.tolist(),'radius_filter':{'nb_points':3,'radius_m':.25},'voxel_m':.05,'counts':{'input':int(len(valid)),'nonfinite_removed':int((~valid).sum()),'outside_roi_removed':int((~keep).sum()),'radius_removed':int(keep.sum()-before_voxel),'before_voxel':before_voxel,'final':len(p)},'map_bounds':[p.min(axis=0).tolist(),p.max(axis=0).tolist()],'initial_imu_xyz_map':xyz[0].tolist(),'trajectory_z_range': [float(xyz[:,2].min()),float(xyz[:,2].max())],'software':{'open3d':o3d.__version__,'numpy':np.__version__},'validation':'PCD reload and rigid-transform checks passed; no relocalization accuracy validation yet','limitations':['No semantic/manual removal of people; geometric cleanup only','Floor plane inferred from mapping observations, not survey ground truth','All-height 3D map; BEV previews are density images, not navigation occupancy grids']}
    # JSON syntax is valid YAML 1.2 and avoids an additional YAML dependency.
    (d/'metadata.yaml').write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
    (d/'README.md').write_text('''# full_lab_v1 固定参考地图\n\n用于三维定位，不是导航障碍栅格。map.pcd 坐标系为 map，单位米，拟合地面为 z=0；导航参考点为 IMU，具有非零离地高度。\n\n由 full_lab_walk 的 registered_xyz.npy 稠密配准点云生成，先应用原重力调平，再小幅对齐拟合地面并平移高度。T_map_from_camera_init.txt 是列向量左乘的完整变换；姿态同样左乘其旋转，不能只修改位置或直接沿用原 yaw。\n\nROI 和地面拟合在原重力调平坐标中进行。清理仅包含显式 ROI、孤立点过滤和 最后一次 0.05 m 体素降采样（之前不做体素降采样），没有人工识别人影。参数、点数和拟合误差见 metadata.yaml。\n\nsource/ 保存可独立重建的原始地图、变换和轨迹；provenance/ 保存处理脚本与建图配置。原始稠密观测也归档在 source/registered_xyz.npy，dense_reference_preview.png 仅供视觉对比。\n\nbev_preview.png 是实际固定地图的全高度密度图；trajectory_preview.png 叠加已统一变换的建图轨迹；floor_check.png 检查侧视及地面拟合。预览不是占据栅格。\n\n加载地图不会重建或覆盖本文件。本次按用户要求重新生成 v1；脚本仅在显式 --replace 时替换已有地图包；地图定位尚待 relocalization_1/2/3 验证。手绘轨迹位于旧重力调平坐标系，精确对比需应用 T_map_from_gravity_level.txt。\n\n复现（--replace 显式允许替换当前地图包；需要 numpy/scipy/open3d/matplotlib）：\n```bash\npython RealWorldG1/tools/freeze_reference_map.py --source RealWorldG1/diagnostics/full_lab_walk/mapping --output RealWorldG1/maps/full_lab_v1 --replace\n```\n''')
    hashes={str(f.relative_to(d)):sha(f) for f in sorted(d.rglob('*')) if f.is_file()};(d/'checksums.sha256').write_text(''.join(f'{h}  {name}\n' for name,h in hashes.items()))
    # Verify package before replacing the requested destination.
    for name,h in hashes.items():
        assert sha(d/name)==h
    backup=target.with_name('.'+target.name+'.previous')
    if backup.exists(): raise RuntimeError('Previous replacement backup exists')
    if target.exists(): target.rename(backup)
    try:
        d.rename(target)
    except Exception:
        if backup.exists(): backup.rename(target)
        raise
    if backup.exists(): shutil.rmtree(backup)
    print(json.dumps(meta,indent=2))
if __name__=='__main__':main()
