"""Compare diagnostic PCDs against the recorded input and isolate the update stage."""
import argparse
import csv
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def pcd(path):
    with path.open('rb') as f:
        meta={}
        while True:
            line=f.readline().decode().strip()
            if line.startswith('#'): continue
            k,*v=line.split();meta[k]=v
            if k=='DATA': break
        if meta['DATA']!=['binary']: raise ValueError('Expected uncompressed binary PCD')
        formats=[{'F':'f','U':'u','I':'i'}[t]+s for t,s in zip(meta['TYPE'],meta['SIZE'])]
        return np.frombuffer(f.read(),dtype=np.dtype(list(zip(meta['FIELDS'],formats))))


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('bag',type=Path);a.add_argument('dump',type=Path)
    args=a.parse_args();recorded=[]
    for db in args.bag.glob('*.db3'):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as con:
            for data, in con.execute("SELECT messages.data FROM messages JOIN topics ON topic_id=topics.id WHERE topics.type='sensor_msgs/msg/PointCloud2' ORDER BY timestamp"):
                m=deserialize_message(data,PointCloud2)
                f={v.name:v for v in m.fields}
                dt=np.dtype(dict(names=['x','y','z','intensity','ring','time'],formats=['<f4']*4+['<u2','<f4'],offsets=[f[k].offset for k in ['x','y','z','intensity','ring','time']],itemsize=m.point_step))
                v=np.frombuffer(m.data,dtype=dt)[::3];xyz=np.column_stack([v[k] for k in ['x','y','z']])
                v=v[np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>.5)&(v['ring']<4)&np.isfinite(v['time'])&(v['time']>=0)]
                v=v[np.argsort(v['time'],kind='stable')]
                recorded.append((m.header.stamp.sec+m.header.stamp.nanosec/1e9,v))
    states=list(csv.reader((args.dump/'states.csv').open()));times={int(r[1]):float(r[2]) for r in states}
    checks=[]
    for path in sorted(args.dump.glob('preprocess_*.pcd')):
        frame=int(path.stem.split('_')[-1]);v=pcd(path)
        stamp,w=min(recorded,key=lambda item:abs(item[0]-times[frame]))
        if len(w)!=len(v): raise ValueError(f'count mismatch frame {frame}: {len(w)} != {len(v)}')
        xyzerr=max(float(np.max(np.abs(v[k]-w[k]))) for k in ['x','y','z'])
        terr=float(np.max(np.abs(v['curvature']-w['time']*np.float32(1e-6))))
        checks.append(dict(frame=frame,points=len(v),max_xyz_error_m=xyzerr,max_time_error_ms=terr,stamp_error_s=abs(stamp-times[frame])))
    stages={}
    for r in states: stages.setdefault(int(r[1]),{})[r[0]]=np.array(r[3:],float)
    updates=[]
    for frame,vals in stages.items():
        if 'update' not in vals: continue
        pre,post=vals['predict'],vals['update']
        updates.append(dict(frame=frame,predict_xyz=pre[:3].tolist(),update_xyz=post[:3].tolist(),update_shift_m=float(np.linalg.norm(post[:3]-pre[:3]))))
    out=dict(preprocess_checks=checks,updates=updates)
    (args.dump/'analysis.json').write_text(json.dumps(out,indent=2)+'\n')
    print('preprocess frames',len(checks),'max xyz error',max(v['max_xyz_error_m'] for v in checks),'max time error',max(v['max_time_error_ms'] for v in checks))
    print('first updates',updates[:5]);print('last update',updates[-1])
    for path in sorted(args.dump.glob('h_*.txt'),key=lambda p:tuple(map(int,p.stem.split('_')[1:])))[:5]:
        h,b=path.read_text().split('RESIDUAL');h=np.array([list(map(float,row.split())) for row in h.strip().splitlines()]);b=np.array(list(map(float,b.split())))
        print(path.name,'n',len(b),'rms residual',np.sqrt(np.mean(b*b)),'least squares correction',np.linalg.lstsq(h,b,rcond=None)[0])

if __name__=='__main__':main()
