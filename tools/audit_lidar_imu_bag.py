"""Read-only audit of a ROS2 SQLite bag; source ROS before running with system Python."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def audit(bag):
    records = {}
    for db in sorted(bag.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True) as con:
            topics = dict((i, (n, t)) for i, n, t in con.execute('SELECT id,name,type FROM topics'))
            for tid, receipt, data in con.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
                name, typ = topics[tid]
                if typ not in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Imu'):
                    continue
                m = deserialize_message(data, PointCloud2 if typ.endswith('PointCloud2') else Imu)
                r = records.setdefault(name, dict(type=typ, stamps=[], receipt_lag=[], frames=set(), values=[]))
                stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                r['stamps'].append(stamp)
                r['receipt_lag'].append((receipt-stamp)/1e9)
                r['frames'].add(m.header.frame_id)
                if isinstance(m, Imu):
                    a, w = m.linear_acceleration, m.angular_velocity
                    r['values'].append([a.x,a.y,a.z,w.x,w.y,w.z])
                else:
                    fields = [dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in m.fields]
                    if 'fields' in r and r['fields'] != fields:
                        raise ValueError('PointCloud2 fields changed within bag')
                    r['fields'] = fields
                    f = {v.name: v for v in m.fields}
                    if f['time'].datatype != 7 or f['ring'].datatype != 4:
                        raise ValueError('Expected float32 time and uint16 ring')
                    endian = '>' if m.is_bigendian else '<'
                    dtype = np.dtype(dict(names=['time','ring'], formats=[endian+'f4',endian+'u2'],
                                          offsets=[f['time'].offset,f['ring'].offset],itemsize=m.point_step))
                    p = np.ndarray((m.height,m.width), dtype=dtype, buffer=m.data,
                                   strides=(m.row_step,m.point_step)).reshape(-1)
                    r['values'].append([len(p),float(p['time'].min()),float(p['time'].max()),
                                        int(p['ring'].min()),int(p['ring'].max())])
                    r['point_step'] = m.point_step
    result = {}
    for name, r in records.items():
        t = np.array(r.pop('stamps'), dtype=np.int64)
        v = np.array(r.pop('values'))
        lag = np.array(r.pop('receipt_lag'))
        dt = np.diff(t)/1e9
        r['frames'] = sorted(r['frames'])
        r.update(count=len(t), stamp_start_ns=int(t[0]), stamp_end_ns=int(t[-1]),
                 interval_seconds_min_median_max=[float(dt.min()),float(np.median(dt)),float(dt.max())],
                 nonincreasing_stamps=int((dt<=0).sum()),
                 receipt_minus_header_seconds_min_median_max=[float(lag.min()),float(np.median(lag)),float(lag.max())],
                 values_min=v.min(axis=0).tolist(),values_max=v.max(axis=0).tolist())
        if r['type'].endswith('Imu'):
            stationary = v[t < t[0]+2_000_000_000]
            r['value_columns'] = ['ax','ay','az','wx','wy','wz']
            r['first_2s_mean'] = stationary.mean(axis=0).tolist()
            r['first_2s_std'] = stationary.std(axis=0).tolist()
        else:
            r['value_columns'] = ['points','time_min','time_max','ring_min','ring_max']
        result[name] = r
    if not result:
        raise ValueError('No supported messages found')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(args.bag)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(args.output)
