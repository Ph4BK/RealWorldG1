"""Recompute final map-pose errors; original experiment files remain unchanged.

python3 diagnostics/posnav_final_approach_20261109/analyze.py
Only figure export requires matplotlib. Other outputs use the standard library.
"""
import csv
from datetime import datetime, timedelta
import hashlib
import json
import math
from pathlib import Path
import statistics

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SESSION = '20261109T151158Z'
BUNDLE = ROOT / 'posnav_experiments' / SESSION


def read(path):
    return json.loads(path.read_text())


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stats(values):
    return dict(count=len(values), mean=statistics.mean(values),
                median=statistics.median(values), minimum=min(values), maximum=max(values),
                rms=math.sqrt(statistics.mean(v*v for v in values)))


def main():
    checked = 0
    for line in (BUNDLE / 'checksums.sha256').read_text().splitlines():
        expected, name = line.split('  ', 1)
        assert hashlib.sha256((BUNDLE / name).read_bytes()).hexdigest() == expected, name
        checked += 1
    session, recording = read(BUNDLE / 'session.json'), read(BUNDLE / 'navigation_recording.json')
    assert session['status'] == 'closed' and session['recording_complete']
    assert recording['recording_complete'] and recording['unfinished_trial'] is None
    assert not recording['writer_error'] and recording['dropped_records'] == 0
    assert not session['localization_writer_error'] and session['localization_dropped_records'] == 0
    events = [json.loads(line) for line in (BUNDLE / 'events.jsonl').read_text().splitlines()]
    rows = []
    for trial in sorted((BUNDLE / 'trials').iterdir()):
        task, result = read(trial / 'task.json'), read(trial / 'result.json')
        assert result['success'] and result['outcome'] == 'arrived' and result['recording_complete']
        ev = [e for e in events if e.get('trial_id') == trial.name]
        position_stop, position_settled, yaw_stop, completed = [
            next(e for e in ev if e['event'] == name) for name in
            ('fine_position_stop', 'fine_position_settled', 'fine_yaw_stop', 'fine_approach_completed')]
        goal, final = task['goal'], result['final_pose']
        l2 = math.hypot(final[0]-goal[0], final[1]-goal[1])
        yaw = wrap(goal[2]-final[2])
        assert final == completed['final_pose']
        assert abs(l2-result['internal_error_m']) < 1e-12
        assert abs(yaw-result['final_yaw_error_rad']) < 1e-12
        assert abs(l2-completed['internal_error_m']) < 1e-12
        assert abs(yaw-completed['final_yaw_error_rad']) < 1e-12
        with (trial / 'trajectory.csv').open() as stream:
            trajectory = list(csv.DictReader(stream))
        latest = max((r for r in trajectory if int(r['seq']) < completed['seq']),
                     key=lambda r: int(r['seq']))
        assert all(abs(final[k]-float(latest[key])) < 1e-12
                   for k, key in enumerate(('x', 'y', 'yaw')))
        rows.append(dict(trial_id=trial.name,
            goal_x=goal[0], goal_y=goal[1], goal_yaw_rad=goal[2],
            final_x=final[0], final_y=final[1], final_yaw_rad=final[2],
            dx_m=final[0]-goal[0], dy_m=final[1]-goal[1],
            l2_m=l2, yaw_error_rad=yaw, abs_yaw_error_rad=abs(yaw),
            abs_yaw_error_deg=abs(math.degrees(yaw)),
            position_stop_l2_m=position_stop['distance_m'],
            position_stop_projection_m=position_stop['projected_remaining_m'],
            position_settled_l2_m=position_settled['internal_error_m'],
            position_settled_projection_m=position_settled['projected_remaining_m'],
            position_wait_s=position_settled['elapsed_s']-position_stop['elapsed_s'],
            xy_shift_after_position_settled_m=math.dist(position_settled['pose'][:2], final[:2]),
            yaw_stop_error_rad=yaw_stop['yaw_error_rad'],
            yaw_wait_s=completed['elapsed_s']-yaw_stop['elapsed_s'],
            final_pose_receive_age_s=result['pose_age_s'],
            completed_timestamp_utc=completed['timestamp'],
            elapsed_task_s=result['elapsed_task_s'], pose_samples=result['pose_samples']))
    assert len(rows) == recording['trial_count']
    summary = dict(session_id=SESSION, raw_bundle_relative_to_repository=f'posnav_experiments/{SESSION}',
        checksum_files_verified=checked, recording_complete=True, trial_count=len(rows), completed_trials=len(rows),
        metric_basis='final_pose after final yaw zero-command wait; map localization estimate',
        yaw_sign='wrap(goal_yaw-final_yaw); aggregate yaw statistics use absolute values',
        measurement='after one final approach cycle; no position tolerance acceptance applied',
        parameters=read(BUNDLE / 'assets/config/navigation_parameters.json')['control'],
        final_l2_m=stats([r['l2_m'] for r in rows]),
        final_abs_yaw_rad=stats([r['abs_yaw_error_rad'] for r in rows]),
        final_abs_yaw_deg=stats([r['abs_yaw_error_deg'] for r in rows]),
        position_settled_l2_m=stats([r['position_settled_l2_m'] for r in rows]),
        final_pose_receive_age_s=stats([r['final_pose_receive_age_s'] for r in rows]),
        descriptive_counts=dict(l2_le_005m=sum(r['l2_m'] <= .05 for r in rows),
            l2_le_010m=sum(r['l2_m'] <= .1 for r in rows),
            abs_yaw_le_01rad=sum(r['abs_yaw_error_rad'] <= .1 for r in rows),
            l2_le_010m_and_abs_yaw_le_01rad=sum(r['l2_m'] <= .1 and r['abs_yaw_error_rad'] <= .1 for r in rows)),
        observations=rows)
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    with (OUT / 'per_trial.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    export_figure(rows)
    export_report(summary, session)
    outputs = ['REPORT.md', 'summary.json', 'per_trial.csv', 'analyze.py',
               'final_pose_errors.png', 'final_pose_errors.pdf']
    (OUT / 'checksums.sha256').write_text(''.join(
        f'{hashlib.sha256((OUT/name).read_bytes()).hexdigest()}  {name}\n'
        for name in outputs if (OUT/name).is_file()))
    print(json.dumps({k: summary[k] for k in ('trial_count', 'final_l2_m',
          'final_abs_yaw_deg', 'descriptive_counts')}, ensure_ascii=False, indent=2))


def export_figure(rows):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), layout='constrained')
    x = list(range(1, len(rows)+1))
    axes[0].bar([i-.18 for i in x], [r['position_settled_l2_m']*100 for r in rows],
                width=.36, label='After position wait')
    axes[0].bar([i+.18 for i in x], [r['l2_m']*100 for r in rows],
                width=.36, label='Final, after yaw wait')
    axes[0].set(xlabel='Trial', ylabel='XY L2 error (cm)', xticks=x, title='Recorded map-pose error')
    axes[0].legend(fontsize=9)
    axes[1].bar(x, [r['abs_yaw_error_deg'] for r in rows], color='C2')
    axes[1].axhline(math.degrees(.1), color='black', ls='--', lw=1,
                    label='0.1 rad (control threshold)')
    axes[1].set(xlabel='Trial', ylabel='Absolute final yaw error (degrees)', xticks=x,
                title='After final yaw wait')
    axes[1].legend(fontsize=9)
    fig.savefig(OUT / 'final_pose_errors.png', dpi=180)
    fig.savefig(OUT / 'final_pose_errors.pdf')


def export_report(summary, session):
    rows = summary['observations']
    d, a, rad = summary['final_l2_m'], summary['final_abs_yaw_deg'], summary['final_abs_yaw_rad']
    local_time = lambda stamp: (datetime.fromisoformat(stamp)+timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
    table = '\n'.join(f"| {i} | {r['l2_m']*100:.2f} | {r['yaw_error_rad']:+.5f} | "
        f"{r['abs_yaw_error_deg']:.2f} | {r['position_settled_l2_m']*100:.2f} |"
        for i, r in enumerate(rows, 1))
    ages = summary['final_pose_receive_age_s']
    text = f'''# G1 PosNav 末端精调实验报告

实验包：`{SESSION}`；实验时间：{local_time(session['timestamp'])} 至 {local_time(session['end']['timestamp'])}（北京时间）。

## 结果

本组 8 次任务均完成完整 PosNav 流程。最终 L2 平均 **{d['mean']*100:.2f} cm**，
中位数 **{d['median']*100:.2f} cm**，范围 **{d['minimum']*100:.2f}–{d['maximum']*100:.2f} cm**。
最终 yaw 误差绝对值平均 **{a['mean']:.2f}°**（{rad['mean']:.5f} rad），
中位数 **{a['median']:.2f}°**，范围 **{a['minimum']:.2f}–{a['maximum']:.2f}°**。
L2 均方根为 {d['rms']*100:.2f} cm，yaw 误差均方根为 {a['rms']:.2f}°。

3/8 次 L2 不超过 5 cm，6/8 次不超过 10 cm；7/8 次最终 yaw 误差绝对值不超过
0.1 rad；同时满足 L2≤10 cm 和 yaw≤0.1 rad 的有 5/8 次。这些是描述性统计，
不是与上层约定的验收规则，也不是总体任务成功率估计。

RViz 中箭头几乎重合与部分试验的厘米级误差相符；但本组第 1、4 次最终 L2
分别为 14.97、17.36 cm，不能据视觉重合认定每次都达到几厘米精度。
可以表述为“8 次完整任务中，最终定位坐标误差平均约 8.1 cm，yaw 绝对误差平均约 3.7°”。

## 指标口径

采用每次 `result.json.final_pose`，即最终 yaw 停止后等待约 0.75 s、记录本轮完成时
控制器使用的定位位姿。与 `fine_approach_completed` 事件及轨迹对应最新样本逐一核对，
结果一致；不是位置阶段刚停止时的位姿，也不是任务途中最接近目标的样本。

```text
L2 = hypot(final_x-goal_x, final_y-goal_y)
e_yaw = atan2(sin(goal_yaw-final_yaw), cos(goal_yaw-final_yaw))
```

角差取 [-π,π] 的最短有符号误差；均值等统计使用绝对值，避免正负抵消。
例如第 4 次目标 yaw 为 +π、最终 yaw 接近 -π，实际误差约 1.39°，不按接近一整圈计算。
最终位姿接收年龄为 {ages['minimum']:.3f}–{ages['maximum']:.3f} s。
指标反映 map 定位坐标与请求目标的差异，本报告不进行独立物理真值标定。

## 逐次结果

| 任务 | 最终 L2（cm） | 最终有符号 yaw 误差（rad） | 最终 yaw 绝对误差（°） | 位置等待结束 L2（cm） |
|---|---:|---:|---:|---:|
{table}

![逐次最终误差与位置阶段误差对比](final_pose_errors.png)

第 8 次最终 yaw 误差为 0.10805 rad，略大于控制停止阈值 0.1 rad。
停止阈值作用于发出零速度的时刻，之后的等待仍可能改变实际记录角度；当前实现没有
停后再次修正或验收。`success=true` 表示本轮流程完成，不能直接解释为误差阈值全部达标。

位置等待结束的 L2 与最终 L2 有时增大、有时减小，CSV 同时保存两个阶段的误差、
阶段间 XY 位移和 yaw 停止瞬间误差。阶段间位移包括最终转向和零速等待期间的定位变化，
不能全部归为纯转向机械位移；最终指标统一使用整个流程结束后的位姿。

## 本次实现与参数

- 所有 STG 针对 pre，pre 位于目标 yaw 反方向 0.60 m，末端以最新实际位姿接近目标。
- 位置阶段开始时固定实际位置到目标的方向，用有符号投影剩余距离停止，`coast=0 m`。
- 投影剩余距离≤0.15 m 时停止角度修正，继续以 `fine_speed=0.25 m/s` 前进，受阻处理仍优先。
- `drive_align=yaw_stop_tol=0.1 rad`，`k_ang=1.5`，`wz_min=0.15 rad/s`，`wz_max=1 rad/s`。
- 位置停止等待 0.50 s，最终 yaw 停止等待 0.75 s；只执行一轮位置接近及 yaw 对准。
- 全组有 8 条完整阶段事件链，无自动脱困事件；本报告不由此推断脱困能力。

参数来自实验包的配置和源码快照。本次分析没有调整运控参数。

## 数据完整性与复现

原包 [本地目录](../../posnav_experiments/{SESSION}/) 完整保留；
原包 `checksums.sha256` 中 **{summary['checksum_files_verified']} 个文件全部校验通过**。
session、navigation_recording 和 8 份 result 均正常收尾，记录器无写盘错误、无队列丢弃。
所有 trial 均纳入统计，没有按误差筛选或剔除异常值；记录器无丢弃不代表 DDS 消息零丢失。

- [逐次数据 CSV](per_trial.csv)：目标与最终位姿、XY 分量、L2、yaw、阶段误差及等待时间。
- [机器可读摘要](summary.json)：完整参数、统计量及逐次结果。
- [图 PNG](final_pose_errors.png) / [图 PDF](final_pose_errors.pdf)。
- [复现脚本](analyze.py)：读取原始包后重新计算，图导出需要 matplotlib，其余使用 Python 标准库。
- [报告产物校验清单](checksums.sha256)。

在 RealWorldG1 根目录执行 `python3 diagnostics/posnav_final_approach_20261109/analyze.py`。
原始实验包留本地；报告、统计数据、配图和脚本按 diagnostics 归档规则保留。
'''
    (OUT / 'REPORT.md').write_text(text)


if __name__ == '__main__':
    main()
