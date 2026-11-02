"""Minimal per-STG measurements; control callbacks only update memory."""
import json
import math
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
import tempfile


class STGErrorRecorder:
    def __init__(self):
        self.current = None
        self.completed = deque()

    def begin(self, pose, stg):
        self.abort('Replanned before STG completion')
        self.current = dict(initial_pose=tuple(pose), stg=tuple(stg),
                            turn_settle_pose=None, walk_settle_pose=None,
                            anomalies=[])

    def anomaly(self, reason):
        if self.current is not None and reason not in self.current['anomalies']:
            self.current['anomalies'].append(reason)

    def turn_settled(self, pose):
        if self.current is not None:
            self.current['turn_settle_pose'] = tuple(pose)

    def walk_settled(self, pose):
        if self.current is None:
            return
        self.current['walk_settle_pose'] = tuple(pose)
        if not self.current['anomalies']:
            x, y, _ = pose
            gx, gy = self.current['stg']
            self.current['error_m'] = math.hypot(x-gx, y-gy)
        self._finish()

    def abort(self, reason):
        if self.current is not None:
            self.anomaly(reason)
            self._finish()

    def _finish(self):
        self.completed.append(self.current)
        self.current = None


class STGErrorLog:
    """One exclusive JSONL file per node session; flush each completed segment."""
    def __init__(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        prefix = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ_')
        self.stream = tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', prefix=prefix, suffix='.jsonl',
            dir=directory, delete=False)
        self.path = Path(self.stream.name)

    def flush(self, recorder):
        while recorder.completed:
            row = recorder.completed[0]
            self.stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
            self.stream.flush()
            recorder.completed.popleft()

    def close(self):
        self.stream.close()
