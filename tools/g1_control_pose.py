"""Latest-odometry control pose; independent of point-cloud/BEV processing.

The reference lease is issued by LiveBEV after its existing ICP/health checks.
Its monotonic deadlines are only valid between processes on the same Linux boot.
Sensor stamps stay in their own clock: never subtract them from wall-clock time.
"""
from functools import lru_cache
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from live_bev_core import checked_transform


@lru_cache(maxsize=1)
def monotonic_clock_id():
    return 'linux-boot:'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def odometry_matrix(message, expected_frame):
    if message.header.frame_id != expected_frame:
        raise ValueError('Unexpected control odometry frame: '+message.header.frame_id)
    p, q = message.pose.pose.position, message.pose.pose.orientation
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    transform[:3, 3] = [p.x, p.y, p.z]
    return checked_transform(transform)


class LatestControlPose:
    def __init__(self, pose_timeout, registration_timeout, clock_id=None):
        # Both limits are inherited from the existing controller/configuration.
        self.pose_timeout = float(pose_timeout)
        self.registration_timeout = float(registration_timeout)
        self.clock_id = clock_id or monotonic_clock_id()
        self.reference = None
        self.reference_sent = -math.inf
        self.odom = None
        self.odom_stamp = None
        self.odom_received = -math.inf
        self.clock_offset = None
        self.clock_reset_session = None
        self.session_id = None
        self.error = 'Waiting for control localization reference'
        self.seen_input = False

    def set_reference(self, data, now):
        self.seen_input = True
        try:
            if data['schema_version'] != 1 or data['clock_id'] != self.clock_id:
                raise ValueError('Control reference protocol/host clock mismatch')
            sent = float(data['sent_monotonic'])
            deadline = float(data['valid_until_monotonic'])
            odom_received = float(data['odom_received_monotonic'])
            latest_stamp = float(data['latest_odom_stamp'])
            registration_stamp = float(data['registration_stamp'])
            if not all(math.isfinite(v) for v in (sent, deadline, odom_received,
                                                  latest_stamp, registration_stamp)):
                raise ValueError('Nonfinite control reference')
            if sent > now + 1e-6 or odom_received > sent + 1e-6:
                raise ValueError('Control reference monotonic time went into the future')
            if sent <= self.reference_sent:
                return False  # A retained/duplicate packet is not a new lease.
            if not isinstance(data['valid'], bool) or not isinstance(data['session_id'], str) or not data['session_id']:
                raise ValueError('Invalid control reference identity/state')
            if int(data['map_epoch']) < 0 or int(data['revision']) < 0:
                raise ValueError('Invalid control reference version')
            transform = None
            if data['valid']:
                transform = checked_transform(data['transform'])
                if latest_stamp <= 0 or registration_stamp <= 0 or int(data['map_stamp_ns']) <= 0:
                    raise ValueError('Valid control reference needs sensor/registration/map stamps')
            previous = self.reference
            if previous and previous['session_id'] == data['session_id'] and (
                    int(data['map_epoch']) < int(previous['map_epoch']) or
                    int(data['revision']) < int(previous['revision'])):
                raise ValueError('Control reference version went backwards')
            if (previous and previous['session_id'] == data['session_id'] and
                    previous['revision'] == data['revision'] and previous['valid'] and data['valid'] and
                    not np.array_equal(previous['matrix'], transform)):
                raise ValueError('Control transform changed without a revision')
            self.reference = dict(data, matrix=transform)
            self.reference_sent = sent
            if self.session_id is not None and self.session_id != data['session_id']:
                self.clock_offset = None
                self.odom = None
                self.odom_stamp = None
                self.odom_received = -math.inf
                self.clock_reset_session = None
            self.session_id = data['session_id']
            self.error = ''
            return True
        except (KeyError, TypeError, ValueError) as exc:
            self.reference = None
            self.error = str(exc)
            return False

    def set_odometry(self, stamp, transform, received):
        self.seen_input = True
        try:
            stamp = float(stamp)
            if not math.isfinite(stamp) or stamp <= 0:
                raise ValueError('Control odometry needs a nonzero finite source stamp')
            transform = checked_transform(transform)
            if self.odom_stamp is not None and stamp <= self.odom_stamp:
                if stamp < self.odom_stamp:
                    self.clock_reset_session = self.reference['session_id'] if self.reference else ''
                    self.error = 'Control odometry clock reset; restart localization session'
                    self.odom = None
                return False  # Duplicate stamps never refresh receipt/sample time.
            self.odom_stamp, self.odom_received, self.odom = stamp, received, transform
            # Relative queue delay, without assuming sensor and Unix clocks agree.
            offset = received-stamp
            self.clock_offset = offset if self.clock_offset is None else min(self.clock_offset, offset)
            self.error = ''
            return True
        except (ValueError, TypeError) as exc:
            self.odom = None
            self.error = str(exc)
            return False

    def snapshot(self, now):
        r = self.reference
        reason = self.error
        if r is None:
            reason = reason or 'Waiting for control localization reference'
        elif self.clock_reset_session is not None:
            reason = 'Control odometry clock reset; restart localization session'
        elif not r['valid']:
            reason = r.get('reason') or 'Control localization reference is invalid'
        elif now >= r['valid_until_monotonic']:
            reason = 'Control localization reference expired'
        elif self.odom is None:
            reason = reason or 'Waiting for latest control odometry'
        elif self.odom_stamp-r['registration_stamp'] > self.registration_timeout:
            reason = 'Control ICP registration is stale in sensor time'
        elif r['latest_odom_stamp']-self.odom_stamp >= self.pose_timeout:
            reason = 'Control odometry is behind the localization reference'
        else:
            # A reference can bound age even when this subscriber starts with an
            # old queued message. Normal subsequent samples use source progress.
            sample_time = min(self.odom_received, self.clock_offset+self.odom_stamp,
                r['odom_received_monotonic'] + self.odom_stamp-r['latest_odom_stamp'])
            if now-sample_time >= self.pose_timeout:
                reason = 'Control odometry is stale or backlogged'
            else:
                mapped = r['matrix'] @ self.odom
                return dict(valid=True, reason='', transform=mapped, source_stamp=self.odom_stamp,
                    sample_time=sample_time, receipt_time=self.odom_received,
                    source_age_lower_bound_s=max(0., now-sample_time),
                    session_id=r['session_id'], revision=r['revision'], map_epoch=r['map_epoch'],
                    required_map_stamp_ns=int(r['map_stamp_ns']))
        return dict(valid=False, reason=reason)
