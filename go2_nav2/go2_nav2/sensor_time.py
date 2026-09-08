"""One immutable source-clock translation for acquisition-stamped sensor streams.

Arrival times estimate the initial epoch offset only. They must never replace
individual acquisition times or change the relationship between cloud and pose.
"""

from collections import Counter
import math


class ClockDiscontinuity(RuntimeError):
    """A running SLAM session must not silently cross a clock discontinuity."""


class SharedSensorClock:
    def __init__(self, calibration_sec=1.0, margin_sec=0.05, max_age_sec=0.5,
                 future_tolerance_sec=0.05, clock_jump_sec=0.5):
        values = (calibration_sec, margin_sec, max_age_sec,
                  future_tolerance_sec, clock_jump_sec)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("clock settings must be finite")
        if calibration_sec <= 0 or margin_sec < 0 or max_age_sec <= margin_sec:
            raise ValueError("positive calibration and max_age > margin >= 0 required")
        if future_tolerance_sec < 0 or clock_jump_sec <= 0:
            raise ValueError("invalid clock discontinuity tolerances")
        self.calibration_ns = round(calibration_sec * 1e9)
        self.margin_ns = round(margin_sec * 1e9)
        self.max_age_ns = round(max_age_sec * 1e9)
        self.future_tolerance_ns = round(future_tolerance_sec * 1e9)
        self.clock_jump_ns = round(clock_jump_sec * 1e9)
        self.offset_ns = None
        self.fault = None
        self.dropped = Counter()
        self._wall_origin = None
        self._start_mono = None
        self._min_offset = None
        self._samples = 0
        self._last_reference = None
        self._last_output = {}

    def _fail(self, reason):
        self.fault = reason
        raise ClockDiscontinuity(reason)

    def _check_wall(self, now_ns, mono_ns):
        if self.fault:
            raise ClockDiscontinuity(self.fault)
        wall_origin = now_ns - mono_ns
        if self._wall_origin is None:
            self._wall_origin = wall_origin
        elif abs(wall_origin - self._wall_origin) > self.clock_jump_ns:
            self._fail("local ROS clock jumped; restart the sensor and SLAM session")

    def observe_odom(self, source_ns, now_ns, mono_ns):
        self._check_wall(now_ns, mono_ns)
        if source_ns <= 0:
            self.dropped['invalid_odom_stamp'] += 1
            return None
        if self._last_reference is not None and source_ns <= self._last_reference:
            if self._last_reference - source_ns > self.clock_jump_ns:
                self._fail("source odometry clock moved backwards; restart SLAM")
            self.dropped['out_of_order_odom'] += 1
            return None
        self._last_reference = source_ns
        if self.offset_ns is None:
            if self._start_mono is None:
                self._start_mono = mono_ns
            candidate = now_ns - source_ns
            self._min_offset = candidate if self._min_offset is None else min(self._min_offset, candidate)
            self._samples += 1
            if mono_ns - self._start_mono < self.calibration_ns or self._samples < 10:
                return None
            # Minimum arrival offset limits queueing bias. The margin keeps TF
            # slightly behind 'now'. Absolute one-way latency remains unknown.
            self.offset_ns = self._min_offset - self.margin_ns
        return self.map_stamp('odom', source_ns, now_ns, mono_ns)

    def map_stamp(self, stream, source_ns, now_ns, mono_ns):
        self._check_wall(now_ns, mono_ns)
        if self.offset_ns is None:
            self.dropped['calibrating'] += 1
            return None
        if source_ns <= 0:
            self.dropped['invalid_stamp'] += 1
            return None
        if source_ns <= self._last_output.get(stream, 0):
            self.dropped['out_of_order_' + stream] += 1
            return None
        stamp = source_ns + self.offset_ns
        age = now_ns - stamp
        if stamp <= 0 or age < -self.future_tolerance_ns:
            self._fail("sensor timestamp is in the future on the shared clock; restart SLAM")
        if age > self.max_age_ns:
            self.dropped['stale_' + stream] += 1
            return None
        self._last_output[stream] = source_ns
        return stamp
