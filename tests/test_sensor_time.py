"""Clock translation regressions, including jitter during synthetic motion."""

import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'go2_nav2/go2_nav2/sensor_time.py'
SPEC = importlib.util.spec_from_file_location('sensor_time', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)
SharedSensorClock = module.SharedSensorClock
ClockDiscontinuity = module.ClockDiscontinuity
SECOND = 10**9
EPOCH = 10000 * SECOND
SOURCE = 9000 * SECOND


def calibrated():
    clock = SharedSensorClock(calibration_sec=.1)
    for i in range(11):
        step = i * 10**7
        clock.observe_odom(SOURCE + step, EPOCH + step, step)
    return clock


class SensorTimeTests(unittest.TestCase):
    def test_waits_for_calibration_and_reference_progress(self):
        clock = SharedSensorClock(calibration_sec=.1)
        self.assertIsNone(clock.observe_odom(SOURCE, EPOCH, 0))
        self.assertIsNone(clock.map_stamp('cloud', SOURCE, EPOCH, 0))
        for i in range(1,20):
            self.assertIsNone(clock.observe_odom(SOURCE, EPOCH+i*10**7, i*10**7))
        self.assertIsNone(clock.offset_ns)

    def test_common_epoch_shift_preserves_exact_inter_sensor_delta(self):
        clock = calibrated()
        odom = clock.observe_odom(SOURCE+SECOND, EPOCH+SECOND, SECOND)
        cloud = clock.map_stamp('cloud', SOURCE+SECOND-12345678, EPOCH+SECOND+40000000,
                                SECOND+40000000)
        self.assertEqual(odom-cloud, 12345678)

    def test_equal_acquisition_times_remain_equal_despite_different_delays(self):
        clock = calibrated()
        odom = clock.observe_odom(SOURCE+SECOND, EPOCH+SECOND, SECOND)
        cloud = clock.map_stamp('cloud', SOURCE+SECOND, EPOCH+SECOND+180000000, SECOND+180000000)
        self.assertEqual(odom, cloud)

    def test_offset_never_tracks_wifi_jitter_after_calibration(self):
        clock = calibrated()
        offset = clock.offset_ns
        for i, delay in enumerate((0, 200000000, 5000000, 150000000),1):
            source = SOURCE+i*SECOND
            corrected = clock.observe_odom(source, EPOCH+i*SECOND+delay, i*SECOND+delay)
            self.assertEqual(corrected-source, offset)
            self.assertEqual(clock.offset_ns, offset)

    def test_late_packet_is_dropped_not_restamped_as_fresh(self):
        clock = calibrated()
        self.assertIsNone(clock.map_stamp('cloud', SOURCE+SECOND, EPOCH+2*SECOND, 2*SECOND))
        self.assertEqual(clock.dropped['stale_cloud'], 1)

    def test_duplicates_and_small_reordering_are_dropped(self):
        clock = calibrated()
        self.assertIsNotNone(clock.observe_odom(SOURCE+SECOND, EPOCH+SECOND, SECOND))
        self.assertIsNone(clock.observe_odom(SOURCE+SECOND, EPOCH+SECOND+1, SECOND+1))
        self.assertIsNone(clock.observe_odom(SOURCE+SECOND-1000000, EPOCH+SECOND+2, SECOND+2))

    def test_zero_timestamp_never_becomes_current_time(self):
        clock = calibrated()
        self.assertIsNone(clock.map_stamp('cloud', 0, EPOCH+SECOND, SECOND))

    def test_source_reset_latches_fault_instead_of_rebasing_running_map(self):
        clock = calibrated()
        with self.assertRaises(ClockDiscontinuity):
            clock.observe_odom(SOURCE-SECOND, EPOCH+SECOND, SECOND)
        with self.assertRaises(ClockDiscontinuity):
            clock.map_stamp('cloud', SOURCE+2*SECOND, EPOCH+2*SECOND, 2*SECOND)

    def test_local_clock_jump_latches_fault(self):
        clock = calibrated()
        with self.assertRaises(ClockDiscontinuity):
            clock.observe_odom(SOURCE+SECOND, EPOCH+10*SECOND, SECOND)

    def test_source_forward_jump_latches_fault(self):
        clock = calibrated()
        with self.assertRaises(ClockDiscontinuity):
            clock.observe_odom(SOURCE+10*SECOND, EPOCH+SECOND, SECOND)

    def test_synthetic_motion_uses_pose_at_scan_time_not_receive_time(self):
        clock = calibrated()
        t0 = clock.observe_odom(SOURCE+SECOND, EPOCH+SECOND, SECOND)
        t1 = clock.observe_odom(SOURCE+2*SECOND, EPOCH+2*SECOND, 2*SECOND)
        # At 1 m/s: a scan acquired at t=1.5 arrives at t=2.1, 600 ms late.
        # Use an increased age limit for this interpolation-only test.
        clock.max_age_ns = SECOND
        scan = clock.map_stamp('cloud', SOURCE+1500000000, EPOCH+2100000000, 2100000000)
        pose_x = 1.0 + (scan-t0)/(t1-t0)
        self.assertAlmostEqual(pose_x, 1.5)
        self.assertNotEqual(scan, EPOCH+2100000000)

    def test_invalid_configuration_rejected(self):
        for kwargs in ({'calibration_sec':0}, {'max_age_sec':-.1}, {'margin_sec':.6},
                       {'clock_jump_sec':float('nan')}, {'future_tolerance_sec':-.1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SharedSensorClock(**kwargs)


if __name__ == '__main__':
    unittest.main()
