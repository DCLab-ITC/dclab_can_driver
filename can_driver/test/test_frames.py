"""Frame formats (no ROS needed): python3 -m pytest test/test_frames.py
or python3 -m unittest discover -s test -p 'test_frames.py'."""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from can_driver import dclab_frames as f  # noqa: E402


class MotorCommands(unittest.TestCase):
    def test_golden_vector(self):
        # 6.0 = 0x40C00000; enable bit 3 + speed mode 2 = 0x0A.
        self.assertEqual(f.motor_command(True, f.MODE_SPEED, 6.0), bytes([0x0A, 0x00, 0xC0, 0x40]))

    def test_goal_keeps_28_bits(self):
        for goal in (0.0, -1.5, 3.14159, 1234.5678, -0.001):
            enabled, mode, back = f.decode_motor_word(f.motor_word(f.MODE_POSITION, goal))
            self.assertEqual((enabled, mode), (True, f.MODE_POSITION))
            self.assertLessEqual(abs(back - goal), abs(goal) * 2 ** -19 + 1e-45)

    def test_m1_only_is_dlc4(self):
        data = f.motor_command(True, f.MODE_VOLTAGE, 3.0, False, 99, math.nan)  # M2 not checked
        self.assertEqual(f.decode_motor_word(data), (True, f.MODE_VOLTAGE, 3.0))
        self.assertEqual(len(data), 4)

    def test_both_is_m1_then_m2(self):
        data = f.motor_command(True, f.MODE_VOLTAGE, 3.0, True, f.MODE_SPEED, -2.0)
        self.assertEqual(len(data), 8)
        self.assertEqual(f.decode_motor_word(data[:4]), (True, f.MODE_VOLTAGE, 3.0))
        self.assertEqual(f.decode_motor_word(data[4:]), (True, f.MODE_SPEED, -2.0))

    def test_m2_only_disables_m1(self):
        data = f.motor_command(False, 99, math.nan, True, f.MODE_SPEED, -2.0)  # M1 not checked
        self.assertEqual(data[:4], bytes(4))
        self.assertFalse(f.decode_motor_word(data[:4])[0])
        self.assertEqual(f.decode_motor_word(data[4:]), (True, f.MODE_SPEED, -2.0))

    def test_ramp_modes_and_refusals(self):
        word = f.motor_word(f.MODE_VOLTAGE_RAMP, 12.0)
        self.assertEqual(f.decode_motor_word(word), (True, 5, 12.0))
        with self.assertRaises(ValueError):
            f.motor_command(False, 0, 0.0, False, 0, 0.0)
        for mode in (6, 7):
            with self.assertRaises(ValueError):
                f.motor_command(True, mode, 1.0)
        with self.assertRaises(ValueError):
            f.motor_command(True, f.MODE_SPEED, math.nan)
        with self.assertRaises(ValueError):
            f.motor_command(True, f.MODE_SPEED, 1.0, True, f.MODE_SPEED, math.inf)


class Feedback(unittest.TestCase):
    def test_single_and_shared(self):
        self.assertEqual(f.encoder_positions(struct.pack("<f", 1.25)), [1.25])
        self.assertEqual(f.encoder_positions(struct.pack("<ff", 1.5, -2.5)), [1.5, -2.5])
        with self.assertRaises(ValueError):
            f.encoder_positions(b"\x00" * 6)

    def test_speed_estimate(self):
        est = f.SpeedEstimator()
        self.assertEqual(est.update((101, 0), 0.0, 1.0), 0.0)
        self.assertAlmostEqual(est.update((101, 0), 0.5, 1.1), 5.0)
        self.assertAlmostEqual(est.update((101, 1), 7.0, 1.1), 0.0)  # its own history


class OtherBoards(unittest.TestCase):
    def test_sensor_inputs(self):
        # analog 200, 4095, 0, 2000, 4095; digital 1 and 3 on (V1 packing).
        a = [200, 4095, 0, 2000, 4095]
        data = bytes([a[0] >> 4, ((a[0] & 15) << 4) | (a[1] >> 8), a[1] & 255,
                      a[2] >> 4, ((a[2] & 15) << 4) | (a[3] >> 8), a[3] & 255,
                      a[4] >> 4, ((a[4] & 15) << 4) | 0b0101])
        analog, digital = f.sensor_inputs(data)
        self.assertEqual([round(x * 4095) for x in analog], a)
        self.assertEqual(digital, [True, False, True, False])

    def test_controller(self):
        pwm = f.controller_pwm([1.0, 0.0, 2.0, -1.0])
        self.assertEqual(pwm[0] >> 6, 2)
        self.assertEqual(((pwm[0] & 0x3F) << 8) | pwm[1], 16383)
        self.assertEqual(((pwm[4] & 0x3F) << 8) | pwm[5], 16383)  # clamped
        self.assertEqual(f.controller_servo([0.5] * 4)[0] >> 6, 3)
        dig = f.controller_digital([True, False, False, True], [False] * 5 + [True])
        self.assertEqual((dig[0] >> 6, dig[1], dig[2]), (1, 0b1001, 0b100000))

    def test_imu(self):
        data = bytes([0x7F, 0xFF, 0x80, 0x00, 0x40, 0x00, 0xC0, 0x00])
        w, x, y, z = f.imu_quaternion(data)
        self.assertAlmostEqual(w, 1.0)
        self.assertEqual(x, -0.0)
        self.assertAlmostEqual(y, 16384 / 32767)
        self.assertAlmostEqual(z, -16384 / 32767)


if __name__ == "__main__":
    unittest.main()
