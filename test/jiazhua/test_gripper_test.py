import unittest

from gripper_test import PwmConfig, angle_to_pulse_us, build_pwm_frames


class GripperPwmTest(unittest.TestCase):
    def setUp(self):
        self.config = PwmConfig()

    def test_angle_mapping(self):
        self.assertEqual(angle_to_pulse_us(0, self.config), 500)
        self.assertEqual(angle_to_pulse_us(90, self.config), 1500)
        self.assertEqual(angle_to_pulse_us(180, self.config), 2500)

    def test_one_frame_shape(self):
        payload = build_pwm_frames(1500, 0.02, self.config)
        self.assertEqual(len(payload), 2000)
        self.assertEqual(
            payload[:150], bytes([self.config.signal_mask]) * 150
        )
        self.assertEqual(payload[150:], bytes(1850))

    def test_rejects_unsafe_values(self):
        with self.assertRaises(ValueError):
            angle_to_pulse_us(181, self.config)
        with self.assertRaises(ValueError):
            build_pwm_frames(2600, 1.0, self.config)


if __name__ == "__main__":
    unittest.main()
