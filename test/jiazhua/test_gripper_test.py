import unittest
from unittest.mock import patch

from gripper_ft232h import GripperController, angle_to_pulse_us, parse_angles


class FakeGpio:
    def __init__(self):
        self.frequency = 10_000.0
        self.is_connected = True
        self.writes = []
        self.closed = False

    def configure(self, *_args, **_kwargs):
        pass

    def write(self, payload):
        self.writes.append(payload)

    def close(self):
        self.is_connected = False
        self.closed = True


class GripperPwmTest(unittest.TestCase):

    def test_angle_mapping(self):
        self.assertAlmostEqual(
            angle_to_pulse_us(31, 500, 2500), 844.444444, places=6
        )
        self.assertAlmostEqual(
            angle_to_pulse_us(70, 500, 2500), 1277.777778, places=6
        )

    def test_only_two_project_angles_are_accepted(self):
        self.assertEqual(parse_angles("31"), [31.0])
        self.assertEqual(parse_angles("70"), [70.0])
        with self.assertRaises(Exception):
            parse_angles("90")

    def test_31_degree_burst_matches_successful_test(self):
        fake = FakeGpio()
        with patch("gripper_ft232h.GpioAsyncController", return_value=fake), patch(
            "gripper_ft232h.time.sleep"
        ) as sleep:
            with GripperController() as gripper:
                pulse_us, high_samples, frames = gripper.move(31, 0.8)
        self.assertAlmostEqual(pulse_us, 844.444444, places=6)
        self.assertEqual(high_samples, 8)
        self.assertEqual(frames, 40)
        self.assertEqual(len(fake.writes[0]), 8000)
        self.assertEqual(fake.writes[-1], 0)
        self.assertTrue(fake.closed)
        sleep.assert_called_once_with(0.8500000000000001)


if __name__ == "__main__":
    unittest.main()
