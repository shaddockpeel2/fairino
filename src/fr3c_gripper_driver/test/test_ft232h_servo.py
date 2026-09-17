import pytest

from fr3c_gripper_driver.ft232h_servo import (
    GripperController,
    angle_to_pulse_us,
    effective_pulse_us,
)


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


def test_project_angles_match_effective_10khz_pulses():
    close_nominal = angle_to_pulse_us(31)
    open_nominal = angle_to_pulse_us(70)
    assert close_nominal == pytest.approx(844.444444)
    assert open_nominal == pytest.approx(1277.777778)
    assert effective_pulse_us(close_nominal, 10_000) == 800
    assert effective_pulse_us(open_nominal, 10_000) == 1300


def test_controller_matches_standalone_burst_shape():
    fake = FakeGpio()
    sleeps = []
    with GripperController(
        controller_factory=lambda: fake,
        sleep=sleeps.append,
    ) as controller:
        result = controller.move_angle(31, 0.8)
    assert result.effective_pulse_us == 800
    assert result.high_samples == 8
    assert result.period_samples == 200
    assert result.frames == 40
    assert result.bytes_written == 8_000
    assert sleeps == [pytest.approx(0.85)]
    assert len(fake.writes) == 2
    assert fake.writes[-1] == 0
    assert fake.closed
