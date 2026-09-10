import pytest

from fr3c_gripper_driver.mapping import (
    GripperCalibration,
    advance_pulse,
    build_pwm_frames,
)


@pytest.fixture
def calibration():
    return GripperCalibration(
        0.0, 0.130, 550, 1650, 500, 2500,
        (0.0, 0.024, 0.060, 0.100, 0.110, 0.130),
        (550, 820, 1100, 1380, 1480, 1650),
    )


def test_endpoint_mapping(calibration):
    assert calibration.position_to_pulse(0.0) == 550
    assert calibration.position_to_pulse(0.130) == 1650
    assert calibration.position_to_pulse(0.060) == 1100
    assert calibration.position_to_pulse(0.110) == 1480
    assert calibration.pulse_to_position(1100) == pytest.approx(0.060)
    assert calibration.position_to_pulse(0.042) == 960


def test_mapping_rejects_out_of_range(calibration):
    with pytest.raises(ValueError):
        calibration.position_to_pulse(-0.001)
    with pytest.raises(ValueError):
        calibration.position_to_pulse(0.131)


def test_pwm_frames():
    payload = build_pwm_frames([550, 1650])
    assert len(payload) == 4000
    assert payload[:55] == bytes([0x80]) * 55
    assert payload[55:2000] == bytes(1945)
    assert payload[2000:2165] == bytes([0x80]) * 165


def test_slew():
    assert advance_pulse(550, 1650, 400.0) == 558
    assert advance_pulse(1648, 1650, 400.0) == 1650
    assert advance_pulse(1650, 550, 400.0) == 1642
