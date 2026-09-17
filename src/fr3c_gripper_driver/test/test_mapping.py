import pytest

from fr3c_gripper_driver.mapping import GripperCalibration


@pytest.fixture
def calibration():
    return GripperCalibration(
        0.026, 0.060, 800, 1300, 500, 2500,
        (0.026, 0.060),
        (800, 1300),
    )


def test_endpoint_mapping(calibration):
    assert calibration.position_to_pulse(0.026) == 800
    assert calibration.position_to_pulse(0.060) == 1300
    assert calibration.pulse_to_position(800) == pytest.approx(0.026)
    assert calibration.pulse_to_position(1300) == pytest.approx(0.060)
    assert calibration.position_to_pulse(0.043) == 1050


def test_mapping_rejects_out_of_range(calibration):
    with pytest.raises(ValueError):
        calibration.position_to_pulse(0.025)
    with pytest.raises(ValueError):
        calibration.position_to_pulse(0.061)
