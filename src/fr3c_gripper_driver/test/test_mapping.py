import pytest

from fr3c_gripper_driver.mapping import GripperCalibration


@pytest.fixture
def calibration():
    return GripperCalibration(
        0.027, 0.060, 820, 1300, 500, 2500,
        (0.027, 0.060),
        (820, 1300),
    )


def test_endpoint_mapping(calibration):
    assert calibration.position_to_pulse(0.027) == 820
    assert calibration.position_to_pulse(0.060) == 1300
    assert calibration.pulse_to_position(820) == pytest.approx(0.027)
    assert calibration.pulse_to_position(1300) == pytest.approx(0.060)
    assert calibration.position_to_pulse(0.0435) == 1060


def test_mapping_rejects_out_of_range(calibration):
    with pytest.raises(ValueError):
        calibration.position_to_pulse(0.026)
    with pytest.raises(ValueError):
        calibration.position_to_pulse(0.061)
