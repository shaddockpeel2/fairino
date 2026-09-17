import pytest

from fr3c_gripper_driver.ft232h_backend import Ft232hPwmBackend


class FakeGpio:
    instances = []

    def __init__(self):
        self.frequency = 10_000.0
        self.is_connected = True
        self.configured = False
        self.writes = []
        self.closed = False
        self.instances.append(self)

    def configure(self, *_args, **_kwargs):
        self.configured = True

    def write(self, payload):
        self.writes.append(payload)

    def close(self):
        self.is_connected = False
        self.closed = True


class FailingGpio(FakeGpio):
    fail_next = True

    def write(self, payload):
        if isinstance(payload, bytes) and type(self).fail_next:
            type(self).fail_next = False
            raise OSError('simulated USB failure')
        super().write(payload)


def make_backend(controller_factory=FakeGpio):
    return Ft232hPwmBackend(
        url='fake', sample_rate_hz=10_000, frame_us=20_000,
        signal_pin=7, command_duration_s=0.8, post_write_delay_s=0.05,
        electrical_min_us=500, electrical_max_us=2500,
        controller_factory=controller_factory, sleep=lambda _seconds: None,
    )


@pytest.mark.parametrize(
    ('angle_deg', 'effective_us', 'high_samples'),
    [(31, 800, 8), (70, 1300, 13)],
)
def test_backend_emits_one_finite_burst_and_pulls_ad7_low(
        angle_deg, effective_us, high_samples):
    FakeGpio.instances.clear()
    backend = make_backend()

    assert backend.command_angle(angle_deg) == effective_us

    gpio = FakeGpio.instances[-1]
    assert gpio.configured
    assert gpio.closed
    assert len(gpio.writes) == 2
    payload, final_low = gpio.writes
    assert final_low == 0
    assert len(payload) == 8_000  # 10 kHz * 0.8 seconds
    frame = payload[:200]
    assert frame[:high_samples] == bytes([0x80]) * high_samples
    assert frame[high_samples:] == bytes(200 - high_samples)
    assert payload == frame * 40
    assert backend.current_us == effective_us
    assert backend.frames_written == 40
    assert backend.bytes_written == 8_000


def test_next_command_reopens_after_usb_error():
    FakeGpio.instances.clear()
    FailingGpio.fail_next = True
    backend = make_backend(FailingGpio)
    with pytest.raises(OSError, match='simulated USB failure'):
        backend.command_angle(31)
    assert 'simulated USB failure' in backend.error

    assert backend.command_angle(70) == 1300
    assert backend.recovery_count == 1
    assert backend.error is None
    assert len(FakeGpio.instances) == 2
