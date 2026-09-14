import time

from fr3c_gripper_driver.ft232h_backend import Ft232hPwmBackend


class FakeGpio:
    instances = []

    def __init__(self):
        self.frequency = 100_000.0
        self.write_count = 0
        self.closed = False
        self.instances.append(self)

    def configure(self, *_args, **_kwargs):
        pass

    def write(self, _payload):
        self.write_count += 1

    def close(self):
        self.closed = True


class RecoveringGpio(FakeGpio):
    def write(self, payload):
        super().write(payload)
        if len(self.instances) == 1:
            raise OSError('simulated USB re-enumeration')


def test_backend_paces_usb_writes_and_closes():
    backend = Ft232hPwmBackend(
        url='fake', sample_rate_hz=100_000, frame_us=20_000,
        signal_pin=7, chunk_frames=2, slew_us_per_s=400.0,
    )
    backend._load_controller = lambda: FakeGpio
    assert backend.command(1103) == 1100
    time.sleep(0.13)
    backend.close()

    gpio = FakeGpio.instances[-1]
    assert 2 <= gpio.write_count <= 5
    assert gpio.closed


def test_next_command_reopens_ftdi_after_writer_error():
    FakeGpio.instances.clear()
    backend = Ft232hPwmBackend(
        url='fake', sample_rate_hz=100_000, frame_us=20_000,
        signal_pin=7, chunk_frames=2, slew_us_per_s=400.0,
    )
    backend._load_controller = lambda: RecoveringGpio
    backend.command(840)
    deadline = time.monotonic() + 1.0
    while backend.error is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert 'simulated USB re-enumeration' in backend.error

    assert backend.command(1100) == 1100
    deadline = time.monotonic() + 1.0
    while len(FakeGpio.instances) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(FakeGpio.instances) == 2
    assert backend.recovery_count == 1
    assert backend.error is None
    backend.close()
