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
