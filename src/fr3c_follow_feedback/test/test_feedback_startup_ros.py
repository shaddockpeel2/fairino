"""Interrupt a real ROS node during a fake SDK RPC; never connect to hardware."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('FR3C_RUN_ROS_TESTS') != '1',
                                reason='isolated ROS domain required')
ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def fake_sdk(tmp_path):
    source = tmp_path/'fake_sdk.cpp'
    source.write_text(r'''
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
void record(const char* message) {
  std::ofstream(std::getenv("FR3C_TEST_RPC_LOG"), std::ios::app) << message << '\n';
}
extern "C" int _ZN7FRRobot3RPCEPKc(void*, const char*) {
  record("rpc_started");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  record("rpc_returned");
  return std::atoi(std::getenv("FR3C_TEST_RPC_CODE"));
}
extern "C" int _ZN7FRRobot8CloseRPCEv(void*) {
  record("rpc_closed");
  return 0;
}
''')
    library = tmp_path/'fake_sdk.so'
    subprocess.run(['c++', '-shared', '-fPIC', '-std=c++17', '-pthread',
                    str(source), '-o', str(library)], check=True, capture_output=True)
    return library


@pytest.mark.parametrize('interrupt,return_code', [(True, 0), (False, 42)])
def test_rpc_is_closed_on_startup_interrupt_or_connection_failure(tmp_path, fake_sdk, interrupt, return_code):
    rpc_log = tmp_path/'rpc.log'
    env = dict(os.environ, LD_PRELOAD=str(fake_sdk), FR3C_TEST_RPC_LOG=str(rpc_log),
               FR3C_TEST_RPC_CODE=str(return_code), ROS_LOG_DIR=str(tmp_path/'ros_logs'))
    executable = ROOT/'install/fr3c_follow_feedback/lib/fr3c_follow_feedback/robot_feedback'
    process = subprocess.Popen([str(executable)], env=env, cwd=tmp_path,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        if interrupt:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline and not rpc_log.exists() and process.poll() is None:
                time.sleep(.01)
            assert rpc_log.exists(), 'node did not enter fake RPC'
            process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == (0 if interrupt else 1), output
        assert rpc_log.read_text().splitlines() == ['rpc_started', 'rpc_returned', 'rpc_closed']
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
