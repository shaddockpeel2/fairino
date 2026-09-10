import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SDK_ROOT = (
    ROOT
    / "sdk_py_1.1.4"
    / "OrbbecSDK_Python_v1.1.4_win_x64_release"
    / "python3.9"
)
PY_LIB = SDK_ROOT / "lib" / "python_lib"
C_LIB = SDK_ROOT / "lib" / "c_lib"
os.add_dll_directory(str(C_LIB))
sys.path.insert(0, str(PY_LIB))

from ObTypes import *  # noqa: E402,F403
import Pipeline  # noqa: E402


pipe = Pipeline.Pipeline(None, None)
info = pipe.getDevice().getDeviceInfo()
print("device_info", info.name(), hex(info.vid()), hex(info.pid()), info.serialNumber(), info.firmwareVersion())
for sensor_name, sensor_type in (("color", OB_PY_SENSOR_COLOR), ("depth", OB_PY_SENSOR_DEPTH)):
    profiles = pipe.getStreamProfileList(sensor_type)
    print(sensor_name, "profile_count", profiles.count(), "methods", [x for x in dir(profiles) if not x.startswith("_")])
    for index in range(profiles.count()):
        profile = profiles.getProfile(index).toConcreteStreamProfile(OB_PY_STREAM_VIDEO)
        print(sensor_name, index, profile.width(), profile.height(), profile.fps(), profile.format(), [x for x in dir(profile) if not x.startswith("_")])

print("pipeline_methods", [x for x in dir(pipe) if not x.startswith("_")])
config = Pipeline.Config()
config.enableStream(pipe.getStreamProfileList(OB_PY_SENSOR_COLOR).getProfile(0).toConcreteStreamProfile(OB_PY_STREAM_VIDEO))
config.enableStream(pipe.getStreamProfileList(OB_PY_SENSOR_DEPTH).getProfile(0).toConcreteStreamProfile(OB_PY_STREAM_VIDEO))
config.setAlignMode(OB_PY_ALIGN_D2C_SW_MODE)
pipe.start(config, None)
try:
    camera_param = pipe.getCameraParam()
    print("camera_param_type", type(camera_param), repr(camera_param))
    print("camera_param_dir", [x for x in dir(camera_param) if not x.startswith("_")])
    frames = pipe.waitForFrames(2000)
    color = frames.colorFrame()
    depth = frames.depthFrame()
    for name, frame in (("color", color), ("depth", depth)):
        print(name, "frame_dir", [x for x in dir(frame) if not x.startswith("_")])
        print(name, "frame", frame.width(), frame.height(), frame.format(), frame.timeStamp(), frame.systemTimeStamp(), frame.index())
    print("depth_scale", depth.getValueScale())
finally:
    pipe.stop()
