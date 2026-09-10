from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys

#  Convert frame data from 16bit to 8bit
def mapUint16ToUint8(img, lowerBound = None, upperBound = None):
  if lowerBound == None:
    lowerBound = np.min(img)
  if upperBound == None:
    upperBound = np.max(img)
  lut = np.concatenate([
    np.zeros(lowerBound, dtype=np.uint16),
    np.linspace(0, 255, upperBound - lowerBound).astype(np.uint16),
    np.ones(2**16 - upperBound, dtype = np.uint16) * 255
  ])
  return lut[ img ].astype(np.uint8)

q = 113
ESC = 27

try:
  # Create a Pipeline, which is the entry point for the entire advanced API 
  # and can be easily opened and closed through the Pipeline Multiple types of streams and get a set of frame data
  # 创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭多种类型的流并获取一组帧数据
  pipe = Pipeline.Pipeline(None, None)
  # Configure which streams to enable or disable in Pipeline by creating a Config
  # 通过创建Config来配置Pipeline要启用或者禁用哪些流
  config = Pipeline.Config()

  windowsWidth = 0
  windowsHeight = 0
  try:
    # Get all stream configurations for the IR camera, including the stream resolution, frame rate, and frame format
    # 获取红外相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    profiles = pipe.getStreamProfileList(OB_PY_SENSOR_IR)

    videoProfile = None
    try:
      # Select the default resolution to open the stream, you can configure the default resolution through the configuration file.
      # 选择默认分辨率打开流，可以通过配置文件配置默认分辨率
      videoProfile = profiles.getProfile(0)
    except ObException as e:
      print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))
      
    depthProfile = videoProfile.toConcreteStreamProfile(OB_PY_STREAM_VIDEO)
    windowsWidth = depthProfile.width()
    windowsHeight = depthProfile.height()
    config.enableStream(depthProfile)
  except ObException as e:
    print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))
    print("Current device is not support IR sensor!")
    sys.exit()

  # Start the stream configured in Config, if no parameters are passed, the default configuration will be used.
  # 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
  pipe.start(config, None)

  # Get whether the mirror property has writable permissions.
  # 创建一个用于渲染的窗口，并设置窗口的分辨率
  if pipe.getDevice().isPropertySupported(OB_PY_PROP_IR_MIRROR_BOOL, OB_PY_PERMISSION_WRITE):
    # Set Mirror.
    # 设置镜像
    pipe.getDevice().setBoolProperty(OB_PY_PROP_IR_MIRROR_BOOL, True)

  while True:
    # wait in a blocking manner for a frame that is a composite frame containing frame data
    # for all the streams enabled in the configuration and set the frame wait timeout to 100ms
    # 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据并设置帧的等待超时时间为100ms
    frameSet = pipe.waitForFrames(100)
    if frameSet == None:
      continue
    else:
      # Render a set of frame data in the window, here only the IR frames are rendered
      # 在窗口中渲染一组帧数据，这里只渲染红外帧
      irFrame = frameSet.irFrame()
      if irFrame != None:
        size = irFrame.dataSize()
        data = irFrame.data()
        ir_format = irFrame.format()
        if size != 0:
          # Resize the frame data to (height,width,2)
          # 调整帧数据的大小为(height,width,2)
          if ir_format == int(OB_PY_FORMAT_Y16):
            data = np.resize(data,(windowsHeight, windowsWidth, 2))
            # Convert frame data from 8bit to 16bit
            # 将帧数据从8位转换为16位
            newData = data[:,:,0]+data[:,:,1]*256
          elif ir_format == int(OB_PY_FORMAT_Y8):
            data = np.resize(data,(windowsHeight, windowsWidth, 1))
            # Convert frame data from 8bit to 16bit
            # 将帧数据从8位转换为16位
            newData = data[:, :, 0]
          # Convert frame data from 16bit to 8bit for rendering
          # 将帧数据从16位转换为8位进行渲染
          newData = mapUint16ToUint8(newData)
          # Convert frame data GRAY to RGB
          # 将帧数据GRAY转换为RGB
          newData = cv2.cvtColor(newData, cv2.COLOR_GRAY2RGB) 

          cv2.namedWindow("InfraredViewer", cv2.WINDOW_NORMAL)

          cv2.imshow("InfraredViewer", newData)

          key = cv2.waitKey(1)
          # Press ESC or 'q' to close the window
          # 按ESC键或者Q键退出运行窗口
          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break

  # Stopping Pipeline will no longer generate frame data
  # 停止Pipeline，将不再产生帧数据
  pipe.stop()

except ObException as e:
  print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))