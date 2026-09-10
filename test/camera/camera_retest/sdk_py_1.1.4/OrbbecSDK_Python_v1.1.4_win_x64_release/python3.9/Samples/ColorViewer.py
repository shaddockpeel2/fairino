from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys

q = 113
ESC = 27

try:
  # Create a Pipeline, which is the entry point for the entire Advanced API.
  # 创建一个Pipeline，Pipeline是整个高级API的入口
  pipe = Pipeline.Pipeline(None, None)
  # Configure which streams to enable or disable in Pipeline.
  # 在Pipeline配置中启用或禁用不同类型的流
  config = Pipeline.Config()

  windowsWidth = 0
  windowsHeight = 0
  try:
    # Get all stream configurations for the color camera, including the stream resolution, frame rate, and frame format
    # 获取彩色相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    profiles = pipe.getStreamProfileList(OB_PY_SENSOR_COLOR)

    videoProfile = None
    try:
      # Select the first to open the stream.
      # 选择第一个打开流
      videoProfile = profiles.getProfile(0)
    except ObException as e:
      print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))

    colorProfile = videoProfile.toConcreteStreamProfile(OB_PY_STREAM_VIDEO)
    windowsWidth = colorProfile.width()
    windowsHeight = colorProfile.height()
    config.enableStream(colorProfile)
  except ObException as e:
    print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))
    print("Current device is not support color sensor!")
    sys.exit()

  # Start the stream configured in Config, if no parameters are passed, it will start the default configuration start stream
  # 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
  pipe.start(config, None)

  # Get whether the mirror property has writable permissions
  # 获取镜像属性是否有可写的权限
  if pipe.getDevice().isPropertySupported(OB_PY_PROP_COLOR_MIRROR_BOOL, OB_PY_PERMISSION_WRITE):
    # Set Mirror
    # 设置镜像
    pipe.getDevice().setBoolProperty(OB_PY_PROP_COLOR_MIRROR_BOOL, True)

  while True:
    # Waiting for a frame in a blocking manner, which is a composite frame containing frame data for all enabled streams in the configuration.
    # And setting the frame's wait timeout to 100ms.
    # 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据并设置帧的等待超时时间为100ms
    frameSet = pipe.waitForFrames(100)
    if frameSet == None:
      continue
    else:
      # Render a set of frame data in a window, rendering only color frames here.
      # 在窗口中渲染一组帧数据，这里只渲染彩色帧
      colorFrame = frameSet.colorFrame()
      if colorFrame != None:
        # To get the size and data of a frame:
        # 获取帧的大小和数据
        size = colorFrame.dataSize()
        data = colorFrame.data()

        if size != 0:
          newData = data
          if colorFrame.format() == OB_PY_FORMAT_MJPG:
              newData = cv2.imdecode(newData,1)
              if newData is not None:
                newData = np.resize(newData,(windowsHeight, windowsWidth, 3))
          elif colorFrame.format() == OB_PY_FORMAT_RGB888:
              newData = np.resize(newData,(windowsHeight, windowsWidth, 3))
              newData = cv2.cvtColor(newData, cv2.COLOR_RGB2BGR)
          elif colorFrame.format() == OB_PY_FORMAT_YUYV:
              newData = np.resize(newData,(windowsHeight, windowsWidth, 2))
              newData = cv2.cvtColor(newData, cv2.COLOR_YUV2BGR_YUYV)
          elif colorFrame.format() == OB_PY_FORMAT_UYVY:
              newData = np.resize(newData,(windowsHeight, windowsWidth, 2))
              newData = cv2.cvtColor(newData, cv2.COLOR_YUV2BGR_UYVY)
          elif colorFrame.format() == OB_PY_FORMAT_I420:
              newData = newData.reshape((windowsHeight * 3 // 2, windowsWidth))
              newData = cv2.cvtColor(newData, cv2.COLOR_YUV2BGR_I420)
              newData = cv2.resize(newData, (windowsWidth, windowsHeight))
              
           
          cv2.namedWindow("ColorViewer", cv2.WINDOW_NORMAL)

          if newData is not None:
            cv2.imshow("ColorViewer", newData)

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