from ObTypes import *
from Property import *
import Pipeline
import StreamProfile
from Error import ObException
import cv2
import numpy as np
import sys
import math

q = 113
ESC = 27

try:
  # Create a Pipeline, which is the entry point for the entire Advanced API and can be easily opened and closed through the Pipeline
  # Multiple types of streams and acquire a set of frame data
  # 创建一个Pipeline，Pipeline是整个高级API的入口，通过Pipeline可以很容易的打开和关闭多种类型的流并获取一组帧数据
  pipe = Pipeline.Pipeline(None, None)
  # Configure which streams to enable or disable in Pipeline by creating a Config
 # 通过创建Config来配置Pipeline要启用或者禁用哪些流
  config = Pipeline.Config()

  windowsWidth = 0
  windowsHeight = 0
  try:
    # Get all stream configurations for the depth camera, including the stream resolution, frame rate, and frame format
    # 获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
    profiles = pipe.getStreamProfileList(OB_PY_SENSOR_DEPTH)

    videoProfile = None
    try:
      # Select the default resolution to open the stream, you can configure the default resolution through the configuration file
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
    print("Current device is not support depth sensor!")
    sys.exit()

  # Start the stream configured in Config, if no parameters are passed, it will start the default configuration start stream.
  # 启动在Config中配置的流，如果不传参数，将启动默认配置启动流
  pipe.start(config, None)

  # Get whether the mirror property has writable permissions
  # 获取镜像属性是否有可写的权限
  if pipe.getDevice().isPropertySupported(OB_PY_PROP_DEPTH_MIRROR_BOOL, OB_PY_PERMISSION_WRITE):
    # Set Mirror
    # 设置镜像
    pipe.getDevice().setBoolProperty(OB_PY_PROP_DEPTH_MIRROR_BOOL, True)

  while True:
    # Waiting in a blocking manner for a frame of data, which is a composite frame containing frame data for all streams enabled in the configuration.
    # and set the frame wait timeout to 100ms
    # 以阻塞的方式等待一帧数据，该帧是一个复合帧，里面包含配置里启用的所有流的帧数据并设置帧的等待超时时间为100ms
    frameSet = pipe.waitForFrames(100)
    if frameSet == None:
      continue
    else:
      # Renders a set of frames in the window, here only the depth frames are rendered
      # 在窗口中渲染一组帧数据，这里只渲染深度帧，
      depthFrame = frameSet.depthFrame()
      if depthFrame != None:
        size = depthFrame.dataSize()
        data = depthFrame.data()
        if size != 0:
          # Resize the frame data to (height,width,2)
          # 调整帧数据的大小为(height,width,2)
          data = np.resize(data,(windowsHeight, windowsWidth, 2))
          
          # Convert frame data from 8bit to 16bit
          # 将帧数据从8位转换为16位
          newData = data[:,:,0]+data[:,:,1]*256          
          # Conversion of frame data to 1mm units
          # 将帧数据转换为1mm单位
          newData = (newData * depthFrame.getValueScale()).astype('uint16')
          # Prin the Image center distance
          # 打印深度图像中心距离
          print("\033[1;32m-------CenterDistance: %d mm ---------\033[0m\n"%(newData[int(windowsHeight/2),int(windowsWidth/2)]))
          # Rendering display
          # 渲染显示
          newData = (newData / 32).astype('uint8')
          # Convert frame data GRAY to RGB
          # 将帧数据GRAY转换为RGB
          newData = cv2.cvtColor(newData, cv2.COLOR_GRAY2RGB) 

          cv2.namedWindow("DepthViewer", cv2.WINDOW_NORMAL)

          cv2.imshow("DepthViewer", newData)
          # Press ESC or 'q' to close the window
          # 按ESC键或者Q键退出运行窗口
          key = cv2.waitKey(1)
          if key == ESC or key == q:
            cv2.destroyAllWindows()
            break
  # Stopping Pipeline will no longer generate frame data
  # 停止Pipeline，将不再产生帧数据 
  pipe.stop()

except ObException as e:
  print("function: %s\nargs: %s\nmessage: %s\ntype: %d\nstatus: %d" %(e.getName(), e.getArgs(), e.getMessage(), e.getExceptionType(), e.getStatus()))