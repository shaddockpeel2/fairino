cmake_minimum_required(VERSION 3.1.15)


add_subdirectory(Sample-HelloOrbbec)
add_subdirectory(Sample-SensorControl)
add_subdirectory(Sample-HotPlugin)
add_subdirectory(Sample-DepthWorkMode)

# opencv required
IF(${OpenCV_FOUND})
    add_subdirectory(Sample-ColorViewer)
    add_subdirectory(Sample-DepthViewer)
    add_subdirectory(Sample-InfraredViewer)
	add_subdirectory(Sample-PointCloud)
	add_subdirectory(Sample-NetDevice)
ENDIF(${OpenCV_FOUND})