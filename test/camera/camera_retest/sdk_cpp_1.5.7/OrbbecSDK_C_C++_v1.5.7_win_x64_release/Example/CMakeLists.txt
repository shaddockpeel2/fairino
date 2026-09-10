cmake_minimum_required(VERSION 3.1.15)
project(OrbbecSDK-Samples LANGUAGES CXX C)

IF(UNIX)
    set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
ENDIF()

# obsensor sdk
set(OrbbecSDK_ROOT_DIR ${CMAKE_SOURCE_DIR}/../SDK)
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)
include_directories(${OrbbecSDK_INCLUDE_DIR})
link_directories(${OrbbecSDK_LIBRARY_DIRS})

# find opencv 
# set(OpenCV_DIR "your/path/to/opencv")  # 可通过该命令指定OpenCV_DIR模块查找路径
find_package(OpenCV QUIET)
if(${OpenCV_FOUND})
    include_directories(${OpenCV_INCLUDE_DIR})
    link_directories(${OpenCV_LIBRARY_DIRS})
ENDIF(${OpenCV_FOUND})

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(c)
add_subdirectory(cpp)
