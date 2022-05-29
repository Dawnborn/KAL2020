# Uncomment the following line to switch to OpenCV 4.
# This file must be placed in the top level folder of the package that require OpenCV.
# Also, make sure to add <depend>libopencv-dev</depend> to your package.xml.
# Please note: The path defined below corresponds to the installation directory in the Jetson
#  Nano. Make sure to adjust it if you installed it somewhere else.

set(OpenCV_DIR /usr/local/include/opencv4/opencv2/) # carla
# set(OpenCV_DIR /usr/local/opencv4/lib/cmake/opencv4/) # jetson

if(DEFINED ENV{OpenCV_DIR})
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()
