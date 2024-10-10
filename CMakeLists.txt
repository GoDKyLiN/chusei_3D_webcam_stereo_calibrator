cmake_minimum_required(VERSION 3.5)
project(camera)

# 添加opencv库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIRS})
include_directories("/usr/local/include/opencv")
include_directories("/usr/local/include/opencv2")
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(camera  camera.cpp)
target_link_libraries(camera ${OpenCV_LIBRARIES})
add_executable(stereo_calibrator  stereo_calibrator.cpp)
target_link_libraries(stereo_calibrator ${OpenCV_LIBRARIES})
