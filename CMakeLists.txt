cmake_minimum_required(VERSION 3.5)
project(image_undistort)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    roscpp
    rosbag
    rospy
    std_msgs
    cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)   # 4.2.0

# ----------------------------------------------------------------------------------------------------------------------------- #
# 开始打包相机模型
# ----------------------------------------------------------------------------------------------------------------------------- #

# 配置子程序
file(GLOB CAMERA_MODEL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_gpl/**.c**
    ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/**.c**
)

add_library(
        CAMERA_MODEL_LIB SHARED
        ${CAMERA_MODEL_FILES}
)

target_include_directories(
    CAMERA_MODEL_LIB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    CAMERA_MODEL_LIB PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
# ----------------------------------------------------------------------------------------------------------------------------- #
# 完成打包相机模型
# ----------------------------------------------------------------------------------------------------------------------------- #

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag rospy std_msgs sensor_msgs cv_bridge
    DEPENDS EIGEN3
)

add_executable(${PROJECT_NAME}_node 
    exe/test_undistorted.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    CAMERA_MODEL_LIB
)