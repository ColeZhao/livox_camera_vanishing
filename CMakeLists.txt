cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_vanishing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
)

find_package(Eigen3)
find_package(OpenCV)
find_package(PCL REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_camera_vanishing
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_test src/livox_camera_vanishing_node.cpp include/lidar_projection.hpp)

target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBS}
)

