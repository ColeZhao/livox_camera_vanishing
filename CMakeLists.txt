cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_vanishing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
)

find_package(Eigen3)
find_package(OpenCV)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

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

add_executable(vanishing_calib src/livox_camera_vanishing_node.cpp include/lidar_projection.hpp)
add_executable(vanishing_calib_multi_group src/livox_camera_vanishing_multi_group_node.cpp include/lidar_projection.hpp)
add_executable(csv2pcd src/trans_test.cpp)
add_executable(test_sim src/test.cpp)

target_link_libraries(vanishing_calib
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(vanishing_calib_multi_group
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(csv2pcd
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBS}
  ${CERES_LIBRARIES}
)

target_link_libraries(test_sim
  ${catkin_LIBRARIES}
  ${Eigen3_LIBS}
  ${OpenCV_LIBS}
  ${PCL_LIBS}
  ${CERES_LIBRARIES}
)
