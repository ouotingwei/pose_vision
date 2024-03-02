cmake_minimum_required(VERSION 3.5)
project(orb_stereo_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  OpenCV REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orb_stereo_odom
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(orb_stereo_odom src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(orb_stereo_odom
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
