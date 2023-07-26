cmake_minimum_required(VERSION 3.0.2)
project(circle_detection_node)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  edcircles
  image_transport
  message_filters
  nav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  CATKIN_DEPENDS edcircles
)


include_directories(
  SYSTEM  
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(circle_detection_node 
  src/circle_detection_node.cpp)


target_link_libraries(circle_detection_node 
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS})