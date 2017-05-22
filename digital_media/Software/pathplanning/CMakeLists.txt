cmake_minimum_required(VERSION 2.8.3)
project(pathplanning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   markerpose.msg
   coord.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(pathplanning_library
	src/astar.cpp
	src/coordTrans.cpp
	src/pathplanningControl.cpp
)


add_executable(pathplanner_node src/astarPP.cpp src/astar.cpp src/coordTrans.cpp src/pathplanningControl.cpp)


 target_link_libraries(pathplanner_node
   ${catkin_LIBRARIES}
   ${OpenCV_INCLUDE_DIRS}
   pathplanning_library
 )




