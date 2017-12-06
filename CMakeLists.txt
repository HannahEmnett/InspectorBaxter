cmake_minimum_required(VERSION 2.8.3)
project(inspector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  trajectory_msgs
  tf
  tf_conversions
  sensor_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rospy
  std_msgs
)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_message_files(DIRECTORY msg
  FILES
  ObjectList.msg
  PclData.msg
  Pcl_Update.msg
  State.msg
  Update.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)
catkin_package(
   CATKIN_DEPENDS geometry_msgs message_generation message_runtime nav_msgs roscpp rospy std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extractor src/cluster_extractor.cpp)

target_link_libraries(cluster_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

