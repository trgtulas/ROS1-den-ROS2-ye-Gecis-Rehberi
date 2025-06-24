cmake_minimum_required(VERSION 2.8.3)
project(limo_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
  roslaunch
  roslint
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
  gazebo_dev
  trajectory_msgs
  gazebo_msgs
  message_generation
)
## Generate services files in the 'srv' folder
add_service_files(
  FILES 
  GetAngles.srv
  GetCoords.srv
  GripperStatus.srv
  PumpStatus.srv
  SetAngles.srv
  SetCoords.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  gazebo_ros_ackerman
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime trajectory_msgs moveit_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# add_library(gazebo_ros_ackerman src/gazebo_ros_ackerman_drive.cpp)
# target_link_libraries(gazebo_ros_ackerman ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# install(TARGETS gazebo_ros_ackerman
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

