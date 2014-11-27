cmake_minimum_required(VERSION 2.8.3)
project(terabot_arm_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_core
  std_srvs
  hardware_interface
  controller_manager
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES terabot_arm_hardware_interface
#  CATKIN_DEPENDS roscpp controller_manager
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${catkin_SRC_DIRS}
  ${Eigen_INCLUDE_DIRS}
  /usr/local/Aria/include/
  ${CMAKE_CURRENT_LIST_DIR}/../terabot_arm_driver/include
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

#########################
## Declare a cpp library

add_executable(Robot src/Robot.cpp src/TerabotArmInterface.cpp) 

message("The path is: ${CMAKE_CURRENT_LIST_DIR}")
link_directories(${catkin_LIBRARIES}/usr/local/Aria/lib ${CMAKE_CURRENT_LIST_DIR}/../terabot_arm_driver/lib)
target_link_libraries(Robot ${catkin_LIBRARIES} /usr/local/Aria/lib/libAria.so ${CMAKE_CURRENT_LIST_DIR}/../terabot_arm_driver/lib/libArTerabotArm.so)


install(PROGRAMS
   scripts/joint_state_publisher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
