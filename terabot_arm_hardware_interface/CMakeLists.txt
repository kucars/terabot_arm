cmake_minimum_required(VERSION 2.8.3)
project(terabot_arm_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  hardware_interface
  controller_manager
  actionlib
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES terabot_arm_hardware_interface
#  CATKIN_DEPENDS roscpp controller_manager
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${catkin_SRC_DIRS} 
  /usr/local/Aria/include/
  /home/marbm/catkin_ws_sdp/src/seekur_comp/ArTerabotArm-2.8.0/include
  /home/marbm/catkin_ws_sdp/src/seekur_comp/terabot_arm/terabot_arm_hardware_interface/include/
)

#########################
## Declare a cpp library

add_executable(Robot src/Robot.cpp src/TerabotArmInterface.cpp) 


link_directories(${catkin_LIBRARIES}/usr/local/Aria/lib /home/marbm/catkin_ws_sdp/src/seekur_comp/ArTerabotArm-2.8.0/lib ${terabot_arm_hardware_interface_SOURCE_DIR}/ArTerabotArm-2.8.0/lib)
target_link_libraries(Robot ${catkin_LIBRARIES} /usr/local/Aria/lib/libAria.so /home/marbm/catkin_ws_sdp/src/seekur_comp/ArTerabotArm-2.8.0/lib/libArTerabotArm.so)


install(PROGRAMS
   scripts/joint_state_publisher
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
