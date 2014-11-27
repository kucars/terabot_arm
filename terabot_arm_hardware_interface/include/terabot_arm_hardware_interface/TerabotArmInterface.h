#ifndef TERABOTARMINTERFACE_H
#include "ros/ros.h" //for the gripper service 
#include <std_srvs/Empty.h> // for the gripper service 

#include <stdio.h> // standard input / output functions
#include <stdlib.h>
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
//#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <boost/thread/mutex.hpp>
#include <math.h>
#include <sstream>
#include "Aria.h"
#include "ArTerabotArm.h"

#define PI_ 3.14159265359
#define DEG_TO_RAD PI_/180.0
#define RAD_TO_DEG 180.0/PI_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>

/*bool isEqual(double & a, double & b, double threshold)
{
    return fabs(a-b)<threshold;

}*/
class TerabotArmInterface : public hardware_interface::RobotHW
{
public:
    TerabotArmInterface(ArRobot *_robot, ArTerabotArm *_arm, ros::NodeHandle & n);
  
    ~TerabotArmInterface();

    bool init();

    bool init(hardware_interface::JointStateInterface &jnt_state_interface_,
              hardware_interface::PositionJointInterface &jnt_pos_interface_);
    
    bool close(std_srvs::Empty::Request&, std_srvs::Empty::Response&);//added for the gripper
    bool open(std_srvs::Empty::Request&, std_srvs::Empty::Response&);//added for the gripper

    void readHW();
    void writeHW();
    bool firstRead; 
    int readCounter;
private:
    static const unsigned int joint_number=5;//6
    ArRobot *robot;
    ArTerabotArm *arm;
    ros::NodeHandle n_;
    ros::ServiceServer close_srv;//added for the gripper
    ros::ServiceServer open_srv;//added for the gripper
    
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    std::vector<double> cmd;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<double> cmd_previous;
    //boost::mutex io_mutex;
  
};
#endif  /*TERABOTARMINTERFACE_H*/