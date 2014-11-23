#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup group("arm");
   // group.setEndEffectorLink("wrist_roll_link");// added newly (was not included)
    group.setPlanningTime(30.0);
    group.setWorkspace(-3.0,-3.0,-3.0,3.0,3.0,3.0);
    // specify that our target will be a random one
     //0.37042; 0.10712; 1.177
     //0.6863; -0.17026; 0.70499; -0.054705
     geometry_msgs::Pose fixed_pose;
     geometry_msgs::Quaternion quat_msg;
     /*
     fixed_pose.position.x =   1.0007;
     fixed_pose.position.y =  -0.0051214;
     fixed_pose.position.z =  0.24196;
     quat_msg.x = 0;
     quat_msg.y = 0;
     quat_msg.z = 0;
     quat_msg.w = 1;
     ROS_INFO("First Planning step");
     */
     //---------------------------------
     /*
     TRANSFORM X: 1.2151
     TRANSFORM Y: -0.00280483
     TRANSFORM Z: -0.0158832
     */
     /*
     fixed_pose.position.x =  0.92427;
     fixed_pose.position.y =  -0.0029563;
     fixed_pose.position.z =  1.079;
     */
     fixed_pose.position.x =  1.2151;
     fixed_pose.position.y =  0.5028;
     fixed_pose.position.z =  -0.015;     
     quat_msg.x = 0;
     quat_msg.y = 0;
     quat_msg.z = 0;
     quat_msg.w = 1;
     ROS_INFO("First Planning step");
     fixed_pose.orientation = quat_msg;
     group.setApproximateJointValueTarget(fixed_pose);
     //group.setJointValueTarget(fixed_pose);
     group.move();
     
     ros::Duration(3).sleep();
     
//      ROS_INFO("Second Planning step");
//      fixed_pose.position.x =  0.95;
//      fixed_pose.position.y =  0;//00028645
//      fixed_pose.position.z =  0.2;//0.168;
//      quat_msg.x = 0.64984;
//      quat_msg.y = 0.27877;
//      quat_msg.z = -0.27843;
//      quat_msg.w = 0.64998;
//      group.setApproximateJointValueTarget(fixed_pose);
//      group.move();
//      
//      ros::Duration(3).sleep();
//      
//      ROS_INFO("Third Planning step");
//      fixed_pose.position.x =  1.148;
//      fixed_pose.position.y =  -2.216e-05;//00028645
//      fixed_pose.position.z =  0.34273;//0.168;
//      quat_msg.x = 0;
//      quat_msg.y = 0;
//      quat_msg.z = 0;
//      quat_msg.w = 1;
//     
     fixed_pose.orientation = quat_msg;

     std::cout << "get end effector link:"<< group.getEndEffectorLink()<<std::endl;
     //group.setPoseTarget(fixed_pose);
     //group.setJointValueTarget(fixed_pose);
     //-------------------------
     //group.setGoalPositionTolerance(0.5);
     //group.setGoalOrientationTolerance(0.5);
     //group.setApproximateJointValueTarget(fixed_pose);
     // group.setRandomTarget();
     // plan the motion and then move the group to the sampled target 
     //sleep(1);
     //group.move();
     //sleep(5);
     ros::waitForShutdown();
}

//home position
// 1.148; -2.216e-05; 0.34273
//0.66566; 0.23854; -0.23856; 0.66565

//0.92427; -0.0029563; 1.079

//1.059; 0.036072; 0.73613
//-3.5313e-05; 0.48751; -2.6597e-06; 0.87312

//***camera_pose 1
//0.84207; 0.016322; 0.2885
//0.032703; 0.70635; -0.70487; 0.056222

//***camera_pose 2
//0.82051; 0.14474; 0.28846
//-0.061438; 0.70443; -0.69114; 0.14944

//***camera_pose 3
//0.77428; 0.24908; 0.28843
//-0.14309; 0.69248; -0.66903; 0.22892

//***camera_pose 4
//0.82044; -0.14473; 0.28843
//-0.14939; -0.69115; 0.70443; 0.061501

//***camera_pose 5
//0.78233; -0.23492; 0.28845
//-0.21778; -0.67273; 0.69476; 0.13154

//***camera_pose 6
//0.80127; -0.2452; 0.24544
//0.17326; 0.68555; -0.68468; -0.17666

//***camera_pose 7
//0.84587; 0.1348; 0.2455
//0.095717; -0.7006; 0.70106; -0.092313

//***camera_pose 8
//0.90135; -0.14999; 0.28731
//0.090634; 0.70127; -0.70036; -0.097434