#include <controller_manager/controller_manager.h>
#include <terabot_arm_hardware_interface/TerabotArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terabot_arm_hardware_interface");
    ros::NodeHandle node;
  std::cout << "argc= "<<argc<<" argv= "<<argv<<std::endl;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    
    TerabotArmInterface robot1(argc,argv);
    std::cout << "after creating the object"<<std::endl;
    robot1.init(jnt_state_interface_, jnt_pos_interface_);
    controller_manager::ControllerManager cm(&robot1, node);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time previous=ros::Time::now();

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::Duration period;
        robot1.readHW();
        ros::Time now=ros::Time::now();
        period=now-previous;
        std::cout << "period:"<<period<<std::endl;
        cm.update(now, period);
        robot1.writeHW();
        rate.sleep();

    }

    spinner.stop();

    return 0;
}
