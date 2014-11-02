#include <controller_manager/controller_manager.h>
#include <terabot_arm_hardware_interface/TerabotArmInterface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terabot_arm_hardware_interface");
    ros::NodeHandle node;
    std::cout << "argc= "<<argc<<" argv= "<<argv<<std::endl;
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;

    //**********************************************************
    //**********************************************************
     Aria::init();
      static float defaultJointSpeed = 15;
      ArLog::init(ArLog::StdErr, ArLog::Normal);
      ArArgumentParser parser(&argc, argv);
      parser.loadDefaultArguments();
      ArRobot robot;
      ArTerabotArm arm(&robot);

      ArRobotConnector robotConnector(&parser, &robot);

      if(!robotConnector.connectRobot())
      {
        ArLog::log(ArLog::Terse, "terabotArm: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed())
        {
            Aria::logOptions();
            Aria::exit(1);
        }
      }
      if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
      {
        Aria::logOptions();
        Aria::exit(1);
      }

      ArLog::log(ArLog::Normal, "terabotArm: Connected to mobile robot.");


      if(!arm.open())
      {
          ArLog::log(ArLog::Terse, "terabotArm: Error opening serial connection to arm");
          Aria::exit(1);
      }


      robot.runAsync(true);
      
      arm.powerOn();
      arm.reset();
      arm.enable();
      arm.setAllJointSpeeds(defaultJointSpeed);
     
      ArUtil::sleep(5000);
      
      robot.lock();
  
      robot.enableMotors(); 
      robot.unlock();
    
    
    
      TerabotArmInterface robot1(&robot, &arm);   
    
    //**********************************************************
    //**********************************************************
    
  //  TerabotArmInterface robot1(&robot, &arm);
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
        //std::cout << "period:"<<period<<std::endl;
        cm.update(now, period);
        robot1.writeHW();
        rate.sleep();

    }

    spinner.stop();

    return 0;
}
