#include <terabot_arm_hardware_interface/TerabotArmInterface.h>
#include <sstream>

ArRobot robot;
ArTerabotArm arm(&robot);

TerabotArmInterface::TerabotArmInterface(int argc, char** argv)
{
    //joint_state_pub=n_priv.advertise<sensor_msgs::JointState>( "joint_states", 1);

      Aria::init();

      ArLog::init(ArLog::StdErr, ArLog::Normal);
      ArArgumentParser parser(&argc, argv);
      parser.loadDefaultArguments();


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
     
      ArUtil::sleep(500); // need to have read some data from the arm for key handler to work
      robot.lock();
      robot.enableMotors(); 
	 
  //ArModeUnguardedTeleop unguardedTeleopMode(&robot, "unguarded teleop", 'u', 'U');
  //ArModeTeleop teleopMode(&robot, "teleop", 't', 'T');
  //ArModeLaser laserMode(&robot, "laser", 'l', 'L');
  //ArModeCommand commandMode(&robot, "direct robot commands", 'd', 'D');

    pos.resize(joint_number);
    vel.resize(joint_number);
    eff.resize(joint_number);
    cmd.resize(joint_number);
    cmd_previous.resize(joint_number);

    //readHW();

    cmd=pos;
    cmd_previous=cmd;
    // convert to radians and add to state
    for(int i=0; i< pos.size(); ++i)
    {
        std::cout << cmd[i] << std::endl;
    }


    std::cout << "Init done!" << '\n';
 
// init();
    return;
}

TerabotArmInterface::~TerabotArmInterface()
{

    //robot.unlock();
    //robot.waitForRunExit();
   Aria::exit(0);
}



bool TerabotArmInterface::init()
{
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_j1("shoulder_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_j1);

    hardware_interface::JointStateHandle state_handle_j2("forearm_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_j2);

    hardware_interface::JointStateHandle state_handle_j3("upper_arm_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_j3);

    hardware_interface::JointStateHandle state_handle_j4("wrist_pitch_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_j4);

    hardware_interface::JointStateHandle state_handle_j5("wrist_roll_joint", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_j5);

    hardware_interface::JointStateHandle state_handle_j6("j6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_j6);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_j1(jnt_state_interface.getHandle("shoulder_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_j1);

    hardware_interface::JointHandle pos_handle_j2(jnt_state_interface.getHandle("forearm_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_j2);

    hardware_interface::JointHandle pos_handle_j3(jnt_state_interface.getHandle("upper_arm_joint"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_j3);

    hardware_interface::JointHandle pos_handle_j4(jnt_state_interface.getHandle("wrist_pitch_joint"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_j4);

    hardware_interface::JointHandle pos_handle_j5(jnt_state_interface.getHandle("wrist_roll_joint"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_j5);

    hardware_interface::JointHandle pos_handle_j6(jnt_state_interface.getHandle("j6"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_j6);

    registerInterface(&jnt_pos_interface);
}

bool TerabotArmInterface::init(hardware_interface::JointStateInterface & jnt_state_interface_,
                                  hardware_interface::PositionJointInterface & jnt_pos_interface_)
{

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_j1("shoulder_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface_.registerHandle(state_handle_j1);

    hardware_interface::JointStateHandle state_handle_j2("forearm_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface_.registerHandle(state_handle_j2);

    hardware_interface::JointStateHandle state_handle_j3("upper_arm_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface_.registerHandle(state_handle_j3);

    hardware_interface::JointStateHandle state_handle_j4("wrist_pitch_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface_.registerHandle(state_handle_j4);

    hardware_interface::JointStateHandle state_handle_j5("wrist_roll_joint", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface_.registerHandle(state_handle_j5);

    hardware_interface::JointStateHandle state_handle_j6("j6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface_.registerHandle(state_handle_j6);

    registerInterface(&jnt_state_interface_);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_j1(jnt_state_interface_.getHandle("shoulder_joint"), &cmd[0]);
    jnt_pos_interface_.registerHandle(pos_handle_j1);

    hardware_interface::JointHandle pos_handle_j2(jnt_state_interface_.getHandle("forearm_joint"), &cmd[1]);
    jnt_pos_interface_.registerHandle(pos_handle_j2);

    hardware_interface::JointHandle pos_handle_j3(jnt_state_interface_.getHandle("upper_arm_joint"), &cmd[2]);
    jnt_pos_interface_.registerHandle(pos_handle_j3);

    hardware_interface::JointHandle pos_handle_j4(jnt_state_interface_.getHandle("wrist_pitch_joint"), &cmd[3]);
    jnt_pos_interface_.registerHandle(pos_handle_j4);

    hardware_interface::JointHandle pos_handle_j5(jnt_state_interface_.getHandle("wrist_roll_joint"), &cmd[4]);
    jnt_pos_interface_.registerHandle(pos_handle_j5);

    hardware_interface::JointHandle pos_handle_j6(jnt_state_interface_.getHandle("j6"), &cmd[5]);
    jnt_pos_interface_.registerHandle(pos_handle_j6);

    registerInterface(&jnt_pos_interface_);

}




void TerabotArmInterface::readHW()
{

    // READ JOINTS STATe
    float* positions;
    positions = arm.getArmPos();

    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=positions[i];
    }



    // convert to radians and add to state be sure
    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=pos[i]*(DEG_TO_RAD);
    }


    eff[0]=0.0;
    eff[1]=0.0;
    eff[2]=0.0;
    eff[3]=0.0;
    eff[4]=0.0;
    eff[5]=0.0;

    vel[0]=0.0;
    vel[1]=0.0;
    vel[2]=0.0;
    vel[3]=0.0;
    vel[4]=0.0;
    vel[5]=0.0;


}


void TerabotArmInterface::writeHW()
{

 /*  if(isEqual(cmd_previous[0],cmd[0],0.00001)&&
       isEqual(cmd_previous[1],cmd[1],0.00001)&&
       isEqual(cmd_previous[2],cmd[2],0.00001)&&
       isEqual(cmd_previous[3],cmd[3],0.00001)&&
       isEqual(cmd_previous[4],cmd[4],0.00001)&&
       isEqual(cmd_previous[5],cmd[5],0.00001))
    {

        cmd_previous=cmd;
        return;
    }*/
    static int new_command_count=0;
    new_command_count++;
    std::cout << "new command:"<< new_command_count << std::endl;
    //boost::mutex::scoped_lock lock(io_mutex);
    // WRITE MOVE to robot
   arm.moveArm(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  
  
}
