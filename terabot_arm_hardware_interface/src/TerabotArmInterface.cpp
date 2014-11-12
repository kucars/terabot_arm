#include <terabot_arm_hardware_interface/TerabotArmInterface.h>
#include <sstream>


TerabotArmInterface::TerabotArmInterface(ArRobot *_robot, ArTerabotArm *_arm) : 
	robot(_robot), 
	arm(_arm) 
{

    pos.resize(joint_number);
    vel.resize(joint_number);
    eff.resize(joint_number);
    cmd.resize(joint_number);
    cmd_previous.resize(joint_number);

    firstRead = true;readCounter = 0;
    cmd=pos;
    cmd_previous=cmd;
    // convert to radians and add to state
    for(int i=0; i< pos.size(); ++i)
    {
        std::cout << cmd[i] << std::endl;
    }
    arm->grip(100);
    pos[5]=0;//0 refers to closed gripper
    std::cout << "Init done!" << '\n';
 
    return;
}

TerabotArmInterface::~TerabotArmInterface()
{

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

    hardware_interface::JointStateHandle state_handle_j6("gripper_l_finger_joint", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_j6);
//     
//     hardware_interface::JointStateHandle state_handle_j7("gripper_r_finger_joint", &pos[6], &vel[6], &eff[6]);
//     jnt_state_interface.registerHandle(state_handle_j7);

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

    hardware_interface::JointHandle pos_handle_j6(jnt_state_interface.getHandle("gripper_l_finger_joint"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_j6);
//     
//     hardware_interface::JointHandle pos_handle_j7(jnt_state_interface.getHandle("gripper_r_finger_joint"), &cmd[6]);
//     jnt_pos_interface.registerHandle(pos_handle_j7);

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

    hardware_interface::JointStateHandle state_handle_j6("gripper_l_finger_joint", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface_.registerHandle(state_handle_j6);
//     
//     hardware_interface::JointStateHandle state_handle_j7("gripper_r_finger_joint", &pos[6], &vel[6], &eff[6]);
//     jnt_state_interface_.registerHandle(state_handle_j7);

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

    hardware_interface::JointHandle pos_handle_j6(jnt_state_interface_.getHandle("gripper_l_finger_joint"), &cmd[5]);
    jnt_pos_interface_.registerHandle(pos_handle_j6);
//     
//     hardware_interface::JointHandle pos_handle_j7(jnt_state_interface_.getHandle("gripper_r_finger_joint"), &cmd[6]);
//     jnt_pos_interface_.registerHandle(pos_handle_j7);

    registerInterface(&jnt_pos_interface_);

}


void TerabotArmInterface::readHW()
{
    
    float positions[6];
    //positions = arm.getArmPos();
    
    positions[0]=-arm->getJointPos(0);
    positions[1]=arm->getJointPos(1);
    positions[2]=arm->getJointPos(2);
    positions[3]=arm->getJointPos(3);
    positions[4]=arm->getJointPos(4);
    
    if(cmd[5]>1 && cmd[5]<40)
      pos[5]=1;
    else pos[5]=0;
    
    
    if(firstRead)
    {
      if(++readCounter>10)
	firstRead = false;
      std::cout <<"\nCURRENT POSITION DEGREES: ";
      for(int i=0; i< pos.size()-1; ++i)
      {
	  cmd[i] = positions[i]*(DEG_TO_RAD);
	  std::cout <<positions[i]<<" , ";
      }
      std::cout << "\n";
    }
    //arm.getArmPos(positions);
    ROS_INFO("Reading current postion POSITION");
    std::cout << "CURRENT POSITION DEGREES: ";
    for(int i=0; i< pos.size(); ++i)
    {
        pos[i]=positions[i];
	std::cout <<pos[i]<<" , ";
    }
    //std::cout << "\n";std::cout << "\n";
    // convert to radians and add to state be sure
    
    std::cout << "CURRENT POSITION RADIANS: ";
    for(int i=0; i< pos.size()-1; ++i)
    {
        pos[i]=pos[i]*(DEG_TO_RAD);
	std::cout <<pos[i]<<" , ";
    }
    
    //pos[5]=cmd[5];
    std::cout << "\n";std::cout << "\n";
    
    std::cout << "COMMANDED POSITION IN RADIANS: ";
    for(int i=0; i< pos.size(); ++i)
    {
        std::cout <<cmd[i]<<" , ";
    }
    std::cout << "\n";std::cout << "\n";
    
    eff[0]=0.0;
    eff[1]=0.0;
    eff[2]=0.0;
    eff[3]=0.0;
    eff[4]=0.0;
    eff[5]=0.0;
    eff[6]=0.0;

    vel[0]=0.0;
    vel[1]=0.0;
    vel[2]=0.0;
    vel[3]=0.0;
    vel[4]=0.0;
    vel[5]=0.0;
    vel[6]=0.0;


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

    //convert command cmd from Radians to Degrees
    for(int i=0; i< cmd.size(); ++i)
    {
        cmd[i]=cmd[i]*(RAD_TO_DEG);
    }
   // cmd[4] = 0; // was used for calibration
    if(readCounter<10)
    {
      std::cout <<"\nCURRENT Command DEGREES: ";
      for(int i=0; i< cmd.size(); ++i)
      {
	  std::cout <<cmd[i]<<" , ";
      }
      std::cout << "\n";
    }    
    //used to flip the movement (will be fixed later)
     cmd[0]=-1*cmd[0];
    // moveArm function takes Degrees only as inputs
   arm->moveArm(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
   if(cmd[5]>1 && cmd[5]<40)
     arm->grip(-100);
   else if(cmd[5]<1) 
	  arm->grip(100);
   // arm.moveArm(0, -90, 0, 0, 0);
}