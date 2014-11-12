/*
ArTerabotArm library for use with ARIA 

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012, 2013 Adept MobileRobots

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
/*
ArTerabotArm library for use with ARIA 

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009, 2010 MobileRobots Inc.
Copyright (C) 2011, 2012 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "Aria.h"
#include "ArTerabotArm.h"


/** @example terabotArmDemo.cpp 
   
    This example shows how to connect to and use the Terabot arm via ArTerabotArm.

    Jump down to the main() function at the bottom of this file to see how to
    connect to and enable the arm.

    After connecting to the arm and enabling it, moves the arm to a "ready"
    position, in front of the mobile robot.  Then it creates several modes which 
    display data and respond to keyboard input to perform actions or switch
    modes. These modes include an arm control mode to move the arm to a few
    preset positions, and modes to drive the mobile robot (as in ARIA demo).

    Make sure the arm has a clear workspace before running and using this
    program, and that you can monitor the arm at all times.
*/

static float park [5]= {149, -94, 155, -149, 0}; // collapsed on back of robot
static float forwardReady [5]= {0, -40, -90, -40, 0};
static float straightForward [5]= {89.831, -79.871, -36.404, -27.131, -0.055};
//static float straightForward [5]= {0, -90, 0, 0, 0};
//static float straightForward [5]= {0.045, -0.570, -90.004, -40.048, 90.117};
//static float pickUpFloor [5]= {0, -95, -76.5, -3.5, -0};
//static float pickUpFloor [5]= {0.045, -94.960, -41.063, -40.044, 12.960};
static float pickUpFloor [5]= {19.304, -79.811, -31.639, -27.312, -0.047};
static float right [5]= {45, -54.52, -69.04, -6.93, 0};
static float left[5] = {-45, -54.52, -69.04, -6.93, 0};

static float defaultJointSpeed = 15;

class TerabotDemoMode : public ArMode
{
public:
	ArRobot *robot;
	ArTerabotArm *arm;
	int selectedJoint;
  float command[5];
	bool excersizing;
	int excersizeState;
	float jointSpeed;
	bool waiting;
	ArTime waitTime;
  ArFunctorC<TerabotDemoMode> incJointCB;
  ArFunctorC<TerabotDemoMode> decJointCB;
  ArFunctorC<TerabotDemoMode> excersizeCB;
  ArFunctorC<ArTerabotArm> haltCB;
  ArFunctor1C<ArTerabotArm, float*> rightPosCB;
  ArFunctor1C<ArTerabotArm, float*> leftPosCB;
  ArFunctor1C<ArTerabotArm, float*> fwdPosCB;
  ArFunctor1C<ArTerabotArm, float*> floorPosCB;
  ArFunctor1C<ArTerabotArm, float*> parkPosCB;
  ArFunctorC<ArTerabotArm> closeGripCB;
  ArFunctorC<ArTerabotArm> openGripCB;
  ArFunctorC<ArTerabotArm> powerOnCB;
  ArFunctorC<ArTerabotArm> powerOffCB;
  ArFunctorC<ArTerabotArm> resetCB;
  ArFunctor1C<ArTerabotArm, unsigned char> enableArmCB;
  ArFunctorC<TerabotDemoMode> incSpeedCB;
  ArFunctorC<TerabotDemoMode> decSpeedCB;
  ArFunctor1C<TerabotDemoMode, int> selectJoint0CB;
  ArFunctor1C<TerabotDemoMode, int> selectJoint1CB;
  ArFunctor1C<TerabotDemoMode, int> selectJoint2CB;
  ArFunctor1C<TerabotDemoMode, int> selectJoint3CB;
  ArFunctor1C<TerabotDemoMode, int> selectJoint4CB;

	TerabotDemoMode(ArRobot *_robot, ArTerabotArm *_arm);

	virtual ~TerabotDemoMode();

	virtual void activate();

	virtual void deactivate();

	void select(int joint) { 
		selectedJoint = joint; 
		printf("Joint %d selected.\n", selectedJoint); 
		arm->getArmPos(command);
	}

	void increase() { 
		if(selectedJoint < 0 || selectedJoint > 4) return;
		command[selectedJoint] += 5;
		printf("Joint %d +\n", selectedJoint); 
		arm->moveJoint(selectedJoint, command[selectedJoint]); 
                //or, you can do this too: arm->moveArm(command);
	}

	void decrease() { 
		if(selectedJoint < 0 || selectedJoint > 4) return;
		command[selectedJoint] -= 5;
		printf("Joint %d -\n", selectedJoint); 
		arm->moveJoint(selectedJoint, command[selectedJoint]); 
                //or, you can do this too: arm->moveArm(command);
	}

	void incSpeed() {
		++jointSpeed;
		arm->setAllJointSpeeds(jointSpeed);
  }

	void decSpeed() {
		--jointSpeed;
		arm->setAllJointSpeeds(jointSpeed);
  }

  virtual void help()
  {
	  	puts("Select joint: (F6) base/shoulder yaw,  (F7) shoulder pitch,  (F8) elbow pitch,  (F9) wrist pitch,   (F10) wrist rotate");
	  	puts("Move selected joint: (Arrow Keys)");
	  	puts("Positions: (1) right, (2) left, (3) Forward ready, (4) Floor pick position, (5) Store" );
		puts("(E) excersize (loop of positions)");
	  	puts("(HOME/g) close gripper, (END/G) open gripper");
	  	puts("(SPACE) halt movement, (F2/H) high power on, (F3/h) high power off, (F4/r) reset errors, then re-enable all joints");
	  	puts("(+) increase joint speed, (-) decrease joint speed");
  }

	void toggleExcersize()
	{
		excersizing = !excersizing;
		excersizeState = 0;
		waiting = false;
	}

  void reset()
  {
	  arm->reset();
	  ArUtil::sleep(5000);
	  arm->enable();
  }

  virtual void userTask();
};

TerabotDemoMode::TerabotDemoMode(ArRobot *_robot, ArTerabotArm *_arm) : 
	ArMode(_robot, "TerabotArm", 'a', 'A'),
	robot(_robot), 
  arm(_arm), 
  selectedJoint(-1), 
  excersizing(false), 
  jointSpeed(defaultJointSpeed),
  incJointCB(this, &TerabotDemoMode::increase),
  decJointCB(this, &TerabotDemoMode::decrease),
  excersizeCB(this, &TerabotDemoMode::toggleExcersize),
  haltCB(arm, &ArTerabotArm::halt),
  rightPosCB(arm, &ArTerabotArm::moveArm, right),
  leftPosCB(arm, &ArTerabotArm::moveArm, left),
  fwdPosCB(arm, &ArTerabotArm::moveArm, forwardReady),
  floorPosCB(arm, &ArTerabotArm::moveArm, pickUpFloor),
  parkPosCB(arm, &ArTerabotArm::moveArm, park),
  closeGripCB(arm, &ArTerabotArm::closeGripper),
  openGripCB(arm, &ArTerabotArm::openGripper),
  powerOnCB(arm, &ArTerabotArm::powerOn),
  powerOffCB(arm, &ArTerabotArm::powerOff),
  resetCB(arm, &ArTerabotArm::reset),
  enableArmCB(arm, &ArTerabotArm::enableArm, 0xFF),
  incSpeedCB(this, &TerabotDemoMode::incSpeed),
  decSpeedCB(this, &TerabotDemoMode::decSpeed),
  selectJoint0CB(this, &TerabotDemoMode::select, 0),
  selectJoint1CB(this, &TerabotDemoMode::select, 1),
  selectJoint2CB(this, &TerabotDemoMode::select, 2),
  selectJoint3CB(this, &TerabotDemoMode::select, 3),
  selectJoint4CB(this, &TerabotDemoMode::select, 4)
{
}

TerabotDemoMode::~TerabotDemoMode()
{
}

void TerabotDemoMode::activate()
{
  if(!baseActivate())
    return;
  addKeyHandler(ArKeyHandler::F6, &selectJoint0CB);
  addKeyHandler(ArKeyHandler::F7, &selectJoint1CB); //new ArFunctor1C<TerabotDemoMode, int>(this, &TerabotDemoMode::select, 1));
  addKeyHandler(ArKeyHandler::F8, &selectJoint2CB); // new ArFunctor1C<TerabotDemoMode, int>(this, &TerabotDemoMode::select, 2));
  addKeyHandler(ArKeyHandler::F9, &selectJoint3CB); // new ArFunctor1C<TerabotDemoMode, int>(this, &TerabotDemoMode::select, 3));
  addKeyHandler(ArKeyHandler::F10, &selectJoint4CB); // new ArFunctor1C<TerabotDemoMode, int>(this, &TerabotDemoMode::select, 4));
  addKeyHandler(ArKeyHandler::UP, &incJointCB); // new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::increase));
  addKeyHandler(ArKeyHandler::DOWN, &decJointCB); //new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::decrease));
  addKeyHandler(ArKeyHandler::RIGHT, &incJointCB); // new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::increase));
  addKeyHandler(ArKeyHandler::LEFT, &decJointCB); //new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::decrease));
  addKeyHandler('e', &excersizeCB); // new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::toggleExcersize));
  addKeyHandler(ArKeyHandler::SPACE, &haltCB); // new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::halt));
  addKeyHandler('1', &rightPosCB); // new ArFunctor1C<ArTerabotArm, float*>(arm, &ArTerabotArm::moveArm, right));
  addKeyHandler('2', &leftPosCB); // new ArFunctor1C<ArTerabotArm, float*>(arm, &ArTerabotArm::moveArm, left));
  addKeyHandler('3', &fwdPosCB); // new ArFunctor1C<ArTerabotArm, float*>(arm, &ArTerabotArm::moveArm, forwardReady));
  addKeyHandler('4', &floorPosCB); // new ArFunctor1C<ArTerabotArm, float*>(arm, &ArTerabotArm::moveArm, pickUpFloor));
  addKeyHandler('5', &parkPosCB); // new ArFunctor1C<ArTerabotArm, float*>(arm, &ArTerabotArm::moveArm, park));
  addKeyHandler(ArKeyHandler::HOME, &closeGripCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::closeGripper));
  addKeyHandler('g', &closeGripCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::closeGripper));
  addKeyHandler(ArKeyHandler::END, &openGripCB); // new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::openGripper));
  addKeyHandler('G', &openGripCB); // new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::openGripper));
  addKeyHandler(ArKeyHandler::F2, &powerOnCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::powerOn));
  addKeyHandler('H', &powerOnCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::powerOn));
  addKeyHandler(ArKeyHandler::F3, &powerOffCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::powerOff));
  addKeyHandler('h', &powerOffCB); //new ArFunctorC<ArTerabotArm>(arm, &ArTerabotArm::powerOff));
  addKeyHandler(ArKeyHandler::F4, &resetCB); //new ArFunctorC<ArTerabotArm>(this, &TerabotDemoMode::reset));
  addKeyHandler('r', &resetCB); //new ArFunctorC<ArTerabotArm>(this, &TerabotDemoMode::reset));
  addKeyHandler('R', &resetCB); //new ArFunctorC<ArTerabotArm>(this, &TerabotDemoMode::reset));
  addKeyHandler('+', &incSpeedCB); //new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::incSpeed));
  addKeyHandler('-', &decSpeedCB); //new ArFunctorC<TerabotDemoMode>(this, &TerabotDemoMode::decSpeed));
}

void TerabotDemoMode::deactivate()
{
  if(!baseDeactivate())
    return;

  remKeyHandler(&selectJoint0CB);
  remKeyHandler(&selectJoint1CB);
  remKeyHandler(&selectJoint2CB);
  remKeyHandler(&selectJoint3CB);
  remKeyHandler(&selectJoint4CB);
  remKeyHandler(&incJointCB);
  remKeyHandler(&decJointCB);
  remKeyHandler(&excersizeCB);
  remKeyHandler(&haltCB);
  remKeyHandler(&rightPosCB);
  remKeyHandler(&leftPosCB);
  remKeyHandler(&fwdPosCB);
  remKeyHandler(&floorPosCB);
  remKeyHandler(&parkPosCB);
  remKeyHandler(&closeGripCB);
  remKeyHandler(&openGripCB);
  remKeyHandler(&powerOnCB);
  remKeyHandler(&powerOffCB);
  remKeyHandler(&resetCB);
  remKeyHandler(&incSpeedCB);
  remKeyHandler(&decSpeedCB);
}


// This is called as a user task by ArRobot every cycle.
// TerabotDemoMode uses this to print new data, and also
// monitor and perform the "excersize" cycle.
void TerabotDemoMode::userTask()
{

  // Print out status
  unsigned short *s = arm->getJointStatus();
  if(excersizing)
    printf("{EXCERSIZING}  ");
  for (int i = 0; i < 5; ++i)
  {
    if(selectedJoint == i)
      printf("{");
    printf("%d:% .2f:", i, arm->getJointPos(i));
    if( !(s[i] & ArTerabotArm::SERVO_ACTIVE) ) printf("OFF!.");
    if(s[i] & ArTerabotArm::ON_TARGET) printf("DONE.");
    if(s[i] & ArTerabotArm::IN_PROGRESS) printf("MOVE.");
    if(s[i] & ArTerabotArm::CONTROLLER_OVER_TEMP) printf("HOT!.");
    if(s[i] & ArTerabotArm::FOLLOWING_ERROR) printf("ERR!.");
    if(s[i] & ArTerabotArm::RESET_COMPLETE) printf("RES..");
    if(selectedJoint == i)
      printf("}");
    if(i != 4)
      printf(", ");
  }
  printf("  Speed=% .2f \r", jointSpeed);


  // Do the excersize cycle, waiting 3 seconds after the arm stops moving at
  // each position.
  if(excersizing && !arm->moving()) 
  {
    if(!waiting)
    {
      waitTime.setToNow();
      waiting = true;
    }
    else
    {
      if(waitTime.secSince() >= 5)
      {
        waiting = false;
        switch(excersizeState)
        {
          case 0:
            arm->moveArm(pickUpFloor);
            arm->closeGripper();
            break;
          case 1:
            arm->moveArm(straightForward);
            break;
          case 2:
            //arm->moveArm(left);
            arm->openGripper();
            break;
          case 3:
            arm->moveArm(right);
            break;
          case 4:
            arm->moveArm(forwardReady);
            break;
        }
        if(++excersizeState > 2) {
          excersizeState = 0;
        }
      }
    }
  }
}




int main(int argc, char **argv)
{
  Aria::init();
  ArLog::init(ArLog::StdErr, ArLog::Normal);
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "terabotArmDemo: Could not connect to the robot.");
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

  ArLog::log(ArLog::Normal, "terabotArmDemo: Connected to mobile robot.");

  ArTerabotArm arm(&robot); // by default, use COM4 to connect to the arm.
  if(!arm.open())
  {
      ArLog::log(ArLog::Terse, "terabotArmDemo: Error opening serial connection to arm");
      Aria::exit(1);
  }

 // to read input from SC Master device and pass the commands through to the
 // arm.
 // ArSerialConnection masterCon;
 // masterCon.open(ArUtil::COM3);
 // arm.setInputDeviceConnection(&masterCon);

  robot.runAsync(true);

  arm.powerOn();
  arm.reset();
  arm.enableArm();
  arm.setAllJointSpeeds(defaultJointSpeed);

  ArLog::log(ArLog::Terse, "\nWARNING\n\nAbout to move the manipulator arm to the front of the robot in 5 seconds. Press Control-C to cancel and exit the program.\n...");
  ArUtil::sleep(5000);
  ArLog::log(ArLog::Terse, "Now moving the arm...");
  arm.moveArm(forwardReady);

 
  float j1, j2, j3, j4, j5;
 

  ArUtil::sleep(500); // need to have read some data from the arm for key handler to work

  robot.lock();
  TerabotDemoMode terabotDemoMode(&robot, &arm);
  ArModeUnguardedTeleop unguardedTeleopMode(&robot, "unguarded teleop", 'u', 'U');
  ArModeTeleop teleopMode(&robot, "teleop", 't', 'T');
  ArModeLaser laserMode(&robot, "laser", 'l', 'L');
  //ArModeConfig configMode(&robot, "show robot config", 'o', 'O');
  ArModeCommand commandMode(&robot, "direct robot commands", 'd', 'D');
  terabotDemoMode.activate();

  robot.enableMotors(); // teleop modes don't

  robot.unlock();

  robot.waitForRunExit();


  ArLog::log(ArLog::Normal, "terabotArmDemo: Either mobile robot or arm disconnected. Exiting.");
  Aria::exit(0);
}


