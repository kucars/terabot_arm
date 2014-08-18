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

#include "Aria.h"
#include "ArTerabotArm.h"

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

  ArTerabotArm arm(&robot, ArUtil::COM3);
  if(!arm.open())
  {
      ArLog::log(ArLog::Terse, "terabotArmDemo: Error opening serial connection to arm (COM3)");
      Aria::exit(1);
  }

  robot.runAsync(true);


  float j1, j2, j3, j4, j5;
  unsigned short s1, s2, s3, s4, s5, sG;
  while(robot.isConnected())
  {
      robot.lock();
      arm.getArmPos(&j1, &j2, &j3, &j4, &j5);
      arm.getJointStatus(&s1, &s2, &s3, &s4, &s5);
      sG = arm.getGripperStatus();
      printf("Arm=(%.2f, %.2f, %.2f, %.2f, %.2f), Gripper=%.2f, Status=(%d, %d, %d, %d, %d, G=%d), Robot Battery=%.2fV\n", j1, j2, j3, j4, j5, arm.getGripperPos(), s1, s2, s3, s4, s5, sG, robot.getRealBatteryVoltage());
      robot.unlock();
      ArUtil::sleep(1000);
  }


  ArLog::log(ArLog::Normal, "terabotArmDemo: Either mobile robot or arm disconnected. Exiting.");
  Aria::exit(0);
}
