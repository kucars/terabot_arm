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

#ifndef ARTERABOTARM_H
#define ARTERABOTARM_H


#include "ariaTypedefs.h"
#include "ariaOSDef.h"
#include "ArSerialConnection.h"
#include "ariaUtil.h"
#include <math.h>
#include <float.h>
#include <limits>

#define _AR_NAN (std::numeric_limits<float>::quiet_NaN())


class ArRobot;


/** Interface to Terabot manipulator arm.
    This class allows joint control of the manipulator arm as well as reading
    joint state and status information, either automatically via an ArRobot task
    callback if an ArRobot object is provided, or manually by calling the read()
    method.
    
    Joint 0 is the rotating base, joint 1 is the shoulder, joint 2
    is the elbow, joint 3 is the wrist pitch, joint 4 is the wrist rotation. The
    Terabot 2-DOF gripper is treated separately.

    Usage:
    <ol>
    <li>Connect to the mobile robot using ArRobot and ArRobotConnector.
    <li>Create an instance of ArTerabotArm, providing the ArRobot object from step 1.
    <li>Call open() and check for error (false return value).
    <li>When you are ready to begin moving the arm, call powerOn() and enable().
    <li>Use moveArm(), moveJoint(), halt(), openGripper(), closeGripper(),
stopGripper() etc. to control the arm and gripper.
    <li>Use getArmPos(), getJointPos(), getJointStatus(), etc. to obtain the most recently received position information from the arm. This data is automatically requested and read from the arm if an ArRobot was given in the constructor, in step 2.
    <li>Use disable() to temporarily disable control of the arm. Use powerOff() to remove power from the arm if it is going to be idle or unused for a while.
    <li>When finished using the arm, call halt(), disable() and powerOff(), and destroy the ArTerabotArm object.
    </ol>
    See terabotArmDemo.cpp for an example of how to connect to and operate the arm.

    @warning No checking for arm collisions with the robot, ground, itself, or any other
    objects is performed. 

*/
class ArTerabotArm
{
public:
  /** 
 * Constructor. To open the connection to the arm, you must also call open().
 * If @a robot is given, then a Sensor Interpretation Task (see ArRobot documentation
 * and ARIA overview for explanation) is added to <b>request</b> new
 * data from the Terabot arm, and a User Task callback is
 * added to then <b>read</b> the returned data. The request task will
 * happen before the SIP status packet is read from the mobile robot, and the 
 * task will happen after
 * the SIP status packet is received from the mobile robot, at priority 100.
 * Therefore, any user tasks at later priority will be able to utilize both the
 * most recent arm and mobile robot data.  Using this approach, communication with
 * the arm happens automatically, and you can simply access the most recent data returned
 * using the accessor methods in this class (see below).
 *
 * However, if @a robot is <b>not</b>
 * given, then you must manually call requestStaus() and  read() periodically to read
 * all pending data from the arm from the serial connection.
 *
 * The default serial port, if @a port is not given, is COM3 (/dev/ttyS2 on Linux).
 */

/* Not implemented yet:
 *
 * If @a robot is given, but @a port is not, then the serial port to use is determined
 * from the robot's parameters (in the robot's parameter file(s)). Therefore, @a robot must
 * have already be connected to the robot via ArRobotConnector, and parameters loaded
 * with Aria::parseArgs(). (See the ARIA overview and the simpleConnect.cpp example.)
 * If @a port is given however, then this serial port is simply used, overriding any parameters.
 *
 * The default serial port, if neither @a port is given nor any valid port is read from the robot
 * parameter file by @a robot is available, is COM3 (/dev/ttyS2).
 */
  AREXPORT ArTerabotArm(ArRobot *robot = NULL, const char *port = NULL);

  /// On object destruction, send halt and power offcommands to arm and close the device connection
  AREXPORT virtual ~ArTerabotArm();

  /** Set a new device connection object to use. If this is called before open(),
   * then @a conn is used instead of a default serial connection. You can use
   * this to use an ArTcpConnection or other kind of device connection instead of
   * a serial port connection */
  void setDeviceConnection(ArDeviceConnection *conn)
  {
    myCon = conn;
  }

  /** Use the given connection for input from an SC master controller device, which is then sent to the arm. (Normally this is not done.) */
  void setInputDeviceConnection(ArDeviceConnection *conn)
  {
	  myMasterCon = conn;
  }
  
  ArDeviceConnection *getDeviceConnection()
  {
    return myCon;
  }

  /** 
    Open the connection to the arm.
    @return true on success, false on error. 
  */
  AREXPORT bool open();

  /// @return true if device connection is open, false if closed.
  /// The device connection may be closed on some errors reading or writing.
  AREXPORT bool isOpen();

  /// Send a request to halt all joints.
  void halt()
  {
    ArLog::log(ArLog::Normal, "ArTerabotArm: Halting movement...");
    Command(0xCF).write(myCon);
  }
  

  /// Send a request to turn on arm power
  void powerOn()
  {
	ArLog::log(ArLog::Normal, "ArTerabotArm: Arm servo power on...");
    ByteCommand(0x20, 1).write(myCon);
  }

  /// Send a request to turn off arm power
  void powerOff()
  {
    ArLog::log(ArLog::Normal, "ArTerabotArm: Arm servo power off...");
    ByteCommand(0x20, 0).write(myCon);
  }

  /// Send a request to enable all or some joints
  void enableArm(unsigned char state = 0x3F) 
  {
    ByteCommand(0x21, state).write(myCon);
  }

  /// @copydoc enableArm()
  void enable(unsigned char state = 0x3F) { enableArm(state); }

  // Send a request to disable all joints
  void disableArm()
  {
    ByteCommand(0x21, 0).write(myCon);
  }

  /// @copydoc disableArm()
  void disable() { disableArm(); }

  /// Clear errors and reset arm status. Follow by calling enableArm().
  void reset()
  {
    Command(0x22).write(myCon);
  }

  // if any of the arm joints is moving (according to its status byte)
  AREXPORT bool moving();


  /// Move all joints of the arm to given positions (in degrees). 
  void moveArm(float pos1 = _AR_NAN, float pos2 = _AR_NAN, float pos3 = _AR_NAN, float pos4 = _AR_NAN, float pos5 = _AR_NAN) 
  {
    float pose[5] = {pos1, pos2, pos3, pos4, pos5};
    moveArm(pose);
  }

  /** Move all joints of the arm to given positions. @a pose must have at
      least 5 elements.  If any value is NaN or inf, it is ignored. If
      The speeds set with setSpeeds() will be used.
  */
  AREXPORT void moveArm(float pose[5]);

  /// Move a single joint to a given position (in degrees). @a joint selects joint where 0 is the first, base joint. All other arm joints are requested to stay in current position (as reported by Terabot arm)
  AREXPORT void moveJoint(int joint, float pos);
  
  /// Set minimum and maximum allowed joint positions (set on the controller)
  void setJointLimits(float min[5], float max[5])
  {
    LimitsCommand(min, max).write(myCon);
  }

  void setSpeeds(float speed[5]) // set speeds to use in moveArm and moveJoint requests.
  {
    for(int i = 0; i < 5; ++i)
      mySpeeds[i] = speed[i];
  }

  void setJointSpeed(int joint, float vel) 
  {
    mySpeeds[joint] = vel;
  }

  void setAllJointSpeeds(float vel)
  {
    for(int i = 0; i < 5; ++i) setJointSpeed(i, vel);
  }

  void setMaxSpeeds(float max[5])
  {
	MaxVelCommand(max).write(myCon);
  }

  /// Control gripper. @a value is amount of movement from 0 to 100% speed/force (close) or 0 to -100% (open) speed/force.
  AREXPORT void grip(short int value);

  /** Start closing the gripper (maximum force/speed). */
  void closeGripper()
  {
    grip(100);
  }

  /** Start opening the gripper (maximum force/speed). */
  void openGripper()
  {
    grip(-100);
  }

  /** Stop moving the gripper */
  void stopGripper()
  {
puts("stop grip");
    grip(0);
  }

  /// Get the joint positions (in degrees)
  void getArmPos(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5)
  {
    float *pos = getArmPos();
    if(pos1) *pos1 = pos[0];
    if(pos2) *pos2 = pos[1];
    if(pos3) *pos3 = pos[2];
    if(pos4) *pos4 = pos[3];
    if(pos5) *pos5 = pos[4];
  }

  void getArmPos(float pos[5])
  {
	float *p = getArmPos();
	for(int i = 0; i < 5; ++i) pos[i] = p[i];
  }

  /// Get the joint positions (in degrees). An array of five values is returned.
  AREXPORT float* getArmPos();

  /// Get a single joint position (in degrees)
  AREXPORT float getJointPos(int joint);


/* not available on TerabotS
  float getGripperPos() {
	return myGripperPos;
  }
*/

  /** Each joint status value is a bitmask of these states.
      For example, if 'status' is an unsigned short joint status value from getJointStatus(), you can
      check if it is active (has no errors) with
       @code
         if (status & ArTerabotArm::SERVO_ACTIVE )
         {
            ...
       @endcode
       And you can check if the joint has finished a motion to reach its requested position with
       @code
         if(status & ArTerabotArm::ON_TARGET)
         {
            ...
       @endcode
  */
  enum JointStatus {
	  SERVO_ACTIVE = ArUtil::BIT0,
	  ON_TARGET = ArUtil::BIT2,
	  IN_PROGRESS = ArUtil::BIT3,
	  CONTROLLER_OVER_TEMP = ArUtil::BIT5,
	  FOLLOWING_ERROR = ArUtil::BIT7,
	  RESET_COMPLETE = ArUtil::BIT8,
  };

  /// @see JointStatus
  void getJointStatus(unsigned short *s1, unsigned short *s2, unsigned short *s3, unsigned short *s4, unsigned short *s5)
  {
	if(s1) *s1 = myJointStatus[0];
	if(s2) *s2 = myJointStatus[1];
	if(s3) *s3 = myJointStatus[2];
	if(s4) *s4 = myJointStatus[3];
	if(s5) *s5 = myJointStatus[4];
  }

  /// @sa JointStatus
  unsigned short * getJointStatus()
  {
	  return myJointStatus;
  }

  /// Get gripper status value from Terabot.  @note not used on Terabot-S.
  unsigned short getGripperStatus()
  {
	return myGripperStatus;
  }




  /** Send a command to the Terabot arm to request status; if you provided an ArRobot object
      to the constructor, this will happen automatically and you do not need to
      call this method. Otherwise this must be called frequently, followed later by
      read() to read any returned data.
  */
  void requestStatus()
  {
    Command(0xA0).write(myCon);
  }

  /** Read all pending data from the arm and store it; If an ArRobot object was given in the
 * constructor, then read() will automatically be called by the ArRobot task
 * cylcle as a user task and you do not need to call this method. Otherwise, you must call this method
 * periodically (about 5 times per second or faster is recomended.)
 * @param timeout Read timeout in miliseconds. Wait this long for data before
 * returning. If 0, return immediately if no data is available. (Default is 10ms)
 */
  AREXPORT void read(int timeout = 10);

private:
  AREXPORT void readMaster(int timeout = 10);


  // todo: we could use ArPacket
  // classes instead perhaps.
  // Note, don't add any virtual methods to Command classes, since we 
  // will serialize it by simply casting it to a character string.
  // The Command class includes the packet header bytes as char members.
  // Subclasses add fields by adding members. The last byte in a command packet
  // is a checksum which is calculated and written separately when sending
  // the packet.
  // (Adding virtual methods would add vtables to the classes which would 
  // end up in the object data that gets written. Non-virtual methods do
  // not result in any data being included in any objects, they belong
  // only to the class.)

  class Message
  {
  public:
    unsigned char message_header;
    unsigned char data_size;
    unsigned char message_id;
  };

  class Command : public Message
  {
  public:
    Command(unsigned char _id, unsigned char _data_size = 0)
    {
        message_header = 0x5A;
        data_size = _data_size;
        message_id = _id;
    }
    AREXPORT void write(ArDeviceConnection *con);
  };


  // command with one byte data argument
  class ByteCommand : public Command
  {
  public:
    char data;
    ByteCommand(unsigned char id, char arg) :
      Command(id, 1),
      data(arg)
    {}
    AREXPORT void write(ArDeviceConnection *con);
  };

  // command with one integer argument
  class IntCommand : public Command
  {
  public:
    short int data;
    IntCommand(unsigned char id, short int arg) :
      Command(id, 2),
      data(arg)
    {}
    AREXPORT void write(ArDeviceConnection *con);
  };

  class JogCommand : public Command
  {
  public:
    float joint_angle[5];
    float joint_velocity[5];
    JogCommand(float a[5], float v[5]) : Command(0xC0, 40) // 40 is the two sets of 5 floats
    {
      for(int i = 0; i < 5; ++i) joint_angle[i] = a[i];
      for(int i = 0; i < 5; ++i) joint_velocity[i] = v[i];
    }
    AREXPORT void write(ArDeviceConnection *con);
  };

  class LimitsCommand : public Command
  {
  public:
    float joint_min[5];
    float joint_max[5];
    LimitsCommand(float min[5], float max[5]) : Command(0xB1, 40) // 40 is two sets of 5 floats
    {
      for(int i = 0; i < 5; ++i) joint_min[i] = min[i];
      for(int i = 0; i < 5; ++i) joint_max[i] = max[i];
    }
	AREXPORT void write(ArDeviceConnection *con);
   };

  class MaxVelCommand : public Command
  {
  public:
    float max[5];
    MaxVelCommand(float _max[5]) : Command(0xB2, 40) // 20 is 5 floats
    {
      for(int i = 0; i < 5; ++i) max[i] = _max[i];
    }
	AREXPORT void write(ArDeviceConnection *con);
   };

  class StatusMessage : public Message
  {
  public:
    float joint_angle[5];
    unsigned short joint_status[5];
    unsigned short grip_status;
  };

  // Generic message with an untyped 200 byte data buffer. This can be cast to
  // another  Message class of equal or lesser total size.
  class DataMessage : public Message
  {
  public:
    char data[200]; // verbose status query response 0xA1 has 109 bytes of data, the longest as of Oct 11, 2011 protocol document
  };
  

  static unsigned char calculate_crc(unsigned char* data, int size);
  AREXPORT const  char *error_str(unsigned char code);

private:
  AREXPORT void process_incoming_msg();
  ArDeviceConnection *myCon;
  ArSerialConnection myOwnCon;
  ArRobot *myRobot; 
  ArFunctor1C<ArTerabotArm, int> myReadCB;
  ArFunctorC<ArTerabotArm> myRequestCB;
  float myArmPos[5];
  float mySpeeds[5];
  float myGripperPos;
  unsigned short myJointStatus[5];
  unsigned short myGripperStatus;
  DataMessage incoming_msg;
  int total_data_read;
  enum {BEGIN, MESSAGE_ID, DATA_SIZE, DATA, CRC} read_state;
  ArTime myBeginPacketTime;
  ArTime myLastPacketReceivedTime;
  ArDeviceConnection *myMasterCon;

};

#endif
