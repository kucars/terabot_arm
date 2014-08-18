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
#include "ArExport.h"
#include "ArTerabotArm.h"

//#define DEBUG(x) {x} // include debug output code
#define DEBUG(x) {} // omit debug output code

AREXPORT ArTerabotArm::ArTerabotArm(ArRobot *robot, const char *port) :
  myCon(NULL),
  myRobot(NULL),
  myReadCB(ArFunctor1C<ArTerabotArm, int>(this, &ArTerabotArm::read, 10)),
  myRequestCB(ArFunctorC<ArTerabotArm>(this, &ArTerabotArm::requestStatus)),
  read_state(BEGIN),
  myMasterCon(NULL)
{
  for(int i = 0; i < 5; ++i)
  {
    myArmPos[i] = _AR_NAN;
    mySpeeds[i] = 0;
    myJointStatus[i] = 0;     
  }
  myGripperStatus = 0;
  if(port)
    myOwnCon.setPort(port);
  else if(robot)
  {
    myOwnCon.setPort(ArUtil::COM4); // default
    // TODO read port, x, y and z from robot parameter file if ArRobot is given.
  }
  else
    myOwnCon.setPort(ArUtil::COM4);
  myOwnCon.setBaud(19200);
  if(robot)
  {
    myRobot = robot;
    myRobot->lock();
    ArLog::log(ArLog::Normal, "ArTerabotArm: Switching on Seekur/SeekurJr power port #29 for 24V arm power...");
    if(myRobot->isConnected())
    {
   	//myRobot->com2Bytes(116, 29, 0);	// cycle arm power on Seekur Jr.
        //myRobot->unlock();
        //ArUtil::sleep(500);
        //myRobot->lock();
        myRobot->com2Bytes(116, 29, 1); // turn on arm power
        myRobot->com2Bytes(116, 7, 1);  // turn on camera power
	ArUtil::sleep(500); // give it a moment to power on
    }
    else
    {
  	myRobot->addConnectCB(new ArRetFunctor3C<bool, ArRobot, unsigned char, char, char>(myRobot, &ArRobot::com2Bytes, 116, 29, 1));
    }
    myRobot->addSensorInterpTask("ArTerabotArm::requestStatus", 400, &myRequestCB);
    myRobot->addUserTask("ArTerabotArm::read", 100, &myReadCB);
    Aria::addExitCallback(new ArFunctorC<ArTerabotArm>(this, &ArTerabotArm::powerOff));
    myRobot->unlock();
  }
}

AREXPORT ArTerabotArm::~ArTerabotArm()
{
  halt();
  powerOff();
  if(myCon)
    myCon->close();
  if(myRobot)
    myRobot->remSensorInterpTask(&myReadCB);
}

AREXPORT bool ArTerabotArm::open()
{
  if(!myCon)
  {
    myCon = &myOwnCon;
    ArLog::log(ArLog::Normal, "ArTerabotArm: opening serial connection on %s at %d baud, %s harware control", myOwnCon.getPort(), myOwnCon.getBaud(), myOwnCon.getHardwareControl()?"with":"without");  
  }
  else 
  {
     ArLog::log(ArLog::Normal, "ArTerabotArm: opening device connection...");
  }
  return myCon->openSimple();
}

AREXPORT bool ArTerabotArm::isOpen()
{
    return myCon && myCon->getStatus() == ArDeviceConnection::STATUS_OPEN;
}

AREXPORT void ArTerabotArm::Command::write(ArDeviceConnection *con)
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;

    /// @todo close connection on write errors?

	unsigned char buf[4];
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	buf[3] = ArTerabotArm::calculate_crc(buf+1, 2);
	bool r = con->write((char*)buf, 4);
  if(!r)
  {
    ArLog::log(ArLog::Terse, "ArTerabotArm: Error writing command 0x%hhx data!", message_id);
    return;
  }
}

AREXPORT void ArTerabotArm::ByteCommand::write(ArDeviceConnection *con)
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;

    /// @todo close connection on write errors?

	unsigned char buf[5];
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	buf[3] = data;
	buf[4] = ArTerabotArm::calculate_crc(buf+1, 3);
	bool r = con->write((char*)buf, 5);
  if(!r)
  {
    ArLog::log(ArLog::Terse, "ArTerabotArm: Error writing ByteCommand 0x%hhx data!", message_id);
    return;
  }
}

AREXPORT void ArTerabotArm::IntCommand::write(ArDeviceConnection *con)
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;

    /// @todo close connection on write errors?

	unsigned char buf[6];
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	//buf[3] = (char)(data & 0xFF);
	//buf[4] = (char)( data >> 8);
	short int* p = (short int*)(buf+3);
	*p = data;
	buf[5] = ArTerabotArm::calculate_crc(buf+1, 4);
	bool r = con->write((char*)buf, 6);
	//printf("int command 0x%x with %d size %d crc %d  => %s)\n", message_id, data, data_size, buf[5], r?"sent":"send error");
  if(!r)
  {
    ArLog::log(ArLog::Terse, "ArTerabotArm: Error writing IntCommand 0x%hhx data!", message_id);
    return;
  }
}

void ArTerabotArm::JogCommand::write(ArDeviceConnection *con) 
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;
	unsigned char buf[4+40];	// 40 is 2 sets of 5 floats
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	float *p = (float*) (buf+3);
	for(int i = 0; i < 5; ++i) { *p = joint_angle[i]; printf("%.2f ", *p); ++p; }
	for(int i = 0; i < 5; ++i) { *p = joint_velocity[i];printf("%.2f ", *p);  ++p; }
	*((char*)p) = ArTerabotArm::calculate_crc(buf+1, 2+40);
	bool r = con->write((char*)buf, 4+40);
	if(!r)
		ArLog::log(ArLog::Terse, "ArTerabotArm::Error writing JogCommand data!");
}

void ArTerabotArm::LimitsCommand::write(ArDeviceConnection *con) 
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;
	unsigned char buf[4+40];	// 40 is 2 sets of 5 floats
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	float *p = (float*) (buf+3);
	for(int i = 0; i < 5; ++i) { (*p) = joint_min[i]; ++p; }
	for(int i = 0; i < 5; ++i) { (*p) = joint_max[i]; ++p; }
	*((char*)p) = ArTerabotArm::calculate_crc(buf+1, 2+40);
	bool r = con->write((char*)buf, 4+40);
	if(!r)
		ArLog::log(ArLog::Terse, "ArTerabotArm::Error writing LimitsCommand data!");
}

void ArTerabotArm::MaxVelCommand::write(ArDeviceConnection *con) 
{
    if(con == NULL || con->getStatus() != ArDeviceConnection::STATUS_OPEN)
        return;
	unsigned char buf[4+40];	// 20 is 1 sets of 5 floats
	buf[0] = message_header;
	buf[1] = data_size;
	buf[2] = message_id;
	float *p = (float*)(buf+3);
	for(int i = 0; i < 5; ++i) { *p = max[i]; ++p; }
	*((char*)p) = ArTerabotArm::calculate_crc(buf+1, 2+20);
	bool r = con->write((char*)buf, 4+20);
	if(!r)
		ArLog::log(ArLog::Terse, "ArTerabotArm::Error writing MaxVelCommand data!");
}

AREXPORT void ArTerabotArm::moveArm(float pose[5])
{
  ArLog::log(ArLog::Normal, "ArTerabotS: Moving arm to: [%.3f, %.3f, %.3f, %.3f, %.3f]. Current pose is: [%.3f, %.3f, %.3f, %.3f, %.3f]", 
	pose[0], pose[1], pose[2], pose[3], pose[4], 
	myArmPos[0], myArmPos[1], myArmPos[2], myArmPos[3], myArmPos[4]
  );
/*
  printf("ArTerabotS: Moving arm to: [%.3f, %.3f, %.3f, %.3f, %.3f]. Current pose is: [%.3f, %.3f, %.3f, %.3f, %.3f]\n", 
	pose[0], pose[1], pose[2], pose[3], pose[4], 
	myArmPos[0], myArmPos[1], myArmPos[2], myArmPos[3], myArmPos[4]
  );
*/
  JogCommand c(pose, mySpeeds);
  c.write(myCon);
}

AREXPORT void ArTerabotArm::moveJoint(int joint, float pos)
{
  if(joint < 0 || joint > 4 || 
#ifdef WIN32
!_finite(pos)
#else
!isfinite(pos)
#endif
  )
    return;
  float p[5];
  for(int i = 0; i < 5; ++i) p[i] = myArmPos[i]; // TODO use last commanded arm position instead, so we can do simultaneous moves.
  p[joint] = pos;
  moveArm(p);
}

AREXPORT float* ArTerabotArm::getArmPos() 
{
  return myArmPos;
}

AREXPORT float ArTerabotArm::getJointPos(int joint)
{
  if(joint < 0 || joint > 4)
    return _AR_NAN;
  return myArmPos[joint];
}

AREXPORT void ArTerabotArm::read(int timeout)
{
	if(myMasterCon) readMaster(timeout);

    if(!isOpen()) return;


    // read bytes from arm until we get 0x5B, followed by message id in the proper range, followed by packet data size. Then read that many bytes followed by crc. If there isn't enough data, return and continue where we left off when read() is called again.
    while (true)
    {
/*
        if(myLastPacketReceivedTime.secSince() >= 2)
        {
            ArLog::log(ArLog::Terse, "ArTerabotArm: Error, have not received any valid packets for 2 seconds, halting arm and closing connection.");
            halt();
            powerOff();
            myCon->close();
        }
        if(myBeginPacketTime.mSecSince() >= 400)
        {
            ArLog::log(ArLog::Terse, "ArTerabotArm: Warning, no valid packet recieved after 400ms, aborting current read.");
	    myBeginPacketTime.setToNow();
            return;
        }
*/
        char c, crc;
        int n;
		char buf[256];
        if(read_state != DATA)
        {
            int n = myCon->read(&c, 1, timeout);
//	    DEBUG(printf("\nread: %d bytes: ", n);)
	    if(myMasterCon) myMasterCon->write(&c, 1);
            // todo handle read error?
            if (n == 0) {
		DEBUG(puts("no data");)
		return;
	    }
            else if(n < 0) {
		DEBUG(puts("error");)
		return;
	    }
        }
/*
        DEBUG({
		if(c == 0x5B  || c == 0x5A)
		 printf("[0x%hhx], ", c);
		else
		 printf("0x%hhx (%d),", c, c);
       })
*/
        switch (read_state) {

            case BEGIN:
                myBeginPacketTime.setToNow();
                if(c == 0x5B || c == 0x5A)
                {
		    //DEBUG(puts("(HEADER)");)
                    read_state = DATA_SIZE;
                    total_data_read = 0;
		    memset(&incoming_msg, 0xAA, sizeof(incoming_msg));
                }
                // else skip this byte and continue reading
                break;

           case DATA_SIZE:
		//DEBUG(puts("(SIZE) ");)
                incoming_msg.data_size = c;
                //if(incoming_msg.data_size == 0)
                //{
                //    // skip data stage
                //    read_state = CRC;
                //}
                /*else*/ if (incoming_msg.data_size > 200)
                {
                    ArLog::log(ArLog::Terse, "ArTerabotS: Error: incoming message can't have data size more than 200! Ignoring this packet.");
                    read_state = BEGIN;
                }
                else
                {
                    read_state = MESSAGE_ID;
                    total_data_read = 0;
                }
                break;

           case MESSAGE_ID:
                // todo check c against range of all messages from protocol spec
		//DEBUG(puts("(ID) ");)
                incoming_msg.message_id = c;
                read_state = DATA;
                break;

            case DATA:
                DEBUG(printf("READING DATA... (%d bytes, total so far=%d, total data size=%d)\n", (incoming_msg.data_size-total_data_read), total_data_read, incoming_msg.data_size);)
		n = myCon->read(buf, incoming_msg.data_size - total_data_read, timeout);
		if(myMasterCon) myMasterCon->write(buf, n);
		DEBUG(printf("...(GOT %d bytes)\n", n);)
		memcpy( &(incoming_msg.data) + total_data_read, buf, n);
                total_data_read += n;
                if(total_data_read < incoming_msg.data_size)
                {
                    return;
                }
                else
                {
                    // we have enough data
                    //assert(total_data_read == incoming_msg.data_size);
                    read_state = CRC;
                }
                break;

            case CRC:
		DEBUG(puts("(CRC) ");)
                crc = calculate_crc( ((unsigned char*)&incoming_msg) + 1 /* skip message header*/, 
					1 /*for data_size byte*/ + 1 /*for message_id byte*/ + incoming_msg.data_size /*for message body data*/);
                if(c == crc)
                {
                    myLastPacketReceivedTime.setToNow();
                    process_incoming_msg();
                }
                else
                {
                    ArLog::log(ArLog::Normal, "ArTerabotS: Warning: incoming message 0x%hhx has wrong checksum (has 0x%hhx, expected 0x%hhx). Ignoring.", incoming_msg.message_id, c, crc);
                }
                read_state = BEGIN;
                return;
        }
    }
}

AREXPORT void ArTerabotArm::readMaster(int timeout)
{
    if(!myMasterCon) return;
	if(myMasterCon->getStatus() != ArDeviceConnection::STATUS_OPEN) return;
	if(!isOpen()) return;

	char buf[128];
	int n = myMasterCon->read(buf, 128, timeout);

        if (n <= 0) return;


	// log for debugging:
	bool isJog = false;
	printf("From MC: ");
	int x = 0;
	for(int i = 0; i < n; ++i)
	{
		if(buf[i] == 0x5A || buf[i] == 0x5B)
		{
			printf("[0x%hhx] ", buf[i]);
			x = 1;
		}
		else
		{
			if(x == 1)
				printf("SIZE=");
			else if(x == 2)
			{
				printf("ID=");
				isJog = (buf[i] == 0xC0);
			}
			else if(x == 3)
				printf("DATA: ");
			printf("0x%hhx (%d) ", buf[i]);
		}
		++x;
	}
	puts("");

	if(isJog)
	{
		printf("Jog data: ");
		float *p = (float*)(buf+3);
		for(int i = 0; i < 5; ++i)
		{
			printf("{%.2f  ", p[i]);
		}
		printf("}\n");
	}

	myCon->write(buf, n);
}
	

AREXPORT void ArTerabotArm::process_incoming_msg()
{
    //StatusMessage *status_msg = 0;
    char *data = incoming_msg.data;
//    char c;
    float v;
    switch (incoming_msg.message_id)
    {
        case 0xA0: // QUERY_STATUS response
	    //DEBUG(puts("\n<-QUERY response!");)
            //status_msg = reinterpret_cast<StatusMessage*>(&incoming_msg);
            for(int j = 0; j < 5; ++j)
	    {
		memcpy(&myArmPos[j], data, 4);
                data += 4;
//                DEBUG(printf("\tp%d=%f\n", j, myArmPos[j]);)
	    }
            for(int j = 0; j < 5; ++j)
	    {
		memcpy(&myJointStatus[j], data, 2);
                data += 2;
//                DEBUG(printf("\ts%d=%d\n", j, myJointStatus[j]);)
	    }
            memcpy(&myGripperStatus, data, 2);
//            DEBUG(printf("\tsG=%d\n", myGripperStatus);)
            break;
	case 0xA2: // SETTINGS response
		DEBUG(puts("\n<- SETTINGS response");)
		for(int j = 0; j < 5; ++j) {
			memcpy(&v, data, 4);
			printf("Joint %d min = %.2f\n", j, v);
		}
		for(int j = 0; j < 5; ++j) {
			memcpy(&v, data, 4);
			printf("Joint %d max = %.2f\n", j, v);
		}
		for(int j = 0; j < 5; ++j) {
			memcpy(&v, data, 4);
			printf("Joint %d max velocity = %.2f\n", j, v);
		}
		break;
    case 0xE0:  // Error
        ArLog::log(ArLog::Terse, "ArTerabotArm: Arm reported error 0x%hhx (%d) handling command 0x%hhx (%d): %s", incoming_msg.data[0], incoming_msg.data[1], error_str(incoming_msg.data[1]));
        break;
    default:
        ArLog::log(ArLog::Terse, "ArTerabotArm: Warning: unrecognized message id 0x%hhx received.", incoming_msg.message_id);
    }
}




/*
 * implemented in moveArm
void jog_robot(void)
{
int joint;
struct jog_packet
	{
	unsigned char message_header;
	unsigned char data_size;
	unsigned char message_id;
	float joint_angle[5];
	float joint_velocity[5];	 
	unsigned char crc;
	}  jog_msg={0x5A,40,0xC0};

for(joint=0;joint<=4;joint++)
	{
	jog_msg.joint_angle[joint]=cmd_joint_angle[joint];
	jog_msg.joint_velocity[joint]=cmd_joint_velocity[joint];
	}
jog_msg.crc=calculate_crc(&jog_msg.data_size,jog_msg.data_size+2);


//send request for data
write_com1(&jog_msg.message_header,jog_msg.data_size+4);
return;
}
*/
/*
// -------------------------------------------------------------------------
void set_servo_power(char value)
{
struct command_packet
	{
	unsigned char message_header;
	unsigned char data_size;
	unsigned char message_id;
	unsigned char state;
	unsigned char crc;
	} command_msg = { 0x5A, 1, 0x20 };

if(value==ON)command_msg.state=ON;
	else command_msg.state=OFF;
command_msg.crc=calculate_crc(&command_msg.data_size,command_msg.data_size+2);
//send request for data
write_com1(&command_msg.message_header,command_msg.data_size+4);
return;
}
// -------------------------------------------------------------------------
void set_servo_state(char value)
{
struct command_packet
	{
	unsigned char message_header;
	unsigned char data_size;
	unsigned char message_id;
	unsigned char state;
	unsigned char crc;
	} command_msg = { 0x5A, 1, 0x21 };


command_msg.state=value&0x1F;// Mask for only valid states
command_msg.crc=calculate_crc(&command_msg.data_size,command_msg.data_size+2);
write_com1(&command_msg.message_header,command_msg.data_size+4);
return;
}
*/

/**
 * @arg value must be between -100 and 100
 */
AREXPORT void ArTerabotArm::grip(short int value)
{
	if (value>100) value=100;//Limit max value
	if (value<-100) value=-100;//Limit min value
	IntCommand(0xC1, value).write(myCon);

	/*
	struct command_packet
	{
		unsigned char message_header;
		unsigned char data_size;
		unsigned char message_id;
		short int grip_value;
		unsigned char crc;
	} command_msg = { 0x5A, 2, 0xC1 };
	command_msg.grip_value=value;
	command_msg.crc=calculate_crc(&command_msg.data_size,command_msg.data_size+2);
	//write_com1(&command_msg.message_header,command_msg.data_size+4);
	myCon->write( (char*)(&command_msg.message_header), command_msg.data_size+4);
	*/

return;
}


AREXPORT const char *ArTerabotArm::error_str(unsigned char code)
{
  switch(code)
  {
    case 1:
      return "Bad Message Header Value";
    case 2:
      return "Unknown Command ID";
    case 3:
      return "Incomplete Message";
    case 4:
      return "Bad CRC";
    default:
      return "(unknown error)";
  }
}

unsigned char ArTerabotArm::calculate_crc(unsigned char *data, int size)
{
	int i;
	unsigned char checksum=0;
	unsigned char *p;
	
	p=data;
//        DEBUG(printf("<<crc: %d bytes: ", size);)
	for(i=0;i<size;++i){
//           DEBUG(printf("0x%hhx, ", *p);)
	   checksum=checksum^(*p); // XOR
	   ++p;
	}
//        DEBUG(printf("= 0x%hhx>> ", checksum);)
	return checksum;
}

AREXPORT bool ArTerabotArm::moving()
{
	for(int i = 0; i < 5; ++i)
	{
		if(myJointStatus[i] & IN_PROGRESS) return true;
	}
}

