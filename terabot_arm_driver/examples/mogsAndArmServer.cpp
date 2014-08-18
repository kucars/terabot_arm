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
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningInterface.h"
#include "ArGPSLocalizationTask.h"
//#include "ArGPSMapTools.h"
#include "ArSystemStatus.h"
#include "ArTerabotArm.h"

void logOptions(const char *progname)
{
  ArLog::log(ArLog::Normal, "Usage: %s [options]\n", progname);
  ArLog::log(ArLog::Normal, "[options] are any program options listed below, or any ARNL configuration");
  ArLog::log(ArLog::Normal, "parameters as -name <value>, see params/arnl.p for list.");
  ArLog::log(ArLog::Normal, "For example, -map <map file>.");
  Aria::logOptions();
}

bool gyroErrored = false;
const char* getGyroStatusString(ArRobot* robot)
{
  if(!robot || !robot->getOrigRobotConfig() || robot->getOrigRobotConfig()->getGyroType() < 2) return "N/A";
  if(robot->getFaultFlags() & ArUtil::BIT4)
  {
    gyroErrored = true;
    return "ERROR/OFF";
  }
  if(gyroErrored)
  {
    return "OK but error before";
  }
  return "OK";
}

void goalDone(ArPose goalPos)
{
  ArLog::log(ArLog::Normal, "ARNL server example: goal reached");
}

void goalFailed(ArPose goalPos)
{
  ArLog::log(ArLog::Normal, "ARNL server example: goal failed");
}

void locFailed(int n)
{
  ArLog::log(ArLog::Normal, "ARNL server example: localization failed");
}
 
ArPathPlanningInterface *pathPlanningTask = NULL;
void pathPlanStateChanged()
{
  char s[256];
  pathPlanningTask->getFailureString(s, 256);
  ArLog::log(ArLog::Normal, "ARNL server example: Path planning state: %s", s);
}



int main(int argc, char **argv)
{
  // Initialize Aria and Arnl global information
  Aria::init();
  Arnl::init();


  // Used to parse the command line arguments.
  ArArgumentParser parser(&argc, argv);
  
  // Load default arguments for this computer (from /etc/Aria.args, environment
  // variables, and other places)
  parser.loadDefaultArguments();

  // Tell the laser connector to always connect the first laser since
  // this program always requires a laser.
  parser.addDefaultArgument("-connectLaser");

  


  // The robot object
  ArRobot robot;

  // This object is used to connect to the robot, which can be configured via
  // command line arguments.
  ArRobotConnector robotConnector(&parser, &robot);

  // Connect to the robot
  if (!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to robot... exiting");
    Aria::exit(3);
  }



  // Set up where we'll look for files. Arnl::init() set Aria's default
  // directory to Arnl's default directory; addDirectories() appends this
  // "examples" directory.
const char *fileDir = "/";
//  char fileDir[1024];
//  ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(), 
//			 "examples");
  
  
  // To direct log messages to a file, or to change the log level, use these  calls:
  //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
  //ArLog::init(ArLog::File, ArLog::Verbose);
 
  // Add a section to the configuration to change ArLog parameters
  ArLog::addToConfig(Aria::getConfig());

  // set up a gyro (if the robot is older and its firmware does not
  // automatically incorporate gyro corrections, then this object will do it)
  ArAnalogGyro gyro(&robot);

  // Our networking server
  ArServerBase server;
  
  // GPS connector.
  ArGPSConnector gpsConnector(&parser);

  // Set up our simpleOpener, used to set up the networking server
  ArServerSimpleOpener simpleOpener(&parser);

  // the laser connector
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  // Take some possible camera PTZ arguments out of the arguments
  const char *cameratype = parser.checkParameterArgument("ptzType");
  const char *cameraport = parser.checkParameterArgument("ptzPort");

  // Parse arguments 
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(1);
  }
  

  // This causes Aria::exit(9) to be called if the robot unexpectedly
  // disconnects
  ArGlobalFunctor1<int> shutdownFunctor(&Aria::exit, 9);
  robot.addDisconnectOnErrorCB(&shutdownFunctor);


  // Create an ArSonarDevice object (ArRangeDevice subclass) and 
  // connect it to the robot.
  ArSonarDevice sonarDev;
  robot.addRangeDevice(&sonarDev);



  // This object will allow robot's movement parameters to be changed through
  // a Robot Configuration section in the ArConfig global configuration facility.
  ArRobotConfig robotConfig(&robot);

  // Include gyro configuration options in the robot configuration section.
  robotConfig.addAnalogGyro(&gyro);

  // Start the robot thread.
  robot.runAsync(true);

  // On the Seekur, power to the GPS receiver is switched on by this command.
  // (A third argument of 0 would turn it off). On other robots this command is
  // ignored. If this fails, you may need to reset the port with ARIA demo or 
  // seekurPower program (turn port off then on again).  If the port is already
  // on, it will have no effect on the GPS (it will remain powered.)
  // Do this now before connecting to lasers to give it plenty of time to power
  // on, initialize, and find a good position before GPS localization begins.
  ArLog::log(ArLog::Normal, "Turning on GPS power... (Seekur/Seekur Jr. power port 6)");
  robot.com2Bytes(116, 6, 1);
  

  // connect the laser(s) if it was requested, this adds them to the
  // robot too, and starts them running in their own threads
  ArLog::log(ArLog::Normal, "Connecting to laser(s) configured in parameters...");
  if (!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Error: Could not connect to laser(s). Exiting.");
    Aria::exit(2);
  }
  ArLog::log(ArLog::Normal, "Done connecting to laser(s).");

  // find the laser we should use for localization and/or mapping,
  // which will be the first laser
  robot.lock();
  ArLaser *firstLaser = robot.findLaser(1);
  if (firstLaser == NULL || !firstLaser->isConnected())
  {
    ArLog::log(ArLog::Normal, "Did not have laser 1 or it is not connected, cannot do localization or mapping without a laser... exiting");
    Aria::exit(2);
  }
  robot.unlock();


    /* Create and set up map object */
  
  // Set up the map object, this will look for files in the examples
  // directory (unless the file name starts with a /, \, or .
  // You can take out the 'fileDir' argument to look in the program's current directory
  // instead.
  // When a configuration file is loaded into ArConfig later, if it specifies a
  // map file, then that file will be loaded as the map.
  ArMap map(fileDir);
  // set it up to ignore empty file names (otherwise if a configuration omits
  // the map file, the whole configuration change will fail)
  map.setIgnoreEmptyFileName(true);
  // ignore the case, so that if someone is using MobileEyes or
  // MobilePlanner from Windows and changes the case on a map name,
  // it will still work.
  map.setIgnoreCase(true);

    
    /* Create localization and path planning threads */


  ArPathPlanningTask pathTask(&robot, &sonarDev, &map);
  pathPlanningTask = &pathTask;
  ArGlobalFunctor1<ArPose> goalDoneCB(&goalDone);
  ArGlobalFunctor1<ArPose> goalFailedCB(&goalFailed);
  ArGlobalFunctor stateCB(&pathPlanStateChanged);
  pathTask.addGoalDoneCB(&goalDoneCB);
  pathTask.addGoalFailedCB(&goalFailedCB);
  pathTask.addStateChangeCB(&stateCB);

  ArGlobalFunctor1<int> locFailedCB(&locFailed);




  ArLog::log(ArLog::Normal, "Connecting to GPS...");

  // Connect to GPS
  ArGPS *gps = gpsConnector.createGPS(&robot);
  if(!gps || !gps->connect())
  {
    ArLog::log(ArLog::Terse, "Error connecting to GPS device."
      "Try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments."
      "Use -help for help. Exiting.");
    Aria::exit(5);
  }

  // set up GPS localization task
  ArLog::log(ArLog::Normal, "Creating GPS localization task");
  ArGPSLocalizationTask gpsLocTask(&robot, gps, &map);
  //gpsLocTask.setFailedCallBack(&locFailedCB);

  // Set some options  and callbacks on each laser that the laser connector 
  // connected to.
  std::map<int, ArLaser *>::iterator laserIt;
  for (laserIt = robot.getLaserMap()->begin();
       laserIt != robot.getLaserMap()->end();
       laserIt++)
  {
    int laserNum = (*laserIt).first;
    ArLaser *laser = (*laserIt).second;

    // Skip lasers that aren't connected
    if(!laser->isConnected())
      continue;

    // add the disconnectOnError CB to shut things down if the laser
    // connection is lost
    laser->addDisconnectOnErrorCB(&shutdownFunctor);
    // set the number of cumulative readings the laser will take
    laser->setCumulativeBufferSize(200);
    // add the lasers to the path planning task
    pathTask.addRangeDevice(laser, ArPathPlanningTask::BOTH);
    // set the cumulative clean offset (so that they don't all fire at once)
    laser->setCumulativeCleanOffset(laserNum * 100);
    // reset the cumulative clean time (to make the new offset take effect)
    laser->resetLastCumulativeCleanTime();

    // Add the packet count to the Aria info strings (It will be included in
    // MobileEyes custom details so you can monitor whether the laser data is
    // being received correctly)
    std::string laserPacketCountName;
    laserPacketCountName = laser->getName();
    laserPacketCountName += " Packet Count";
    Aria::getInfoGroup()->addStringInt(
	    laserPacketCountName.c_str(), 10, 
	    new ArRetFunctorC<int, ArLaser>(laser, 
					 &ArLaser::getReadingCount));
  }




    /* Start the server */

  // Open the networking server
  if (!simpleOpener.open(&server, fileDir, 240))
  {
    ArLog::log(ArLog::Normal, "Error: Could not open server.");
    exit(2);
  }



    /* Create various services that provide network access to clients (such as
     * MobileEyes), as well as add various additional features to ARNL */





  /* Add additional range devices to the robot and path planning task (so it
     avoids obstacles detected by these devices) */
  
  // Add IR range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  robot.lock();
  ArIRs irs;
  robot.addRangeDevice(&irs);
  pathTask.addRangeDevice(&irs, ArPathPlanningTask::CURRENT);

  // Add bumpers range device to robot and path planning task (so it avoids obstacles
  // detected by this device)
  ArBumpers bumpers;
  robot.addRangeDevice(&bumpers);
  pathTask.addRangeDevice(&bumpers, ArPathPlanningTask::CURRENT);

  // Add range device which uses forbidden regions given in the map to give virtual
  // range device readings to ARNL.  (so it avoids obstacles
  // detected by this device)
  ArForbiddenRangeDevice forbidden(&map);
  robot.addRangeDevice(&forbidden);
  pathTask.addRangeDevice(&forbidden, ArPathPlanningTask::CURRENT);

  robot.unlock();


  // Action to slow down robot when localization score drops but not lost.
  ArActionSlowDownWhenNotCertain actionSlowDown(&gpsLocTask);
  pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

  // Action to stop the robot when localization is "lost" (score too low)
  ArActionLost actionLostPath(&gpsLocTask, &pathTask);
  pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

  // Arnl uses this object when it must replan its path because its
  // path is completely blocked.  It will use an older history of sensor
  // readings to replan this new path.  This should not be used with SONARNL
  // since sonar readings are not accurate enough and may prevent the robot
  // from planning through space that is actually clear.
  ArGlobalReplanningRangeDevice replanDev(&pathTask);

  
  // Service to provide drawings of data in the map display :
  ArServerInfoDrawings drawings(&server);
  drawings.addRobotsRangeDevices(&robot);
  drawings.addRangeDevice(&replanDev);

  /* Draw a box around the local path planning area use this 
    (You can enable this particular drawing from custom commands 
    which is set up down below in ArServerInfoPath) */
  ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
  ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorP(&pathTask, &ArPathPlanningTask::drawSearchRectangle);
  drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP); 

  /* Draw a box showing the safety clearance used by the path planning task to */
  drawings.addDrawing(
    new ArDrawingData("polySegments", ArColor(166, 166, 166), 1, 60, 100, "DefaultOff"),
    "Path Planning Clearances",
    new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(&pathTask, &ArPathPlanningTask::drawRobotBounds)
  );


  /* Show the positions calculated by GPS localization */

  ArDrawingData drawingDataG("polyDots", ArColor(100,100,255), 130, 61);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG(&gpsLocTask, &ArGPSLocalizationTask::drawGPSPoints);
  drawings.addDrawing(&drawingDataG, "GPS Points", &drawingFunctorG);

  ArDrawingData drawingDataG2("polyDots", ArColor(255,100,100), 100, 62);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG2(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanPoints);
  drawings.addDrawing(&drawingDataG2, "Kalman Points", &drawingFunctorG2);

  ArDrawingData drawingDataG3("polyDots", ArColor(100,255,100), 70, 63);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG3(&gpsLocTask, &ArGPSLocalizationTask::drawOdoPoints);
  drawings.addDrawing(&drawingDataG3, "Odom. Points", &drawingFunctorG3);

  ArDrawingData drawingDataG4("polyDots", ArColor(255,50,50), 100, 75);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG4(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanRangePoints);
  drawings.addDrawing(&drawingDataG4, "KalRange Points", &drawingFunctorG4);

  ArDrawingData drawingDataG5("polySegments", ArColor(100,0,255), 1, 78);
  ArFunctor2C<ArGPSLocalizationTask, ArServerClient *, ArNetPacket *> 
    drawingFunctorG5(&gpsLocTask, &ArGPSLocalizationTask::drawKalmanVariance);
  drawings.addDrawing(&drawingDataG5, "VarGPS", &drawingFunctorG5);

  // "Custom" commands. You can add your own custom commands here, they will
  // be available in MobileEyes' custom commands (enable in the toolbar or
  // access through Robot Tools)
  ArServerHandlerCommands commands(&server);


  // These provide various kinds of information to the client:
  ArServerInfoRobot serverInfoRobot(&server, &robot);
  ArServerInfoSensor serverInfoSensor(&server, &robot);
  ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
  serverInfoPath.addSearchRectangleDrawing(&drawings);
  serverInfoPath.addControlCommands(&commands);

  // Provides localization info and allows the client (MobileEyes) to relocalize at a given
  // pose:
  ArServerInfoLocalization serverInfoLocalization(&server, &robot, &gpsLocTask);
  ArServerHandlerLocalization serverLocHandler(&server, &robot, &gpsLocTask);

  // If you're using MobileSim, ArServerHandlerLocalization sends it a command
  // to move the robot's true pose if you manually do a localization through 
  // MobileEyes.  To disable that behavior, use this constructor call instead:
  // ArServerHandlerLocalization serverLocHandler(&server, &robot, true, false);
  // The fifth argument determines whether to send the command to MobileSim.

  // Provide the map to the client (and related controls):
  ArServerHandlerMap serverMap(&server, &map);

  // These objects add some simple (custom) commands to 'commands' for testing and debugging:
  ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
  ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
  ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
//  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


  // service that allows the client to monitor the communication link status
  // between the robot and the client.
  //
  ArServerHandlerCommMonitor handlerCommMonitor(&server);



  // service that allows client to change configuration parameters in ArConfig 
  ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
				      Arnl::getTypicalDefaultParamFileName(),
				      Aria::getDirectory());



  /* Set up the possible modes for remote control from a client such as
   * MobileEyes:
   */

  // Mode To go to a goal or other specific point:
    ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map,
			    gpsLocTask.getRobotHome(),
			    gpsLocTask.getRobotHomeCallback());

  // Add a special command to Custom Commands that tours a list of goals rather
  // than all:
  modeGoto.addTourGoalsInListSimpleCommand(&commands);

  // Mode To stop and remain stopped:
  ArServerModeStop modeStop(&server, &robot);

  // Cause the sonar to turn off automatically
  // when the robot is stopped, and turn it back on when commands to move
  // are sent. (Note, if using SONARNL to localize, then don't do this
  // since localization may get lost)
  ArSonarAutoDisabler sonarAutoDisabler(&robot);

  // Teleoperation modes To drive by keyboard, joystick, etc:
  ArServerModeRatioDrive modeRatioDrive(&server, &robot);  
//  ArServerModeDrive modeDrive(&server, &robot);            // Older mode for compatability



  // Prevent normal teleoperation driving if localization is lost using
  // a high-priority action, which enables itself when the particular mode is
  // active.
  // (You have to enter unsafe drive mode to drive when lost.)
  ArActionLost actionLostRatioDrive(&gpsLocTask, &pathTask, &modeRatioDrive);
  modeRatioDrive.getActionGroup()->addAction(&actionLostRatioDrive, 110);

  // Add drive mode section to the configuration, and also some custom (simple) commands:
  modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
  modeRatioDrive.addControlCommands(&commands);

  // Wander mode (also prevent wandering if lost):
  ArServerModeWander modeWander(&server, &robot);
  ArActionLost actionLostWander(&gpsLocTask, &pathTask, &modeWander);
  modeWander.getActionGroup()->addAction(&actionLostWander, 110);


  // This provides a small table of interesting information for the client
  // to display to the operator. You can add your own callbacks to show any
  // data you want.
  ArServerInfoStrings stringInfo(&server);
  Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());
  
  // The following statements add fields to a set of informational data called
  // the InfoGroup. These are served to MobileEyes for displayi (turn on by enabling Details
  // and Custom Details in the View menu of MobileEyes.)

  Aria::getInfoGroup()->addStringInt(
	  "Motor Packet Count", 10, 
	  new ArConstRetFunctorC<int, ArRobot>(&robot, 
					       &ArRobot::getMotorPacCount));


  const char *dopfmt = "%2.4f";
  const char *posfmt = "%2.8f";
  const char *altfmt = "%3.6f m";
  const char *errfmt = "%2.4f m";
  Aria::getInfoGroup()->addStringString(
	    "GPS Fix Mode", 25,
	    new ArConstRetFunctorC<const char*, ArGPS>(gps, &ArGPS::getFixTypeName)
    );
  Aria::getInfoGroup()->addStringInt(
	    "GPS Num. Satellites", 4,
	    new ArConstRetFunctorC<int, ArGPS>(gps, &ArGPS::getNumSatellitesTracked)
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS HDOP", 12,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getHDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS VDOP", 5,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getVDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS PDOP", 5,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getPDOP),
      dopfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Latitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Longitude", 15,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitude),
      posfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "Altitude", 8,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitude),
      altfmt
    );

  // only some GPS receivers provide these, but you can uncomment them
  // here to enable them if yours does.
  /*
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lat. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLatitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Lon. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getLongitudeError),
      errfmt
    );
  Aria::getInfoGroup()->addStringDouble(
	    "GPS Alt. Err.", 6,
	    new ArConstRetFunctorC<double, ArGPS>(gps, &ArGPS::getAltitudeError),
      errfmt
    );
  */

  Aria::getInfoGroup()->addStringDouble(
    "MOGS Localization Score", 8,
    new ArRetFunctorC<double, ArGPSLocalizationTask>(
      &gpsLocTask, &ArGPSLocalizationTask::getLocalizationScore),
    "%.03f"
  );


  // Display gyro status if gyro is enabled and is being handled by the firmware (gyro types 2, 3, or 4).
  // (If the firmware detects an error communicating with the gyro or IMU it
  // returns a flag, and stops using it.)
  // (This gyro type parameter, and fault flag, are only in ARCOS, not Seekur firmware)
  if(robot.getOrigRobotConfig() && robot.getOrigRobotConfig()->getGyroType() > 1)
  {
    Aria::getInfoGroup()->addStringString(
          "Gyro/IMU Status", 10,
          new ArGlobalRetFunctor1<const char*, ArRobot*>(&getGyroStatusString, &robot)
      );
  }

  // Display system CPU and wireless network status
  ArSystemStatus::startPeriodicUpdate(1000); // update every 1 second
  Aria::getInfoGroup()->addStringDouble("CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
  Aria::getInfoGroup()->addStringInt("Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Link Noise", 9, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt("Wireless Signal", 9, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
  




  // Add some "custom commands" for setting up initial GPS offset and heading.
  gpsLocTask.addLocalizationInitCommands(&commands);
  
  // Add some commands for manually creating map objects based on GPS positions:
//  ArGPSMapTools gpsMapTools(gps, &robot, &commands, &map);

  // Add command to set simulated GPS position manually
  if(gpsConnector.getGPSType() == ArGPSConnector::Simulator)
  {
    ArSimulatedGPS *simGPS = dynamic_cast<ArSimulatedGPS*>(gps);
//    simGPS->setDummyPosition(42.80709, -71.579047, 100);
    commands.addStringCommand("GPS:setDummyPosition", 
      "Manually set a new dummy position for simulated GPS. Provide latitude (required), longitude (required) and altitude (optional)", 
      new ArFunctor1C<ArSimulatedGPS, ArArgumentBuilder*>(simGPS, &ArSimulatedGPS::setDummyPositionFromArgs)
    );
  }


  // Make Stop mode the default (If current mode deactivates without entering
  // a new mode, then Stop Mode will be selected)
  modeStop.addAsDefaultMode();
    // TODO move up near where stop mode is created?





  /* Services that allow the client to initiate scanning with the laser to
     create maps in Mapper3 (So not possible with SONARNL): */

  ArServerHandlerMapping handlerMapping(&server, &robot, firstLaser, 
					fileDir, "", true);


  // Save GPS positions in the .2d scan log when making a map
  handlerMapping.addLocationData("robotGPS", 
			    gpsLocTask.getPoseInterpPositionCallback());

  // add the starting latitude and longitude info to the .2d scan log
  handlerMapping.addMappingStartCallback(
	  new ArFunctor1C<ArGPSLocalizationTask, ArServerHandlerMapping *>
	  (&gpsLocTask, &ArGPSLocalizationTask::addScanInfo, 
	   &handlerMapping));

  // Make it so our "lost" actions don't stop us while mapping
  handlerMapping.addMappingStartCallback(actionLostPath.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostRatioDrive.getDisableCB());
  handlerMapping.addMappingStartCallback(actionLostWander.getDisableCB());

  // And then let them make us stop as usual when done mapping
  handlerMapping.addMappingEndCallback(actionLostPath.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostRatioDrive.getEnableCB());
  handlerMapping.addMappingEndCallback(actionLostWander.getEnableCB());

  // don't let forbidden lines show up as obstacles while mapping
  // (they'll just interfere with driving while mapping, and localization is off anyway)
  handlerMapping.addMappingStartCallback(forbidden.getDisableCB());

  // let forbidden lines show up as obstacles again as usual after mapping
  handlerMapping.addMappingEndCallback(forbidden.getEnableCB());


  /*
  // If we are on a simulator, move the robot back to its starting position,
  // and reset its odometry.
  // This will allow localizeRobotAtHomeBlocking() below will (probably) work (it
  // tries current odometry (which will be 0,0,0) and all the map
  // home points.
  // (Ignored by a real robot)
  //robot.com(ArCommands::SIM_RESET);
  */


  // create a pose storage class, this will let the program keep track
  // of where the robot is between runs...  after we try and restore
  // from this file it will start saving the robot's pose into the
  // file
  ArPoseStorage poseStorage(&robot);
  /// if we could restore the pose from then set the sim there (this
  /// won't do anything to the real robot)... if we couldn't restore
  /// the pose then just reset the position of the robot (which again
  /// won't do anything to the real robot)
  if (poseStorage.restorePose("robotPose"))
    serverLocHandler.setSimPose(robot.getPose());
  //else
 //   robot.com(ArCommands::SIM_RESET);



  /* File transfer services: */
  
#ifdef WIN32
  // Not implemented for Windows yet.
  ArLog::log(ArLog::Normal, "Note, file upload/download services are not implemented for Windows; not enabling them.");
#else
  // This block will allow you to set up where you get and put files
  // to/from, just comment them out if you don't want this to happen
  // /*
  ArServerFileLister fileLister(&server, fileDir);
  ArServerFileToClient fileToClient(&server, fileDir);
  ArServerFileFromClient fileFromClient(&server, fileDir, "/tmp");
  ArServerDeleteFileOnServer deleteFileOnServer(&server, fileDir);
  // */
#endif

    /* Video image streaming, and camera controls (Requires SAVserver or ACTS) */

  // Forward any video if either ACTS or SAV server are running.
  // You can find out more about SAV and ACTS on our website
  // http://robots.activmedia.com. ACTS is for color tracking and is
  // a seperate product. SAV just does software A/V transmitting and is
  // free to all our customers. Just run ACTS or SAV server before you
  // start this program and this class here will forward video from the
  // server to the client.
  ArHybridForwarderVideo videoForwarder(&server, "localhost", 7070);
  
  // Camera PTZ control to provide in addition to the video images. The camera collection collects
  // multiple PTZ cameras into one interface to manage things like their
  // ArNetworking interfaces.
  ArPTZ *camera = NULL;
  ArServerHandlerCamera *handlerCamera = NULL;
  ArCameraCollection *cameraCollection = NULL;

  // if we have video then set up camera PTZ control.
  if (videoForwarder.isForwardingVideo())
  {
    if(cameraport != NULL)
      ArLog::log(ArLog::Normal, "Using serial port %s for %s camera pan/tilt/zoom control (from -ptzPort argument)", cameratype, cameraport);
    if(cameratype == NULL || strcmp(cameratype, "vcc") == 0 || strcmp(cameratype, "vcc4") == 0)
      camera = new ArVCC4(&robot);
    else if(strcmp(cameratype, "rvision") == 0)
    {
      camera = new ArRVisionPTZ(&robot);
      if(cameraport == NULL)
        cameraport = ArUtil::COM3;
    }
    else if(strcmp(cameratype, "sony") == 0)
      camera = new ArSonyPTZ(&robot);
    else
    {
      ArLog::log(ArLog::Terse, "Error: unknown PTZ camera type \"%s\" given with -ptzType argument: expected \"vcc\" (for Canon VC-C4 or VC-C5), \"rvision\" (for RVision SEE) or \"sony\".", cameratype);
      Aria::exit(8);
    }

    if(cameraport)    // if cameraport is NULL, then the camera PTZ class uses its default connection method (which is actually via the robot for VCC and Sony)
    {
      ArSerialConnection *con = new ArSerialConnection();
      con->setPort(cameraport);
      if(int err = con->open(cameraport) != 0)
      {
        switch(err)
        {
          case ArSerialConnection::OPEN_COULD_NOT_OPEN_PORT:
            ArLog::log(ArLog::Terse, "Error opening serial port %s for camera.", cameraport);
            Aria::exit(8);
          case ArSerialConnection::OPEN_COULD_NOT_SET_UP_PORT:
            ArLog::log(ArLog::Terse, "Error setting up serial port %s for camera.", cameraport);
            Aria::exit(8);
          case ArSerialConnection::OPEN_INVALID_BAUD_RATE:
            ArLog::log(ArLog::Terse, "Error: invalid baud rate for serial port %s for camera.", cameraport);
            Aria::exit(8);
          case ArSerialConnection::OPEN_COULD_NOT_SET_BAUD:
            ArLog::log(ArLog::Terse, "Error setting baud rate for serial port %s for camera.", cameraport);
            Aria::exit(8);
        }
      }
      camera->setDeviceConnection(con);
    }
      
    // On a Seekur or Seekur Jr. this resets power to make sure the camera is on
    robot.com2Bytes(116, 12, 0);
    ArUtil::sleep(300);
    robot.com2Bytes(116, 12, 1);
    ArUtil::sleep(300);

    camera->init();

    cameraCollection = new ArCameraCollection();
    cameraCollection->addCamera("Cam1", cameratype, "Camera", cameratype);

    videoForwarder.setCameraName("Cam1");
    videoForwarder.addToCameraCollection(*cameraCollection);

    handlerCamera = new ArServerHandlerCamera("Cam1", 
		                                           &server, 
					                                     &robot,
					                                     camera, 
					                                     cameraCollection);

/*
    // ArServerHandlerCamera has a special feature that continuously moves the
    // camera to look at a position. This is an example of how to use that to 
    // look at the path planning task's next goal via its callbacks. You can 
    // also set the ArServerHandhlerCamera to look at any map position instead.
    pathTask.addNewGoalCB(
	    new ArFunctor1C<ArServerHandlerCamera, ArPose>(
		    handlerCamera, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalSetGoal));
    pathTask.addGoalFinishedCB(
	    new ArFunctorC<ArServerHandlerCamera>(
		    handlerCamera, 
		    &ArServerHandlerCamera::cameraModeLookAtGoalClearGoal));
*/

  } // end if (videoForwarder.isForwardingVideo())

  if (cameraCollection != NULL) {
    new ArServerHandlerCameraCollection(&server, cameraCollection);
  }


  /* Connect to arm */
  ArTerabotArm arm(&robot);
  if(!arm.open())
  {
    ArLog::log(ArLog::Terse, "mogsAndArmServer: Error opening connection to arm");
    //Aria::exit(22);
  }

  arm.powerOn();
  arm.reset();

  float parkPose [5]= {149.5, -94.2, 155.7, -149.9, -90.9}; // collapsed on back of robot
  float readyPose [5]= {0, -40, -90, -40, 0};
//  float straightForward [5]= {0, -90, 0, 0, 0};
  float pickFrontPose [5]= {0, -95, -76.5, -3.5, -0};
  float pickRightPose [5]= {20, -95, -76.5, -3.5, -0};
  float pickLeftPose [5]= {-20, -95, -76.5, -3.5, -0};
//  float right [5]= {45, -54.52, -69.04, -6.93, 0};
//  float left[5] = {-45, -54.52, -69.04, -6.93, 0};

  commands.addCommand("Arm:Park", "Move arm to parked position on robot", new ArFunctor1C<ArTerabotArm, float*>(&arm, &ArTerabotArm::moveArm, parkPose));
  commands.addCommand("Arm:Ready", "Move arm to ready position in front of robot", new ArFunctor1C<ArTerabotArm, float*>(&arm, &ArTerabotArm::moveArm, readyPose));
  commands.addCommand("Arm:PickFront", "Move arm to pick position in front of robot", new ArFunctor1C<ArTerabotArm, float*>(&arm, &ArTerabotArm::moveArm, pickFrontPose));
  commands.addCommand("Arm:PickRight", "Move arm to pick position  right of robot", new ArFunctor1C<ArTerabotArm, float*>(&arm, &ArTerabotArm::moveArm, pickRightPose));
  commands.addCommand("Arm:PickLeft", "Move arm to pick position  left of robot", new ArFunctor1C<ArTerabotArm, float*>(&arm, &ArTerabotArm::moveArm, pickLeftPose));
  commands.addCommand("Arm:GripperOpen", "Open the gripper", new ArFunctorC<ArTerabotArm>(&arm, &ArTerabotArm::openGripper));
  commands.addCommand("Arm:GripperClose", "Close the gripper", new ArFunctorC<ArTerabotArm>(&arm, &ArTerabotArm::closeGripper));
  commands.addCommand("Arm:Reset", "Reset errors", new ArFunctorC<ArTerabotArm>(&arm, &ArTerabotArm::reset));
  commands.addCommand("Arm:Disable", "Disable all joints", new ArFunctor1C<ArTerabotArm, unsigned char>(&arm, &ArTerabotArm::enableArm, 0x0));
  commands.addCommand("Arm:Enable", "Enable all joints", new ArFunctor1C<ArTerabotArm, unsigned char>(&arm, &ArTerabotArm::enableArm, 0xFF));




    /* Load configuration values, map, and begin! */

  
  // When parsing the configuration file, also look at the program's command line options 
  // from the command-line argument parser as well as the configuration file.
  // (So you can use any argument on the command line, namely -map.) 
  Aria::getConfig()->useArgumentParser(&parser);

  // Read in parameter files.
  ArLog::log(ArLog::Normal, "Loading config file %s into ArConfig...", Arnl::getTypicalParamFileName());
  if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
  {
    ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
    Aria::exit(5);
  }

  // Warn about unknown params.
  if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
  {
    logOptions(argv[0]);
    Aria::exit(6);
  }

  // Warn if there is no map
  if (map.getFileName() == NULL || strlen(map.getFileName()) <= 0)
  {
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal, "### No map file is set up, you can make a map with the following procedure");
    ArLog::log(ArLog::Normal, "\n   See docs/GPSMapping.txt for instructions on creating a map for GPS localization");
  }

  // Print a log message notifying user of the directory for map files
  ArLog::log(ArLog::Normal, "");
  ArLog::log(ArLog::Normal, 
	     "Directory for maps and file serving: %s", fileDir);
  
  ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
  ArLog::log(ArLog::Normal, "");

  // Do an initial localization of the robot. ARNL and SONARNL try all the home points
  // in the map, as well as the robot's current odometric position, as possible
  // places the robot is likely to be at startup.   If successful, it will
  // also save the position it found to be the best localized position as the
  // "Home" position, which can be obtained from the localization task (and is
  // used by the "Go to home" network request).
  // MOGS instead just initializes at the current GPS position.
  // (You will stil have to drive the robot so it can determine the robot's
  // heading, however. See GPS Mapping instructions.)
  gpsLocTask.localizeRobotAtHomeBlocking();
  

  // Start the networking server's thread
  server.runAsync();



  // Enable the motors and wait until the robot exits (disconnection, etc.) or this program is
  // canceled.
  robot.enableMotors();
  robot.waitForRunExit();
  Aria::exit(0);
}

