
Adept MobileRobots
Interface Library for Terabot Manipulator Arms

Copyright 2012, 2013 Adept Technology Inc.

The ArTerabotArm library is used to control the Terabot 5-DOF 
manipulator arm, an optional feature of MobileRobots Seekur platforms. 
It is a C++ library, used in conjunction with the core ARIA 
library to communicate with the arm via an RS-232 serial link.

Currently the Terabot-S model is tested and supported.

ArTerabotArm is provided under the terms of the GNU General
Public License. See the LICENSE.txt file for full license
text.

The prebuilt library is installed in the ARIA lib directory
(../lib). On Windows, the DLLs are in the ARIA bin directory
(..\bin).   The header files are in the include subdirectory.

See the docs and examples directories for API documentation and
example programs.

The source code to the library is provided in the src directory.

The ArTerabotArm library provides the ArTerabotArm C++ class.
An ArTerabotArm object, once connected to the arm, can be used
to send joint control commands, set parameters such as velocity
and acceleration, and can be used to read state data from the arm.
Reading state data from the arm can optionally be done 
automatically in conjunction with an ARIA ArRobot object, or it
can be performed under control of your program.

By default, ArTerabotArm uses the COM4 serial port to communicate
with the arm. It will also switch on Seekur/Seekur Jr. power ports
#29 (arm 24V) and #7 (small camera power) when connecting to the arm.

Linux
-----

To compile and link a program using the ArTerabotArm in Linux with
the GNU C++ compile g++, use the following compilation flags:

  -fPIC -I/usr/local/Aria/include -I/usr/local/Aria/ArTerabotArm/include

And use the following linker flags:
  
  -L/usr/local/Aria/lib -lArTerabotArm -lAria -lm -ldl -lrt

Windows
-------

To compile and link a program in Windows with Microsoft Visual
C++ 2010, ...
  XXX TODO

  ******************************************************************
  * Always excersise caution and operate the robot and arm safely  *
  * -- at safe speeds and in a clear area -- at all times.         *
  ******************************************************************


======================================

Adept MobileRobots
support@mobilerobots.com
http://www.activrobots.com
10 Columbia Drive
Amherst, NH 03031

