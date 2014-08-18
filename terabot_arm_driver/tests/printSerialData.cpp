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
//#include "ArTerabotArm.h"

int main(int argc, char **argv)
{
  Aria::init();
  ArLog::init(ArLog::StdErr, ArLog::Normal);
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }
  
  ArSerialConnection con;
  con.setPort(ArUtil::COM4);
  con.setBaud(19200);
  if(!con.openSimple())
    ArLog::log(ArLog::Terse, "could not open COM4");

  char buf[512];
  
  while (true)
  {
    int n = con.read(buf, 512, 10);
    if(n < 0)
    {
      ArLog::log(ArLog::Terse, "Error reading.");
      Aria::exit(n);
    }
    if(n == 0)
      continue;

    // log for debugging:
    char cmd = 0;
    int x = 0;
    int size = 0;
    for(int i = 0; i < n; ++i)
    {
      if(buf[i] == 0xc1 || buf[i] == 0x5a) puts("");
      printf("0x%hhX (%u)    ", buf[i], buf[i]);
  }
  
   }   
    

  Aria::exit(0);
}
