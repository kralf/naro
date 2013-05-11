#! /usr/bin/python

###########################################################################
#   Copyright (C) 2013 by Ralf Kaestner                                   #
#   ralf.kaestner@gmail.com                                               #
#                                                                         #
#   This program is free software; you can redistribute it and/or modify  #
#   it under the terms of the GNU General Public License as published by  #
#   the Free Software Foundation; either version 2 of the License, or     #
#   (at your option) any later version.                                   #
#                                                                         #
#   This program is distributed in the hope that it will be useful,       #
#   but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#   GNU General Public License for more details.                          #
#                                                                         #
#   You should have received a copy of the GNU General Public License     #
#   along with this program; if not, write to the                         #
#   Free Software Foundation, Inc.,                                       #
#   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             #
###########################################################################

import roslib;
roslib.load_manifest("naro_usc_srvs")

import sys, rospy, math
from naro_usc_srvs.srv import *

def setPosition(channel, angle):
  rospy.wait_for_service("usc_server/set_positions")
  try:
    request = rospy.ServiceProxy("usc_server/set_positions", SetPositions)
    request([channel], [angle])
  except rospy.ServiceException, exception:
    print "SetPosition request failed: %s" % exception

def usage():
  return "%s CHANNEL ANGLE" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    channel = int(sys.argv[1])
    angle = float(sys.argv[2])*math.pi/180.0
  else:
    print usage()
    sys.exit(1)

  setPosition(channel, angle)
  