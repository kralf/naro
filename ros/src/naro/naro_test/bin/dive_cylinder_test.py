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
roslib.load_manifest("naro_smc_srvs")

import sys, rospy
from naro_smc_srvs.srv import *

def start():
  rospy.wait_for_service("/smc_server/start")
  try:
    request = rospy.ServiceProxy("/smc_server/start", Start)
    request()
  except rospy.ServiceException, exception:
    print "Start request failed: %s" % exception

def setSpeed(speed):
  rospy.wait_for_service("/smc_server/set_speed")
  try:
    request = rospy.ServiceProxy("/smc_server/set_speed", SetSpeed)
    request(speed)
  except rospy.ServiceException, exception:
    print "SetSpeed request failed: %s" % exception

def getLimits():
  rospy.wait_for_service("/smc_server/get_limits")
  try:
    request = rospy.ServiceProxy("/smc_server/get_limits", GetLimits)
    return request().limits
  except rospy.ServiceException, exception:
    print "GetLimits request failed: %s" % exception
    return 0

def usage():
  return "%s SPEED NUM_CYCLES" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    speed = float(sys.argv[1])
    numCycles = int(sys.argv[2])
  else:
    print usage()
    sys.exit(1)

  if not numCycles:
    sys.exit(0)

  for i in range(1, numCycles):
    start()
    setSpeed(speed)

    if speed > 0:
      limitMask = GetLimitsResponse.ANALOG1
    else:
      limitMask = GetLimitsResponse.ANALOG2

    while not getLimits() & limitMask:
      rospy.sleep(0.1)

    speed *= -1

  setSpeed(0)
  