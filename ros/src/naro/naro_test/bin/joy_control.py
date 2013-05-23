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

import sys, math, rospy
from sensor_msgs.msg import *
from naro_smc_srvs.srv import *
from naro_usc_srvs.srv import *

class JoyControl(object):
  def __init__(self, name = "joy_control"):
    object.__init__(self)

    rospy.init_node(name)

    self.smcServerName = "/smc_server"
    self.uscServerName = "/usc_server"

    self.getParameters()

    rospy.Subscriber("/joy", Joy, self.receiveJoy, queue_size = 1)

    angle = 30.0*math.pi/180.0
    self.setPosition([0, 2, 4, 6], [1.5*angle, -angle, -angle, 1.5*angle])
    self.start()

  def getParameters(self):
    self.smcServerName = rospy.get_param("servers/smc/name",
      self.smcServerName)
    self.uscServerName = rospy.get_param("servers/usc/name",
      self.uscServerName)

  def getLimits(self):
    rospy.wait_for_service(self.smcServerName+"/get_limits")
    try:
      request = rospy.ServiceProxy(self.smcServerName+"/get_limits", GetLimits)
      return request().limits
    except rospy.ServiceException, exception:
      print "GetLimits request failed: %s" % exception
      return 0

  def start(self):
    rospy.wait_for_service(self.smcServerName+"/start")
    try:
      request = rospy.ServiceProxy(self.smcServerName+"/start", Start)
      request()
    except rospy.ServiceException, exception:
      print "Start request failed: %s" % exception

  def setSpeed(self, speed):
    rospy.wait_for_service(self.smcServerName+"/set_speed")
    try:
      request = rospy.ServiceProxy(self.smcServerName+"/set_speed", SetSpeed)
      request(speed)
    except rospy.ServiceException, exception:
      print "SetSpeed request failed: %s" % exception

  def setPosition(self, channels, angles):
    rospy.wait_for_service(self.uscServerName+"/set_positions")
    try:
      request = rospy.ServiceProxy(self.uscServerName+"/set_positions",
        SetPositions)
      request(channels, angles)
    except rospy.ServiceException, exception:
      print "SetPosition request failed: %s" % exception

  def receiveJoy(self, joy):
    limits = self.getLimits()
    speed = joy.axes[4];
    
    if limits & GetLimitsResponse.ANALOG1:
      if speed < 0: 
        self.start()
        self.setSpeed(joy.axes[4])
    elif limits & GetLimitsResponse.ANALOG2:
      if speed > 0:
        self.start()
        self.setSpeed(joy.axes[4])
    else:
      self.setSpeed(joy.axes[4])

    angle = joy.axes[1]*45.0*math.pi/180.0
    self.setPosition([1, 3, 5, 7], [-angle, angle, angle, -angle])

if __name__ == "__main__":
  control = JoyControl()
  
  rospy.spin()
  