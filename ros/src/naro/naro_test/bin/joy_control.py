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

import sys, math, roslib, rospy
from sensor_msgs.msg import *
from naro_smc_srvs.srv import *
from naro_usc_srvs.srv import *
from naro_fin_ctrl.srv import *

class JoyControl(object):
  def __init__(self, name = "joy_control"):
    object.__init__(self)

    rospy.init_node(name)

    self.smcServerName = "/smc_server"
    self.uscServerName = "/usc_server"
    self.finServerName = "/fin_controller"

    self.getParameters()

    rospy.Subscriber("/joy", Joy, self.receiveJoy, queue_size = 1)
    self.start()

  def getParameters(self):
    self.smcServerName = rospy.get_param("servers/smc/name",
      self.smcServerName)
    self.uscServerName = rospy.get_param("servers/usc/name",
      self.uscServerName)
    self.finServerName = rospy.get_param("servers/fin/name",
      self.finServerName)

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

  def getHomes(self, servos):
    rospy.wait_for_service(self.finServerName+"/get_homes")
    try:
      request = rospy.ServiceProxy(self.finServerName+"/get_homes", GetHomes)
      return request(servos).home
    except rospy.ServiceException, exception:
      print "GetHomes request failed: %s" % exception

  def setHomes(self, servos, home):
    rospy.wait_for_service(self.finServerName+"/set_homes")
    try:
      request = rospy.ServiceProxy(self.finServerName+"/set_homes", SetHomes)
      request(servos, home)
    except rospy.ServiceException, exception:
      print "SetHomes request failed: %s" % exception

  def setCommands(self, servos, frequency, amplitude, phase, offset):
    rospy.wait_for_service(self.finServerName+"/set_commands")
    try:
      request = rospy.ServiceProxy(self.finServerName+"/set_commands",
        SetCommands)
      request(servos, frequency, amplitude, phase, offset)
    except rospy.ServiceException, exception:
      print "SetCommands request failed: %s" % exception

  def receiveJoy(self, joy):
    limits = self.getLimits()

    if (joy.axes[2] < 1.0):
      speed = (1.0-joy.axes[2])*0.5
    elif (joy.axes[5] < 1.0):
      speed = -(1.0-joy.axes[5])*0.5
    else:
      speed = 0.0
    
    if limits & GetLimitsResponse.ANALOG1:
      if speed < 0.0:
        self.start()
        self.setSpeed(speed)
    elif limits & GetLimitsResponse.ANALOG2:
      if speed > 0.0:
        self.start()
        self.setSpeed(speed)
    else:
      self.start()
      self.setSpeed(speed)

    try:
      button = joy.buttons.index(1)
    except ValueError:
      button = -1

    if button >= 0 and button < 4:
      servos = [2*button, 2*button+1]
      home = self.getHomes(servos)

      home = [home[0]+joy.axes[1]*1.0*math.pi/180.0,
        home[1]+joy.axes[4]*1.0*math.pi/180.0]

      self.setHomes(servos, home)
    else:
      servos = range(8)
      frequency = [2.0]*len(servos)
      amplitude = [0.0]*len(servos)
      phase = [0.0]*len(servos)
      offset = [0.0]*len(servos)

      if (joy.axes[3] > 0.0):
        left_gain = 1.0-joy.axes[3]
        right_gain = 1.0
      else:
        left_gain = 1.0
        right_gain = 1.0+joy.axes[3]

      # left and right flap
      amplitude[0] = joy.axes[1]*left_gain*20.0*math.pi/180.0
      amplitude[2] = -joy.axes[1]*right_gain*20.0*math.pi/180.0

      # left and right pitch
      amplitude[1] = joy.axes[1]*left_gain*25.0*math.pi/180.0
      amplitude[3] = -joy.axes[1]*right_gain*25.0*math.pi/180.0

      phase[1] = -90*math.pi/180.0
      phase[3] = -90*math.pi/180.0

      # top pitch
      offset[7] = joy.axes[3]*20.0*math.pi/180.0

      # back flap
      amplitude[4] = joy.axes[1]*20.0*math.pi/180.0
      offset[4] = joy.axes[3]*30.0*math.pi/180.0
      
      self.setCommands(servos, frequency, amplitude, phase, offset)

if __name__ == "__main__":
  control = JoyControl()
  
  rospy.spin()
  