#! /usr/bin/python

###########################################################################
#   Copyright (C) 2014 by Ralf Kaestner                                   #
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
from naro_blinkm_srvs.srv import *
from naro_fin_ctrl.srv import *

class MusicControl(object):
  def __init__(self, name = "music_control", platforms = []):
    object.__init__(self)

    rospy.init_node(name)

    self.smcServerName = "/smc_server"
    self.uscServerName = "/usc_server"
    self.blinkmServerName = "/blinkm_server"
    self.finServerName = "/fin_controller"

    self.getParameters()

    self.platforms = platforms
    self.colors = {
      0: [0.0, 1.0, 0.0],
      1: [1.0, 0.0, 0.0],
      2: [0.0, 0.0, 1.0],
      3: [1.0, 0.3, 0.0]
    }

    self.start()

    self.timer = rospy.Timer(rospy.Rate(10), self.update)

  def getParameters(self):
    self.smcServerName = rospy.get_param("servers/smc/name",
      self.smcServerName)
    self.uscServerName = rospy.get_param("servers/usc/name",
      self.uscServerName)
    self.blinkmServerName = rospy.get_param("servers/blinkm/name",
      self.blinkmServerName)
    self.finServerName = rospy.get_param("servers/fin/name",
      self.finServerName)

  def start(self):
    pass

  def setColor(self, color):
    rospy.wait_for_service(self.blinkmServerName+"/set_color")
    try:
      request = rospy.ServiceProxy(self.blinkmServerName+"/set_color",
        SetColor)
      request(color)
    except rospy.ServiceException, exception:
      print "SetColor request failed: %s" % exception

  def receiveJoy(self, joy):
    try:
      button = joy.buttons.index(1)
    except ValueError:
      button = -1

    if button >= 0 and button < 4:
      if joy.buttons[5]:
        self.setColor(self.colors[button])
      else:
        servos = [2*button, 2*button+1]
        home = self.getHomes(servos)

        home = [home[0]+joy.axes[1]*1.0*math.pi/180.0,
          home[1]+joy.axes[4]*1.0*math.pi/180.0]

        self.setHomes(servos, home)
    else:
      if self.configuration == "fish_advanced":
        self.actuateFishAdvanced(joy)
      elif self.configuration == "turtle_standard":
        self.actuateTurtleStandard(joy)
      else:
        self.actuateFishStandard(joy)

  def update(self, event):
    pass

if __name__ == "__main__":
  if len(sys.argv) > 1:
    control = MusicControl(configuration = sys.argv[1])
  else:
    control = MusicControl()
  
  rospy.spin()
  
