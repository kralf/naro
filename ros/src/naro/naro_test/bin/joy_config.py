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

import sys, math, roslib, rospy, re
from naro_cmd_srvs.srv import *
from naro_cmd_srvs.msg import *

class JoyConfig(object):
  def __init__(self, name = "joy_config"):
    object.__init__(self)

    rospy.init_node(name)

    self.cmdServerName = "/joy_command"

    self.fins = {}
    self.actuators = ["pitch", "flap"]
    self.outputChannels = ["frequency", "amplitude", "offset", "phase"]
    self.transferFunctions = {
      "identity": Coefficient.IDENTITY,
      "ramp": Coefficient.RAMP,
      "slope": Coefficient.SLOPE,
      "step": Coefficient.STEP,
      "absolute": Coefficient.ABSOLUTE,
      "square": Coefficient.SQUARE,
      "exponential": Coefficient.EXPONENTIAL
    }

    self.getParameters()
    
  def getParameters(self):
    self.cmdServerName = rospy.get_param("servers/cmd/name",
      self.cmdServerName)

  def load(self):    
    parameters = rospy.get_param_names()
    
    self.fins = {}
    finsNamespace = rospy.get_name()+"/fins"
    
    for parameter in parameters:
      match = re.match(finsNamespace+"/([^/]*)/.*", parameter)
      if match and not match.group(1) in self.fins:
        self.fins[match.group(1)] = Fin()
    
    for fin in self.fins:
      finNamespace = finsNamespace+"/"+fin
      
      for actuator in self.actuators:
        actuatorNamespace = finNamespace+"/"+actuator
        actuator = getattr(self.fins[fin], actuator)
        
        setattr(actuator, "servo",
          rospy.get_param(actuatorNamespace+"/servo", -1))
          
        for outputChannel in self.outputChannels:
          outputChannelNamespace = actuatorNamespace+"/"+outputChannel
          outputChannel = getattr(actuator, outputChannel)
          
          constant = rospy.get_param(outputChannelNamespace+"/constant", 0.0)
          setattr(outputChannel, "constant", constant*math.pi/180.0)

          coefficients = {}
          coefficientsNamespace = outputChannelNamespace+"/coefficients"
          
          for parameter in parameters:
            match = re.match(coefficientsNamespace+"/([^/]*)/.*", parameter)
            if match and not match.group(1) in coefficients:
              coefficients[match.group(1)] = Coefficient()

          for coefficient in coefficients:
            coefficientNamespace = coefficientsNamespace+"/"+coefficient
            
            setattr(coefficients[coefficient], "input_channel",
              rospy.get_param(coefficientNamespace+"/input_channel", 0))
            transferFunction = rospy.get_param(
              coefficientNamespace+"/transfer_function", "identity")
            setattr(coefficients[coefficient], "transfer_function",
              self.transferFunctions[transferFunction]);
            setattr(coefficients[coefficient], "invert_arguments",
              rospy.get_param(coefficientNamespace+"/invert_arguments", False))
            setattr(coefficients[coefficient], "invert_values",
              rospy.get_param(coefficientNamespace+"/invert_values", False))
           
          setattr(outputChannel, "coefficients", coefficients.values())
    
  def set(self):
    rospy.wait_for_service(self.cmdServerName+"/set_fins")
    try:
      request = rospy.ServiceProxy(self.cmdServerName+"/set_fins", SetFins)
      request(self.fins.values())
    except rospy.ServiceException, exception:
      print "SetFins request failed: %s" % exception
          
if __name__ == "__main__":
  config = JoyConfig()

  config.load()
  config.set()
