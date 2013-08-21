/***************************************************************************
 *   Copyright (C) 2013 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <limits>
#include <map>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include "naro_blinkm_srvs/FadeToColor.h"

#include "naro_led_ctrl/GetEnabled.h"
#include "naro_led_ctrl/GetDefault.h"
#include "naro_led_ctrl/GetCommand.h"
#include "naro_led_ctrl/SetDefault.h"
#include "naro_led_ctrl/SetCommand.h"
#include "naro_led_ctrl/Enable.h"
#include "naro_led_ctrl/Disable.h"

using namespace naro_led_ctrl;
using namespace naro_blinkm_srvs;

std::string blinkmServerName = "blinkm_server";
double connectionRetry = 0.1;
double controllerFrequency = 1.0;
float defaultColor[] = {1.0f, 1.0f, 1.0f};

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient fadeToColorClient;

ros::ServiceServer getEnabledService;
ros::ServiceServer getDefaultService;
ros::ServiceServer getCommandService;
ros::ServiceServer setDefaultService;
ros::ServiceServer setCommandService;
ros::ServiceServer enableService;
ros::ServiceServer disableService;

class Controller {
public:
  class Parameters {
  public:
    Parameters(float red = -1.0f, float green = -1.0f, float blue = -1.0f,
      float period = std::numeric_limits<float>::infinity()) :
      period(period) {
      color[0] = red;
      color[1] = green;
      color[2] = blue;
    };

    float color[3];
    float period;
  };
  
  Controller() :
    enabled(false),
    high(false) {
  };
  
  void reset() {
    high = false;
  };

  bool enabled;
  bool high;
  Parameters command;
};

Controller controller;
ros::Time lastTime;

template <typename S, typename D> void copy(const S& srcColor, D& dstColor) {
  dstColor[0] == srcColor[0];
  dstColor[1] == srcColor[1];
  dstColor[2] == srcColor[2];
}

template <typename S, typename D> bool equals(const S& firstColor, const D&
    secondColor) {
  return
    (firstColor[0] == secondColor[0]) &&
    (firstColor[1] == secondColor[1]) &&
    (firstColor[2] == secondColor[2]);
}

inline bool parameterToColor(const ros::NodeHandle& node, const
    std::string& key, float* color) {
  XmlRpc::XmlRpcValue value;
  node.getParam(key, value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    if (value.size() == 3) {
      color[0] = static_cast<double>(value[0]);
      color[1] = static_cast<double>(value[1]);
      color[2] = static_cast<double>(value[2]);
    }
    else {
      ROS_WARN("Invalid array size for color parameter %s: %d", key.c_str(),
        (unsigned int)value.size());
      return false;
    }
  }
  else {
    ROS_WARN("Invalid type for color parameter %s: expecting array",
      key.c_str(), (unsigned int)value.size());
    return false;
  }
  
  return true;
}

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/blinkm/name", blinkmServerName,
    blinkmServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);

  node.param<double>("controller/frequency", controllerFrequency,
    controllerFrequency);
  
  parameterToColor(node, "default/color", defaultColor);
}

bool getEnabled(GetEnabled::Request& request, GetEnabled::Response& response) {
  response.enabled = controller.enabled;
  return true;
}

bool getDefault(GetDefault::Request& request, GetDefault::Response& response) {
  copy(defaultColor, response.color);
  return true;
}

bool getCommand(GetCommand::Request& request, GetCommand::Response& response) {
  copy(controller.command.color, response.color);
  response.period = controller.command.period;
  
  return true;
}

bool setDefault(SetDefault::Request& request, SetDefault::Response& response) {
  copy(request.color, defaultColor);

  if (!controller.enabled) {
    FadeToColor fadeToColor;
    copy(defaultColor, fadeToColor.request.rgb);
    
    return fadeToColorClient.call(fadeToColor);
  }
  else
    return true;
}

bool setCommand(SetCommand::Request& request, SetCommand::Response& response) {
  copy(request.color, controller.command.color);
  controller.command.period = request.period;
  
  return true;
}

bool enable(Enable::Request& request, Enable::Response& response) {
  controller.enabled = true;
  return true;
}

bool disable(Disable::Request& request, Disable::Response& response) {
  controller.enabled = false;
  controller.reset();
  lastTime.fromSec(0.0);

  FadeToColor fadeToColor;
  copy(defaultColor, fadeToColor.request.rgb);
  fadeToColor.request.speed = 1.0f/controllerFrequency;
  
  return fadeToColorClient.call(fadeToColor);
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!fadeToColorClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (!fadeToColorClient)
    fadeToColorClient = ros::NodeHandle("~").serviceClient<FadeToColor>(
      "/"+blinkmServerName+"/fade_to_color", true);
}

void updateControl(const ros::TimerEvent& event) {
  if (!controller.enabled) {
    diagnoseFrequency->tick();
    return;
  }
  
  float dt = controller.command.period;
  if (!lastTime.isZero())
    dt = (ros::Time::now()-lastTime).toSec();
  
  if ((dt >= 0.5f*controller.command.period) && controller.high) {
    FadeToColor fadeToColor;
    float color[] = {0.0f, 0.0f, 0.0f};
    
    copy(color, fadeToColor.request.rgb);
    fadeToColor.request.speed = 1.0f/controllerFrequency;

    if (fadeToColorClient.call(fadeToColor)) {
      controller.high = false;
      diagnoseFrequency->tick();
    }
  }
  else if ((dt >= controller.command.period) && !controller.high) {
    FadeToColor fadeToColor;
    
    copy(controller.command.color, fadeToColor.request.rgb);
    fadeToColor.request.speed = 1.0f/controllerFrequency;

    if (fadeToColorClient.call(fadeToColor)) {
      controller.high = true;
      lastTime = ros::Time::now();
      
      diagnoseFrequency->tick();
    }
  }
  else
    diagnoseFrequency->tick();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "led_controller");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getEnabledService = node.advertiseService("get_enabled", getEnabled);
  getDefaultService = node.advertiseService("get_default", getDefault);
  getCommandService = node.advertiseService("get_command", getCommand);
  setDefaultService = node.advertiseService("set_default", setDefault);
  setCommandService = node.advertiseService("set_command", setCommand);
  enableService = node.advertiseService("enable", enable);
  disableService = node.advertiseService("disable", disable);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);
  ros::Timer controllerTimer = node.createTimer(
    ros::Duration(1.0/controllerFrequency), updateControl);

  tryConnect();

  ros::spin();

  return 0;
}
