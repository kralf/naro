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

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_fin_ctrl/SetCommands.h>
#include <naro_fin_ctrl/Enable.h>
#include <naro_fin_ctrl/Disable.h>

#include "naro_cmd_srvs/GetConfiguration.h"
#include "naro_cmd_srvs/SetConfiguration.h"

#include "sensor_msgs/Joy.h"

using namespace naro_cmd_srvs;
using namespace naro_fin_ctrl;
using namespace sensor_msgs;

std::string finServerName = "fin_controller";
double connectionRetry = 0.1;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient setCommandsClient;
ros::ServiceClient enableClient;
ros::ServiceClient disableClient;

ros::ServiceServer getConfigurationService;
ros::ServiceServer setConfigurationService;

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/fin/name", finServerName, finServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);
}

bool getConfiguration(GetConfiguration::Request& request,
    GetConfiguration::Response& response) {
  return true;
}

bool setConfiguration(SetConfiguration::Request& request,
    SetConfiguration::Response& response) {
  return true;
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!setCommandsClient || !enableClient || !disableClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void subscribe() {
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (!setCommandsClient)
    setCommandsClient = ros::NodeHandle("~").serviceClient<SetCommands>(
      "/"+finServerName+"/set_commands", true);
  if (!enableClient)
    setCommandsClient = ros::NodeHandle("~").serviceClient<SetCommands>(
      "/"+finServerName+"/set_commands", true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_command");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
//   diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
//     diagnostic_updater::FrequencyStatusParam(&controllerFrequency,
//     &controllerFrequency)));
//   updater->add("Frequency", &*diagnoseFrequency,
//     &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getConfigurationService = node.advertiseService("get_configuration",
    getConfiguration);
  setConfigurationService = node.advertiseService("set_configuration",
    setConfiguration);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);

  subscribe();
  tryConnect();

  ros::spin();

  return 0;
}
