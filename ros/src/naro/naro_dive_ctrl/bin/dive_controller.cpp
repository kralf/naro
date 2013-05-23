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

#include <fstream>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_smc_srvs/GetLimits.h>
#include <naro_smc_srvs/SetSpeed.h>
#include <naro_smc_srvs/Start.h>
#include <naro_usc_srvs/GetInputs.h>

#include "naro_dive_ctrl/SetDepth.h"
#include "naro_dive_ctrl/Emerge.h"

using namespace naro_dive_ctrl;
using namespace naro_smc_srvs;
using namespace naro_usc_srvs;

std::string smcServerName = "smc_server";
std::string uscServerName = "usc_server";
double controllerFrequency = 5.0;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getLimitsClient;
ros::ServiceClient startClient;
ros::ServiceClient setSpeedClient;
ros::ServiceClient getInputsClient;

ros::ServiceServer setDepthService;
ros::ServiceServer emergeService;

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/smc/name", smcServerName, smcServerName);
  node.param<std::string>("server/usc/name", uscServerName, uscServerName);
  node.param<double>("controller/frequency", controllerFrequency,
    controllerFrequency);
}

bool setDepth(SetDepth::Request& request, SetDepth::Response& response) {
}

bool emerge(Emerge::Request& request, Emerge::Response& response) {
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void updateControl(const ros::TimerEvent& event) {
  diagnoseFrequency->tick();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dive_controller");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
    diagnostic_updater::FrequencyStatusParam(&controllerFrequency,
    &controllerFrequency)));
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getLimitsClient = node.serviceClient<GetLimits>(
    "/"+smcServerName+"/get_limits");
  startClient = node.serviceClient<Start>(
    "/"+smcServerName+"/start");
  setSpeedClient = node.serviceClient<SetSpeed>(
    "/"+smcServerName+"/set_speed");
  getInputsClient = node.serviceClient<GetInputs>(
    "/"+uscServerName+"/get_inputs");

  setDepthService = node.advertiseService("setDepth", setDepth);
  emergeService = node.advertiseService("emerge", emerge);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer controllerTimer = node.createTimer(
    ros::Duration(1.0/controllerFrequency), updateControl);

  ros::spin();

  return 0;
}
