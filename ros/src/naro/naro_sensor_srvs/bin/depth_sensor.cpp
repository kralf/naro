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

#include <deque>
#include <limits>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_usc_srvs/GetChannels.h>
#include <naro_usc_srvs/GetInputs.h>

#include "naro_sensor_srvs/GetPressure.h"
#include "naro_sensor_srvs/GetDepth.h"
#include "naro_sensor_srvs/GetElevation.h"

using namespace naro_sensor_srvs;
using namespace naro_usc_srvs;

std::string uscServerName = "usc_server";
double connectionRetry = 0.1;
double sensorFrequency = 25.0;
int sensorInputChannel = 11;
double sensorInputVoltage = 5.0;
double sensorTransferCoefficient = 4e-6;
double sensorTransferOffset = -0.04;
double sensorStandardAtmosphere = 101325.0;
double sensorMeterSeaWater = 9807.0;
double sensorBarometricConstant = 7990.0;
int filterWindowSize = 50;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getChannelsClient;
ros::ServiceClient getInputsClient;

ros::ServiceServer getPressureService;
ros::ServiceServer getDepthService;
ros::ServiceServer getElevationService;

int input = -1;
std::deque<float> readings;
float sumReadings = 0.0;

double voltageToPressure(double voltage) {
  return (voltage/sensorInputVoltage-sensorTransferOffset)/
    sensorTransferCoefficient;
}

double voltageToDepth(double voltage) {
  return (voltageToPressure(voltage)-sensorStandardAtmosphere)/
    sensorMeterSeaWater;
}

double voltageToElevation(double voltage) {
  return -(log(voltageToPressure(voltage))-log(sensorStandardAtmosphere))*
    sensorBarometricConstant;
}

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/usc/name", uscServerName, uscServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);

  node.param<double>("sensor/frequency", sensorFrequency, sensorFrequency);
  node.param<int>("sensor/input_channel", sensorInputChannel,
    sensorInputChannel);
  node.param<double>("sensor/input_voltage", sensorInputVoltage,
    sensorInputVoltage);
  node.param<double>("sensor/transfer_coefficient", sensorTransferCoefficient,
    sensorTransferCoefficient);
  node.param<double>("sensor/transfer_offset", sensorTransferOffset,
    sensorTransferOffset);
  node.param<double>("sensor/standard_atmosphere", sensorStandardAtmosphere,
    sensorStandardAtmosphere);
  node.param<double>("sensor/meter_sea_water", sensorMeterSeaWater,
    sensorMeterSeaWater);
  node.param<double>("sensor/barometric_constant", sensorBarometricConstant,
    sensorBarometricConstant);

  node.param<int>("filter/window_size", filterWindowSize, filterWindowSize);
}

void initializeInput() {
  input = -1;

  getChannelsClient = ros::NodeHandle("~").serviceClient<GetChannels>(
    "/"+uscServerName+"/get_channels");
  GetChannels getChannels;

  if (getChannelsClient.call(getChannels)) {
    if ((sensorInputChannel >= 0) &&
      (sensorInputChannel <
        getChannels.response.mode.size()) &&
      (getChannels.response.mode[sensorInputChannel] ==
        GetChannels::Response::INPUT)) {
      input = sensorInputChannel;
      ROS_INFO("USC server reported an available input at channel %d.", input);
    }
    else
      ROS_FATAL("No input available: Channel %d not in input mode.",
        sensorInputChannel);
  }
  else
    ROS_FATAL("No input available: GetChannels request failed.");
}

bool getPressure(GetPressure::Request& request, GetPressure::Response&
    response) {
  if (!readings.empty())
    response.raw = voltageToPressure(readings.back());
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (readings.size() == filterWindowSize)
    response.filtered = voltageToPressure(sumReadings/filterWindowSize);
  else
    response.filtered = std::numeric_limits<float>::quiet_NaN();

  return true;
}

bool getDepth(GetDepth::Request& request, GetDepth::Response& response) {
  if (!readings.empty())
    response.raw = voltageToDepth(readings.back());
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (readings.size() == filterWindowSize)
    response.filtered = voltageToDepth(sumReadings/filterWindowSize);
  else
    response.filtered = std::numeric_limits<float>::quiet_NaN();

  return true;
}

bool getElevation(GetElevation::Request& request, GetElevation::Response&
    response) {
  if (!readings.empty())
    response.raw = voltageToElevation(readings.back());
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (readings.size() == filterWindowSize)
    response.filtered = voltageToElevation(sumReadings/filterWindowSize);
  else
    response.filtered = std::numeric_limits<float>::quiet_NaN();

  return true;
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!getInputsClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void diagnoseInput(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (input >= 0)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "Input available at channel %d.", input);
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "No input available.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (input < 0)
    initializeInput();

  if (!getInputsClient)
    getInputsClient = ros::NodeHandle("~").serviceClient<GetInputs>(
      "/"+uscServerName+"/get_inputs", true);
}

void acquireReading(const ros::TimerEvent& event) {
  if (input < 0)
    return;

  GetInputs getInputs;
  getInputs.request.channels.push_back(input);
  if (!getInputsClient.call(getInputs))
    return;

  if (readings.size() == filterWindowSize) {
    float reading = readings.front();
    readings.pop_front();
    sumReadings -= reading;
  }
  readings.push_back(getInputs.response.voltage[0]);
  sumReadings += getInputs.response.voltage[0];

  diagnoseFrequency->tick();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_sensor");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
  updater->add("Input", diagnoseInput);
  diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
    diagnostic_updater::FrequencyStatusParam(&sensorFrequency,
    &sensorFrequency)));
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getPressureService = node.advertiseService("get_pressure", getPressure);
  getDepthService = node.advertiseService("get_depth", getDepth);
  getElevationService = node.advertiseService("get_elevation", getElevation);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);
  ros::Timer sensorTimer = node.createTimer(
    ros::Duration(1.0/sensorFrequency), acquireReading);

  initializeInput();
  tryConnect();

  ros::spin();

  return 0;
}
