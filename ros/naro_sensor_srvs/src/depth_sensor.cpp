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

#include <vector>
#include <deque>
#include <limits>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_usc_srvs/GetChannels.h>
#include <naro_usc_srvs/GetInputs.h>

#include "naro_sensor_srvs/Calibrate.h"
#include "naro_sensor_srvs/GetPressure.h"
#include "naro_sensor_srvs/GetDepth.h"
#include "naro_sensor_srvs/GetElevation.h"

using namespace naro_sensor_srvs;
using namespace naro_usc_srvs;

std::string uscServerName = "usc_server";
double connectionRetry = 0.1;
float modelStandardAtmosphere = 101325.0f;
float modelMeterSeaWater = 9625.0f;
float modelBarometricConstant = 7990.0f;
double sensorFrequency = 25.0;
int sensorInputChannel = 11;
float sensorInputVoltage = 5.0f;
float sensorTransferCoefficient = 4e-6f;
float sensorTransferOffset = -0.04f;
int filterWindowSize = 50;
int calibrationWindowSize = 100;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getChannelsClient;
ros::ServiceClient getInputsClient;

ros::ServiceServer calibrateService;
ros::ServiceServer getPressureService;
ros::ServiceServer getDepthService;
ros::ServiceServer getElevationService;

int input = -1;
std::vector<float> calibrationReadings;
size_t calibrationNumReadings = 0;
float depthOffset = 0.0;
std::deque<float> filterReadings;
float filterSumReadings = 0.0;

inline float voltageToPressure(float voltage) {
  return (voltage/sensorInputVoltage-sensorTransferOffset)/
    sensorTransferCoefficient;
}

inline float voltageToDepth(float voltage) {
  return (voltageToPressure(voltage)-modelStandardAtmosphere)/
    modelMeterSeaWater;
}

inline float voltageToElevation(float voltage) {
  return -(logf(voltageToPressure(voltage))-logf(modelStandardAtmosphere))*
    modelBarometricConstant;
}

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/usc/name", uscServerName, uscServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);

  double modelStandardAtmosphere = ::modelStandardAtmosphere;
  node.param<double>("model/standard_atmosphere", modelStandardAtmosphere,
    modelStandardAtmosphere);
  ::modelStandardAtmosphere = modelStandardAtmosphere;
  double modelMeterSeaWater = ::modelMeterSeaWater;
  node.param<double>("model/meter_sea_water", modelMeterSeaWater,
    modelMeterSeaWater);
  ::modelMeterSeaWater = modelMeterSeaWater;
  double modelBarometricConstant = ::modelBarometricConstant;
  node.param<double>("model/barometric_constant", modelBarometricConstant,
    modelBarometricConstant);
  ::modelBarometricConstant = modelBarometricConstant;

  node.param<double>("sensor/frequency", sensorFrequency, sensorFrequency);
  node.param<int>("sensor/input_channel", sensorInputChannel,
    sensorInputChannel);
  double sensorInputVoltage = ::sensorInputVoltage;
  node.param<double>("sensor/input_voltage", sensorInputVoltage,
    sensorInputVoltage);
  ::sensorInputVoltage = sensorInputVoltage;
  double sensorTransferCoefficient = ::sensorTransferCoefficient;
  node.param<double>("sensor/transfer_coefficient", sensorTransferCoefficient,
    sensorTransferCoefficient);
  ::sensorTransferCoefficient = sensorTransferCoefficient;
  double sensorTransferOffset = ::sensorTransferOffset;
  node.param<double>("sensor/transfer_offset", sensorTransferOffset,
    sensorTransferOffset);
  ::sensorTransferOffset = sensorTransferOffset;

  node.param<int>("filter/window_size", filterWindowSize, filterWindowSize);

  node.param<int>("calibration/window_size", calibrationWindowSize,
    calibrationWindowSize);
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

bool calibrate(Calibrate::Request& request, Calibrate::Response& response) {
  if (request.window > 0) {
    calibrationReadings.resize(request.window);
    calibrationNumReadings = 0;
  }
  else
    return false;

  return true;
}

bool getPressure(GetPressure::Request& request, GetPressure::Response&
    response) {
  if (!filterReadings.empty())
    response.raw = voltageToPressure(filterReadings.back());
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (filterReadings.size() == filterWindowSize)
    response.filtered = voltageToPressure(filterSumReadings/filterWindowSize);
  else
    response.filtered = std::numeric_limits<float>::quiet_NaN();

  return true;
}

bool getDepth(GetDepth::Request& request, GetDepth::Response& response) {
  if (!filterReadings.empty())
    response.raw = fmaxf(0.0f, voltageToDepth(filterReadings.back())+
      depthOffset);
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (filterReadings.size() == filterWindowSize)
    response.filtered = fmaxf(0.0f, voltageToDepth(filterSumReadings/
      filterWindowSize)+depthOffset);
  else
    response.filtered = std::numeric_limits<float>::quiet_NaN();

  return true;
}

bool getElevation(GetElevation::Request& request, GetElevation::Response&
    response) {
  if (!filterReadings.empty())
    response.raw = fmaxf(0.0f, voltageToElevation(filterReadings.back()));
  else
    response.raw = std::numeric_limits<float>::quiet_NaN();

  if (filterReadings.size() == filterWindowSize)
    response.filtered = fmaxf(0.0f, voltageToElevation(filterSumReadings/
      filterWindowSize));
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

  if (calibrationNumReadings < calibrationReadings.size()) {
    calibrationReadings[calibrationNumReadings] =
      getInputs.response.voltage[0];
    ++calibrationNumReadings;

    std::nth_element(calibrationReadings.begin(),
      calibrationReadings.begin()+calibrationNumReadings/2,
      calibrationReadings.begin()+calibrationNumReadings);
    depthOffset = -voltageToDepth(
      calibrationReadings[calibrationNumReadings/2]);
  }

  if (filterReadings.size() == filterWindowSize) {
    float reading = filterReadings.front();
    filterReadings.pop_front();
    filterSumReadings -= reading;
  }
  filterReadings.push_back(getInputs.response.voltage[0]);
  filterSumReadings += getInputs.response.voltage[0];

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

  calibrateService = node.advertiseService("calibrate", calibrate);
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

  calibrationReadings.resize(calibrationWindowSize);
  calibrationNumReadings = 0;

  ros::spin();

  return 0;
}
