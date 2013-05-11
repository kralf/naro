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
#include <limits>

#include <usb/context.h>
#include <usb/error.h>
#include <config/document.h>
#include <usc/device.h>
#include <usc/usb/getfirmwareversion.h>
#include <usc/usb/mini/setsettings.h>
#include <usc/usb/mini/getsettings.h>
#include <usc/usb/mini/getvariables.h>
#include <usc/usb/mini/getservovariables.h>
#include <usc/usb/reinitialize.h>
#include <usc/usb/clearerrors.h>
#include <usc/usb/settarget.h>
#include <usc/usb/setspeed.h>
#include <usc/usb/setacceleration.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "naro_usc_srvs/GetErrors.h"
#include "naro_usc_srvs/GetChannels.h"
#include "naro_usc_srvs/GetPositions.h"
#include "naro_usc_srvs/GetSpeeds.h"
#include "naro_usc_srvs/GetAccelerations.h"
#include "naro_usc_srvs/GetInputs.h"
#include "naro_usc_srvs/Initialize.h"
#include "naro_usc_srvs/ClearErrors.h"
#include "naro_usc_srvs/SetPositions.h"
#include "naro_usc_srvs/SetSpeeds.h"
#include "naro_usc_srvs/SetAccelerations.h"
#include "naro_usc_srvs/SetOutputs.h"

using namespace naro_usc_srvs;

double connectionRetry = 0.1;
std::string deviceAddress = "/dev/naro/usc";
double deviceTimeout = 0.1;
double servosTransmission = 0.001;
std::string configurationFile = "etc/usc.xml";

boost::shared_ptr<diagnostic_updater::Updater> updater;
std::string configurationError;

Pololu::Pointer<Pololu::Usb::Context> context;
Pololu::Pointer<Pololu::Usb::Interface> interface;
Pololu::Pointer<Pololu::Usc::Device> device;
Pololu::Usc::Usb::Mini::Settings settings(0);

ros::ServiceServer getErrorsService;
ros::ServiceServer getChannelsService;
ros::ServiceServer getPositionsService;
ros::ServiceServer getSpeedsService;
ros::ServiceServer getAccelerationsService;
ros::ServiceServer getInputsService;
ros::ServiceServer initializeService;
ros::ServiceServer clearErrorsService;
ros::ServiceServer setPositionsService;
ros::ServiceServer setSpeedsService;
ros::ServiceServer setAccelerationsService;

template <typename T> inline T clamp(T x, T min, T max) {
  return x < min ? min : (x > max ? max : x);
}

inline bool isServo(int channel) {
  return (channel >= 0) && (channel < settings.channels.size()) &&
    ((settings.channels[channel].mode ==
      Pololu::Usc::Usb::Settings::Channel::modeServo) ||
    (settings.channels[channel].mode ==
      Pololu::Usc::Usb::Settings::Channel::modeServoMultiplied));
}

inline bool isInput(int channel) {
  return (channel >= 0) && (channel < settings.channels.size()) &&
    (settings.channels[channel].mode ==
      Pololu::Usc::Usb::Settings::Channel::modeInput);
}

inline bool isOutput(int channel) {
  return (channel >= 0) && (channel < settings.channels.size()) &&
    (settings.channels[channel].mode ==
      Pololu::Usc::Usb::Settings::Channel::modeOutput);
}

inline double qusToRad(int channel, unsigned short position) {
  return (position-settings.channels[channel].neutral)*M_PI*
    0.25e-6/servosTransmission;
}

inline unsigned short radToQus(int channel, double angle) {
  return round(angle/M_PI*servosTransmission/0.25e-6)+
    settings.channels[channel].neutral;
}

void getParameters() {
  ros::param::param<double>("connection/retry", connectionRetry,
    connectionRetry);
  ros::param::param<std::string>("device/address", deviceAddress,
    deviceAddress);
  ros::param::param<double>("device/timeout", deviceTimeout, deviceTimeout);
  ros::param::param<double>("servos/transmission", servosTransmission,
    servosTransmission);
  ros::param::param<std::string>("configuration/file", configurationFile,
    configurationFile);
}

void diagnoseContext(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!context.isNull())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "Pololu USB context created.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Pololu USB context not created.");
}

void diagnoseInterface(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!interface.isNull())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "Pololu USB interface created at %s.", interface->getAddress().c_str());
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Pololu USB interface not created.");
}

void diagnoseDevice(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!device.isNull())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "%s device created.", device->getName().c_str());
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Pololu device not created.");
}

void diagnoseConnection(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!device.isNull() && device->isConnected())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "%s device connected at %s.", device->getName().c_str(),
      device->getInterface()->getAddress().c_str());
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Pololu device not connected.");
}

void diagnoseConfiguration(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!device.isNull() && device->isConnected()) {
    if (configurationError.empty())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Device configuration set from %s.", configurationFile.c_str());
    else
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
        "Pololu device configuration failed: %s", configurationError.c_str());
  }
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Pololu device not connected.");
}

void diagnoseTransfer(diagnostic_updater::DiagnosticStatusWrapper
    &status) {
  if (!device.isNull() && device->isConnected()) {
    Pololu::Usc::Usb::GetFirmwareVersion request;
    try {
      device->send(request);
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Transfer to Pololu device succeeded: Firmware version is %d.%d.",
        request.getResponse().getMajor(), request.getResponse().getMinor());
    }
    catch (const Pololu::Exception& exception) {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Transfer to Pololu device failed: %s", exception.what());
    }
  }
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Transfer to Pololu device failed: No connection.");
}

void syncSettings(const std::string& filename) {
  std::ifstream file(configurationFile.c_str());

  if (file.is_open()) {
    try {
      file >> settings;
      Pololu::Usc::Usb::Mini::SetSettings setSettingsRequest(
        settings.channels.size());
      setSettingsRequest.setSettings(settings);
      interface->transfer(setSettingsRequest);
    }
    catch (const Pololu::Exception& exception) {
      ROS_WARN("Failed to set device configuration: %s", exception.what());
      configurationError = exception.what();
    }

    file.close();
  }
  else {
    ROS_WARN("Failed to open configuration file: %s", filename.c_str());
    configurationError = "Failed to open "+filename;
  }

  try {
    Pololu::Usc::Usb::Mini::GetSettings getSettingsRequest(
      settings.channels.size());
    interface->transfer(getSettingsRequest);
    settings = getSettingsRequest.getResponse();
  }
  catch (const Pololu::Exception& exception) {
    ROS_WARN("Failed to get device configuration: %s", exception.what());
  }
}

bool connect() {
  try {
    if (context.isNull())
      context = new Pololu::Usb::Context();
    if (interface.isNull()) {
      interface = context->getInterface(deviceAddress);
      interface->setTimeout(deviceTimeout);
    }
    if (device.isNull())
      device = interface->discoverDevice().typeCast<Pololu::Usc::Device>();
    updater->force_update();

    device->setInterface(interface.typeCast<Pololu::Interface>());
    device->connect();
    settings.channels.resize(device->getNumChannels());
    updater->force_update();

    syncSettings(configurationFile);
    updater->force_update();

    ROS_INFO("%s device connected at %s.", device->getName().c_str(),
      device->getInterface()->getAddress().c_str());

    return true;
  }
  catch (const Pololu::Exception& exception) {
    if (!device.isNull())
      device.free();
    if (!interface.isNull())
      interface.free();

    ROS_FATAL("Connecting device failed: %s", exception.what());
  }

  return false;
}

void disconnect() {
  if (!device.isNull() && device->isConnected()) {
    try {
      device->disconnect();
      settings.channels.clear();
      updater->force_update();

      ROS_INFO("Device disconnected.");
    }
    catch (const Pololu::Exception& exception) {
      ROS_FATAL("Disonnecting device failed: %s", exception.what());
    }
  }
}

bool transfer(Pololu::Usb::Request& request, const std::string& name) {
  if (!device.isNull() && device->isConnected()) {
    try {
      interface->transfer(request);
    }
    catch (const Pololu::Usb::Error& error) {
      ROS_WARN("%s request failed: %s", name.c_str(), error.what());

      if (error == Pololu::Usb::Error::device) {
        ROS_INFO("Retrying connection now.");

        disconnect();
        connect();
      }
    }
    catch (const Pololu::Exception& exception) {
      ROS_WARN("%s request failed: %s", name.c_str(), exception.what());
      return false;
    }
  }
  else {
    ROS_WARN("%s request failed: Device not connected.", name.c_str());
    return false;
  }

  return true;
}

bool getErrors(GetErrors::Request& request, GetErrors::Response& response) {
  Pololu::Usc::Usb::Mini::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.errors = getVariablesRequest.getResponse().errorOccurred;
  else
    return false;

  return true;
}

bool getChannels(GetChannels::Request& request, GetChannels::Response&
    response) {
  if (settings.channels.size()) {
    response.mode.resize(settings.channels.size());

    for (int i = 0; i < settings.channels.size(); ++i) {
      if (settings.channels[i].mode ==
          Pololu::Usc::Usb::Settings::Channel::modeOutput)
        response.mode[i] = GetChannels::Response::OUTPUT;
      else if (settings.channels[i].mode ==
          Pololu::Usc::Usb::Settings::Channel::modeInput)
        response.mode[i] = GetChannels::Response::INPUT;
      else
        response.mode[i] = GetChannels::Response::SERVO;
    }

    return true;
  }
  else
    return false;
}

bool getPositions(GetPositions::Request& request, GetPositions::Response&
    response) {
  Pololu::Usc::Usb::Mini::GetServoVariables
    getServoVariablesRequest(settings.channels.size());

  if (transfer(getServoVariablesRequest, "GetServoVariables")) {
    Pololu::Usc::Usb::Variables::Servos variables =
      getServoVariablesRequest.getResponse();
    response.actual.resize(request.channels.size());
    response.target.resize(request.channels.size());

    for (int i = 0; i < request.channels.size(); ++i) {
      if (isServo(request.channels[i])) {
        response.actual[i] = qusToRad(request.channels[i],
          variables[request.channels[i]].position);
        response.target[i] = qusToRad(request.channels[i],
          variables[request.channels[i]].target);
      }
      else {
        response.actual[i] = std::numeric_limits<float>::quiet_NaN();
        response.target[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  else
    return false;

  return true;
}

bool getSpeeds(GetSpeeds::Request& request, GetSpeeds::Response& response) {
  Pololu::Usc::Usb::Mini::GetServoVariables
    getServoVariablesRequest(settings.channels.size());

  if (transfer(getServoVariablesRequest, "GetServoVariables")) {
    Pololu::Usc::Usb::Variables::Servos variables =
      getServoVariablesRequest.getResponse();
    response.speed.resize(request.channels.size());

    for (int i = 0; i < request.channels.size(); ++i) {
      if (isServo(request.channels[i]))
        response.speed[i] = qusToRad(request.channels[i],
          variables[request.channels[i]].speed);
      else
        response.speed[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  else
    return false;

  return true;
}

bool getAccelerations(GetAccelerations::Request& request,
    GetAccelerations::Response& response) {
  Pololu::Usc::Usb::Mini::GetServoVariables
    getServoVariablesRequest(settings.channels.size());

  if (transfer(getServoVariablesRequest, "GetServoVariables")) {
    Pololu::Usc::Usb::Variables::Servos variables =
      getServoVariablesRequest.getResponse();
    response.acceleration.resize(request.channels.size());

    for (int i = 0; i < request.channels.size(); ++i) {
      if (isServo(request.channels[i]))
        response.acceleration[i] = qusToRad(request.channels[i],
          variables[request.channels[i]].acceleration);
      else
        response.acceleration[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  else
    return false;

  return true;
}

bool getInputs(GetInputs::Request& request, GetInputs::Response& response) {
  Pololu::Usc::Usb::Mini::GetServoVariables
    getServoVariablesRequest(settings.channels.size());

  if (transfer(getServoVariablesRequest, "GetServoVariables")) {
    Pololu::Usc::Usb::Variables::Servos variables =
      getServoVariablesRequest.getResponse();
    response.voltage.resize(request.channels.size());

    for (int i = 0; i < request.channels.size(); ++i) {
      if (isInput(request.channels[i]))
        response.voltage[i] =
          variables[request.channels[i]].position/1023.0*5.0;
      else
        response.voltage[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  else
    return false;

  return true;
}

bool initialize(Initialize::Request& request, Initialize::Response&
    response) {
  Pololu::Usc::Usb::Reinitialize reinitializeRequest;

  if (!transfer(reinitializeRequest, "Reinitialize"))
    return false;

  return true;
}

bool clearErrors(ClearErrors::Request& request, ClearErrors::Response&
    response) {
  Pololu::Usc::Usb::ClearErrors clearErrorsRequest;

  if (!transfer(clearErrorsRequest, "ClearErrors"))
    return false;

  return true;
}

bool setPositions(SetPositions::Request& request, SetPositions::Response&
    response) {
  Pololu::Usc::Usb::SetTarget setTargetRequest(settings.channels.size());
  bool result = true;

  for (int i = 0; i < request.channels.size(); ++i) {
    if (isServo(request.channels[i])) {
      setTargetRequest.setServo(request.channels[i]);
      setTargetRequest.setValue(radToQus(request.channels[i],
        request.position[i]));

      if (!transfer(setTargetRequest, "SetTarget"))
        result = false;
    }
    else {
      ROS_WARN("SetTarget request failed: Channel %d not in servo mode.",
        request.channels[i]);
      result = false;
    }
  }

  return result;
}

bool setSpeeds(SetSpeeds::Request& request, SetSpeeds::Response& response) {
  Pololu::Usc::Usb::SetSpeed setSpeedRequest(settings.channels.size());
  bool result = true;

  for (int i = 0; i < request.channels.size(); ++i) {
    if (isServo(request.channels[i])) {
      setSpeedRequest.setServo(request.channels[i]);
      setSpeedRequest.setValue(radToQus(request.channels[i],
        request.speed[i]));

      if (!transfer(setSpeedRequest, "SetSpeed"))
        result = false;
    }
    else {
      ROS_WARN("SetSpeed request failed: Channel %d not in servo mode.",
        request.channels[i]);
      result = false;
    }
  }

  return result;
}

bool setAccelerations(SetAccelerations::Request& request,
    SetAccelerations::Response& response) {
  Pololu::Usc::Usb::SetAcceleration setAccelerationRequest(
    settings.channels.size());
  bool result = true;

  for (int i = 0; i < request.channels.size(); ++i) {
    if (isServo(request.channels[i])) {
      setAccelerationRequest.setServo(request.channels[i]);
      setAccelerationRequest.setValue(radToQus(request.channels[i],
        request.acceleration[i]));

      if (!transfer(setAccelerationRequest, "SetAcceleration"))
        result = false;
    }
    else {
      ROS_WARN("SetAcceleration request failed: Channel %d not in servo mode.",
        request.channels[i]);
      result = false;
    }
  }

  return result;
}

bool setOutputs(SetOutputs::Request& request, SetOutputs::Response&
    response) {
  Pololu::Usc::Usb::SetTarget setTargetRequest(settings.channels.size());
  bool result = true;

  for (int i = 0; i < request.channels.size(); ++i) {
    if (isOutput(request.channels[i])) {
      setTargetRequest.setServo(request.channels[i]);
      setTargetRequest.setValue(request.high[i] ? 6000 : 0);

      if (!transfer(setTargetRequest, "SetTarget"))
        result = false;
    }
    else {
      ROS_WARN("SetTarget request failed: Channel %d not in output mode.",
        request.channels[i]);
      result = false;
    }
  }

  return result;
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void tryConnect(const ros::TimerEvent& event) {
  if (device.isNull() || !device->isConnected()) {
    if (!connect())
      ROS_INFO("Retrying in %.2f second(s).", connectionRetry);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usc_server");
  ros::NodeHandle node;

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Context", diagnoseContext);
  updater->add("Interface", diagnoseInterface);
  updater->add("Device", diagnoseDevice);
  updater->add("Connection", diagnoseConnection);
  updater->add("Configuration", diagnoseConfiguration);
  updater->add("Transfer", diagnoseTransfer);
  updater->force_update();

  getParameters();

  connect();

  getErrorsService = node.advertiseService(
    ros::this_node::getName()+"/get_errors", getErrors);
  getChannelsService = node.advertiseService(
    ros::this_node::getName()+"/get_channels", getChannels);
  getPositionsService = node.advertiseService(
    ros::this_node::getName()+"/get_positions", getPositions);
  getSpeedsService = node.advertiseService(
    ros::this_node::getName()+"/get_speeds", getSpeeds);
  getAccelerationsService = node.advertiseService(
    ros::this_node::getName()+"/get_acceleration", getAccelerations);
  getInputsService = node.advertiseService(
    ros::this_node::getName()+"/get_inputs", getInputs);
  initializeService = node.advertiseService(
    ros::this_node::getName()+"/initialize", initialize);
  clearErrorsService = node.advertiseService(
    ros::this_node::getName()+"/clear_errors", clearErrors);
  setPositionsService = node.advertiseService(
    ros::this_node::getName()+"/set_positions", setPositions);
  setSpeedsService = node.advertiseService(
    ros::this_node::getName()+"/set_speeds", setSpeeds);
  setAccelerationsService = node.advertiseService(
    ros::this_node::getName()+"/set_acceleration", setAccelerations);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);

  ros::spin();

  disconnect();

  return 0;
}
