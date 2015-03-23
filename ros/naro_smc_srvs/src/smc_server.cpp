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

#include <usb/context.h>
#include <usb/error.h>
#include <config/document.h>
#include <smc/device.h>
#include <smc/usb/getfirmwareversion.h>
#include <smc/usb/setsettings.h>
#include <smc/usb/getsettings.h>
#include <smc/usb/getvariables.h>
#include <smc/usb/exitsafestart.h>
#include <smc/usb/setusbkill.h>
#include <smc/usb/setspeed.h>
#include <smc/usb/setbrake.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "naro_smc_srvs/GetErrors.h"
#include "naro_smc_srvs/GetLimits.h"
#include "naro_smc_srvs/GetInputs.h"
#include "naro_smc_srvs/GetVoltage.h"
#include "naro_smc_srvs/GetTemperature.h"
#include "naro_smc_srvs/GetSpeed.h"
#include "naro_smc_srvs/GetBrake.h"
#include "naro_smc_srvs/Start.h"
#include "naro_smc_srvs/Kill.h"
#include "naro_smc_srvs/SetSpeed.h"
#include "naro_smc_srvs/SetBrake.h"

using namespace naro_smc_srvs;

double connectionRetry = 0.1;
std::string deviceAddress = "/dev/naro/smc";
double deviceTimeout = 0.1;
std::string configurationFile = "etc/smc.xml";

boost::shared_ptr<diagnostic_updater::Updater> updater;
std::string configurationError;

Pololu::Pointer<Pololu::Usb::Context> context;
Pololu::Pointer<Pololu::Usb::Interface> interface;
Pololu::Pointer<Pololu::Smc::Device> device;
Pololu::Smc::Usb::Settings settings;

ros::ServiceServer getErrorsService;
ros::ServiceServer getLimitsService;
ros::ServiceServer getInputsService;
ros::ServiceServer getVoltageService;
ros::ServiceServer getTemperatureService;
ros::ServiceServer getSpeedService;
ros::ServiceServer getBrakeService;
ros::ServiceServer startService;
ros::ServiceServer killService;
ros::ServiceServer setSpeedService;
ros::ServiceServer setBrakeService;

template <typename T> inline T clamp(T x, T min, T max) {
  return x < min ? min : (x > max ? max : x);
}

void getParameters(const ros::NodeHandle& node) {
  node.param<double>("connection/retry", connectionRetry, connectionRetry);
  node.param<std::string>("device/address", deviceAddress, deviceAddress);
  node.param<double>("device/timeout", deviceTimeout, deviceTimeout);
  node.param<std::string>("configuration/file", configurationFile,
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
    Pololu::Smc::Usb::GetFirmwareVersion request;
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
      Pololu::Smc::Usb::SetSettings setSettingsRequest(settings);
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
    Pololu::Smc::Usb::GetSettings getSettingsRequest;
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
      device = interface->discoverDevice().typeCast<Pololu::Smc::Device>();
    updater->force_update();

    device->setInterface(interface.typeCast<Pololu::Interface>());
    device->connect();
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
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.errors = getVariablesRequest.getResponse().errorOccurred;
  else
    return false;

  return true;
}

bool getLimits(GetLimits::Request& request, GetLimits::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.limits = getVariablesRequest.getResponse().limitStatus;
  else
    return false;

  return true;
}

bool getInputs(GetInputs::Request& request, GetInputs::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables")) {
    Pololu::Smc::Usb::Variables variables = getVariablesRequest.getResponse();

    response.raw[GetInputs::Response::RC1] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelRc1].rawValue*0.25e-6f;
    response.scaled[GetInputs::Response::RC1] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelRc1].scaledValue/3200.0f;

    response.raw[GetInputs::Response::RC2] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelRc2].rawValue*0.25e-6f;
    response.scaled[GetInputs::Response::RC2] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelRc2].scaledValue/3200.0f;

    response.raw[GetInputs::Response::ANALOG1] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelAnalog1].rawValue/4095.0f*3.3f;
    response.scaled[GetInputs::Response::ANALOG1] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelAnalog1].scaledValue/3200.0f;

    response.raw[GetInputs::Response::ANALOG2] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelAnalog2].rawValue/4095.0f*3.3f;
    response.scaled[GetInputs::Response::ANALOG2] = variables.inputChannels[
      Pololu::Smc::Device::inputChannelAnalog2].scaledValue/3200.0f;
  }
  else
    return false;

  return true;
}

bool getVoltage(GetVoltage::Request& request, GetVoltage::Response&
    response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.voltage = getVariablesRequest.getResponse().vinMv*1e-3f;
  else
    return false;

  return true;
}

bool getTemperature(GetTemperature::Request& request,
    GetTemperature::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.temperature =
      getVariablesRequest.getResponse().temperature*1e-1f;
  else
    return false;

  return true;
}

bool getSpeed(GetSpeed::Request& request, GetSpeed::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables")) {
    Pololu::Smc::Usb::Variables variables = getVariablesRequest.getResponse();
    response.actual = variables.speed/3200.0f;
    response.target = variables.targetSpeed/3200.0f;
  }
  else
    return false;

  return true;
}

bool getBrake(GetBrake::Request& request, GetBrake::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  if (transfer(getVariablesRequest, "GetVariables"))
    response.brake = getVariablesRequest.getResponse().brakeAmount/32.0f;
  else
    return false;

  return true;
}

bool start(Start::Request& request, Start::Response& response) {
  Pololu::Smc::Usb::ExitSafeStart startRequest;

  if (!transfer(startRequest, "ExitSafeStart"))
    return false;

  return true;
}

bool kill(Kill::Request& request, Kill::Response& response) {
  Pololu::Smc::Usb::SetUsbKill killRequest;

  if (!transfer(killRequest, "SetUsbKill"))
    return false;

  return true;
}

bool setSpeed(SetSpeed::Request& request, SetSpeed::Response& response) {
  Pololu::Smc::Usb::SetSpeed setSpeedRequest(
    round(clamp<float>(request.speed, -1.0f, 1.0f)*3200.0f));
  Pololu::Smc::Usb::ExitSafeStart startRequest;

  if (!transfer(setSpeedRequest, "SetSpeed"))
    return false;
  if (request.start && !transfer(startRequest, "ExitSafeStart"))
    return false;

  return true;
}

bool setBrake(SetBrake::Request& request, SetBrake::Response& response) {
  Pololu::Smc::Usb::SetBrake setBrakeRequest(
    round(clamp<float>(request.brake, 0.0f, 1.0f)*32.0f));

  if (!transfer(setBrakeRequest, "SetBrake"))
    return false;

  return true;
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
  ros::init(argc, argv, "smc_server");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Context", diagnoseContext);
  updater->add("Interface", diagnoseInterface);
  updater->add("Device", diagnoseDevice);
  updater->add("Connection", diagnoseConnection);
  updater->add("Configuration", diagnoseConfiguration);
  updater->add("Transfer", diagnoseTransfer);
  updater->force_update();

  getParameters(node);

  connect();

  getErrorsService = node.advertiseService("get_errors", getErrors);
  getLimitsService = node.advertiseService("get_limits", getLimits);
  getInputsService = node.advertiseService("get_inputs", getInputs);
  getVoltageService = node.advertiseService("get_voltage", getVoltage);
  getTemperatureService = node.advertiseService("get_temperature",
    getTemperature);
  getSpeedService = node.advertiseService("get_speed", getSpeed);
  getBrakeService = node.advertiseService("get_brake", getBrake);
  startService = node.advertiseService("start", start);
  killService = node.advertiseService("kill", kill);
  setSpeedService = node.advertiseService("set_speed", setSpeed);
  setBrakeService = node.advertiseService("set_brake", setBrake);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);

  ros::spin();

  disconnect();

  return 0;
}
