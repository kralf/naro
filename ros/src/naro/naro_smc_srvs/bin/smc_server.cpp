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
#include <config/document.h>
#include <smc/device.h>
#include <smc/usb/protocol.h>
#include <smc/usb/getfirmwareversion.h>
#include <smc/usb/setsettings.h>
#include <smc/usb/getvariables.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "naro_smc_srvs/GetErrors.h"
#include "naro_smc_srvs/GetLimits.h"
#include "naro_smc_srvs/GetVoltage.h"
#include "naro_smc_srvs/GetTemperature.h"

using namespace naro_smc_srvs;

std::string deviceAddress = "/dev/naro/smc";
double deviceTimeout = 0.1;
std::string configurationFile = "etc/smc.xml";

boost::shared_ptr<diagnostic_updater::Updater> updater;
std::string configurationError;

Pololu::Pointer<Pololu::Usb::Context> context;
Pololu::Pointer<Pololu::Usb::Interface> interface;
Pololu::Pointer<Pololu::Smc::Device> device;

void getParameters() {
  ros::param::param<std::string>("device/address", deviceAddress,
    deviceAddress);
  ros::param::param<double>("device/timeout", deviceTimeout, deviceTimeout);
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
      "%s device connected on %s.", device->getName().c_str(),
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
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Transfer to Pololu device failed: %s", exception.what());
    }
  }
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Transfer to Pololu device failed: No connection.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void setSettings(const std::string& filename) {
  std::ifstream file(configurationFile.c_str());

  if (file.is_open()) {
    try {
      Pololu::Smc::Usb::SetSettings settings;
      file >> settings;

      Pololu::Smc::Usb::SetSettings setSettingsRequest(settings);
      interface->transfer(setSettingsRequest);
    }
    catch (const Pololu::Exception& exception) {
      ROS_WARN_STREAM("Failed to set device configuration: " <<
        exception.what());
      configurationError = exception.what();
    }

    file.close();
  }
  else {
    ROS_WARN_STREAM("Failed to open configuration file: " << filename);
    configurationError = "Failed to open "+filename;
  }
}

bool getErrors(GetErrors::Request& request, GetErrors::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  try {
    interface->transfer(getVariablesRequest);
    response.errors = getVariablesRequest.getResponse().errorOccurred;
  }
  catch (const Pololu::Exception& exception) {
    ROS_WARN_STREAM("GetVariables request failed: " << exception.what());
    return false;
  }

  return true;
}

bool getLimits(GetLimits::Request& request, GetLimits::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  try {
    interface->transfer(getVariablesRequest);
    response.limits = getVariablesRequest.getResponse().limitStatus;
  }
  catch (const Pololu::Exception& exception) {
    ROS_WARN_STREAM("GetVariables request failed: " << exception.what());
    return false;
  }

  return true;
}

bool getVoltage(GetVoltage::Request& request, GetVoltage::Response&
    response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  try {
    interface->transfer(getVariablesRequest);
    response.voltage = getVariablesRequest.getResponse().vinMv*1e-3;
  }
  catch (const Pololu::Exception& exception) {
    ROS_WARN_STREAM("GetVariables request failed: " << exception.what());
    return false;
  }

  return true;
}

bool getTemperature(GetTemperature::Request& request,
    GetTemperature::Response& response) {
  Pololu::Smc::Usb::GetVariables getVariablesRequest;

  try {
    interface->transfer(getVariablesRequest);
    response.temperature =
      getVariablesRequest.getResponse().temperature*1e-1;
  }
  catch (const Pololu::Exception& exception) {
    ROS_WARN_STREAM("GetVariables request failed: " << exception.what());
    return false;
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "smc_server");
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

  try {
    context = new Pololu::Usb::Context();
    interface = context->getInterface(deviceAddress);
    interface->setTimeout(deviceTimeout);
    device = interface->discoverDevice().typeCast<Pololu::Smc::Device>();
  }
  catch (const Pololu::Exception& exception) {
    ROS_FATAL_STREAM("Connecting device failed: " << exception.what());
  }
  updater->force_update();

  if (!device.isNull()) {
    device->setInterface(interface.typeCast<Pololu::Interface>());
    device->connect();
    updater->force_update();

    setSettings(configurationFile);
    updater->force_update();

    ros::ServiceServer getErrorsService = node.advertiseService(
      "smc_server/get_errors", getErrors);
    ros::ServiceServer getLimitsService = node.advertiseService(
      "smc_server/get_limits", getLimits);
    ros::ServiceServer getVoltageService = node.advertiseService(
      "smc_server/get_voltage", getVoltage);
    ros::ServiceServer getTemperatureService = node.advertiseService(
      "smc_server/get_temperature", getTemperature);

    ros::Timer timer = node.createTimer(ros::Duration(0.1),
      updateDiagnostics);
    ros::spin();

    device->disconnect();
  }
  else
    return -1;

  return 0;
}
