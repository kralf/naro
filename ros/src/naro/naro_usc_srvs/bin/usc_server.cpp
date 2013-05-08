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
#include <usc/device.h>
#include <usc/usb/getfirmwareversion.h>
#include <usc/usb/mini/setsettings.h>
#include <usc/usb/mini/getvariables.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "naro_usc_srvs/GetErrors.h"

using namespace naro_usc_srvs;

double connectionRetry = 1.0;
std::string deviceAddress = "/dev/naro/usc";
double deviceTimeout = 0.1;
std::string configurationFile = "etc/usc.xml";

boost::shared_ptr<diagnostic_updater::Updater> updater;
std::string configurationError;

Pololu::Pointer<Pololu::Usb::Context> context;
Pololu::Pointer<Pololu::Usb::Interface> interface;
Pololu::Pointer<Pololu::Usc::Device> device;

void getParameters() {
  ros::param::param<double>("connection/retry", connectionRetry,
    connectionRetry);
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

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void setSettings(const std::string& filename) {
  std::ifstream file(configurationFile.c_str());

  if (file.is_open()) {
    try {
      Pololu::Usc::Usb::Mini::SetSettings settings(device->getNumChannels());
      file >> settings;

      Pololu::Usc::Usb::Mini::SetSettings setSettingsRequest(settings);
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
}

void connect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (device.isNull() || !device->isConnected()) {
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
      updater->force_update();

      setSettings(configurationFile);
      updater->force_update();

      ROS_INFO("%s device connected at %s.", device->getName().c_str(),
        device->getInterface()->getAddress().c_str());
    }
    catch (const Pololu::Exception& exception) {
      if (!device.isNull())
        device.free();
      if (!interface.isNull())
        interface.free();

      ROS_FATAL("Connecting device failed: %s", exception.what());
      ROS_INFO("Retrying in %.2f second(s).", connectionRetry);
    }
  }
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

  ros::ServiceServer getErrorsService = node.advertiseService(
    "usc_server/get_errors", getErrors);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectTimer = node.createTimer(
    ros::Duration(connectionRetry), connect);

  ros::spin();

  disconnect();

  return 0;
}
