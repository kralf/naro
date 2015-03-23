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
#include <signal.h>

#include <device.h>
#include <getfirmwareversion.h>
#include <setcolor.h>
#include <setfadespeed.h>
#include <setstartupparameters.h>
#include <fadetocolor.h>
#include <stopscript.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "naro_blinkm_srvs/SetColor.h"
#include "naro_blinkm_srvs/FadeToColor.h"

using namespace naro_blinkm_srvs;

double connectionRetry = 0.1;
std::string deviceAddress = "/dev/naro/blinkm";
double deviceTimeout = 0.1;
float ledStartupColor[] = {0.0f, 0.0f, 0.0f};
float ledStartupSpeed = 1.0f;
float ledShutdownColor[] = {0.0f, 0.0f, 0.0f};
float ledShutdownSpeed = 1.0f;

boost::shared_ptr<diagnostic_updater::Updater> updater;

BlinkM::Pointer<BlinkM::Device> device;

ros::ServiceServer setColorService;
ros::ServiceServer fadeToColorService;

template <typename T> inline T clamp(const T& x,
    const T& min = std::numeric_limits<T>::min(),
    const T& max = std::numeric_limits<T>::max()) {
  return x < min ? min : (x > max ? max : x);
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

inline unsigned char speedToUnits(float speed) {
  return clamp<float>(roundf(speed*255.0f/30.0f), 1.0f, 255.0f);
}

void getParameters(const ros::NodeHandle& node) {
  node.param<double>("connection/retry", connectionRetry, connectionRetry);
  node.param<std::string>("device/address", deviceAddress, deviceAddress);
  node.param<double>("device/timeout", deviceTimeout, deviceTimeout);

  parameterToColor(node, "led/startup/color", ledStartupColor);
  parameterToColor(node, "led/shutdown/color", ledShutdownColor);
}

void diagnoseDevice(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!device.isNull())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "BlinkM device created.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "BlinkM device not created.");
}

void diagnoseConnection(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!device.isNull() && device->isConnected())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "BlinkM device connected at %s.",
      device->getAdapter().getAddress().c_str());
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "BlinkM device not connected.");
}

void diagnoseTransfer(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!device.isNull() && device->isConnected()) {
    BlinkM::GetFirmwareVersion request;
    try {
      device->send(request);
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Transfer to BlinkM device succeeded: Firmware version is %d.%d.",
        request.getResponse().getMajor(), request.getResponse().getMinor());
    }
    catch (const BlinkM::Exception& exception) {
      status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Transfer to BlinkM device failed: %s", exception.what());
    }
  }
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Transfer to BlinkM device failed: No connection.");
}

void stopStartupScript() {
  try {
    BlinkM::StopScript stopScriptRequest;
    device->send(stopScriptRequest);
  }
  catch (const BlinkM::Exception& exception) {
    ROS_WARN("Failed to stop startup script: %s", exception.what());
  }
}

void setStartupParameters() {
  try {
    BlinkM::SetStartupParameters setStartupParametersRequest(
      BlinkM::SetStartupParameters::none);
    device->send(setStartupParametersRequest);
  }
  catch (const BlinkM::Exception& exception) {
    ROS_WARN("Failed to set device startup parameters: %s", exception.what());
  }
}

void fadeToStartupColor() {
  try {
    BlinkM::SetFadeSpeed setFadeSpeedRequest(speedToUnits(ledStartupSpeed));
    BlinkM::FadeToColor fadeToColorRequest(BlinkM::Color::Rgb(
      clamp<float>(ledStartupColor[0], 0.0f, 1.0f),
      clamp<float>(ledStartupColor[1], 0.0f, 1.0f),
      clamp<float>(ledStartupColor[2], 0.0f, 1.0f)));

    device->send(setFadeSpeedRequest);
    device->send(fadeToColorRequest);
  }
  catch (const BlinkM::Exception& exception) {
    ROS_WARN("Failed to fade to device startup color: %s", exception.what());
  }
}

void fadeToShutdownColor() {
  try {
    BlinkM::SetFadeSpeed setFadeSpeedRequest(speedToUnits(ledShutdownSpeed));
    BlinkM::FadeToColor fadeToColorRequest(BlinkM::Color::Rgb(
      clamp<float>(ledShutdownColor[0], 0.0f, 1.0f),
      clamp<float>(ledShutdownColor[1], 0.0f, 1.0f),
      clamp<float>(ledShutdownColor[2], 0.0f, 1.0f)));

    device->send(setFadeSpeedRequest);
    device->send(fadeToColorRequest);
  }
  catch (const BlinkM::Exception& exception) {
    ROS_WARN("Failed to fade to device shutdown color: %s", exception.what());
  }
}

bool connect() {
  try {
    if (device.isNull())
      device = new BlinkM::Device();
    updater->force_update();

    device->getAdapter().setAddress(deviceAddress);
    device->connect();
    updater->force_update();

    ROS_INFO("BlinkM device connected at %s.",
      device->getAdapter().getAddress().c_str());

    stopStartupScript();
    setStartupParameters();
    fadeToStartupColor();

    return true;
  }
  catch (const BlinkM::Exception& exception) {
    if (!device.isNull())
      device.free();

    ROS_FATAL("Connecting device failed: %s", exception.what());
  }

  return false;
}

void disconnect() {
  if (!device.isNull() && device->isConnected()) {
    fadeToShutdownColor();
    
    try {
      device->disconnect();
      updater->force_update();

      ROS_INFO("Device disconnected.");
    }
    catch (const BlinkM::Exception& exception) {
      ROS_FATAL("Disonnecting device failed: %s", exception.what());
    }
  }
}

void shutdown(int signal) {
  fadeToShutdownColor();
  ros::shutdown();
}

bool transfer(BlinkM::Request& request, const std::string& name) {
  if (!device.isNull() && device->isConnected()) {
    try {
      device->send(request);
    }
    catch (const BlinkM::Exception& exception) {
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

bool setColor(SetColor::Request& request, SetColor::Response& response) {
  BlinkM::SetColor setColorRequest(BlinkM::Color::Rgb(
    clamp<float>(request.rgb[0], 0.0f, 1.0f),
    clamp<float>(request.rgb[1], 0.0f, 1.0f),
    clamp<float>(request.rgb[2], 0.0f, 1.0f)));

  if (!transfer(setColorRequest, "SetColor"))
    return false;

  return true;
}

bool fadeToColor(FadeToColor::Request& request, FadeToColor::Response&
    response) {
  BlinkM::SetFadeSpeed setFadeSpeedRequest(speedToUnits(request.speed));
  BlinkM::FadeToColor fadeToColorRequest(BlinkM::Color::Rgb(
    clamp<float>(request.rgb[0], 0.0f, 1.0f),
    clamp<float>(request.rgb[1], 0.0f, 1.0f),
    clamp<float>(request.rgb[2], 0.0f, 1.0f)));

  if (!transfer(setFadeSpeedRequest, "SetFadeSpeed"))
    return false;
  if (!transfer(fadeToColorRequest, "FadeToColor"))
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
  ros::init(argc, argv, "blinkm_server");
  ros::NodeHandle node("~");

  signal(SIGINT, shutdown);
  
  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Device", diagnoseDevice);
  updater->add("Connection", diagnoseConnection);
  updater->force_update();

  getParameters(node);

  connect();

  setColorService = node.advertiseService("set_color", setColor);
  fadeToColorService = node.advertiseService("fade_to_color", fadeToColor);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);

  ros::spin();

  disconnect();

  return 0;
}
