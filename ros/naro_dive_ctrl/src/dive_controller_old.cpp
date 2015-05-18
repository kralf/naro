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

#include <naro_smc_srvs/GetLimits.h>
#include <naro_smc_srvs/SetSpeed.h>
#include <naro_sensor_srvs/GetDepth.h>

#include "naro_dive_ctrl/GetEnabled.h"
#include "naro_dive_ctrl/GetGains.h"
#include "naro_dive_ctrl/GetCommand.h"
#include "naro_dive_ctrl/GetActual.h"
#include "naro_dive_ctrl/GetError.h"
#include "naro_dive_ctrl/SetGains.h"
#include "naro_dive_ctrl/SetCommand.h"
#include "naro_dive_ctrl/Enable.h"
#include "naro_dive_ctrl/Disable.h"
#include "naro_dive_ctrl/Emerge.h"

using namespace naro_dive_ctrl;
using namespace naro_smc_srvs;
using namespace naro_sensor_srvs;

std::string smcServerName = "smc_server";
std::string sensorServerName = "depth_sensor";
double connectionRetry = 0.1;
float modelGravitationalAcceleration = 9.80665f;  // [m/s^2]
float modelFluidDensity = 1000.0f;                // [kg/m^3]
float modelPlatformMass = 10.0f;                  // [kg]
float modelPlatformArea = 0.1f;                   // [m^2]
float modelPlatformVolume = 0.012f;               // [m^3]
float modelPlatformDragCoefficient = 1.0f;
float modelActuatorMaxFlowRate = 20e-6f;          // [m^3/s]
int actuatorLimitsMinInputChannel = 128;
int actuatorLimitsMaxInputChannel = 256;
bool actuatorInverted = true;
double controllerFrequency = 5.0;
float controllerToleranceDepth = 0.1f;             // [m]
float controllerToleranceVelocity = 0.0f;          // [m/s]
float controllerGainProportional = 1e-2f;
float controllerGainIntegral = 1e-1f;
float controllerGainDifferential = 0.0f;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getLimitsClient;
ros::ServiceClient setSpeedClient;
ros::ServiceClient getDepthClient;

ros::ServiceServer getEnabledService;
ros::ServiceServer getGainsService;
ros::ServiceServer getCommandService;
ros::ServiceServer getActualService;
ros::ServiceServer getErrorService;
ros::ServiceServer setGainsService;
ros::ServiceServer setCommandService;
ros::ServiceServer enableService;
ros::ServiceServer disableService;
ros::ServiceServer emergeService;

class Controller {
public:
  class Parameters {
  public:
    Parameters(float depth = 0.0f, float velocity = 0.0f) :
      depth(depth),
      velocity(velocity) {
    };

    float depth;
    float velocity;
  };

  Controller(float lastError = std::numeric_limits<float>::quiet_NaN(),
      float integralError = 0.0f) :
    lastError(lastError),
    integralError(integralError),
    enabled(false) {
  };
  
  void reset() {
    integralError = 0.0f;
    lastError = std::numeric_limits<float>::quiet_NaN();
  };

  bool enabled;
  float lastError;
  float integralError;
  Parameters command;
  Parameters actual;
};

Controller controller;
ros::Time lastTime;

template <typename T> inline T clamp(T x, T min, T max) {
  return x < min ? min : (x > max ? max : x);
}

/** Calculate the net force [N] acting on the platform for a given platform
  * velocity [m/s]
  */
inline float velocityToForce(float velocity) {
  float gravitationalForce = modelPlatformMass*
    modelGravitationalAcceleration;
  float buoyancyForce = modelPlatformVolume*modelFluidDensity*
    modelGravitationalAcceleration;
  float dragForce = 0.5f*velocity*velocity*modelFluidDensity*
    modelPlatformArea*modelPlatformDragCoefficient;

  if (velocity > 0.0f)
    return gravitationalForce-buoyancyForce-dragForce;
  else
    return gravitationalForce-buoyancyForce+dragForce;
}

/** Calculate the platform acceleration [m/s^2] for a given platform
  * velocity [m/s]
  */
inline float velocityToAcceleration(float velocity) {
  return velocityToForce(velocity)/modelPlatformMass;
}

inline float outputToSpeed(float output) {
  float sign = actuatorInverted ? -1.0f : 1.0f;
  return clamp(sign*output/modelActuatorMaxFlowRate, -1.0f, 1.0f);
}

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/smc/name", smcServerName, smcServerName);
  node.param<std::string>("server/sensor/name", sensorServerName,
    sensorServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);

  double modelGravitationalAcceleration = ::modelGravitationalAcceleration;
  node.param<double>("model/gravitational_acceleration",
    modelGravitationalAcceleration, modelGravitationalAcceleration);
  ::modelGravitationalAcceleration = modelGravitationalAcceleration;
  double modelFluidDensity = ::modelFluidDensity;
  node.param<double>("model/fluid/density",
    modelFluidDensity, modelFluidDensity);
  ::modelFluidDensity = modelFluidDensity;
  double modelPlatformMass = ::modelPlatformMass;
  node.param<double>("model/platform/mass",
    modelPlatformMass, modelPlatformMass);
  ::modelPlatformMass = modelPlatformMass;
  double modelPlatformArea = ::modelPlatformArea;
  node.param<double>("model/platform/area",
    modelPlatformArea, modelPlatformArea);
  ::modelPlatformArea = modelPlatformArea;
  double modelPlatformVolume = ::modelPlatformVolume;
  node.param<double>("model/platform/volume",
    modelPlatformVolume, modelPlatformVolume);
  ::modelPlatformVolume = modelPlatformVolume;
  double modelPlatformDragCoefficient = ::modelPlatformDragCoefficient;
  node.param<double>("model/platform/drag_coefficient",
    modelPlatformDragCoefficient, modelPlatformDragCoefficient);
  ::modelPlatformDragCoefficient = modelPlatformDragCoefficient;
  double modelActuatorMaxFlowRate = ::modelActuatorMaxFlowRate;
  node.param<double>("model/actuator/max_flow_rate",
    modelActuatorMaxFlowRate, modelActuatorMaxFlowRate);
  ::modelActuatorMaxFlowRate = modelActuatorMaxFlowRate;

  node.param<int>("actuator/limits/minimum/input_channel",
    actuatorLimitsMinInputChannel, actuatorLimitsMinInputChannel);
  node.param<int>("actuator/limits/maximum/input_channel",
    actuatorLimitsMaxInputChannel, actuatorLimitsMaxInputChannel);
  node.param<bool>("actuator/inverted", actuatorInverted,
    actuatorInverted);

  node.param<double>("controller/frequency", controllerFrequency,
    controllerFrequency);
  double controllerToleranceDepth = ::controllerToleranceDepth;
  node.param<double>("controller/tolerance/depth",
    controllerToleranceDepth, controllerToleranceDepth);
  ::controllerToleranceDepth = controllerToleranceDepth;
  double controllerToleranceVelocity = ::controllerToleranceVelocity;
  node.param<double>("controller/tolerance/velocity",
    controllerToleranceVelocity, controllerToleranceVelocity);
  ::controllerToleranceVelocity = controllerToleranceVelocity;
  double controllerGainProportional = ::controllerGainProportional;
  node.param<double>("controller/gain/proportional",
    controllerGainProportional, controllerGainProportional);
  ::controllerGainProportional = controllerGainProportional;
  double controllerGainIntegral = ::controllerGainIntegral;
  node.param<double>("controller/gain/integral",
    controllerGainIntegral, controllerGainIntegral);
  ::controllerGainIntegral = controllerGainIntegral;
  double controllerGainDifferential = ::controllerGainDifferential;
  node.param<double>("controller/gain/differential",
    controllerGainDifferential, controllerGainDifferential);
  ::controllerGainDifferential = controllerGainDifferential;
}

bool getEnabled(GetEnabled::Request& request, GetEnabled::Response& response) {
  response.enabled = controller.enabled;
  return true;
}

bool getGains(GetGains::Request& request, GetGains::Response& response) {
  response.proportional = controllerGainProportional;
  response.differential = controllerGainDifferential;
  response.integral = controllerGainIntegral;

  return true;
}

bool getCommand(GetCommand::Request& request, GetCommand::Response&
    response) {
  response.depth = controller.command.depth;
  response.velocity = controller.command.velocity;
  
  return true;
}

bool getActual(GetActual::Request& request, GetActual::Response& response) {
  response.depth = controller.actual.depth;
  response.velocity = controller.actual.velocity;
  
  return true;
}

bool getError(GetError::Request& request, GetError::Response& response) {
  response.depth = controller.command.depth-controller.actual.depth;
  response.velocity = controller.command.velocity-controller.actual.velocity;
  
  return true;
}

bool setGains(SetGains::Request& request, SetGains::Response& response) {
  controllerGainProportional = request.proportional;
  controllerGainDifferential = request.differential;
  controllerGainIntegral = request.integral;
}

bool setCommand(SetCommand::Request& request, SetCommand::Response&
    response) {
  controller.command.depth = request.depth;
  controller.command.velocity = request.velocity;
  
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
  
  return true;
}

bool emerge(Emerge::Request& request, Emerge::Response& response) {
  controller.enabled = false;
  controller.reset();

  GetLimits getLimits;
  if (!getLimitsClient.call(getLimits))
    return false;
  
  if (!(getLimits.response.limits & actuatorLimitsMinInputChannel)) {
    SetSpeed setSpeed;
  
    float sign = actuatorInverted ? 1.0f : -1.0f;
    setSpeed.request.speed = sign*1.0f;
    setSpeed.request.start = true;
    
    return setSpeedClient.call(setSpeed);
  }

  return true;
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!getLimitsClient || !setSpeedClient || !getDepthClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void updateControl(const ros::TimerEvent& event) {
  if (!controller.enabled)
    return;
  
  GetDepth getDepth;
  GetLimits getLimits;

  if (!getDepthClient.call(getDepth) ||
      (getDepth.response.filtered != getDepth.response.filtered))
    return;

  if (lastTime.isZero()) {
    controller.actual.depth = getDepth.response.filtered;
    lastTime = ros::Time::now();
    
    return;
  }
  
  float dt = (ros::Time::now()-lastTime).toSec();
  lastTime = ros::Time::now();

  controller.actual.velocity = (getDepth.response.filtered-
    controller.actual.depth)/dt;
  controller.actual.depth = getDepth.response.filtered;

  if (!getLimitsClient.call(getLimits))
    return;
  
  /** On-off depth control with tolerances
    */ 
  float commandVelocity = 0.0f;
  if (controller.actual.depth > controller.command.depth+
      controllerToleranceDepth)
    commandVelocity = -controller.command.velocity;
  else if (controller.actual.depth < controller.command.depth-
      controllerToleranceDepth)
    commandVelocity = controller.command.velocity;
  
  float error = commandVelocity-controller.actual.velocity;
  controller.integralError += error*dt;
  float derivativeError = 0.0f;
  if (!(controller.lastError != controller.lastError))
    derivativeError = (error-controller.lastError)/dt;
  controller.lastError = error;
  
  /** PID velocity control with tolerances
    */ 
  float output =
    controllerGainProportional*error+
    controllerGainIntegral*controller.integralError+
    controllerGainDifferential*derivativeError;

  /** Check limits to saturate control output
    */ 
  bool saturate = true;
  bool minLimit = (getLimits.response.limits & actuatorLimitsMinInputChannel);
  bool maxLimit = (getLimits.response.limits & actuatorLimitsMaxInputChannel);
  
  if (fabsf(error) > controllerToleranceVelocity) {
    if (minLimit) {
      if (output > 0.0f)
        saturate = false;
    }
    else if (maxLimit) {
      if (output < 0.0f)
        saturate = false;
    }
    else {
      saturate = false;
    }
  }
  
  if (!saturate) {
    SetSpeed setSpeed;
    setSpeed.request.speed = outputToSpeed(output);
    setSpeed.request.start = true;

    if (setSpeedClient.call(setSpeed))
      diagnoseFrequency->tick();
  }
  else
    diagnoseFrequency->tick();
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (!getLimitsClient)
    getLimitsClient = ros::NodeHandle("~").serviceClient<GetLimits>(
      "/"+smcServerName+"/get_limits", true);
  if (!setSpeedClient)
    setSpeedClient = ros::NodeHandle("~").serviceClient<SetSpeed>(
      "/"+smcServerName+"/set_speed", true);
  if (!getDepthClient)
    getDepthClient = ros::NodeHandle("~").serviceClient<GetDepth>(
      "/"+sensorServerName+"/get_depth", true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dive_controller");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
  diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
    diagnostic_updater::FrequencyStatusParam(&controllerFrequency,
    &controllerFrequency)));
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getEnabledService = node.advertiseService("get_enabled", getEnabled);
  getGainsService = node.advertiseService("get_gains", getGains);
  getCommandService = node.advertiseService("get_command", getCommand);
  getActualService = node.advertiseService("get_actual", getActual);
  getErrorService = node.advertiseService("get_error", getError);
  setGainsService = node.advertiseService("set_gains", setGains);
  setCommandService = node.advertiseService("set_command", setCommand);
  enableService = node.advertiseService("enable", enable);
  disableService = node.advertiseService("disable", disable);
  emergeService = node.advertiseService("emerge", emerge);

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
