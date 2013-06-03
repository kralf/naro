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

#include <Eigen/Dense>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_smc_srvs/GetLimits.h>
#include <naro_smc_srvs/SetSpeed.h>
#include <naro_smc_srvs/Start.h>
#include <naro_usc_srvs/GetInputs.h>

#include "naro_dive_ctrl/GetGains.h"
#include "naro_dive_ctrl/GetCommand.h"
#include "naro_dive_ctrl/Emerge.h"
#include "naro_dive_ctrl/SetGains.h"
#include "naro_dive_ctrl/SetCommand.h"

using namespace Eigen;

using namespace naro_dive_ctrl;
using namespace naro_smc_srvs;
using namespace naro_usc_srvs;

std::string smcServerName = "smc_server";
std::string uscServerName = "usc_server";
float modelGravitationalAcceleration = 9.80665f;  // [m/s^2]
float modelFluidDensity = 1000.0f;                // [kg/m^3]
float modelPlatformMass = 10.0f;                  // [kg]
float modelPlatformArea = 0.1f;                   // [m^2]
float modelPlatformVolume = 0.012f;               // [m^3]
float modelPlatformDragCoefficient = 1.0f;
float filterInitialStateNoiseDepth = 0.02f;
float filterInitialStateNoiseVelocity = 0.01f;
float filterInitialStateNoiseAcceleration = 0.01f;
float filterProcessNoiseDepth = 0.01f;
float filterProcessNoiseVelocity = 0.01f;
float filterProcessNoiseAcceleration = 0.1f;
float filterObservationNoiseDepth = 0.02f;
double controllerFrequency = 5.0;
float controllerGainProportional = 1.0f;
float controllerGainDifferential = 0.0f;
float controllerGainIntegral = 0.0f;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getLimitsClient;
ros::ServiceClient startClient;
ros::ServiceClient setSpeedClient;
ros::ServiceClient getInputsClient;

ros::ServiceServer getGainsService;
ros::ServiceServer getCommandService;
ros::ServiceServer emergeService;
ros::ServiceServer setGainsService;
ros::ServiceServer setCommandService;

/** Experimente:
    Abtauchen auf 3.50m mit vollem Zylinder ca. 24s
    Auftauchen von 3.50 mit leerem Zylinder ca. 17s
  */

class KalmanFilter {
public:
  KalmanFilter() :
    sigma_z(0.0f),
    s_t(0.0f),
    time(0.0) {
  };

  void initialize(float z_0) {
    I = Matrix<float, 3, 3>::Identity();
    h = Matrix<float, 1, 3>::Zero();
    Sigma_x = Matrix<float, 3, 3>::Zero();

    F_t = Matrix<float, 3, 3>::Identity();
    P_t = Matrix<float, 3, 3>::Zero();
    k_t = Matrix<float, 3, 1>::Zero();
    mu_t = Matrix<float, 3, 1>::Zero();
    Sigma_t = Matrix<float, 3, 3>::Zero();

    h(0, 0) = 1.0f;
    Sigma_x(0, 0) = filterProcessNoiseDepth;
    Sigma_x(1, 1) = filterProcessNoiseVelocity;
    Sigma_x(2, 2) = filterProcessNoiseAcceleration;
    sigma_z = filterObservationNoiseDepth;

    mu_t(0, 0) = z_0;
    Sigma_t(0, 0) = filterInitialStateNoiseDepth;
    Sigma_t(1, 1) = filterInitialStateNoiseVelocity;
    Sigma_t(2, 2) = filterInitialStateNoiseAcceleration;

    time = ros::Time::now().toSec();
  };

  void update(float z_t) {
    double time = ros::Time::now().toSec();
    float dt = time-this->time;

    F_t(0, 1) = dt;
    F_t(0, 2) = 0.5f*dt*dt;
    F_t(1, 2) = dt;

    P_t = F_t*Sigma_t*F_t.transpose()+Sigma_x;
    s_t = h*P_t*h.transpose()+sigma_z;
    k_t = P_t*h.transpose()*1.0f/s_t;
    mu_t = F_t*mu_t+k_t*(z_t-h*F_t*mu_t);
    Sigma_t = (I-k_t*h)*P_t;

    this->time = time;
  };

  Matrix<float, 3, 3> I;
  Matrix<float, 1, 3> h;
  Matrix<float, 3, 3> Sigma_x;
  float sigma_z;

  Matrix<float, 3, 3> F_t;
  Matrix<float, 3, 3> P_t;
  Matrix<float, 3, 1> k_t;
  Matrix<float, 3, 1> mu_t;
  Matrix<float, 3, 3> Sigma_t;
  float s_t;

  double time;
};

float command = 0.0f;

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

inline float velocityToAcceleration(float velocity) {
  return velocityToForce(velocity)/modelPlatformMass;
}

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/smc/name", smcServerName, smcServerName);
  node.param<std::string>("server/usc/name", uscServerName, uscServerName);

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

  double filterInitialStateNoiseDepth = ::filterInitialStateNoiseDepth;
  node.param<double>("filter/initial_state/noise/depth",
    filterInitialStateNoiseDepth, filterInitialStateNoiseDepth);
  ::filterInitialStateNoiseDepth = filterInitialStateNoiseDepth;
  double filterInitialStateNoiseVelocity = ::filterInitialStateNoiseVelocity;
  node.param<double>("filter/initial_state/noise/velocity",
    filterInitialStateNoiseVelocity, filterInitialStateNoiseVelocity);
  ::filterInitialStateNoiseVelocity = filterInitialStateNoiseVelocity;
  double filterInitialStateNoiseAcceleration =
    ::filterInitialStateNoiseAcceleration;
  node.param<double>("filter/initial_state/noise/acceleration",
    filterInitialStateNoiseAcceleration, filterInitialStateNoiseAcceleration);
  ::filterInitialStateNoiseAcceleration = filterInitialStateNoiseAcceleration;
  double filterProcessNoiseDepth = ::filterProcessNoiseDepth;
  node.param<double>("filter/process/noise/depth",
    filterProcessNoiseDepth, filterProcessNoiseDepth);
  ::filterProcessNoiseDepth = filterProcessNoiseDepth;
  double filterProcessNoiseVelocity = ::filterProcessNoiseVelocity;
  node.param<double>("filter/process/noise/velocity",
    filterProcessNoiseVelocity, filterProcessNoiseVelocity);
  ::filterProcessNoiseVelocity = filterProcessNoiseVelocity;
  double filterProcessNoiseAcceleration = ::filterProcessNoiseAcceleration;
  node.param<double>("filter/process/noise/acceleration",
    filterProcessNoiseAcceleration, filterProcessNoiseAcceleration);
  ::filterProcessNoiseAcceleration = filterProcessNoiseAcceleration;
  double filterObservationNoiseDepth = ::filterObservationNoiseDepth;
  node.param<double>("filter/observation/noise/depth",
    filterObservationNoiseDepth, filterObservationNoiseDepth);
  ::filterObservationNoiseDepth = filterObservationNoiseDepth;

  node.param<double>("controller/frequency", controllerFrequency,
    controllerFrequency);
  double controllerGainProportional = ::controllerGainProportional;
  node.param<double>("controller/gain/proportional",
    controllerGainProportional, controllerGainProportional);
  ::controllerGainProportional = controllerGainProportional;
  double controllerGainDifferential = ::controllerGainDifferential;
  node.param<double>("controller/gain/differential",
    controllerGainDifferential, controllerGainDifferential);
  ::controllerGainDifferential = controllerGainDifferential;
  double controllerGainIntegral = ::controllerGainIntegral;
  node.param<double>("controller/gain/integral",
    controllerGainIntegral, controllerGainIntegral);
  ::controllerGainIntegral = controllerGainIntegral;
}

bool getGains(GetGains::Request& request, GetGains::Response& response) {
  response.proportional = controllerGainProportional;
  response.differential = controllerGainDifferential;
  response.integral = controllerGainIntegral;

  return true;
}

bool getCommand(GetCommand::Request& request, GetCommand::Response&
    response) {
  response.depth = command;
  return true;
}

bool emerge(Emerge::Request& request, Emerge::Response& response) {
  command = 0.0;
  return true;
}

bool setGains(SetGains::Request& request, SetGains::Response& response) {
  controllerGainProportional = request.proportional;
  controllerGainDifferential = request.differential;
  controllerGainIntegral = request.integral;

  return true;
}

bool setCommand(SetCommand::Request& request, SetCommand::Response&
    response) {
  command = request.depth;
  return true;
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

  getGainsService = node.advertiseService("get_gains", getGains);
  getCommandService = node.advertiseService("get_command", getCommand);
  emergeService = node.advertiseService("emerge", emerge);
  setGainsService = node.advertiseService("set_gains", setGains);
  setCommandService = node.advertiseService("set_command", setCommand);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer controllerTimer = node.createTimer(
    ros::Duration(1.0/controllerFrequency), updateControl);

  ros::spin();

  return 0;
}
