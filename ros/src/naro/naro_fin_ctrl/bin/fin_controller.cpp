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
#include <limits>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_usc_srvs/GetChannels.h>
#include <naro_usc_srvs/GetPositions.h>
#include <naro_usc_srvs/SetProfiles.h>

#include "naro_fin_ctrl/GetServos.h"
#include "naro_fin_ctrl/GetHomes.h"
#include "naro_fin_ctrl/GetGains.h"
#include "naro_fin_ctrl/GetFrequencies.h"
#include "naro_fin_ctrl/GetAmplitudes.h"
#include "naro_fin_ctrl/GetPhases.h"
#include "naro_fin_ctrl/GetOffsets.h"
#include "naro_fin_ctrl/GetCommands.h"
#include "naro_fin_ctrl/GetActuals.h"
#include "naro_fin_ctrl/SetHomes.h"
#include "naro_fin_ctrl/SetGains.h"
#include "naro_fin_ctrl/SetFrequencies.h"
#include "naro_fin_ctrl/SetAmplitudes.h"
#include "naro_fin_ctrl/SetPhases.h"
#include "naro_fin_ctrl/SetOffsets.h"
#include "naro_fin_ctrl/SetCommands.h"

using namespace naro_fin_ctrl;
using namespace naro_usc_srvs;

std::string uscServerName = "usc_server";
double connectionRetry = 0.1;
int controllerMaxServos = 8;
double controllerFrequency = 50.0;
float controllerFrequencyGain = 0.9f;
float controllerAmplitudeGain = 0.9f;
float controllerPhaseGain = 0.9f;
float controllerOffsetGain = 0.9f;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient getChannelsClient;
ros::ServiceClient getPositionsClient;
ros::ServiceClient setProfilesClient;

ros::ServiceServer getServosService;
ros::ServiceServer getHomesService;
ros::ServiceServer getGainsService;
ros::ServiceServer getFrequenciesService;
ros::ServiceServer getAmplitudesService;
ros::ServiceServer getPhasesService;
ros::ServiceServer getOffsetsService;
ros::ServiceServer getCommandsService;
ros::ServiceServer getActualsService;
ros::ServiceServer setHomesService;
ros::ServiceServer setGainsService;
ros::ServiceServer setFrequenciesService;
ros::ServiceServer setAmplitudesService;
ros::ServiceServer setPhasesService;
ros::ServiceServer setOffsetsService;
ros::ServiceServer setCommandsService;

const float pi2 = M_PI*2.0f;

class Servo {
public:
  class Parameters {
  public:
    Parameters(float frequency = 0.0f, float amplitude = 0.0f, float
        offset = 0.0f, float phase = 0.0f) :
      frequency(frequency),
      amplitude(amplitude),
      phase(phase),
      offset(offset) {
    };

    float frequency;
    float amplitude;
    float phase;
    float offset;
  };

  Servo(int channel = -1) :
    channel(channel),
    home(0.0f) {
  };

  int channel;

  float home;
  Parameters gain;
  Parameters command;
  Parameters actual;
};

std::vector<Servo> servos;
ros::Time timeOffset;
ros::Time lastTime;

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/usc/name", uscServerName, uscServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);

  node.param<int>("controller/max_servos", controllerMaxServos,
    controllerMaxServos);
  node.param<double>("controller/frequency", controllerFrequency,
    controllerFrequency);
  double controllerFrequencyGain = ::controllerFrequencyGain;
  node.param<double>("controller/gain/frequency", controllerFrequencyGain,
    controllerFrequencyGain);
  ::controllerFrequencyGain = controllerFrequencyGain;
  double controllerAmplitudeGain = ::controllerAmplitudeGain;
  node.param<double>("controller/gain/amplitude", controllerAmplitudeGain,
    controllerAmplitudeGain);
  ::controllerAmplitudeGain = controllerAmplitudeGain;
  double controllerPhaseGain = ::controllerPhaseGain;
  node.param<double>("controller/gain/phase", controllerPhaseGain,
    controllerPhaseGain);
  ::controllerPhaseGain = controllerPhaseGain;
  double controllerOffsetGain = ::controllerOffsetGain;
  node.param<double>("controller/gain/offset", controllerOffsetGain,
    controllerOffsetGain);
  ::controllerOffsetGain = controllerOffsetGain;
}

void initializeServos() {
  servos.clear();

  getChannelsClient = ros::NodeHandle("~").serviceClient<GetChannels>(
    "/"+uscServerName+"/get_channels");
  GetChannels getChannels;

  if (getChannelsClient.call(getChannels)) {
    for (int i = 0; (i < getChannels.response.mode.size()) &&
        (servos.size() < controllerMaxServos); ++i)
      if (getChannels.response.mode[i] == GetChannels::Response::SERVO) {
      servos.push_back(i);

      servos.back().gain.frequency = controllerFrequencyGain;
      servos.back().gain.amplitude = controllerAmplitudeGain;
      servos.back().gain.phase = controllerPhaseGain;
      servos.back().gain.offset = controllerOffsetGain;
    }

    ROS_INFO("USC server reported %d available servo(s).",
      (unsigned int)servos.size());
  }
  else
    ROS_FATAL("No servos available: GetChannels request failed.");
}

bool getServos(GetServos::Request& request, GetServos::Response& response) {
  response.servos = servos.size();
  return true;
}

bool getHomes(GetHomes::Request& request, GetHomes::Response& response) {
  response.home.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      response.home[i] = servos[request.servos[i]].home;
    else
      response.home[i] = std::numeric_limits<float>::quiet_NaN();
  }

  return true;
}

bool getGains(GetGains::Request& request, GetGains::Response& response) {
  response.frequency.resize(request.servos.size());
  response.amplitude.resize(request.servos.size());
  response.phase.resize(request.servos.size());
  response.offset.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.frequency[i] = servos[request.servos[i]].gain.frequency;
      response.amplitude[i] = servos[request.servos[i]].gain.amplitude;
      response.phase[i] = servos[request.servos[i]].gain.phase;
      response.offset[i] = servos[request.servos[i]].gain.offset;
    }
    else {
      response.frequency[i] = std::numeric_limits<float>::quiet_NaN();
      response.amplitude[i] = std::numeric_limits<float>::quiet_NaN();
      response.phase[i] = std::numeric_limits<float>::quiet_NaN();
      response.offset[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getFrequencies(GetFrequencies::Request& request,
    GetFrequencies::Response& response) {
  response.command.resize(request.servos.size());
  response.actual.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.command[i] = servos[request.servos[i]].command.frequency;
      response.actual[i] = servos[request.servos[i]].actual.frequency;
    }
    else {
      response.command[i] = std::numeric_limits<float>::quiet_NaN();
      response.actual[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getAmplitudes(GetAmplitudes::Request& request, GetAmplitudes::Response&
    response) {
  response.command.resize(request.servos.size());
  response.actual.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.command[i] = servos[request.servos[i]].command.amplitude;
      response.actual[i] = servos[request.servos[i]].actual.amplitude;
    }
    else {
      response.command[i] = std::numeric_limits<float>::quiet_NaN();
      response.actual[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getPhases(GetPhases::Request& request, GetPhases::Response& response) {
  response.command.resize(request.servos.size());
  response.actual.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.command[i] = servos[request.servos[i]].command.phase;
      response.actual[i] = servos[request.servos[i]].actual.phase;
    }
    else {
      response.command[i] = std::numeric_limits<float>::quiet_NaN();
      response.actual[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getOffsets(GetOffsets::Request& request, GetOffsets::Response& response) {
  response.command.resize(request.servos.size());
  response.actual.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.command[i] = servos[request.servos[i]].command.offset;
      response.actual[i] = servos[request.servos[i]].actual.offset;
    }
    else {
      response.command[i] = std::numeric_limits<float>::quiet_NaN();
      response.actual[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getCommands(GetCommands::Request& request, GetCommands::Response&
    response) {
  response.frequency.resize(request.servos.size());
  response.amplitude.resize(request.servos.size());
  response.phase.resize(request.servos.size());
  response.offset.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.frequency[i] = servos[request.servos[i]].command.frequency;
      response.amplitude[i] = servos[request.servos[i]].command.amplitude;
      response.phase[i] = servos[request.servos[i]].command.phase;
      response.offset[i] = servos[request.servos[i]].command.offset;
    }
    else {
      response.frequency[i] = std::numeric_limits<float>::quiet_NaN();
      response.amplitude[i] = std::numeric_limits<float>::quiet_NaN();
      response.phase[i] = std::numeric_limits<float>::quiet_NaN();
      response.offset[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool getActuals(GetActuals::Request& request, GetActuals::Response&
    response) {
  response.frequency.resize(request.servos.size());
  response.amplitude.resize(request.servos.size());
  response.phase.resize(request.servos.size());
  response.offset.resize(request.servos.size());

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      response.frequency[i] = servos[request.servos[i]].actual.frequency;
      response.amplitude[i] = servos[request.servos[i]].actual.amplitude;
      response.phase[i] = servos[request.servos[i]].actual.phase;
      response.offset[i] = servos[request.servos[i]].actual.offset;
    }
    else {
      response.frequency[i] = std::numeric_limits<float>::quiet_NaN();
      response.amplitude[i] = std::numeric_limits<float>::quiet_NaN();
      response.phase[i] = std::numeric_limits<float>::quiet_NaN();
      response.offset[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }

  return true;
}

bool setHomes(SetHomes::Request& request, SetHomes::Response& response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      servos[request.servos[i]].home = request.home[i];
    else {
      ROS_WARN("SetHomes request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setGains(SetGains::Request& request, SetGains::Response& response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      servos[request.servos[i]].gain.frequency = request.frequency[i];
      servos[request.servos[i]].gain.amplitude = request.amplitude[i];
      servos[request.servos[i]].gain.phase = request.phase[i];
      servos[request.servos[i]].gain.offset = request.offset[i];
    }
    else {
      ROS_WARN("SetGains request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setFrequencies(SetFrequencies::Request& request,
    SetFrequencies::Response& response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      servos[request.servos[i]].command.frequency = request.command[i];
    else {
      ROS_WARN("SetFrequencies request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setAmplitudes(SetAmplitudes::Request& request, SetAmplitudes::Response&
    response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      servos[request.servos[i]].command.amplitude = request.command[i];
    else {
      ROS_WARN("SetAmplitudes request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setPhases(SetPhases::Request& request, SetPhases::Response& response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      servos[request.servos[i]].command.phase = request.command[i];
    else {
      ROS_WARN("SetPhases request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setOffsets(SetOffsets::Request& request, SetOffsets::Response& response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size())
      servos[request.servos[i]].command.offset = request.command[i];
    else {
      ROS_WARN("SetOffsets request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

bool setCommands(SetCommands::Request& request, SetCommands::Response&
    response) {
  bool result = true;

  for (int i = 0; i < request.servos.size(); ++i) {
    if (request.servos[i] < servos.size()) {
      servos[request.servos[i]].command.frequency = request.frequency[i];
      servos[request.servos[i]].command.amplitude = request.amplitude[i];
      servos[request.servos[i]].command.phase = request.phase[i];
      servos[request.servos[i]].command.offset = request.offset[i];
    }
    else {
      ROS_WARN("SetCommands request failed: Servo %d does not exist.",
        request.servos[i]);
      result = false;
    }
  }

  return result;
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!getPositionsClient || !setProfilesClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void diagnoseServos(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!servos.empty())
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "%d servos available.", (unsigned int)servos.size());
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "No servos available.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (servos.empty())
    initializeServos();

  if (!getPositionsClient)
    getPositionsClient = ros::NodeHandle("~").serviceClient<GetPositions>(
      "/"+uscServerName+"/get_positions", true);
  if (!setProfilesClient)
    setProfilesClient = ros::NodeHandle("~").serviceClient<SetProfiles>(
      "/"+uscServerName+"/set_profiles", true);
}

void updateControl(const ros::TimerEvent& event) {
  SetProfiles setProfiles;
  setProfiles.request.channels.resize(servos.size());
  setProfiles.request.position.resize(servos.size());
  setProfiles.request.speed.resize(servos.size());
  setProfiles.request.acceleration.resize(servos.size());

  float dt = 0.0f;
  if (!lastTime.isZero())
    dt = (ros::Time::now()-lastTime).toSec();
  lastTime = ros::Time::now();

  float t_0 = (lastTime-timeOffset).toSec();
  float t_2 = t_0+2.0f/controllerFrequency;

  for (int i = 0; i < servos.size(); ++i) {
    if (dt > 0.0f) {
      servos[i].actual.frequency += servos[i].gain.frequency*dt*
        (servos[i].command.frequency-servos[i].actual.frequency);
      servos[i].actual.amplitude += servos[i].gain.amplitude*dt*
        (servos[i].command.amplitude-servos[i].actual.amplitude);
      servos[i].actual.phase += servos[i].gain.phase*dt*
        (servos[i].command.phase-servos[i].actual.phase);
      servos[i].actual.offset += servos[i].gain.offset*dt*
        (servos[i].command.offset-servos[i].actual.offset);
    }

    float omega_i = pi2*servos[i].actual.frequency;
    setProfiles.request.channels[i] = servos[i].channel;
    setProfiles.request.position[i] = servos[i].home+
      servos[i].actual.offset+servos[i].actual.amplitude*
      sin(omega_i*t_2+servos[i].actual.phase);
    setProfiles.request.speed[i] = omega_i*servos[i].actual.amplitude*
      cos(omega_i*t_0+servos[i].actual.phase);
    setProfiles.request.acceleration[i] =
      std::numeric_limits<float>::infinity();
  }

  if (setProfilesClient.call(setProfiles))
    diagnoseFrequency->tick();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fin_controller");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
  updater->add("Servos", diagnoseServos);
  diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
    diagnostic_updater::FrequencyStatusParam(&controllerFrequency,
    &controllerFrequency)));
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getServosService = node.advertiseService("get_servos", getServos);
  getHomesService = node.advertiseService("get_homes", getHomes);
  getGainsService = node.advertiseService("get_gains", getGains);
  getFrequenciesService = node.advertiseService("get_frequencies",
    getFrequencies);
  getAmplitudesService = node.advertiseService("get_amplitudes",
    getAmplitudes);
  getPhasesService = node.advertiseService("get_phases", getPhases);
  getOffsetsService = node.advertiseService("get_offsets", getOffsets);
  getCommandsService = node.advertiseService("get_commands", getCommands);
  getActualsService = node.advertiseService("get_actuals", getActuals);
  setHomesService = node.advertiseService("set_homes", setHomes);
  setGainsService = node.advertiseService("set_gains", setGains);
  setFrequenciesService = node.advertiseService("set_frequencies",
    setFrequencies);
  setAmplitudesService = node.advertiseService("set_amplitudes",
    setAmplitudes);
  setPhasesService = node.advertiseService("set_phases", setPhases);
  setOffsetsService = node.advertiseService("set_offsets", setOffsets);
  setCommandsService = node.advertiseService("set_commands", setCommands);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);
  ros::Timer controllerTimer = node.createTimer(
    ros::Duration(1.0/controllerFrequency), updateControl);

  initializeServos();
  tryConnect();
  timeOffset = ros::Time::now();

  ros::spin();

  return 0;
}
