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
#include <vector>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <naro_fin_ctrl/SetCommands.h>

#include "naro_cmd_srvs/GetOutputs.h"
#include "naro_cmd_srvs/GetCoefficient.h"
#include "naro_cmd_srvs/GetCoefficients.h"
#include "naro_cmd_srvs/GetOutputChannel.h"
#include "naro_cmd_srvs/GetOutputChannels.h"
#include "naro_cmd_srvs/GetActuator.h"
#include "naro_cmd_srvs/GetActuators.h"
#include "naro_cmd_srvs/GetFin.h"
#include "naro_cmd_srvs/GetFins.h"
#include "naro_cmd_srvs/SetCoefficient.h"
#include "naro_cmd_srvs/SetCoefficients.h"
#include "naro_cmd_srvs/SetOutputChannel.h"
#include "naro_cmd_srvs/SetOutputChannels.h"
#include "naro_cmd_srvs/SetActuator.h"
#include "naro_cmd_srvs/SetActuators.h"
#include "naro_cmd_srvs/SetFin.h"
#include "naro_cmd_srvs/SetFins.h"
#include "naro_cmd_srvs/AddFin.h"
#include "naro_cmd_srvs/RemoveFin.h"
#include "naro_cmd_srvs/Connect.h"
#include "naro_cmd_srvs/Disconnect.h"

#include "sensor_msgs/Joy.h"

using namespace naro_cmd_srvs;
using namespace naro_fin_ctrl;
using namespace sensor_msgs;

std::string finServerName = "fin_controller";
double connectionRetry = 0.1;
std::string subscriberTopic = "joy";
int subscriberQueueSize = 1;
double subscriberFrequency = 1.0;

boost::shared_ptr<diagnostic_updater::Updater> updater;
boost::shared_ptr<diagnostic_updater::FrequencyStatus> diagnoseFrequency;

ros::ServiceClient setCommandsClient;

ros::ServiceServer getOutputsService;
ros::ServiceServer getCoefficientService;
ros::ServiceServer getCoefficientsService;
ros::ServiceServer getOutputChannelService;
ros::ServiceServer getOutputChannelsService;
ros::ServiceServer getActuatorService;
ros::ServiceServer getActuatorsService;
ros::ServiceServer getFinService;
ros::ServiceServer getFinsService;
ros::ServiceServer setCoefficientService;
ros::ServiceServer setCoefficientsService;
ros::ServiceServer setOutputChannelService;
ros::ServiceServer setOutputChannelsService;
ros::ServiceServer setActuatorService;
ros::ServiceServer setActuatorsService;
ros::ServiceServer setFinService;
ros::ServiceServer setFinsService;
ros::ServiceServer addFinService;
ros::ServiceServer removeFinService;
ros::ServiceServer connectService;
ros::ServiceServer disconnectService;

ros::Subscriber subscriber;

class _Fin {
public:
  class Actuator {
  public:
    class OutputChannel {
    public:
      class TransferFunction {
      public:
        enum Type {
          identity,
          ramp,
          slope,
          step,
          absolute,
          square,
          exponential
        };
        
        TransferFunction(Type type = identity, bool invertArguments = false,
            bool invertValues = false) :
          type(type),
          invertArguments(invertArguments),
          invertValues(invertValues) {
        };
        
        inline float operator()(float x) const {
          if (invertArguments)
            x = -x;
          
          float y = 0.0f;
          if (type == ramp)
            y = (x > 0.0f) ? x : 0.0f;
          else if (type == slope)
            y = (x > 0.0f) ? 1.0f-x : 1.0f;
          else if (type == step)
            y = (x > 0.0f) ? 1.0f : 0.0f;
          else if (type == absolute)
            y = fabs(x);
          else if (type == square)
            y = x*x;
          else if (type == exponential)
            y = (x > 0.0f) ? x*exp(x-1.0f) : 0.0f;
          else
            y = x;
            
          if (invertValues)
            return -y;
          else
            return y;
        };
        
        Type type;
        
        bool invertArguments;
        bool invertValues;
      };

      OutputChannel(float constant = 0.0f) :
        constant(constant) {
      };

      inline float operator()(const std::vector<float>& inputs) const {
        float output = constant;
        
        for (int i = 0; i < coefficients.size(); ++i)
          output *= coefficients[i](inputs[inputChannels[i]]);
        
        return output;
      };
      
      inline void connect(int inputChannel, const TransferFunction&
          coefficient = TransferFunction()) {
        inputChannels.push_back(inputChannel);
        coefficients.push_back(coefficient);
      };
      
      inline void disconnect(int inputChannel) {
        std::vector<int>::iterator it = std::find(inputChannels.begin(),
          inputChannels.end(), inputChannel);
        
        if (it != inputChannels.end()) {
          coefficients.erase(coefficients.begin()+*it);
          inputChannels.erase(it);
        }
      };
      
      float constant;
      std::vector<TransferFunction> coefficients;
      
      std::vector<int> inputChannels;
    };

    class Commands {
    public:
      Commands(float frequency = 0.0f, float amplitude = 0.0f, float
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
    
    Actuator(int servo = -1) :
      servo(servo) {
    };

    inline Commands operator()(const std::vector<float>& inputs) const {
      Commands commands;
      
      if (servo >= 0) {
        commands.frequency = frequency(inputs);
        commands.amplitude = amplitude(inputs);
        commands.phase = phase(inputs);
        commands.offset = offset(inputs);
      }

      return commands;
    };
    
    OutputChannel& getOutputChannel(int id) {
      if (id == ::OutputChannel::AMPLITUDE)
        return amplitude;
      else if (id == ::OutputChannel::PHASE)
        return phase;
      else if (id == ::OutputChannel::OFFSET)
        return offset;
      else
        return frequency;
    };
    
    const OutputChannel& getOutputChannel(int id) const {
      if (id == ::OutputChannel::AMPLITUDE)
        return amplitude;
      else if (id == ::OutputChannel::PHASE)
        return phase;
      else if (id == ::OutputChannel::OFFSET)
        return offset;
      else
        return frequency;
    };
    
    int servo;
    
    OutputChannel frequency;
    OutputChannel amplitude;
    OutputChannel phase;
    OutputChannel offset;
  };
  
  _Fin() {
  };
  
  Actuator& getActuator(int id) {
    if (id == ::Actuator::FLAP)
      return flap;
    else
      return pitch;
  };
  
  const Actuator& getActuator(int id) const {
    if (id == ::Actuator::FLAP)
      return flap;
    else
      return pitch;
  };
  
  Actuator pitch;
  Actuator flap;
};

std::vector<_Fin> fins;

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("server/fin/name", finServerName, finServerName);
  node.param<double>("server/connection/retry", connectionRetry,
    connectionRetry);
  
  node.param<std::string>("subscriber/topic", subscriberTopic,
    subscriberTopic);
  node.param<int>("subscriber/queue_size", subscriberQueueSize,
    subscriberQueueSize);
  node.param<double>("subscriber/frequency", subscriberFrequency,
    subscriberFrequency);
}

bool getOutputs(GetOutputs::Request& request, GetOutputs::Response& response) {
  size_t numServos = 0;
  for (int i = 0; i < fins.size(); ++i) {
    numServos += (fins[i].pitch.servo >= 0);
    numServos += (fins[i].flap.servo >= 0);
  }
  
  int j = 0;
  response.outputs.resize(numServos);
  for (int i = 0; i < fins.size(); ++i) {
    if (fins[i].pitch.servo >= 0) {
      _Fin::Actuator::Commands commands = fins[i].pitch(request.inputs);
      
      response.outputs[j].servo = fins[i].pitch.servo;
      response.outputs[j].frequency = commands.frequency;
      response.outputs[j].amplitude = commands.amplitude;
      response.outputs[j].phase = commands.phase;
      response.outputs[j].offset = commands.offset;
      
      ++j;
    }

    if (fins[i].flap.servo >= 0) {
      _Fin::Actuator::Commands commands = fins[i].flap(request.inputs);
      
      response.outputs[j].servo = fins[i].flap.servo;
      response.outputs[j].frequency = commands.frequency;
      response.outputs[j].amplitude = commands.amplitude;
      response.outputs[j].phase = commands.phase;
      response.outputs[j].offset = commands.offset;
      
      ++j;
    }    
  }
  
  return true;
}

bool getCoefficient(GetCoefficient::Request& request,
    GetCoefficient::Response& response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetCoefficient request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "GetCoefficient request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN(
      "GetCoefficient request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }
  const _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);
  if ((request.id < 0) || (request.id >= outputChannel.coefficients.size())) {
    ROS_WARN(
      "GetCoefficient request failed: Coefficient %d does not exist "
      "for fin %d, actuator %d, output channel %d.", request.id, request.fin,
      request.actuator, request.output_channel);
    return false;
  }

  response.coefficient.input_channel =
    outputChannel.inputChannels[request.id];
  response.coefficient.transfer_function = 
    outputChannel.coefficients[request.id].type;
  response.coefficient.invert_arguments = 
    outputChannel.coefficients[request.id].invertArguments;
  response.coefficient.invert_values = 
    outputChannel.coefficients[request.id].invertValues;
  
  return true;
}

bool getCoefficients(GetCoefficients::Request& request,
    GetCoefficients::Response& response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetCoefficients request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "GetCoefficients request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN(
      "GetCoefficients request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }

  const _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);
  response.coefficients.resize(outputChannel.coefficients.size());
  for (int i = 0; i < outputChannel.coefficients.size(); ++i) {
    GetCoefficient::Request getCoefficientRequest;
    GetCoefficient::Response getCoefficientResponse;
    
    getCoefficientRequest.fin = request.fin;
    getCoefficientRequest.actuator = request.actuator;
    getCoefficientRequest.output_channel = request.output_channel;
    getCoefficientRequest.id = i;
    result |= getCoefficient(getCoefficientRequest, getCoefficientResponse);
    response.coefficients[i] = getCoefficientResponse.coefficient;
  }
  
  return result;
}

bool getOutputChannel(GetOutputChannel::Request& request,
    GetOutputChannel::Response& response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetOutputChannel request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "GetOutputChannel request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.id < OutputChannel::FREQUENCY) ||
      (request.id > OutputChannel::OFFSET)) {
    ROS_WARN(
      "GetOutputChannel request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.id, request.fin, request.actuator);
    return false;
  }

  const _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.id);
  GetCoefficients::Request getCoefficientsRequest;
  GetCoefficients::Response getCoefficientsResponse;
  
  response.output_channel.constant = outputChannel.constant;
  getCoefficientsRequest.fin = request.fin;
  getCoefficientsRequest.actuator = request.actuator;
  getCoefficientsRequest.output_channel = request.id;
  bool result = getCoefficients(getCoefficientsRequest,
    getCoefficientsResponse);
  response.output_channel.coefficients = getCoefficientsResponse.coefficients;
  
  return result;
}

bool getOutputChannels(GetOutputChannels::Request& request,
    GetOutputChannels::Response& response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetOutputChannels request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "GetOutputChannels request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }

  GetOutputChannel::Request getOutputChannelRequest;
  GetOutputChannel::Response getOutputChannelResponse;
  
  getOutputChannelRequest.fin = request.fin;
  getOutputChannelRequest.actuator = request.actuator;
  getOutputChannelRequest.id = OutputChannel::FREQUENCY;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.frequency = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::AMPLITUDE;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.amplitude = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::OFFSET;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.offset = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::PHASE;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.phase = getOutputChannelResponse.output_channel;
  
  return result;
}

bool getActuator(GetActuator::Request& request, GetActuator::Response&
    response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetActuator request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.id < Actuator::PITCH) || (request.id > Actuator::FLAP)) {
    ROS_WARN(
      "GetActuator request failed: Actuator %d does not exist for fin %d.",
      request.id, request.fin);
    return false;
  }

  const _Fin::Actuator& actuator = fins[request.fin].getActuator(request.id);
  GetOutputChannel::Request getOutputChannelRequest;
  GetOutputChannel::Response getOutputChannelResponse;

  response.actuator.servo = actuator.servo;
  getOutputChannelRequest.fin = request.fin;
  getOutputChannelRequest.actuator = request.id;
  getOutputChannelRequest.id = OutputChannel::FREQUENCY;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.actuator.frequency = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::AMPLITUDE;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.actuator.amplitude = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::PHASE;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.actuator.phase = getOutputChannelResponse.output_channel;
  getOutputChannelRequest.id = OutputChannel::OFFSET;
  result |= getOutputChannel(getOutputChannelRequest,
    getOutputChannelResponse);
  response.actuator.offset = getOutputChannelResponse.output_channel;
  
  return result;
}

bool getActuators(GetActuators::Request& request, GetActuators::Response&
    response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("GetActuators request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }

  GetActuator::Request getActuatorRequest;
  GetActuator::Response getActuatorResponse;
  
  getActuatorRequest.fin = request.fin;
  getActuatorRequest.id = Actuator::PITCH;
  result |= getActuator(getActuatorRequest, getActuatorResponse);
  response.pitch = getActuatorResponse.actuator;
  getActuatorRequest.id = Actuator::FLAP;
  result |= getActuator(getActuatorRequest, getActuatorResponse);
  response.flap = getActuatorResponse.actuator;
  
  return result;
}

bool getFin(GetFin::Request& request, GetFin::Response& response) {
  if ((request.id < 0) || (request.id >= fins.size())) {
    ROS_WARN("GetFin request failed: Fin %d does not exist.", request.id);
    return false;
  }
  
  GetActuators::Request getActuatorsRequest;
  GetActuators::Response getActuatorsResponse;
  
  getActuatorsRequest.fin = request.id;
  bool result = getActuators(getActuatorsRequest, getActuatorsResponse);
  response.fin.pitch = getActuatorsResponse.pitch;
  response.fin.flap = getActuatorsResponse.flap;
  
  return result;
}

bool getFins(GetFins::Request& request, GetFins::Response& response) {
  bool result = true;
  
  response.fins.resize(fins.size());

  for (int i = 0; i < fins.size(); ++i) {
    GetFin::Request getFinRequest;
    GetFin::Response getFinResponse;

    getFinRequest.id = i;
    result |= getFin(getFinRequest, getFinResponse);    
    response.fins[i] = getFinResponse.fin;
  }
  
  return result;
}

bool setCoefficient(SetCoefficient::Request& request,
    SetCoefficient::Response& response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetCoefficient request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "SetCoefficient request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN(
      "SetCoefficient request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }
  _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);
  if ((request.id < 0) || (request.id >= outputChannel.coefficients.size())) {
    ROS_WARN(
      "SetCoefficient request failed: Coefficient %d does not exist "
      "for fin %d, actuator %d, output channel %d.", request.id, request.fin,
      request.actuator, request.output_channel);
    return false;
  }

  outputChannel.inputChannels[request.id] =
    request.coefficient.input_channel;
  outputChannel.coefficients[request.id].type =
    (_Fin::Actuator::OutputChannel::TransferFunction::Type)
    request.coefficient.transfer_function;
  outputChannel.coefficients[request.id].invertArguments =
    request.coefficient.invert_arguments;
  outputChannel.coefficients[request.id].invertValues =
    request.coefficient.invert_values;
  
  return true;
}

bool setCoefficients(SetCoefficients::Request& request,
    SetCoefficients::Response& response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetCoefficients request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "SetCoefficients request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN(
      "SetCoefficients request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }

  _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);
  outputChannel.coefficients.resize(request.coefficients.size());
  outputChannel.inputChannels.resize(request.coefficients.size());
  for (int i = 0; i < request.coefficients.size(); ++i) {
    SetCoefficient::Request setCoefficientRequest;
    SetCoefficient::Response setCoefficientResponse;
    
    setCoefficientRequest.fin = request.fin;
    setCoefficientRequest.actuator = request.actuator;
    setCoefficientRequest.output_channel = request.output_channel;
    setCoefficientRequest.id = i;
    setCoefficientRequest.coefficient = request.coefficients[i];
    result |= setCoefficient(setCoefficientRequest, setCoefficientResponse);
  }
  
  return result;
}

bool setOutputChannel(SetOutputChannel::Request& request,
    SetOutputChannel::Response& response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetOutputChannel request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "SetOutputChannel request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.id < OutputChannel::FREQUENCY) ||
      (request.id > OutputChannel::OFFSET)) {
    ROS_WARN(
      "SetOutputChannel request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.id, request.fin, request.actuator);
    return false;
  }

  _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.id);
  SetCoefficients::Request setCoefficientsRequest;
  SetCoefficients::Response setCoefficientsResponse;
  
  outputChannel.constant = request.output_channel.constant;
  setCoefficientsRequest.fin = request.fin;
  setCoefficientsRequest.actuator = request.actuator;
  setCoefficientsRequest.output_channel = request.id;
  setCoefficientsRequest.coefficients = request.output_channel.coefficients;
  bool result = setCoefficients(setCoefficientsRequest,
    setCoefficientsResponse);
  
  return result;
}

bool setOutputChannels(SetOutputChannels::Request& request,
    SetOutputChannels::Response& response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetOutputChannels request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN(
      "SetOutputChannels request failed: Actuator %d does not exist "
      "for fin %d.", request.actuator, request.fin);
    return false;
  }

  SetOutputChannel::Request setOutputChannelRequest;
  SetOutputChannel::Response setOutputChannelResponse;
  
  setOutputChannelRequest.fin = request.fin;
  setOutputChannelRequest.actuator = request.actuator;
  setOutputChannelRequest.id = OutputChannel::FREQUENCY;
  setOutputChannelRequest.output_channel = request.frequency;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::AMPLITUDE;
  setOutputChannelRequest.output_channel = request.amplitude;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::OFFSET;
  setOutputChannelRequest.output_channel = request.offset;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::PHASE;
  setOutputChannelRequest.output_channel = request.phase;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  
  return result;
}

bool setActuator(SetActuator::Request& request, SetActuator::Response&
    response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetActuator request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  if ((request.id < Actuator::PITCH) || (request.id > Actuator::FLAP)) {
    ROS_WARN(
      "SetActuator request failed: Actuator %d does not exist for fin %d.",
      request.id, request.fin);
    return false;
  }

  _Fin::Actuator& actuator = fins[request.fin].getActuator(request.id);
  SetOutputChannel::Request setOutputChannelRequest;
  SetOutputChannel::Response setOutputChannelResponse;

  actuator.servo = request.actuator.servo;
  setOutputChannelRequest.fin = request.fin;
  setOutputChannelRequest.actuator = request.id;
  setOutputChannelRequest.id = OutputChannel::FREQUENCY;
  setOutputChannelRequest.output_channel = request.actuator.frequency;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::AMPLITUDE;
  setOutputChannelRequest.output_channel = request.actuator.amplitude;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::PHASE;
  setOutputChannelRequest.output_channel = request.actuator.phase;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  setOutputChannelRequest.id = OutputChannel::OFFSET;
  setOutputChannelRequest.output_channel = request.actuator.offset;
  result |= setOutputChannel(setOutputChannelRequest,
    setOutputChannelResponse);
  
  return result;
}

bool setActuators(SetActuators::Request& request, SetActuators::Response&
    response) {
  bool result = true;
  
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("SetActuators request failed: Fin %d does not exist.",
      request.fin);
    return false;
  }
  
  SetActuator::Request setActuatorRequest;
  SetActuator::Response setActuatorResponse;
  
  setActuatorRequest.fin = request.fin;
  setActuatorRequest.id = Actuator::PITCH;
  setActuatorRequest.actuator = request.pitch;
  result |= setActuator(setActuatorRequest, setActuatorResponse);
  setActuatorRequest.id = Actuator::FLAP;
  setActuatorRequest.actuator = request.flap;
  result |= setActuator(setActuatorRequest, setActuatorResponse);
  
  return result;
}

bool setFin(SetFin::Request& request, SetFin::Response& response) {
  if ((request.id < 0) || (request.id >= fins.size())) {
    ROS_WARN("SetFin request failed: Fin %d does not exist.", request.id);
    return false;
  }
  
  SetActuators::Request setActuatorsRequest;
  SetActuators::Response setActuatorsResponse;
  
  setActuatorsRequest.fin = request.id;
  setActuatorsRequest.pitch = request.fin.pitch;
  setActuatorsRequest.flap = request.fin.flap;
  
  return setActuators(setActuatorsRequest, setActuatorsResponse);
}

bool setFins(SetFins::Request& request, SetFins::Response& response) {
  bool result = true;
  
  fins.resize(request.fins.size());
  for (int i = 0; i < request.fins.size(); ++i) {
    SetFin::Request setFinRequest;
    SetFin::Response setFinResponse;

    setFinRequest.id = i;
    setFinRequest.fin = request.fins[i];
    result |= setFin(setFinRequest, setFinResponse);
  }
  
  return result;
}

bool addFin(AddFin::Request& request, AddFin::Response& response) {
  SetFin::Request setFinRequest;
  SetFin::Response setFinResponse;
  
  setFinRequest.id = fins.size();
  setFinRequest.fin = request.fin;
  fins.resize(fins.size()+1);
  response.id = setFinRequest.id;

  return setFin(setFinRequest, setFinResponse);
}

bool removeFin(RemoveFin::Request& request, RemoveFin::Response& response) {
  if ((request.id < 0) || (request.id >= fins.size())) {
    ROS_WARN("RemoveFin request failed: Fin %d does not exist.", request.id);
    return false;
  }
  
  fins.erase(fins.begin()+request.id);
  
  return true;
}

bool connect(Connect::Request& request, Connect::Response& response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("Connect request failed: Fin %d does not exist.", request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN("Connect request failed: Actuator %d does not exist for fin %d.",
      request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN("Connect request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }

  _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);

  _Fin::Actuator::OutputChannel::TransferFunction coefficient(
    (_Fin::Actuator::OutputChannel::TransferFunction::Type)
    request.transfer_function, request.invert_arguments,
    request.invert_values);
  outputChannel.coefficients.push_back(coefficient);
  outputChannel.inputChannels.push_back(request.input_channel);
  
  response.coefficient = outputChannel.coefficients.size()-1;
  
  return true;
}

bool disconnect(Disconnect::Request& request, Disconnect::Response&
    response) {
  if ((request.fin < 0) || (request.fin >= fins.size())) {
    ROS_WARN("Disconnect request failed: Fin %d does not exist.", request.fin);
    return false;
  }
  if ((request.actuator < Actuator::PITCH) ||
      (request.actuator > Actuator::FLAP)) {
    ROS_WARN("Disconnect request failed: Actuator %d does not exist for "
      "fin %d.", request.actuator, request.fin);
    return false;
  }
  if ((request.output_channel < OutputChannel::FREQUENCY) ||
      (request.output_channel > OutputChannel::OFFSET)) {
    ROS_WARN("Disconnect request failed: Output channel %d does not exist "
      "for fin %d, actuator %d.", request.output_channel, request.fin,
      request.actuator);
    return false;
  }
  _Fin::Actuator::OutputChannel& outputChannel =
    fins[request.fin].getActuator(request.actuator).getOutputChannel(
    request.output_channel);
  if ((request.coefficient < 0) ||
      (request.coefficient >= outputChannel.coefficients.size())) {
    ROS_WARN(
      "Disconnect request failed: Coefficient %d does not exist "
      "for fin %d, actuator %d, output channel %d.", request.coefficient,
      request.fin, request.actuator, request.output_channel);
    return false;
  }
  
  outputChannel.coefficients.erase(outputChannel.coefficients.begin()+
    request.coefficient);
  outputChannel.inputChannels.erase(outputChannel.inputChannels.begin()+
    request.coefficient);
  
  return true;
}

void receiveJoy(const Joy::ConstPtr& message) {
  size_t numServos = 0;
  for (int i = 0; i < fins.size(); ++i) {
    numServos += (fins[i].pitch.servo >= 0);
    numServos += (fins[i].flap.servo >= 0);
  }
  
  if (!numServos) {
    diagnoseFrequency->tick();
    return;
  }

  SetCommands setCommands;
  setCommands.request.servos.resize(numServos);
  setCommands.request.frequency.resize(numServos);
  setCommands.request.amplitude.resize(numServos);
  setCommands.request.phase.resize(numServos);
  setCommands.request.offset.resize(numServos);
  
  int j = 0;
  for (int i = 0; i < fins.size(); ++i) {
    if (fins[i].pitch.servo >= 0) {
      _Fin::Actuator::Commands commands = fins[i].pitch(message->axes);
      
      setCommands.request.servos[j] = fins[i].pitch.servo;
      setCommands.request.frequency[j] = commands.frequency;
      setCommands.request.amplitude[j] = commands.amplitude;
      setCommands.request.phase[j] = commands.phase;
      setCommands.request.offset[j] = commands.offset;
      
      ++j;
    }

    if (fins[i].flap.servo >= 0) {
      _Fin::Actuator::Commands commands = fins[i].flap(message->axes);
      
      setCommands.request.servos[j] = fins[i].flap.servo;
      setCommands.request.frequency[j] = commands.frequency;
      setCommands.request.amplitude[j] = commands.amplitude;
      setCommands.request.phase[j] = commands.phase;
      setCommands.request.offset[j] = commands.offset;
      
      ++j;
    }    
  }
  
  if (setCommandsClient.call(setCommands))
    diagnoseFrequency->tick();
}

void diagnoseConnections(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!setCommandsClient)
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Not all required services are connected.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "All required services are connected.");
}

void updateDiagnostics(const ros::TimerEvent& event) {
  updater->update();
}

void tryConnect(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (!setCommandsClient)
    setCommandsClient = ros::NodeHandle("~").serviceClient<SetCommands>(
      "/"+finServerName+"/set_commands", true);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_command");
  ros::NodeHandle node("~");

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("none");

  updater->add("Connections", diagnoseConnections);
  diagnoseFrequency.reset(new diagnostic_updater::FrequencyStatus(
    diagnostic_updater::FrequencyStatusParam(&subscriberFrequency,
    &subscriberFrequency)));
  updater->add("Frequency", &*diagnoseFrequency,
    &diagnostic_updater::FrequencyStatus::run);
  updater->force_update();

  getParameters(node);

  getOutputsService = node.advertiseService("get_outputs", getOutputs);
  getCoefficientService = node.advertiseService("get_coefficient",
    getCoefficient);
  getCoefficientsService = node.advertiseService("get_coefficients",
    getCoefficients);
  getOutputChannelService = node.advertiseService("get_output_channel",
    getOutputChannel);
  getOutputChannelsService = node.advertiseService("get_output_channels",
    getOutputChannels);
  getActuatorService = node.advertiseService("get_actuator", getActuator);
  getActuatorsService = node.advertiseService("get_actuators", getActuators);
  getFinService = node.advertiseService("get_fin", getFin);
  getFinsService = node.advertiseService("get_fins", getFins);
  setCoefficientService = node.advertiseService("set_coefficient",
    setCoefficient);
  setCoefficientsService = node.advertiseService("set_coefficients",
    setCoefficients);
  setOutputChannelService = node.advertiseService("set_output_channel",
    setOutputChannel);
  setOutputChannelsService = node.advertiseService("set_output_channels",
    setOutputChannels);
  setActuatorService = node.advertiseService("set_actuator", setActuator);
  setActuatorsService = node.advertiseService("set_actuators", setActuators);
  setFinService = node.advertiseService("set_fin", setFin);
  setFinsService = node.advertiseService("set_fins", setFins);
  addFinService = node.advertiseService("add_fin", addFin);
  removeFinService = node.advertiseService("remove_fin", removeFin);
  connectService = node.advertiseService("connect", connect);
  disconnectService = node.advertiseService("disconnect", disconnect);

  ros::Timer diagnosticsTimer = node.createTimer(
    ros::Duration(1.0), updateDiagnostics);
  ros::Timer connectionTimer = node.createTimer(
    ros::Duration(connectionRetry), tryConnect);

  subscriber = ros::NodeHandle("~").subscribe("/"+subscriberTopic,
    subscriberQueueSize, receiveJoy);
  
  tryConnect();

  ros::spin();

  return 0;
}
