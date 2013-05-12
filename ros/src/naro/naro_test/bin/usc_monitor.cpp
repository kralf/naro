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

#include <ros/ros.h>

#include "naro_usc_srvs/GetErrors.h"
#include "naro_usc_srvs/GetChannels.h"
#include "naro_usc_srvs/GetPositions.h"
#include "naro_usc_srvs/GetInputs.h"

using namespace naro_usc_srvs;

std::string serverName = "usc_server";
double clientUpdate = 0.1;

ros::ServiceClient getErrorsClient;
ros::ServiceClient getChannelsClient;
ros::ServiceClient getPositionsClient;
ros::ServiceClient getInputsClient;

unsigned int numLines = 0;

void update(const ros::TimerEvent& event) {
  GetErrors getErrors;
  if (getErrorsClient.call(getErrors)) {
    char errorBits[10];

    int j = sizeof(errorBits)-2;
    for (int i = 0; i+1 < sizeof(errorBits); ++i, --j)
      errorBits[i] = (getErrors.response.errors & (1 << j)) ? '1' : '0';
    errorBits[sizeof(errorBits)-1] = 0;

    printf("\r%14s: %12s\n", "Errors", errorBits);
  }
  else
    printf("\r%14s: %12s\n", "Errors", "n/a");

  GetChannels getChannels;
  if (getChannelsClient.call(getChannels)) {
    char channelModes[getChannels.response.mode.size()+1];

    for (int i = 0; i < getChannels.response.mode.size(); ++i) {
      if (getChannels.response.mode[i] == GetChannels::Response::OUTPUT)
        channelModes[i] = 'O';
      else if (getChannels.response.mode[i] == GetChannels::Response::INPUT)
        channelModes[i] = 'I';
      else
        channelModes[i] = 'S';
    }
    channelModes[getChannels.response.mode.size()] = 0;

    printf("\r%14s: %12s\n", "Channels", channelModes);
  }
  else
    printf("\r%14s: %12s\n", "Channels", "n/a");

  GetPositions getPositions;
  for (int i = 0; i < getChannels.response.mode.size(); ++i)
    if (getChannels.response.mode[i] == GetChannels::Response::SERVO)
      getPositions.request.channels.push_back(i);
  if (getPositionsClient.call(getPositions)) {
    for (int i = 0; i < getPositions.request.channels.size(); ++i)
      printf("\r%11s %2d: %12.2f deg\n", "Channel",
        getPositions.request.channels[i],
        getPositions.response.actual[i]*180.0/M_PI);
  }
  else {
    for (int i = 0; i < getPositions.request.channels.size(); ++i)
      printf("\r%11s %2d: %12s    \n", "Channel",
        getPositions.request.channels[i], "n/a");
  }

  GetInputs getInputs;
  for (int i = 0; i < getChannels.response.mode.size(); ++i)
    if (getChannels.response.mode[i] == GetChannels::Response::INPUT)
      getInputs.request.channels.push_back(i);
  if (getInputsClient.call(getInputs)) {
    for (int i = 0; i < getInputs.request.channels.size(); ++i)
      printf("\r%11s %2d: %12.2f V\n", "Channel",
        getInputs.request.channels[i],
        getInputs.response.voltage[i]);
  }
  else {
    for (int i = 0; i < getInputs.request.channels.size(); ++i)
      printf("\r%11s %2d: %12s  \n", "Channel",
        getInputs.request.channels[i], "n/a");
  }

  numLines = 2+getPositions.response.actual.size()+
    getInputs.response.voltage.size();
  printf("%c[%dA\r", 0x1B, numLines);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usc_monitor");
  ros::NodeHandle node;

  ros::param::param<std::string>(ros::this_node::getName()+"/server/name",
    serverName, serverName);
  ros::param::param<double>(ros::this_node::getName()+"/client/update",
    clientUpdate, clientUpdate);

  getErrorsClient = node.serviceClient<GetErrors>(
    serverName+"/get_errors");
  getChannelsClient = node.serviceClient<GetChannels>(
    serverName+"/get_channels");
  getPositionsClient = node.serviceClient<GetPositions>(
    serverName+"/get_positions");
  getInputsClient = node.serviceClient<GetInputs>(
    serverName+"/get_inputs");

  ros::Timer updateTimer = node.createTimer(
    ros::Duration(clientUpdate), update);
  ros::spin();

  printf("%c[%dB\n", 0x1B, numLines);

  return 0;
}
