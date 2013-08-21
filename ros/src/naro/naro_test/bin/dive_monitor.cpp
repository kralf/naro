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

#include "naro_dive_ctrl/GetActual.h"
#include "naro_dive_ctrl/GetCommand.h"
#include "naro_dive_ctrl/GetError.h"

using namespace naro_dive_ctrl;

std::string serverName = "dive_controller";
double clientUpdate = 0.1;

ros::ServiceClient getCommandClient;
ros::ServiceClient getActualClient;
ros::ServiceClient getErrorClient;

unsigned int numLines = 0;

void update(const ros::TimerEvent& event) {
  GetCommand getCommand;
  if (getCommandClient.call(getCommand))
    printf("\rCmd: D %8.2f m  V %8.2f m/s\n",
      getCommand.response.depth,
      getCommand.response.velocity);
  else 
    printf("\rCmd: n/a\n");

  GetActual getActual;
  if (getActualClient.call(getActual))
    printf("\rAct: D %8.2f m  V %8.2f m/s\n",
      getActual.response.depth,
      getActual.response.velocity);
  else 
    printf("\rAct: n/a\n");

  GetError getError;
  if (getErrorClient.call(getError))
    printf("\rErr: D %8.2f m  V %8.2f m/s\n",
      getActual.response.depth,
      getActual.response.velocity);
  else 
    printf("\rErr: n/a\n");
  
  numLines = 3;
  printf("%c[%dA\r", 0x1B, numLines);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dive_monitor");
  ros::NodeHandle node("~");

  node.param<std::string>("server/name", serverName, serverName);
  node.param<double>("client/update", clientUpdate, clientUpdate);

  getCommandClient = node.serviceClient<GetCommand>(
    "/"+serverName+"/get_command");
  getActualClient = node.serviceClient<GetActual>(
    "/"+serverName+"/get_actual");
  getErrorClient = node.serviceClient<GetError>(
    "/"+serverName+"/get_error");

  ros::Timer updateTimer = node.createTimer(
    ros::Duration(clientUpdate), update);
  ros::spin();

  printf("%c[%dB\n", 0x1B, numLines);

  return 0;
}
