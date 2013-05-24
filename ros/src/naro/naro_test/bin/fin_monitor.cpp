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

#include "naro_fin_ctrl/GetServos.h"
#include "naro_fin_ctrl/GetActuals.h"

using namespace naro_fin_ctrl;

std::string serverName = "fin_controller";
double clientUpdate = 0.1;

ros::ServiceClient getServosClient;
ros::ServiceClient getActualsClient;

unsigned int numLines = 0;

void update(const ros::TimerEvent& event) {
  GetServos getServos;
  getServosClient.call(getServos);

  GetActuals getActuals;
  for (int i = 0; i < getServos.response.servos; ++i)
    getActuals.request.servos.push_back(i);
  if (getActualsClient.call(getActuals)) {
    for (int i = 0; i < getActuals.request.servos.size(); ++i)
      printf("\r%s %2d: F %8.2f Hz  A %8.2f deg  P %8.3f deg  O %8.2f deg\n",
        "Servo", getActuals.request.servos[i],
        getActuals.response.frequency[i],
        getActuals.response.amplitude[i]*180.0/M_PI,
        getActuals.response.phase[i]*180.0/M_PI,
        getActuals.response.offset[i]*180.0/M_PI);
  }
  else {
    for (int i = 0; i < getActuals.request.servos.size(); ++i)
      printf("\r%s %2d: n/a\n", "Servo", getActuals.request.servos[i]);
  }

  numLines = getActuals.request.servos.size();
  printf("%c[%dA\r", 0x1B, numLines);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fin_monitor");
  ros::NodeHandle node("~");

  node.param<std::string>("server/name", serverName, serverName);
  node.param<double>("client/update", clientUpdate, clientUpdate);

  getServosClient = node.serviceClient<GetServos>(
    "/"+serverName+"/get_servos");
  getActualsClient = node.serviceClient<GetActuals>(
    "/"+serverName+"/get_actuals");

  ros::Timer updateTimer = node.createTimer(
    ros::Duration(clientUpdate), update);
  ros::spin();

  printf("%c[%dB\n", 0x1B, numLines);

  return 0;
}
