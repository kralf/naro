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

#include "naro_sensor_srvs/GetPressure.h"
#include "naro_sensor_srvs/GetDepth.h"
#include "naro_sensor_srvs/GetElevation.h"

using namespace naro_sensor_srvs;

std::string serverName = "depth_sensor";
double clientUpdate = 0.1;

ros::ServiceClient getPressureClient;
ros::ServiceClient getDepthClient;
ros::ServiceClient getElevationClient;

void update(const ros::TimerEvent& event) {
  GetPressure getPressure;
  if (getPressureClient.call(getPressure))
    printf("\r%10s: %8.2f / %8.2f kPa\n", "Pressure",
      getPressure.response.raw*1e-3, getPressure.response.filtered*1e-3);
  else
    printf("\r%10s: %19s\n", "Pressure", "n/a");

  GetDepth getDepth;
  if (getDepthClient.call(getDepth))
    printf("\r%10s: %8.2f / %8.2f m\n", "Depth",
      getDepth.response.raw, getDepth.response.filtered);
  else
    printf("\r%10s: %19s\n", "Depth", "n/a");

  GetElevation getElevation;
  if (getElevationClient.call(getElevation))
    printf("\r%10s: %8.2f / %8.2f m\n", "Elevation",
      getElevation.response.raw, getElevation.response.filtered);
  else
    printf("\r%10s: %19s\n", "Elevation", "n/a");

  printf("%c[3A\r", 0x1B);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_monitor");
  ros::NodeHandle node("~");

  node.param<std::string>("server/name", serverName, serverName);
  node.param<double>("client/update", clientUpdate, clientUpdate);

  getPressureClient = node.serviceClient<GetPressure>(
    "/"+serverName+"/get_pressure");
  getDepthClient = node.serviceClient<GetDepth>(
    "/"+serverName+"/get_depth");
  getElevationClient = node.serviceClient<GetElevation>(
    "/"+serverName+"/get_elevation");

  ros::Timer updateTimer = node.createTimer(
    ros::Duration(clientUpdate), update);
  ros::spin();

  printf("%c[3B\n", 0x1B);

  return 0;
}
