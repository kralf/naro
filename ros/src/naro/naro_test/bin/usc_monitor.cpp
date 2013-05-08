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

using namespace naro_usc_srvs;

ros::ServiceClient getErrorsClient;

void update(const ros::TimerEvent& event) {
  GetErrors getErrors;
  if (getErrorsClient.call(getErrors)) {
    char errorBits[10];

    int j = sizeof(errorBits)-2;
    for (int i = 0; i+1 < sizeof(errorBits); ++i, --j)
      errorBits[i] = (getErrors.response.errors & (1 << j)) ? '1' : '0';
    errorBits[sizeof(errorBits)-1] = 0;

    printf("\r%14s: %10s\n", "Errors", errorBits);
  }
  else
    printf("\r%14s: %10s\n", "Errors", "n/a");

  printf("%c[1A\r", 0x1B);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usc_monitor");
  ros::NodeHandle node;

  getErrorsClient = node.serviceClient<GetErrors>(
    "usc_server/get_errors");

  ros::Timer timer = node.createTimer(ros::Duration(0.1), update);
  ros::spin();

  printf("%c[1B\n", 0x1B);

  return 0;
}
