/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/Node.h>

#include "naro_sensor_srvs/HallSensorNode.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "hall_sensor");

  Node<HallSensorNode> node;

  ros::spin();

  return 0;
}
