#include <roscpp_nodewrap/Node.h>

#include "naro_tank_ctrl/TankPosition.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "naro_tank_ctrl");

  Node<TankPosition> node;

  ros::spin();

  return 0;
}