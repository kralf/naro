#include <roscpp_nodewrap/Node.h>

#include "naro_tank_ctrl/TankCtrl.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "naro_imu");

  Node<TankCtrl> node;

  ros::spin();

  return 0;
}
