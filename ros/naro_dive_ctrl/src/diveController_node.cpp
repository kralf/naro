#include <roscpp_nodewrap/Node.h>

#include "naro_dive_ctrl/DiveController.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "naro_dive_ctrl");

  Node<DiveController> node;

  ros::spin();

  return 0;
}
