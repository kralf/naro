#include <roscpp_nodewrap/Node.h>

#include "naro_imu/NaroImu.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");

  Node<NaroImu> node;

  ros::spin();

  return 0;
}
