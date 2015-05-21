#include <roscpp_nodewrap/Node.h>

#include "naro_monitoring/MonitoringClass.h"

using namespace nodewrap;

int main(int argc, char** argv) {
  ros::init(argc, argv, "naro_monitoring");

  Node<MonitoringClass> node;

  ros::spin();

  return 0;
}
