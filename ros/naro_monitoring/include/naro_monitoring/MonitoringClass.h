#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include "naro_monitoring/LoggingClass.h"
#include "ros/ros.h"
#include <vector>

namespace nodewrap {

class MonitoringClass:
public NodeImpl{

public:
	MonitoringClass();
	virtual ~MonitoringClass();

protected:
	void init();
	void cleanup();

	ros::NodeHandle n;

	ros::Timer testTimer;
	void callback(const ros::TimerEvent& event);


	LoggingClass logger;
};

};
