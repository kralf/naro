/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_monitoring/MonitoringClass.h"

//using namespace naro_monitoring;

namespace nodewrap {

MonitoringClass::MonitoringClass() {}
MonitoringClass::~MonitoringClass() {}

void MonitoringClass::init() {

	logger.createPublisher("test1");
	testTimer = n.createTimer(ros::Duration(1.0), &MonitoringClass::callback, this);

}

void MonitoringClass::cleanup() {};

void MonitoringClass::callback(const ros::TimerEvent& event) {
	std::vector<float> array;
	array[0] = 10; array[1] = 12;
	logger.log(array,"test1");
	std::vector<float> array2;
	array2[0] = 20; array2[1]=24;
	logger.log(array2,"test2");
}

}
