/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_sensor_srvs/HallSensorNode.h"

NODEWRAP_EXPORT_CLASS(naro_hall, nodewrap::HallSensorNode)

using namespace naro_sensor_srvs;
namespace nodewrap {

HallSensorNode::HallSensorNode(){}

HallSensorNode::~HallSensorNode(){
	delete sensor;
}

void HallSensorNode::init() {
	NODEWRAP_INFO("Initialize <HallSensorNode>");

	// setup services
	getPositionService = advertiseService("getPosition", "/getPosition", &HallSensorNode::getPosition);
	resetCounterService = advertiseService("resetCounter", "/resetCounter", &HallSensorNode::resetCounter);
	setFrequencyService = advertiseService("setFrequency", "/setFrequency", &HallSensorNode::setFrequency);
	getFrequencyService = advertiseService("getFrequency", "/getFrequency", &HallSensorNode::getFrequency);

	// start reading sensor data
	sensor = new HallSensor("199");
	sensor->startReadingSensor();
}

void HallSensorNode::cleanup() {
	NODEWRAP_INFO("Shutting down <HallSensorNode>");
	sensor->join();
}

// return position of piston tank
bool HallSensorNode::getPosition(GetPosition::Request& request, GetPosition::Response& response) {
	NODEWRAP_INFO("getPosition service called");
	response.position = sensor->getCount();

	return true;
}

// set reading frequency of HallSensor
bool HallSensorNode::setFrequency(SetFrequency::Request& request, SetFrequency::Response& response){
	sensor->setFrequency(request.frequency);
	response.set = true;
	return 1;
}

bool HallSensorNode::getFrequency(GetFrequency::Request& request, GetFrequency::Response& response) {
	response.frequency = sensor->getFrequency();
	return 1;
}

// reset the counter
bool HallSensorNode::resetCounter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
	sensor->resetCount();
	ROS_INFO("Counter reset to 0");
	return 1;
}

}
