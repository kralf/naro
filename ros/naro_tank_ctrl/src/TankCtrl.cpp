/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_tank_ctrl/TankCtrl.h"

namespace nodewrap {

TankCtrl::TankCtrl() {};
TankCtrl::~TankCtrl() {};

void TankCtrl::init() {
	NODEWRAP_INFO("Initialize <TankCtrl>");

	positionClient = n.serviceClient<naro_sensor_srvs::GetPosition>("hallSensor_front");
	timerPosition = n.createTimer(ros::Duration(1), &TankCtrl::readPosition, this);

	ticksOld = 0;

}

void TankCtrl::cleanup() {
	NODEWRAP_INFO("Shutting down: TankCtrl");
}

void TankCtrl::readPosition(const ros::TimerEvent& event) {
	if(positionClient.call(srv)) {
		NODEWRAP_INFO("Call successful!!!");

		int ticksNew = srv.response.position;
		int diff = ticksNew-ticksOld; // calculate diff in position
		// change position
		if(speedDirection<0) { // tank filling
			float posTmp = position+diff/totalTicks; // add normalized difference
			position = posTmp;
		} else if(speedDirection > 0) {
			float posTmp = position-diff/totalTicks; // subtract normalized difference
			position = posTmp;
		} else {
			// no change in position
		}

		ticksOld = ticksNew;

	}
	else {
		NODEWRAP_INFO("Call FAILED");
	}
}

}
