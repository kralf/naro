/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_tank_ctrl/TankCtrl.h"
#include <iostream>

using namespace naro_tank_ctrl;
namespace nodewrap {

TankCtrl::TankCtrl() {};
TankCtrl::~TankCtrl() {};

void TankCtrl::init() {
	NODEWRAP_INFO("Initialize <TankCtrl>");

	positionClient = n.serviceClient<naro_sensor_srvs::GetPosition>("hallSensor_rear/getPosition");
	speedClient = n.serviceClient<naro_smc_srvs::SetSpeed>("/smc_server_rear/set_speed");

	timerHallSensor = n.createTimer(ros::Duration(1/300), &TankCtrl::readPosition, this); // timer for reading HallSensor
	checkPositionTimer = n.createTimer(ros::Duration(1/300), &TankCtrl::checkPosition, this); // timer for checking position

	getTankPositionService = advertiseService("getTankPosition", "getTankPosition", &TankCtrl::getTankPosition);
	setTankPositionService = advertiseService("setTankPosition", "setTankPosition", &TankCtrl::setTankPosition);

	startup();

	ticksOld = 0.0;
	totalTicks = 950.0;
	speedDirection = 0.0;
	position = 0.0;
	positionRequest = 0.0;
	positionThreshold = 0.01;
	finalPosition = true;

}

void TankCtrl::cleanup() {
	NODEWRAP_INFO("Shutting down: TankCtrl");
}

/*
* get information from hall sensor and transform into normalized tank position
*/
void TankCtrl::readPosition(const ros::TimerEvent& event) {
	if(positionClient.call(posSrv)) {
		float ticksNew = posSrv.response.position;
		float diff = ticksNew-ticksOld; // calculate diff in position
		// change position
		if(speedDirection<0) { // tank filling
			float posTmp = position+diff/totalTicks; // add normalized difference
			position = posTmp;
		} else if(speedDirection > 0) {
			float posTmp = position-diff/totalTicks; // subtract normalized difference
			position = posTmp;
		} else { // no speed
			if(diff!=0) {
				NODEWRAP_INFO("Somebody messed with the system!");
				float posTmp = position+diff/totalTicks; // add normalized difference
				position = posTmp;
			}
		}

		ticksOld = ticksNew;

	}
	else {
		NODEWRAP_INFO("HallSensor service not available");
	}
}

bool TankCtrl::getTankPosition(GetTankPosition::Request& request, GetTankPosition::Response& response) {
	response.position = this->position;
	return true;
}

/*
* sets correct motor speed according actual position and requested position of piston tank
*/

bool TankCtrl::setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response) {
	positionRequest = (float)(request.position);

	std::cout << "---" << std::endl;
	std::cout << "Position Request:" << positionRequest << std::endl;

	float speed = abs((float)(request.speed));

	std::cout << "Speed Request: " << speed << std::endl;

	float diff = abs(positionRequest-position);

	std::cout << "Position:" << position << std::endl;
	std::cout << "Position Diff: " << diff << std::endl;

	if(diff>positionThreshold) { // check ifnot at position
		if(position > positionRequest) {
			speedDirection = 1.0; // drive in
			NODEWRAP_INFO("set speed direction +");
		} else {
			speedDirection = -1.0; // drive in
			NODEWRAP_INFO("set speed direction -");
		}
		callSpeedClient(speedDirection*speed);
		finalPosition = false;
	} else {
		NODEWRAP_INFO("Already at final position");
	}

	return true;
}

/*
* set the speed of the motor 
*/
void TankCtrl::callSpeedClient(float speed) {
	std::cout << "Speed in call class: " << speed << std::endl;
	speedSrv.request.speed = speed;
	std::cout << speed << std::endl;
	speedSrv.request.start = true;
	if(speedClient.call(speedSrv)) {
		NODEWRAP_INFO("speed set to: %f", speed);
		
	} else {
		NODEWRAP_INFO("smc node not avialable");
	}
}

/*
* check position and stop if correct position within certain threshold
*/
void TankCtrl::checkPosition(const ros::TimerEvent& event) {
	if(abs(position-positionRequest)<positionThreshold) {
		if(!finalPosition) {
			std::cout << "Diff: " << abs(position-positionRequest) << std::endl;
			NODEWRAP_INFO("Reached final position");
			callSpeedClient(0.0);
			speedDirection = 0.0;
			finalPosition = true;
		}
	}
}

/*
* startup procedure for piston tank
*/
void TankCtrl::startup() {
	callSpeedClient(0.0);
}

}
