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

	ticksOld = 0;
	totalTicks = 950;
	speedDirection = 0;
	position = 0;
	positionRequest = 0;
	positionThreshold = 0.01;
	finalPosition = true;

}

void TankCtrl::cleanup() {
	NODEWRAP_INFO("Shutting down: TankCtrl");
}

void TankCtrl::readPosition(const ros::TimerEvent& event) {
	if(positionClient.call(posSrv)) {
		int ticksNew = posSrv.response.position;
		int diff = ticksNew-ticksOld; // calculate diff in position
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
			}
		}

		ticksOld = ticksNew;

		std::cout << "Tank position inside: " << position << std::endl;

	}
	else {
		NODEWRAP_INFO("HallSensor service not available");
	}
	std::cout << "Tank position outside: " << position << std::endl;
}

bool TankCtrl::getTankPosition(GetTankPosition::Request& request, GetTankPosition::Response& response) {
	response.position = this->position;
	return true;
}

/*
* sets correct motor speed according actual position and requested position of piston tank
*/

bool TankCtrl::setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response) {
	finalPosition = false;
	positionRequest = request.position;

	std::cout << "Position Request:" << positionRequest << std::endl;

	float speed = abs(request.speed);

	std::cout << "Speed: " << speed << std::endl;

	if(abs(positionRequest - this->position)>positionThreshold) { // check ifnot at position
		if(position > positionRequest) {
			speedDirection = 1; // drive in
			NODEWRAP_INFO("set speed direction +");
		} else {
			speedDirection = -1; // drive in
			NODEWRAP_INFO("set speed direction -");
		}
		speed = 0.5f;
		callSpeedClient(speedDirection*speed);
	}

	return true;
}

/*
* set the speed of the motor 
*/
void TankCtrl::callSpeedClient(float speed) {
	speedSrv.request.speed = -0.5;
	std::cout << speed << std::endl;
	speedSrv.request.start = true;
	if(speedClient.call(speedSrv)) {
		NODEWRAP_INFO("speed set to: %d", speed);
		
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
			NODEWRAP_INFO("Reached final position");
			callSpeedClient(0);
			speedDirection = 0;
			finalPosition = true;
		}
	}
}

}
