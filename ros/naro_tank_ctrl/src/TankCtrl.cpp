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

	// SERVICES
	// -> subscribe
	speedClient = n.serviceClient<naro_smc_srvs::SetSpeed>("/smc_server_rear/set_speed");
	positionClient = n.serviceClient<naro_tank_ctrl::GetTankPosition>("/tankPos/getTankPosition");
	directionClient = n.serviceClient<naro_tank_ctrl::SetDirection>("/tankPos/setDirection");

	// -> advertise
	setTankPositionService = advertiseService("setTankPosition", "setTankPosition", &TankCtrl::setTankPosition);

	// TIMER
	checkPositionTimer = n.createTimer(ros::Duration(1/300), &TankCtrl::checkPosition, this); // timer for checking position

	startup();

	// INIT VAROIABLES
	speedDirection = 0.0;
	positionRequest = 0.0;
	positionThreshold = 0.01;
	finalPosition = true;

}

void TankCtrl::cleanup() {
	NODEWRAP_INFO("Shutting down: TankCtrl");
}


/*
* set correct motor speed according actual position and requested position of piston tank
*/
bool TankCtrl::setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response) {
	positionRequest = (float)(request.position);
	float speed = fabs((float)(request.speed));

	float position = getPosition();

	if(fabs(positionRequest-position)>positionThreshold) { // check ifnot at position
		float direction = speedDirection;
		if(position > positionRequest) {
			direction = 1.0; // drive in
			NODEWRAP_INFO("set speed direction +");
		} else {
			direction = -1.0; // drive in
			NODEWRAP_INFO("set speed direction -");
		}

		if(direction!=speedDirection) { // check if change in direction
			// stop motor
			callSpeedClient(0);
			// change direction
			setDirection(direction);
			speedDirection = direction;
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
	speedSrv.request.speed = speed;
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
	float position = getPosition();
	if(fabs(position-positionRequest)<positionThreshold) {
		if(!finalPosition) {
			NODEWRAP_INFO("Reached final position");
			callSpeedClient(0.0);
			speedDirection = 0;
			setDirection(0);
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

/*
* call TankPosition node and update direction for position counting
*/
void TankCtrl::setDirection(float direction) {
	directionSrv.request.direction = direction;
	if(directionClient.call(directionSrv)) {
		NODEWRAP_INFO("direction set to: %f", direction);	
	} else {
		NODEWRAP_INFO("TankPosition node not avialable");
	}
}

/*
* get tank position from PositionNode
*/
float TankCtrl::getPosition() {
	if(positionClient.call(posSrv)) {
		return posSrv.response.position;	
	} else {
		NODEWRAP_INFO("TankPosition node not avialable");
		return false;
	}
}


}
