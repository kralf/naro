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
	// GET PARAMETERS
	nodeName = getParam("control/name", nodeName);
	double tmpPos = getParam("positionThreshold", tmpPos);
	positionThreshold = (float)tmpPos;
	smcServerName = getParam("control/smcServer", smcServerName);
	tankPosName = getParam("position/name", tankPosName);
	double positionCheckFreq = getParam("control/checkPosFreq", positionCheckFreq);
	double tmp = getParam("motorSpeed", tmp);
	motorSpeed = tmp;

	// INIT VARIABLES
	speedDirection = 0.0;
	positionRequest = 0.0;
	finalPosition = true;

	NODEWRAP_INFO("Initialize: <%s>", nodeName.c_str());

	// SERVICES
	// -> subscribe
	connectServices();

	// -> advertise
	setTankPositionService = advertiseService("setTankPosition", "setTankPosition", &TankCtrl::setTankPosition);
	resetTankPositionService = advertiseService("resetTankPosition", "resetTankPosition", &TankCtrl::resetTankPosition);

	// TIMER
	checkPositionTimer = n.createTimer(ros::Duration(1.0/positionCheckFreq), &TankCtrl::checkPosition, this); // timer for checking position

	//startup();

}

void TankCtrl::cleanup() {
	NODEWRAP_INFO("Shutting down: <%s>", nodeName.c_str());
	//resetPosition();
	setSpeed(motorSpeed); // empty tanks (don't check for limits, as already shut down)
}


/*
* set correct motor speed according actual position and requested position of piston tank
*/
bool TankCtrl::setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response) {
	positionRequest = clamp((float)(request.position), 0.0, 1.0);
	float speed = clamp(fabs((float)(request.speed)), 0.0, 1.0);

	float position = getPosition();

	if(fabs(positionRequest-position)>positionThreshold) { // check ifnot at position
		float direction = speedDirection;
		if(position > positionRequest) {
			direction = 1.0; // drive in
			//NODEWRAP_INFO("set speed direction +");
		} else {
			direction = -1.0; // drive in
			//NODEWRAP_INFO("set speed direction -");
		}

		if(direction!=speedDirection) { // check if change in direction
			// stop motor
			setSpeed(0);
			// change direction
			setDirection(direction);
			speedDirection = direction;
		}

		setSpeed(speedDirection*speed);

		finalPosition = false;
	} else {
		//NODEWRAP_INFO("Already at final position");
	}

	return true;
}

/*
* set the speed of the motor 
*/
void TankCtrl::setSpeed(float speed) {
	if(!speedClient)
			connectServices();

	speedSrv.request.speed = speed;
	speedSrv.request.start = true;
	if(speedClient.call(speedSrv)) {
		//NODEWRAP_INFO("speed set to: %f", speed);
		
	} else {
		NODEWRAP_INFO("smc node not avialable");
	}
}

/*
* check position and stop if correct position within certain threshold
*/
void TankCtrl::checkPosition(const ros::TimerEvent& event) {
	if(!finalPosition) {
		float position = getPosition();
		if(fabs(position-positionRequest)<positionThreshold) {
			//NODEWRAP_INFO("Reached final position");
			setSpeed(0.0);
			//speedDirection = 0;
			//setDirection(0);
			finalPosition = true;
		}
	}
}

/*
* startup procedure for piston tank
*/
void TankCtrl::startup() {
	resetPosition();
}

/*
* call TankPosition node and update direction for position counting
*/
void TankCtrl::setDirection(float direction) {
	if(!directionClient)
		connectServices();

	directionSrv.request.direction = direction;
	if(directionClient.call(directionSrv)) {
		//NODEWRAP_INFO("direction set to: %f", direction);	
	} else {
		NODEWRAP_INFO("setDirection: TankPosition node not available");
	}
}

/*
* get tank position from PositionNode
*/
float TankCtrl::getPosition() {
	if(!positionClient)
		connectServices();

	if(positionClient.call(posSrv)) {
		return posSrv.response.position;	
	} else {
		NODEWRAP_INFO("getPosition: TankPosition node not avialable");
		return false;
	}
}

/*
* reset tank position service
*/
bool TankCtrl::resetTankPosition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	return resetPosition();
}

/*
* reset tank position from (unknown) position to 0
*/
bool TankCtrl::resetPosition() {
	if(getLimit()!=257) { // already in limit
		setSpeed(0.0); // stop tank
		setDirection(1.0);
		setSpeed(0.5); // start driving in

		while(true) { // check if in limit switch
			if(getLimit()==257) { // in limit
				break;
			}

			ros::Duration(2.0).sleep();
		}
	} else {
		//NODEWRAP_INFO("Already at position 0");
	}

	setSpeed(0.0);

	// reset position counter
	if(resetClient.call(resetSrv)) {
		NODEWRAP_INFO("%s reset to position 0", nodeName.c_str());
		return true;
	} else {
		NODEWRAP_INFO("tank position reset service not available");
		return false;
	}
}

int TankCtrl::getLimit() {
	if(!limitClient)
		connectServices();

	if(limitClient.call(limitSrv)) {
		return limitSrv.response.limits;
	} else {
		NODEWRAP_INFO("smc limit service not available");
		return false;
	}
}

/*
 * check value
 */
float TankCtrl::clamp(float value, float min, float max) {
	if(value<min) {
		return min;
	} else if(value>max) {
		return max;
	} else {
		return value;
	}
}

/*
 * connect or reconnect to service clients
 */
void TankCtrl::connectServices() {
	if(!speedClient)
		speedClient = n.serviceClient<naro_smc_srvs::SetSpeed>("/"+smcServerName+"/set_speed", true);
	if(!limitClient)
		limitClient = n.serviceClient<naro_smc_srvs::GetLimits>("/"+smcServerName+"/get_limits", true);
	if(!positionClient)
		positionClient = n.serviceClient<naro_tank_ctrl::GetTankPosition>("/"+tankPosName+"/getTankPosition", true);
	if(!directionClient)
		directionClient = n.serviceClient<naro_tank_ctrl::SetDirection>("/"+tankPosName+"/setDirection", true);
	if(!resetClient)
		resetClient = n.serviceClient<std_srvs::Empty>("/"+tankPosName+"/resetPositionCounter", true);
}

// ---- END
}
