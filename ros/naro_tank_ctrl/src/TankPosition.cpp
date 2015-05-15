/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_tank_ctrl/TankPosition.h"
#include <iostream>

using namespace naro_tank_ctrl;

namespace nodewrap {

TankPosition::TankPosition() {};
TankPosition::~TankPosition() {};

void TankPosition::init() {
	// get parameters
	nodeName = getParam("position/name", nodeName);
	int tmpTicks = getParam("position/totalTicks", tmpTicks);
	totalTicks = (float)tmpTicks;
	double tmpPos = getParam("positionThreshold", tmpPos);
	positionThreshold = (float)tmpPos;

    NODEWRAP_INFO("Initialize: <%s>", nodeName.c_str());

    positionClient = n.serviceClient<naro_sensor_srvs::GetPosition>("hallSensor_rear/getPosition");

    timerHallSensor = n.createTimer(ros::Duration(1/300), &TankPosition::readPosition, this); // timer for reading HallSensor

    // services
    setDirectionService = advertiseService("setDirection", "setDirection", &TankPosition::setSpeedDirection);
    getPositionService = advertiseService("getTankPosition", "getTankPosition", &TankPosition::getTankPosition);
    resetPositionCounterService = advertiseService("resetPositionCounter", "resetPositionCounter", &TankPosition::resetPositionCounter);

    ticksOld = 0.0;
    speedDirection = 0.0;
    position = 0.0;

}

void TankPosition::cleanup() {
    NODEWRAP_INFO("Shutting down: <%s>", nodeName.c_str());
}

/*
* get information from hall sensor and transform into normalized tank position
*/
void TankPosition::readPosition(const ros::TimerEvent& event) {
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
            if(diff>positionThreshold) {
                NODEWRAP_INFO("Somebody messed with the system!");
            }
        }

        ticksOld = ticksNew;

    }
    else {
        NODEWRAP_INFO("HallSensor service not available");
    }
}

bool TankPosition::setSpeedDirection(SetDirection::Request& request, SetDirection::Response& response) {
    this->speedDirection = request.direction;
    return true;
}

bool TankPosition::getTankPosition(GetTankPosition::Request& request, GetTankPosition::Response& response) {
    response.position = this->position;
    return true;
}

bool TankPosition::resetPositionCounter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	this->position = 0.0;
	NODEWRAP_INFO("position counter reset to 0");
	return 1;
}

}
