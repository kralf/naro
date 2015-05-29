/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_dive_ctrl/DiveController.h"
#include <iostream>

using namespace naro_dive_ctrl;

namespace nodewrap {

DiveController::DiveController() {};
DiveController::~DiveController() {};

void DiveController::init() {
	NODEWRAP_INFO("Initializing <DiveCtrl> node");

	// get parameters
	double freqDepth = getParam("Depth/Frequency", freqDepth);
	double freqPitch = getParam("Pitch/Frequency", freqPitch);
	tankFront = getParam("Services/tankFrontCtrl", tankFront);
	tankRear = getParam("Services/tankRearCtrl", tankRear);
	depthName = getParam("Services/depthService", depthName);
	pitchName = getParam("Services/pitchService", pitchName);

	double ms = getParam("Tank/Speed", ms);
	tankMotorSpeed = (float)ms;
	refDepth = 0.0;
	refPitch = 0.0;

	double iDepth = getParam("Depth/InitialInput", iDepth);
	double iPitch = getParam("Pitch/InitialInput", iPitch);
	controlInputPitch = iPitch;
	controlInputDepth = iDepth;

	// init controller
	double Kp = getParam("Depth/Kp", Kp);
	double Ki = getParam("Depth/Ki", Ki);
	double Kd = getParam("Depth/Kd", Kd);
	double freq = getParam("Depth/Frequency", freq);
	double dt = 1.0/freq;
	depthCtrl.setGains((float)Kp,(float)Ki,(float)Kd);
	depthCtrl.setTimestep((float)dt);

	Kp = getParam("Pitch/Kp", Kp);
	Ki = getParam("Pitch/Ki", Ki);
	Kd = getParam("Pitch/Kd", Kd);
	freq = getParam("Pitch/Frequency", freq);
	dt = 1.0/freq;
	pitchCtrl.setGains((float)Kp,(float)Ki,(float)Kd);
	pitchCtrl.setTimestep((float)dt);

	// setup logging topics
	logger.createPublisher("ctrlInputs");
	logger.createPublisher("depthLog");
	logger.createPublisher("pitchLog");

	// SERVICES
	// -> subscribe
	connectServices();
	//-> advertise
	depthService = advertiseService("setRefDepth", "setRefDepth", &DiveController::setDepth);
	getDepthService = advertiseService("getRefDepth", "getRefDepth", &DiveController::getRefDepth);
	pitchService = advertiseService("setRefPitch", "setRefPitch", &DiveController::setPitch);
	getPitchService = advertiseService("getRefPitch", "getRefPitch", &DiveController::getRefPitch);
	enableService = advertiseService("enable", "enable", &DiveController::enable);
	disableService = advertiseService("disable", "disable", &DiveController::disable);
	tankPosService = advertiseService("setTankPos", "setTankPos", &DiveController::setTankPosService);
	setGainDepthService = advertiseService("setGainsDepthPID", "setGainsDepthPID", &DiveController::setGainsDepth);
	setGainPitchService = advertiseService("setGainsPitchPID", "setGainsPitchPID", &DiveController::setGainsPitch);

	// TIMER, not starting yet
	depthTimer = n.createTimer(ros::Duration(1.0/((float)freqDepth)), &DiveController::depthCallback, this, 0, 0);
	pitchTimer = n.createTimer(ros::Duration(1.0/((float)freqPitch)), &DiveController::pitchCallback, this, 0, 0);

}

void DiveController::cleanup() {
	NODEWRAP_INFO("Shutting down: <DiveCtrl>");
}

/*
 * depth control loop
 */
void DiveController::depthCallback(const ros::TimerEvent& event) {
	float actualDepth = getDepth();
	float error = refDepth-actualDepth;
	float inputDepth = depthCtrl.updateControl(error);

	setControlInput(inputDepth, controlInputPitch);

	controlInputDepth = inputDepth;

	// Log data
	std::vector<float> depthData(2);
	depthData[0] = refDepth; depthData[1] = actualDepth;
	logger.log(depthData, "depthLog");
}

/*
 * pitch control loop
 */
void DiveController::pitchCallback(const ros::TimerEvent& event) {
	float actualPitch = getPitch();
	float error = refPitch-actualPitch;
	float inputPitch = pitchCtrl.updateControl(error);

	setControlInput(controlInputDepth, inputPitch);

	controlInputPitch = inputPitch;

	// log data
	std::vector<float> pitchData(2);
	pitchData[0] = refPitch; pitchData[1] = actualPitch;
	logger.log(pitchData, "pitchLog");
}

/*
 * perform service calls to the tankCtrl for the tank positions
 */
void DiveController::setTankPosition(ros::ServiceClient client, float input) {
	tankSrv.request.position = input;
	tankSrv.request.speed = tankMotorSpeed;

	if(!client)
		connectServices();

	if(client.call(tankSrv)) {

	} else {
		NODEWRAP_INFO("TankCtrl node not available");
	}
}

/*
 * combine the control inputs from the depth and pitch controller
 */
void DiveController::setControlInput(float inputDepth, float inputPitch) {
	// divide control inputs on front and rear tank
	float inputFront = 0.0;
	float inputRear = 0.0;
	if(inputDepth>0.0 && (inputDepth)<1.0) {
		inputFront = inputDepth-inputPitch;
		inputRear = inputDepth+inputPitch;
	} else {
		if(inputDepth == 0) {
			if(inputPitch>0) {
				inputFront = 2*inputPitch;
				inputRear = 0;
			}
			else {
				inputFront = 0;
				inputRear = 2*inputPitch;
			}
		}
		else {
			inputFront = inputDepth-inputPitch;
			inputRear = inputDepth+inputPitch;
		}
	}

	// set tank inputs
	setTankPosition(controlFrontClient, inputFront);
	setTankPosition(controlRearClient, inputRear);

	// log data
	std::vector<float> inputData(2);
	inputData[0] = inputFront; inputData[1] = inputRear;
	logger.log(inputData, "ctrlInputs");
}

/*
 * handle service call for depth
 */
float DiveController::getDepth() {
	if(!depthClient)
		connectServices();

	if(depthClient.call(depthSrv)) {
		return depthSrv.response.filtered;
	} else {
		NODEWRAP_INFO("depth sensor service not available");
		return 0;
	}
}

/*
 * handle service call for pitch
 */
float DiveController::getPitch() {
	if(!pitchClient)
		connectServices();
	if(pitchClient.call(pitchSrv)) {
		return pitchSrv.response.pitch;
	} else {
		NODEWRAP_INFO("imu pitch service not available");
		return 0;
	}
}

/*
 * connect with the service clients
 */
void DiveController::connectServices() {
	if(!controlFrontClient)
		controlFrontClient = n.serviceClient<naro_tank_ctrl::SetTankPosition>("/"+tankFront+"/setTankPosition", true);
	if(!controlRearClient)
		controlRearClient  = n.serviceClient<naro_tank_ctrl::SetTankPosition>("/"+tankRear+"/setTankPosition", true);
	if(!depthClient)
		depthClient = n.serviceClient<naro_sensor_srvs::GetDepth>("/"+depthName+"/get_depth", true);
	if(!pitchClient)
		pitchClient = n.serviceClient<naro_imu::GetPitch>("/"+pitchName+"/getPitch", true);
}

/*
 * advertised service functions
 */

bool DiveController::setPitch(SetPitch::Request& request, SetPitch::Response& response) {
	refPitch = request.pitch;
	return 1;
}

bool DiveController::setDepth(SetDepth::Request& request, SetDepth::Response& response) {
	refDepth = request.depth;
	return 1;
}

bool DiveController::getRefPitch(GetRefPitch::Request& request, GetRefPitch::Response& response) {
	response.refPitch = refPitch;
	return 1;
}

bool DiveController::getRefDepth(GetRefDepth::Request& request, GetRefDepth::Response& response) {
	response.refDepth = refDepth;
	return 1;
}

bool DiveController::enable(Enable::Request& request, Enable::Response& response) {
	if(request.enablePitchCtrl) {
		pitchTimer.start();
		NODEWRAP_INFO("PitchCtrl ENABLED!");
	}
	if(request.enableDepthCtrl) {
		depthTimer.start();
		NODEWRAP_INFO("DepthCtrl ENABLED!");
	}

	return 1;
}

bool DiveController::disable(Disable::Request& request, Disable::Response& response) {
	pitchTimer.stop();
	depthTimer.stop();
	NODEWRAP_INFO("DiveCtrl disabled!");
	return 1;
}

bool DiveController::setTankPosService(SetTankPos::Request& request, SetTankPos::Response& response) {
	// set tank inputs
	setTankPosition(controlFrontClient, (float)request.position);
	setTankPosition(controlRearClient, (float)request.position);

	return 1;
}


bool DiveController::setGainsDepth(SetGains::Request& request, SetGains::Response& response) {
	depthCtrl.setGains(request.proportional, request.integral, request.differential);
	return 1;
}

bool DiveController::setGainsPitch(SetGains::Request& request, SetGains::Response& response) {
	pitchCtrl.setGains(request.proportional, request.integral, request.differential);
	return 1;
}

}
