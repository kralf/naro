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

	tankMotorSpeed = getParam("Tank/Speed", tankMotorSpeed);
	refDepth = 0.0;
	refPitch = 0.0;

	// init controller
	double Kp = getParam("Depth/Kp", Kp);
	double Ki = getParam("Depth/Ki", Ki);
	double Kd = getParam("Depth/Kd", Kd);
	double freq = getParam("Depth/Frequency", freq);
	double dt = 1.0/freq;
	depthCtrl.setGains(Kp,Ki,Kd);
	depthCtrl.setTimestep(dt);

	Kp = getParam("Pitch/Kp", Kp);
	Ki = getParam("Pitch/Ki", Ki);
	Kd = getParam("Pitch/Kd", Kd);
	freq = getParam("Depth/Frequency", freq);
	dt = 1.0/freq;
	pitchCtrl.setGains(Kp,Ki,Kd);
	pitchCtrl.setTimestep(dt);

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

	// TIMER, not starting
	depthTimer = n.createTimer(ros::Duration(1.0/freqDepth), &DiveController::depthCallback, this, 0, 0);
	pitchTimer = n.createTimer(ros::Duration(1.0/freqPitch), &DiveController::pitchCallback, this, 0, 0);

}

void DiveController::cleanup() {
	NODEWRAP_INFO("Shutting down: <DiveCtrl>");
}

void DiveController::depthCallback(const ros::TimerEvent& event) {
	double actualDepth = getDepth();
	double error = refDepth-actualDepth;
	double inputDepth = depthCtrl.updateControl(error);

	std::vector<float> depthData{(float)refDepth, (float)actualDepth};
	logger.log(depthData, "depthLog");

	setControlInput(inputDepth, controlInputPitch);

	controlInputDepth = inputDepth;
}

void DiveController::pitchCallback(const ros::TimerEvent& event) {
	double actualPitch = getPitch();
	double error = refPitch-actualPitch;
	double inputPitch = pitchCtrl.updateControl(error);

	std::vector<float> pitchData{(float)refPitch, (float)actualPitch};
	logger.log(pitchData, "pitchLog");

	NODEWRAP_INFO("Pitch error: %f", (float)error);
	NODEWRAP_INFO("Input Pitch: %f", (float)inputPitch);

	setControlInput(controlInputDepth, inputPitch);

	controlInputPitch = inputPitch;
}

void DiveController::setTankPosition(ros::ServiceClient client, double input) {
	tankSrv.request.position = input;
	tankSrv.request.speed = tankMotorSpeed;

	if(!client)
		connectServices();

	if(client.call(tankSrv)) {

	} else {
		NODEWRAP_INFO("TankCtrl node not available");
	}
}

void DiveController::setControlInput(double inputDepth, double inputPitch) {
	// divide control inputs on front and rear tank
	double inputFront = 0.0;
	double inputRear = 0.0;
	if(inputDepth>0 && (inputDepth+inputPitch)<1) {
		inputFront = inputDepth+inputPitch;
		inputRear = inputDepth-inputPitch;
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
			inputFront = inputDepth+inputPitch;
			inputRear = inputDepth-inputPitch;
		}
	}

	// set tank inputs
	setTankPosition(controlFrontClient, inputFront);
	setTankPosition(controlRearClient, inputRear);

	NODEWRAP_INFO("Input Front: %f", (float)inputFront);
	NODEWRAP_INFO("Input Rear: %f", (float)inputRear);
	std::vector<float> inputData{(float)inputFront, (float)inputRear};
	logger.log(inputData, "ctrlInputs");
}

double DiveController::getDepth() {
	if(!depthClient)
		connectServices();

	if(depthClient.call(depthSrv)) {
		return depthSrv.response.filtered;
	} else {
		NODEWRAP_INFO("depth sensor service not available");
		return 0;
	}
}

double DiveController::getPitch() {
	if(!pitchClient)
		connectServices();
	if(pitchClient.call(pitchSrv)) {
		return pitchSrv.response.pitch;
	} else {
		NODEWRAP_INFO("imu pitch service not available");
		return 0;
	}
}

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

}
