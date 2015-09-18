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
	double freqLQR = getParam("LQR/Frequeny", freqLQR);
	tankFront = getParam("Services/tankFrontCtrl", tankFront);
	tankRear = getParam("Services/tankRearCtrl", tankRear);
	depthName = getParam("Services/depthService", depthName);
	imuName = getParam("Services/imuService", imuName);

	double ms = getParam("Tank/Speed", ms);
	tankMotorSpeed = (float)ms;

	double iDepth = getParam("Depth/InitialInput", iDepth);
	double iPitch = getParam("Pitch/InitialInput", iPitch);
	controlInputPitch = iPitch;
	controlInputDepth = iDepth;

	// init PID controller
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

	// init LQR controller
	double i0 = getParam("LQR/Input0", i0); // neutral buoyant
	input0 = (float)i0;
	// init kalman Gain
	std::vector<double> l1 = getParam("LQR/KalmanGain_Line1", l1);
	std::vector<double> l2 = getParam("LQR/KalmanGain_Line2", l2);
	std::vector<double> ref = getParam("LQR/RefState", ref); // [depth, w_dot, pitch, q_dot]
	for(int i=0; i!=l1.size(); ++i) {
		kalmanGain[0][i] = l1[i];
		kalmanGain[1][i] = l2[i];
		refState[i] = ref[i];
	}
	refDepth = refState[0];
	refPitch = refState[2];


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
	enablePIDService = advertiseService("enablePIDCtrl", "enablePIDCtrl", &DiveController::enablePIDCtrl);
	enableLQRService = advertiseService("enableLQRCtrl", "enableLQRCtrl", &DiveController::enableLQRCtrl);
	disableService = advertiseService("disableService", "disableService", &DiveController::disable);
	tankPosService = advertiseService("setTankPos", "setTankPos", &DiveController::setTankPosService);
	setGainDepthService = advertiseService("setGainsDepthPID", "setGainsDepthPID", &DiveController::setGainsDepth);
	setGainPitchService = advertiseService("setGainsPitchPID", "setGainsPitchPID", &DiveController::setGainsPitch);

	// TIMER, not starting yet
	depthTimer = n.createTimer(ros::Duration(1.0/((float)freqDepth)), &DiveController::depthCallback, this, 0, 0);
	pitchTimer = n.createTimer(ros::Duration(1.0/((float)freqPitch)), &DiveController::pitchCallback, this, 0, 0);
	lqrTimer = n.createTimer(ros::Duration(1.0/((float)freqLQR)), &DiveController::lqrCallback, this, 0, 0);

}

void DiveController::cleanup() {

	disableFunc();
	NODEWRAP_INFO("Shutting down: <DiveCtrl>");
}

/*
 * =============================================================
 * PID CONTROLLER
 * =============================================================
 */

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
 * =============================================================
 * LQR CONTROLLER
 * =============================================================
 */

void DiveController::lqrCallback(const ros::TimerEvent& event) {
	getState(); // update state
	// calc error
	float error[4];
	for(int i=0; i<4; i++) {
		error[i] = refState[i]-state[i];
	}
	// calculate new input
	float input[2];
	for(int i=0; i<2; i++) {
		for(int j=0; j<4; j++) {
			input[i] = kalmanGain[i][j]*error[j];
		}
	}

	// add feed-forward term
	input[0] += input0;
	input[1] += input0;

	// set tank inputs
	setTankPosition(controlFrontClient, input[0]);
	setTankPosition(controlRearClient, input[1]);

	// log data
	std::vector<float> inputData(2);
	inputData[0] = input[0]; inputData[1] = input[1];
	logger.log(inputData, "ctrlInputs");
}

/*
 * =============================================================
 * SERVICE FUNCTIONS
 * =============================================================
 */

void DiveController::disableFunc() {
	pitchTimer.stop();
	depthTimer.stop();
	lqrTimer.stop();
	NODEWRAP_INFO("DiveCtrl disabled!");
}

// ======= SERVICE CALLS ========

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
 * handle service call for state [depth, w_dot, pitch, q_dot]
 */
void DiveController::getState() {
	// get depth
	state[0] = getDepth();

	// get state information from imu
	if(!imuStateClient)
		connectServices();
	if(imuStateClient.call(imuStateSrv)) {
		state[1] = imuStateSrv.response.w_dot;
		state[2] = imuStateSrv.response.pitch;
		state[3] = imuStateSrv.response.q_dot;
	} else {
		NODEWRAP_INFO("imu state service not available");
	}
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
		pitchClient = n.serviceClient<naro_imu::GetPitch>("/"+imuName+"/getPitch", true);
	if(!imuStateClient)
		imuStateClient = n.serviceClient<naro_imu::GetState>("/"+imuName+"/getState", true);
}

// ======= ADVERTISE SERVICE FUNCTIONS ========

bool DiveController::setPitch(SetPitch::Request& request, SetPitch::Response& response) {
	refPitch = request.pitch;
	refState[2] = refPitch;
	return 1;
}

bool DiveController::setDepth(SetDepth::Request& request, SetDepth::Response& response) {
	refDepth = request.depth;
	refState[0] = refDepth;
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

bool DiveController::setRefState(SetRefState::Request& request, SetRefState::Response& response) {
	refDepth = request.depth;
	refState[0] = refDepth;
	refState[1] = request.w_dot;
	refPitch = request.pitch;
	refState[2] = refPitch;
	refState[3] = request.q_dot;

}

bool DiveController::enablePIDCtrl(EnablePIDCtrl::Request& request, EnablePIDCtrl::Response& response) {
	if(request.enablePitchCtrl) {
		pitchTimer.start();
		NODEWRAP_INFO("Pitch PID-control ENABLED!");
	}
	if(request.enableDepthCtrl) {
		depthTimer.start();
		NODEWRAP_INFO("Depth PID-control ENABLED!");
	}

	return 1;
}

bool DiveController::enableLQRCtrl(EnableLQRCtrl::Request& request, EnableLQRCtrl::Response& response) {
	// stop PID ctrl
	pitchTimer.stop();
	depthTimer.stop();

	// start LQR ctrl
	if(request.enableLQRCtrl) {
		lqrTimer.start();
		NODEWRAP_INFO("LQR control ENABLED!");
	}

	return 1;

}

bool DiveController::disable(Disable::Request& request, Disable::Response& response) {
	disableFunc();
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
