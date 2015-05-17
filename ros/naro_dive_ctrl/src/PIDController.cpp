/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_dive_ctrl/PIDController.h"

PIDController::PIDController() {
	Kp = 1.0;
	Ki = 0.0;
	Kd = 0.0;
	integralError = 0.0;
	lastError = 0.0;
	timeStep = 1.0;
}

void PIDController::setGains(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PIDController::setTimestep(double dt) {
	this->timeStep = dt;
}

double PIDController::updateControl(double error) {
	// update errors
	this->integralError += error*timeStep;
	double derivativeError = 0.0;
	if(lastError != error)
		derivativeError = (error-lastError)/timeStep;

	bool output = Kp*error+Ki*integralError+Kd*derivativeError;
	lastError = error;

	return output;
}
