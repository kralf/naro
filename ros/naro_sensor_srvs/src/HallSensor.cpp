/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */
#include "naro_sensor_srvs/HallSensor.h"
#include <unistd.h>
#include <iostream>


using namespace std;

HallSensor::HallSensor() {};

HallSensor::HallSensor(string gpioId) {
	running = true;
	duration = 1.0/300.0;
	counter = 0;
	valueOld = "0";

	// Init GPIO
	gpioName = gpioId;
	gpio = new GPIOClass(gpioName);

	// setup ros timer
	timer = n.createTimer(ros::Duration(duration), &HallSensor::readSensor, this);
}

HallSensor::~HallSensor() {
	delete gpio;
}

void HallSensor::readSensor(const ros::TimerEvent& event) {
	string value;

	gpio->getval_gpio(value);
	if((valueOld == "0") && (value == "1")) {
		this->counter++;
		//std::cout << counter << endl;
	}
	valueOld = value;

}

int HallSensor::getCount() {
	return this->counter;
}

void HallSensor::resetCount() {
	this->counter = 0.0;
}

void HallSensor::setFrequency(int freq) {
	this->duration = 1.0/freq; // convert from Hz -> seconds
	this->timer.setPeriod(ros::Duration(duration));
}

int HallSensor::getFrequency() {
	return (1.0/duration);
}
