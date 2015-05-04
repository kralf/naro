/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */
#include "naro_sensor_srvs/HallSensor.h"
#include <iostream>
#include <unistd.h>


using namespace std;

HallSensor::HallSensor() {};

HallSensor::HallSensor(string gpioId) {
	running = true;
	waitMicroSec = 100000;
	counter = 0;
	// Init GPIO
	gpioName = gpioId;
	gpio = new GPIOClass(gpioName);
}

HallSensor::~HallSensor() {
	delete gpio;
}

void HallSensor::startReadingSensor()  {
	this->running = true;
	readingThread = thread(&HallSensor::work, this);
}

void HallSensor::work() {
	string valueOld = "0";
	string value;
	while(running){
		gpio->getval_gpio(value);
		if((valueOld == "0") && (value == "1")) {
			counter++;
		}
		valueOld = value;
		usleep(this->waitMicroSec);
	}
}

void HallSensor::join() {
	this->running = false;
	readingThread.join();
}

int HallSensor::getCount() {
	return this->counter;
}

void HallSensor::resetCount() {
	this->counter = 0;
}

void HallSensor::setFrequency(int freq) {
	int msec = 1000000/freq; // convert from Hz -> microsec
	this->waitMicroSec = msec;
}

int HallSensor::getFrequency() {
	return this->waitMicroSec*1000000;
}
