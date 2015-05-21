/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_monitoring/LoggingClass.h"


LoggingClass::LoggingClass() {

}

LoggingClass::~LoggingClass() {}


void LoggingClass::log(vector<float> vector, string topic) {
	if(publisher[topic]) {
		std_msgs::Float32MultiArray msg;
		msg.data = vector;
		publisher[topic].publish(msg);
	} else {
		cout << "no publisher on topic " << topic << " available. Create new one with createPublisher()" << endl;
	}
}

void LoggingClass::createPublisher(string topic) {
	publisher[topic] = n.advertise<std_msgs::Float32MultiArray>(topic, 1000);
}

