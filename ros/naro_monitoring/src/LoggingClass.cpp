/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include "naro_monitoring/LoggingClass.h"


LoggingClass::LoggingClass() {
	startTime = ros::Time::now();

}

LoggingClass::~LoggingClass() {}


void LoggingClass::log(vector<float> vector, string topic) {
	if(publisher[topic]) {
		// add time to logging data
		float time = (ros::Time::now()-startTime).toSec();
		vector.push_back(time);

		std_msgs::Float32MultiArray msg;
		msg.data = vector;
		publisher[topic].publish(msg);
	} else {
		cout << "no publisher on topic " << topic << " available. Create new one with createPublisher()" << endl;
	}
}

void LoggingClass::createPublisher(string topic) {
	publisher[topic] = n.advertise<std_msgs::Float32MultiArray>("/naro_monitoring/"+topic, 1000);
}

