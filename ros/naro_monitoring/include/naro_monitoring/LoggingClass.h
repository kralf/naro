/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <iostream>

using namespace std;

class LoggingClass {
public:
	LoggingClass();
	~LoggingClass();

	ros::NodeHandle n;
	map<string,ros::Publisher> publisher;

	void log(vector<float> vector, string topic);
	void createPublisher(string topic);

private:

};
