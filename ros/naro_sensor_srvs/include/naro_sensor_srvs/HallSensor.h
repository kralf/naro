/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */
#include <thread>
#include <string>
#include "naro_sensor_srvs/GPIOClass.h"
#include "ros/ros.h"

using namespace std;

class HallSensor {
	public:
		HallSensor();
		HallSensor(string gpioId);
		~HallSensor();
		int getCount();
		void resetCount();
		void setFrequency(int freq);
		int getFrequency();

	private:
		int counter;
		float duration;
		bool running;
		string gpioName;
		GPIOClass* gpio;
		ros::Timer timer;
		ros::NodeHandle n;
		ros::Duration period;
		void readSensor(const ros::TimerEvent& event);
};
