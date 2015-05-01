/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */
#include <thread>
#include <string>
#include "naro_sensor_srvs/GPIOClass.h"

using namespace std;

class HallSensor {
	public:
		HallSensor();
		HallSensor(string gpioId);
		int getCount();
		void resetCount();
		void setFrequency(int freq);
		int getFrequency();
		void startReadingSensor();
		void join();

	private:
		int counter;
		int waitMicroSec;
		bool running;
		string gpioName;
		GPIOClass gpio;
		thread readingThread;

		void work();
};
