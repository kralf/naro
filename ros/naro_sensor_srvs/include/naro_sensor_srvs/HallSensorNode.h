/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include "naro_sensor_srvs/HallSensor.h"
#include "naro_sensor_srvs/GetPosition.h"
#include "naro_sensor_srvs/SetFrequency.h"
#include "naro_sensor_srvs/GetFrequency.h"

using namespace naro_sensor_srvs;
namespace nodewrap {

class HallSensorNode:
		public NodeImpl{

		public:
			HallSensorNode();
			virtual ~HallSensorNode();

		protected:

			ros::ServiceServer getPositionService;
			ros::ServiceServer setFrequencyService;
			ros::ServiceServer resetCounterService;
			ros::ServiceServer getFrequencyService;

			HallSensor* sensor;

			string name;
			

			void init();
			void cleanup();

			// return position of piston tank
			bool getPosition(GetPosition::Request& request, GetPosition::Response& response);

			// set reading frequency of HallSensor
			bool setFrequency(SetFrequency::Request& request, SetFrequency::Response& response);

			bool getFrequency(GetFrequency::Request& request, GetFrequency::Response& response);

			// reset the counter
			bool resetCounter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);


};

};
