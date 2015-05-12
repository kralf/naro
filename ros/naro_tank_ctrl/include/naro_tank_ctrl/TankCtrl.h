/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include "std_msgs/String.h"

#include "naro_sensor_srvs/GetPosition.h"

//using namespace naro_tank_ctrl;

namespace nodewrap {

class TankCtrl:
		public NodeImpl {

		public:
			TankCtrl();
			virtual ~TankCtrl();

		protected:
			void init();
			void cleanup();

			double position;
			int totalTicks;
			int ticksOld;
			int speedDirection;

			ros::NodeHandle n;
			ros::Subscriber subscribeCtrl;
			ros::Timer timerPosition;
			ros::ServiceClient positionClient;
			naro_sensor_srvs::GetPosition srv;

			void handleCtrlInput(const std_msgs::String::ConstPtr& msg);
			void readPosition(const ros::TimerEvent& event);

};

};
