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
#include "naro_smc_srvs/SetSpeed.h"
#include "naro_tank_ctrl/GetTankPosition.h"
#include "naro_tank_ctrl/SetTankPosition.h"

using namespace naro_tank_ctrl;

namespace nodewrap {

class TankCtrl:
		public NodeImpl {

		public:
			TankCtrl();
			virtual ~TankCtrl();

		protected:
			void init();
			void cleanup();

			float position;
			float positionRequest;
			float positionThreshold;
			float totalTicks;
			float ticksOld;
			float speedDirection;
			bool finalPosition;

			ros::NodeHandle n;

			ros::Timer timerHallSensor;
			ros::Timer checkPositionTimer;

			ros::ServiceClient positionClient;
			ros::ServiceClient speedClient;
			naro_smc_srvs::SetSpeed speedSrv;
			naro_sensor_srvs::GetPosition posSrv;

			ros::ServiceServer getTankPositionService;
			ros::ServiceServer setTankPositionService;

			void readPosition(const ros::TimerEvent& event);
			void checkPosition(const ros::TimerEvent& event);
			bool getTankPosition(GetTankPosition::Request& request, GetTankPosition::Response& response);
			bool setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response);

			void callSpeedClient(float speed);

			void startup();

};

};
