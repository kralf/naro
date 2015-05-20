/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include "ros/ros.h"

#include "naro_sensor_srvs/GetPosition.h"
#include "naro_smc_srvs/SetSpeed.h"
#include "naro_smc_srvs/GetLimits.h"
#include "naro_tank_ctrl/GetTankPosition.h"
#include "naro_tank_ctrl/SetTankPosition.h"
#include "naro_tank_ctrl/SetDirection.h"
#include <std_srvs/Empty.h>

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

			float positionRequest;
			float positionThreshold;
			float totalTicks;
			float ticksOld;
			float speedDirection;
			bool finalPosition;
			std::string nodeName;

			ros::NodeHandle n;

			ros::Timer timerHallSensor;
			ros::Timer checkPositionTimer;

			ros::ServiceClient positionClient;
			naro_tank_ctrl::GetTankPosition posSrv;
			ros::ServiceClient directionClient;
			naro_tank_ctrl::SetDirection directionSrv;
			ros::ServiceClient resetClient;
			std_srvs::Empty resetSrv;

			ros::ServiceClient speedClient;
			naro_smc_srvs::SetSpeed speedSrv;
			ros::ServiceClient limitClient;
			naro_smc_srvs::GetLimits limitSrv;

			std::string smcServerName;
			std::string tankPosName;

			ros::ServiceServer setTankPositionService;
			ros::ServiceServer resetTankPositionService;

			void readPosition(const ros::TimerEvent& event);
			void checkPosition(const ros::TimerEvent& event);
			bool setTankPosition(SetTankPosition::Request& request, SetTankPosition::Response& response);
			bool resetTankPosition(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
			bool resetPosition();

			void setSpeed(float speed);
			void setDirection(float direction);
			float getPosition();
			int getLimit();

			float clamp(float value, float min, float max);

			void startup();

			void connectServices();

};

};
