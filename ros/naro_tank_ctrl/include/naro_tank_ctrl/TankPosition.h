/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

#include "naro_sensor_srvs/GetPosition.h"
#include "naro_tank_ctrl/SetDirection.h"
#include "naro_tank_ctrl/GetTankPosition.h"

using namespace naro_tank_ctrl;

namespace nodewrap {
class TankPosition:
    public NodeImpl {
        public:
            TankPosition();
            virtual ~TankPosition();

        protected:
            void init();
            void cleanup();

            float position;
            float positionThreshold;
            float totalTicks;
            float ticksOld;
            float speedDirection;

            ros::NodeHandle n;

            ros::Timer timerHallSensor;

            ros::ServiceClient positionClient;
            naro_sensor_srvs::GetPosition posSrv;

            ros::ServiceServer setDirectionService;
            ros::ServiceServer getPositionService;

            void readPosition(const ros::TimerEvent& event);
            bool setSpeedDirection(SetDirection::Request& request, SetDirection::Response& response);
            bool getTankPosition(GetTankPosition::Request& request, GetTankPosition::Response& response);

    };
};