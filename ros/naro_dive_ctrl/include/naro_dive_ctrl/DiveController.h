/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include "naro_dive_ctrl/PIDController.h"
#include "naro_tank_ctrl/SetTankPosition.h"

using namespace naro_dive_ctrl;

namespace nodewrap {

class DiveController:
	public NodeImpl {

	public:
		DiveController();
		~DiveController();

	protected:
		void init();
		void cleanup();

		PIDController depthCtrl;
		PIDController pitchCtrl;

		ros::Timer depthTimer;
		ros::Timer pitchTimer;

		ros::ServiceClient *controlFrontClient;
		ros::ServiceClient *controlRearClient;
		naro_tank_ctrl::SetTankPosition tankSrv;

		void setControlInput(ros::ServiceClient& client, double input);
};

};
