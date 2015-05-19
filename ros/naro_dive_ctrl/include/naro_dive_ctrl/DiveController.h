/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include "ros/ros.h"
#include "naro_dive_ctrl/PIDController.h"
#include "naro_tank_ctrl/SetTankPosition.h"
#include "naro_sensor_srvs/GetDepth.h"
#include "naro_dive_ctrl/SetDepth.h"
#include "naro_dive_ctrl/SetPitch.h"
#include "naro_dive_ctrl/GetRefDepth.h"
#include "naro_dive_ctrl/GetRefPitch.h"
#include "naro_dive_ctrl/Enable.h"
#include "naro_dive_ctrl/Disable.h"
#include "naro_imu/GetPitch.h"


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

		ros::NodeHandle n;

		PIDController depthCtrl;
		PIDController pitchCtrl;

		ros::Timer depthTimer;
		ros::Timer pitchTimer;

		ros::ServiceClient controlFrontClient;
		ros::ServiceClient controlRearClient;
		naro_tank_ctrl::SetTankPosition tankSrv;

		ros::ServiceClient depthClient;
		naro_sensor_srvs::GetDepth depthSrv;

		ros::ServiceClient pitchClient;
		naro_imu::GetPitch pitchSrv;

		ros::ServiceServer depthService;
		ros::ServiceServer getDepthService;
		ros::ServiceServer pitchService;
		ros::ServiceServer getPitchService;
		ros::ServiceServer enableService;
		ros::ServiceServer disableService;

		double tankMotorSpeed;
		double refDepth;
		double refPitch;
		double controlInputDepth;
		double controlInputPitch;

		void depthCallback(const ros::TimerEvent& event);
		void pitchCallback(const ros::TimerEvent& event);
		void setTankPosition(ros::ServiceClient client, double input);
		void setControlInput(double inputDepth, double inputPitch);
		double getDepth();
		double getPitch();

		bool setPitch(SetPitch::Request& request, SetPitch::Response& response);
		bool getRefPitch(GetRefPitch::Request& request, GetRefPitch::Response& response);
		bool setDepth(SetDepth::Request& request, SetDepth::Response& response);
		bool getRefDepth(GetRefDepth::Request& request, GetRefDepth::Response& response);
		bool enable(Enable::Request& request, Enable::Response& response);
		bool disable(Disable::Request& request, Disable::Response& response);
};

};
