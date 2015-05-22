/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include <ros/ros.h>
#include <naro_tank_ctrl/SetTankPosition.h>
#include <naro_sensor_srvs/GetDepth.h>
#include <naro_imu/GetPitch.h>
#include <naro_monitoring/LoggingClass.h>
#include <std_srvs/Empty.h>

#include "naro_dive_ctrl/PIDController.h"
#include "naro_dive_ctrl/SetDepth.h"
#include "naro_dive_ctrl/SetPitch.h"
#include "naro_dive_ctrl/GetRefDepth.h"
#include "naro_dive_ctrl/GetRefPitch.h"
#include "naro_dive_ctrl/Enable.h"
#include "naro_dive_ctrl/Disable.h"
#include "naro_dive_ctrl/SetGains.h"
#include "naro_dive_ctrl/SetTankPos.h"

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

		ros::ServiceClient resetFrontCient;
		ros::ServiceClient resetRearClient;
		std_srvs::Empty resetSrv;

		ros::ServiceClient depthClient;
		naro_sensor_srvs::GetDepth depthSrv;

		ros::ServiceClient pitchClient;
		naro_imu::GetPitch pitchSrv;

		std::string tankFront;
		std::string tankRear;
		std::string depthName;
		std::string pitchName;

		ros::ServiceServer depthService;
		ros::ServiceServer getDepthService;
		ros::ServiceServer pitchService;
		ros::ServiceServer getPitchService;
		ros::ServiceServer enableService;
		ros::ServiceServer disableService;
		ros::ServiceServer tankPosService;
		ros::ServiceServer resetTankPosService;
		ros::ServiceServer setGainDepthService;
		ros::ServiceServer setGainPitchService;

		LoggingClass logger;

		float tankMotorSpeed;
		float refDepth;
		float refPitch;
		float controlInputDepth;
		float controlInputPitch;

		void depthCallback(const ros::TimerEvent& event);
		void pitchCallback(const ros::TimerEvent& event);
		void setTankPosition(ros::ServiceClient client, float input);
		void setControlInput(float inputDepth, float inputPitch);
		float getDepth();
		float getPitch();
		void connectServices();

		bool setPitch(SetPitch::Request& request, SetPitch::Response& response);
		bool getRefPitch(GetRefPitch::Request& request, GetRefPitch::Response& response);
		bool setDepth(SetDepth::Request& request, SetDepth::Response& response);
		bool getRefDepth(GetRefDepth::Request& request, GetRefDepth::Response& response);
		bool enable(Enable::Request& request, Enable::Response& response);
		bool disable(Disable::Request& request, Disable::Response& response);
		bool setTankPosService(SetTankPos::Request& request, SetTankPos::Response& response);
		bool setGainsDepth(SetGains::Request& request, SetGains::Response& response);
		bool setGainsPitch(SetGains::Request& request, SetGains::Response& response);
		bool resetTankPos(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

};
