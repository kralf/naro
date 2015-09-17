/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "naro_tank_ctrl/SetTankPosition.h"
#include "naro_sensor_srvs/GetDepth.h"
#include "naro_imu/GetPitch.h"
#include "naro_imu/GetState.h"
#include "naro_monitoring/LoggingClass.h"

#include "naro_dive_ctrl/PIDController.h"
#include "naro_dive_ctrl/SetDepth.h"
#include "naro_dive_ctrl/SetPitch.h"
#include "naro_dive_ctrl/SetRefState.h"
#include "naro_dive_ctrl/GetRefDepth.h"
#include "naro_dive_ctrl/GetRefPitch.h"
#include "naro_dive_ctrl/EnablePIDCtrl.h"
#include "naro_dive_ctrl/EnableLQRCtrl.h"
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
		ros::Timer lqrTimer;

		ros::ServiceClient controlFrontClient;
		ros::ServiceClient controlRearClient;
		naro_tank_ctrl::SetTankPosition tankSrv;

		ros::ServiceClient depthClient;
		naro_sensor_srvs::GetDepth depthSrv;

		ros::ServiceClient pitchClient;
		naro_imu::GetPitch pitchSrv;

		ros::ServiceClient imuStateClient;
		naro_imu::GetState imuStateSrv;

		std::string tankFront;
		std::string tankRear;
		std::string depthName;
		std::string imuName;

		ros::ServiceServer depthService;
		ros::ServiceServer getDepthService;
		ros::ServiceServer pitchService;
		ros::ServiceServer getPitchService;
		ros::ServiceServer enablePIDService;
		ros::ServiceServer enableLQRService;
		ros::ServiceServer disableService;
		ros::ServiceServer tankPosService;
		ros::ServiceServer setGainDepthService;
		ros::ServiceServer setGainPitchService;

		LoggingClass logger;

		float tankMotorSpeed;
		float refDepth;
		float refPitch;
		float refState[4];
		float state[4];
		float controlInputDepth;
		float controlInputPitch;
		float input0;

		float kalmanGain[2][4];

		void depthCallback(const ros::TimerEvent& event);
		void pitchCallback(const ros::TimerEvent& event);
		void lqrCallback(const ros::TimerEvent& event);
		void setTankPosition(ros::ServiceClient client, float input);
		void setControlInput(float inputDepth, float inputPitch);
		float getDepth();
		float getPitch();
		void getState();
		void connectServices();
		void disableFunc();

		bool setPitch(SetPitch::Request& request, SetPitch::Response& response);
		bool getRefPitch(GetRefPitch::Request& request, GetRefPitch::Response& response);
		bool setDepth(SetDepth::Request& request, SetDepth::Response& response);
		bool getRefDepth(GetRefDepth::Request& request, GetRefDepth::Response& response);
		bool setRefState(SetRefState::Request& request, SetRefState::Response& response);
		bool enablePIDCtrl(EnablePIDCtrl::Request& request, EnablePIDCtrl::Response& response);
		bool enableLQRCtrl(EnableLQRCtrl::Request& request, EnableLQRCtrl::Response& response);
		bool disable(Disable::Request& request, Disable::Response& response);
		bool setTankPosService(SetTankPos::Request& request, SetTankPos::Response& response);
		bool setGainsDepth(SetGains::Request& request, SetGains::Response& response);
		bool setGainsPitch(SetGains::Request& request, SetGains::Response& response);
};

};
