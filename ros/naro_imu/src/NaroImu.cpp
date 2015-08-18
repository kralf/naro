#include "naro_imu/NaroImu.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <math.h>

NODEWRAP_EXPORT_CLASS(naro_imu, nodewrap::NaroImu)

namespace nodewrap {

NaroImu::NaroImu() {}
NaroImu::~NaroImu() {}

/* --- Methods --- */

void NaroImu::init() {
	NODEWRAP_INFO("Initialize <NaroImu>");

	resetPose();
	g = 9.81;

	subscriber = subscribe("bla", "/imu/data", 100, &NaroImu::getImuData);
	publisher = advertise<geometry_msgs::Pose>("bla", "/base/pose", 10);

	// Service
	pitchService = advertiseService("getPitch", "getPitch", &NaroImu::getPitch);
	stateService = advertiseService("getState", "getState", &NaroImu::getState);
}

void NaroImu::cleanup() {
	NODEWRAP_INFO("Shutting down <NaroImu>");
}

void NaroImu::getImuData(const sensor_msgs::Imu::ConstPtr& msg) {

	geometry_msgs::Pose poseBase = geometry_msgs::Pose();
	// convert sensor_msgs
	// orientation
	geometry_msgs::QuaternionStamped orientation_IMU = geometry_msgs::QuaternionStamped();
	orientation_IMU.header = msg->header;
	orientation_IMU.quaternion = msg->orientation;

	angularVel = msg->angular_velocity;
	linearAcc = msg->linear_acceleration;

	// Transform
	try {
		geometry_msgs::QuaternionStamped orientation_base;

		// orientation
		transformQuaternion("base_link", ros::Time(0), orientation_IMU, orientation_base);
		poseBase.orientation = orientation_base.quaternion;
	}
	catch(tf::TransformException& ex) {
			//NODEWRAP_ERROR("Transform base: %s", ex.what());
		}

	// position
	geometry_msgs::Point point = geometry_msgs::Point();
	integrateImuToPosition(msg, point);
	poseBase.position = point;

	//publisher.publish(orientation_base);
	lastPose = poseBase;
	publisher.publish(poseBase);

}

/**
 * Transform quaternion to target_frame at specific time
 */
void NaroImu::transformQuaternion(const std::string& target_frame, const ros::Time& time,
		const geometry_msgs::QuaternionStamped& msg_in, geometry_msgs::QuaternionStamped& msg_out) {

	 tf::assertQuaternionValid(msg_in.quaternion);
	 tf::Stamped<tf::Quaternion> pin, pout;
	 tf::quaternionStampedMsgToTF(msg_in, pin);
	 tf::StampedTransform transform;
	 transformer.lookupTransform(target_frame, msg_in.header.frame_id, time, transform);
	 pout.setData( transform * pin);
	 pout.stamp_ = transform.stamp_;
	 pout.frame_id_ = target_frame;
	 tf::quaternionStampedTFToMsg(pout, msg_out);
}

/**
 * Compute position of base from integration of IMU
 */
void NaroImu::integrateImuToPosition(const sensor_msgs::Imu::ConstPtr& msg, geometry_msgs::Point& position) {
	tf::Vector3 accel_IMU;
	tf::vector3MsgToTF(msg->linear_acceleration, accel_IMU);

	//subtract gravity
	tf::Vector3 gravity;
	tf::vector3MsgToTF(getGravity("imu_link"), gravity);
	accel_IMU = accel_IMU+gravity;

	newTime = ros::Time::now().toNSec();
	double dt = (newTime-oldTime)*pow(10,-9);
	if(startIntegration) dt=0;
	// calculate velocity and position
	newVelocity = accel_IMU*dt+oldVelocity;
	newPosition = 0.5*accel_IMU*dt*dt+oldVelocity+oldPosition;

	// store values
	oldTime = newTime;
	oldPosition = newPosition;
	oldVelocity = newVelocity;

	// convert to point
	geometry_msgs::Vector3 tmpVec;
	tf::vector3TFToMsg(newPosition, tmpVec);
	float x = tmpVec.x/pow(10,6);
	float y = tmpVec.y/pow(10,6);
	float z = tmpVec.z/pow(10,6);
	position.x = x;
	position.y = y;
	position.z = z;

	startIntegration = false;

}

/**
 * get gravity vector in specific target_frame
 */

geometry_msgs::Vector3 NaroImu::getGravity(const std::string& target_frame) {
	tf::Stamped<tf::Vector3> gin, gout;
	geometry_msgs::Vector3Stamped gravityMap;
	gravityMap.vector.z = -g;
	tf::vector3StampedMsgToTF(gravityMap, gin);

	tf::StampedTransform transform;
	try {
		transformer.lookupTransform(target_frame, "map", ros::Time(0), transform);
	} catch(tf::TransformException& ex) {
			NODEWRAP_ERROR("Transform gravity: %s", ex.what());
		}
	gout.setData(transform*gin);

	tf::Vector3 origin = tf::Vector3(0,0,0);
	tf::Vector3 output = (transform * gin) - (transform * origin);
	gout.setData( output);

	gout.stamp_ = transform.stamp_;
	gout.frame_id_ = target_frame;

	geometry_msgs::Vector3 gravity;
	tf::vector3TFToMsg(gout, gravity);
	return gravity;
}

/**
 * reset pose of imu
 */

void NaroImu::resetPose() {
	oldTime = 0.0;
	oldPosition.setValue(0.0,0.0,0.0);
	oldVelocity.setValue(0.0,0.0,0.0);
	startIntegration = true;
}

/**
 * pitch service
 */
bool NaroImu::getPitch(GetPitch::Request& request, GetPitch::Response& response) {
	 double roll, pitch, yaw;
	 tf::Quaternion quat;
	 tf::quaternionMsgToTF(lastPose.orientation, quat);
	 tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	 response.pitch = -pitch;
	 return 1;
}

bool NaroImu::getState(GetState::Request& request, GetState::Response& response) {
	 double roll, pitch, yaw;
	 tf::Quaternion quat;
	 tf::quaternionMsgToTF(lastPose.orientation, quat);
	 tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	 response.pitch = -pitch;

	 response.q_dot = -angularVel.y;
	 response.w_dot = linearAcc.z-9.81;
}

}
