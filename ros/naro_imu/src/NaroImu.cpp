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
	subscriber = subscribe("bla", "/imu/data", 100, &NaroImu::getImuData);
	//publisher = advertise<geometry_msgs::QuaternionStamped>("bla", "/base/orientation", 10);
	publisher = advertise<geometry_msgs::Pose>("bla", "/base/pose", 10);
	resetPosition();
}

void NaroImu::cleanup() {
	NODEWRAP_INFO("Shutting down <NaroImu>");
}

void NaroImu::getImuData(const sensor_msgs::Imu::ConstPtr& msg) {
	// convert sensor_msgs
	// orientation
	geometry_msgs::QuaternionStamped orientation_IMU = geometry_msgs::QuaternionStamped();
	orientation_IMU.header = msg->header;
	orientation_IMU.quaternion = msg->orientation;

	// Transform
	try {
		geometry_msgs::QuaternionStamped orientation_base;

		// orientation
		transformQuaternion("base_link", ros::Time(0), orientation_IMU, orientation_base);
		// position
		geometry_msgs::Point point = geometry_msgs::Point();
		integrateIMUtoPosition(msg, point);

		geometry_msgs::Pose poseBase = geometry_msgs::Pose();

		poseBase.position = point;
		poseBase.orientation = orientation_base.quaternion;

		//publisher.publish(orientation_base);
		publisher.publish(poseBase);

	}
	catch(tf::TransformException& ex) {
		NODEWRAP_ERROR("Received an exception: %s", ex.what());
	}

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

void NaroImu::integrateIMUtoPosition(const sensor_msgs::Imu::ConstPtr& msg, geometry_msgs::Point& position) {
	tf::Vector3 accel_IMU;
	tf::vector3MsgToTF(msg->linear_acceleration, accel_IMU);

	// TODO subtract gravity

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

void NaroImu::resetPosition() {
	oldTime = 0.0;
	oldPosition.setValue(0.0,0.0,0.0);
	oldVelocity.setValue(0.0,0.0,0.0);
	startIntegration = true;
}

}
