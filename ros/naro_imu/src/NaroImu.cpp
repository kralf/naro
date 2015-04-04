#include "naro_imu/NaroImu.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

NODEWRAP_EXPORT_CLASS(naro_imu, nodewrap::NaroImu)

namespace nodewrap {

NaroImu::NaroImu() {}
NaroImu::~NaroImu() {}

/* --- Methods --- */

void NaroImu::init() {
	NODEWRAP_INFO("Initialize <NaroImu>");
	subscriber = subscribe("bla", "/imu/data", 100, &NaroImu::getImuData);
	publisher = advertise<geometry_msgs::QuaternionStamped>("bla", "/base/orientation", 10);
}

void NaroImu::cleanup() {
	NODEWRAP_INFO("Shutting down <NaroImu>");
}

void NaroImu::getImuData(const sensor_msgs::Imu::ConstPtr& msg) {
	//NODEWRAP_INFO("Got data");
	// convert sensor_msgs
	geometry_msgs::QuaternionStamped orientation_IMU = geometry_msgs::QuaternionStamped();
	orientation_IMU.header = msg->header;
	orientation_IMU.quaternion = msg->orientation;

	// Transform
	try {
		geometry_msgs::QuaternionStamped orientation_base;

		transformQuaternion("base_link", ros::Time(0), orientation_IMU, orientation_base);

		publisher.publish(orientation_base);

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

}
