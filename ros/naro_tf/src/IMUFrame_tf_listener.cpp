#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

void poseCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(2.0, 0.0, 0.0)); // adapt for correct dynamic position

	geometry_msgs::Quaternion orientation  = geometry_msgs::Quaternion();
	orientation = msg->orientation;
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation, q);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "baseFrame_tf_broadcaster");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe<sensor_msgs::Imu>("imu/data", 10, &poseCallback);

	ros::spin();

  return 0;
};


void transformOrientation(const tf::TransformListener& listener, const sensor_msgs::Imu::ConstPtr& msg) {
	// convert sensor_msgs
	geometry_msgs::Quaternion orientation_IMU;
	orientation_IMU = msg->orientation;

	// Transform
	try {
		geometry_msgs::Quaternion orientation_base;
		listener.transformQuaternion("base_link", orientation_IMU, orientation_base);
	}
	catch {
		ROS_ERROR("Received an exception trying to transform a point from \base_IMU to \base_link: %s", ex.what());
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "IMUFrame_tf_listener");


}
