#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

void poseCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(2.0, 0.0, 0.0)); // adapt for correct dynamic position

	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->quaternion, q);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "baseFrame_tf_broadcaster");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe<geometry_msgs::QuaternionStamped>("/base/orientation", 10, &poseCallback);

	ros::spin();

  return 0;
};
