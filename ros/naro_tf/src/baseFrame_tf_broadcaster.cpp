#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Point positionBase;
	tf::pointMsgToTF(msg->position,positionBase);
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->orientation, q);

	tf::Vector3 dummyPosition = tf::Vector3(0.0, 2.0, 1.0);
	transform.setOrigin(positionBase); // adapt for correct dynamic position
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "baseFrame_tf_broadcaster");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe<geometry_msgs::Pose>("/base/pose", 10, &poseCallback);

	ros::spin();

  return 0;
};
