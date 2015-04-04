#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

void poseCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	ROS_INFO("%s", &msg->orientation);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "baseFrame_tf_broadcaster");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("imu/data", 10, &poseCallback);

	ros::spin();

  return 0;
};
