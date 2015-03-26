#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void poseCallback() {
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(2.0, 0.0, 0.0)); // adapt for dynamic position
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "baseFrame_tf_broadcaster");
	ros::NodeHandle node;

	ros::Rate rate(10.0);
	while(node.ok()) {
		poseCallback();
		rate.sleep();
	}

  return 0;
};
