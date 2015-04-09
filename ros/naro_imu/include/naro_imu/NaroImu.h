#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


namespace nodewrap {

class NaroImu:
		public NodeImpl{
			public:
				NaroImu();
				virtual ~NaroImu();

			protected:
				ros::Subscriber subscriber;
				ros::Publisher publisher;
				tf::TransformListener transformer;

				tf::Vector3 oldPosition;
				tf::Vector3 newPosition;
				tf::Vector3 oldVelocity;
				tf::Vector3 newVelocity;
				double oldTime;
				double newTime;
				bool startIntegration;
				double g;



				void init();
				void cleanup();

				void getImuData(const sensor_msgs::Imu::ConstPtr& msg);

				void transformQuaternion(const std::string& target_frame, const ros::Time& time,
						const geometry_msgs::QuaternionStamped& msg_in, geometry_msgs::QuaternionStamped& msg_out);

				geometry_msgs::Vector3 getGravity(const std::string& target_frame);

				void resetPose();

				void integrateImuToPosition(const sensor_msgs::Imu::ConstPtr& msg, geometry_msgs::Point& position);

};
};
