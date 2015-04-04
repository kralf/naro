#include "naro_imu/TestNode.h"

NODEWRAP_EXPORT_CLASS(naro_imu, nodewrap::TestNode)

namespace nodewrap {

	TestNode::TestNode() {}

	TestNode::~TestNode() {}

	void TestNode::init() {
		 publisher = advertise<std_msgs::String>("jonas", "/jonas", 100,
		    boost::bind(&TestNode::connect, this, _1));
		 NODEWRAP_INFO("Publishing to: %s", publisher.getTopic().c_str());
	}

	void TestNode::cleanup() {
		NODEWRAP_INFO("Good bye");
	}

	void TestNode::connect(const ros::SingleSubscriberPublisher& pub) {
		NODEWRAP_INFO("In connect");
		while(!publisher)
			ros::Duration(0.1).sleep();

	    	std_msgs::StringPtr msg(new std_msgs::String);
	    	msg->data = "Hello Jonas!";

			publisher.publish(msg);
			NODEWRAP_DEBUG("I said: [%s]", msg->data.c_str());
	}

}
