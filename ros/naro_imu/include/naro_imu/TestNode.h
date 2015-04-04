#ifndef TEST_JONAS
#define TEST_JONAS

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace nodewrap {

class TestNode :
		public NodeImpl {
  public:

    TestNode();


    virtual ~TestNode();

  protected:

    ros::Publisher publisher;

    ros::Subscriber subscriber;

    ros::ServiceServer server;

    std::string name;

    bool initiate;

    std::string say;

    void init();

    void cleanup();

    void connect(const ros::SingleSubscriberPublisher& pub);

    void chat(const std_msgs::String::ConstPtr& msg);


    bool call(std_srvs::Empty::Request& request, std_srvs::Empty::Response&
      response);
  };
};


#endif
