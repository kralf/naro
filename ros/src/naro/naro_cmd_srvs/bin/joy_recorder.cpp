/***************************************************************************
 *   Copyright (C) 2013 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <limits>

#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include "naro_cmd_srvs/GetState.h"
#include "naro_cmd_srvs/Record.h"
#include "naro_cmd_srvs/Play.h"
#include "naro_cmd_srvs/Stop.h"
#include "naro_cmd_srvs/List.h"

#include "sensor_msgs/Joy.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

using namespace naro_cmd_srvs;
using namespace sensor_msgs;

std::string subscriberTopic = "joy";
int subscriberQueueSize = 1;
std::string publisherTopic = "joy";
int publisherQueueSize = 1;
std::string libraryPath = "/tmp";

ros::ServiceServer getStateService;
ros::ServiceServer recordService;
ros::ServiceServer playService;
ros::ServiceServer stopService;
ros::ServiceServer listService;

ros::Subscriber subscriber;
ros::Publisher publisher;

rosbag::Bag bag;
boost::shared_ptr<rosbag::View> bagView;
rosbag::View::const_iterator bagIterator;

enum State {
  stopped,
  recording,
  playing
};

State state;

ros::Time bagStartTime;
ros::Timer bagPlayTimer;
ros::Timer bagStopTimer;

void getParameters(const ros::NodeHandle& node) {
  node.param<std::string>("subscriber/topic", subscriberTopic,
    subscriberTopic);
  node.param<int>("subscriber/queue_size", subscriberQueueSize,
    subscriberQueueSize);

  node.param<std::string>("publisher/topic", publisherTopic,
    publisherTopic);
  node.param<int>("publisher/queue_size", publisherQueueSize,
    publisherQueueSize);

  node.param<std::string>("library/path", libraryPath, libraryPath);
}

void receiveJoy(const Joy::ConstPtr& message) {
  if (state == recording)
    bag.write("/"+subscriberTopic, ros::Time::now(), message);
}

float stop(const ros::TimerEvent& event = ros::TimerEvent()) {
  if (state == recording) {
    ROS_INFO("Finished recording to %s.", bag.getFileName().c_str());
    
    subscriber.shutdown();
    bagStopTimer.stop();
    bag.close();
    state = stopped;

    return (ros::Time::now()-bagStartTime).toSec();
  }
  else if (state == playing) {
    ROS_INFO("Finished playing from %s.", bag.getFileName().c_str());    

    publisher.shutdown();
    bagPlayTimer.stop();
    bagStopTimer.stop();
    bag.close();
    state = stopped;

    return (ros::Time::now()-bagStartTime).toSec();
  }
  else
    return 0.0f;
}

void playNext(const ros::TimerEvent& event) {
  publisher.publish(bagIterator->instantiate<Joy>());
  ++bagIterator;
  
  if (bagIterator != bagView->end()) {
    bagPlayTimer = ros::NodeHandle("~").createTimer(
      (bagIterator->getTime()-bagView->getBeginTime())-
      (event.current_real-bagStartTime), playNext, true);   
  }
  else
    stop();
}

bool getState(GetState::Request& request, GetState::Response& response) {
  response.state = state;
  
  if ((state == playing) || (state == recording)) {
    response.name = boost::filesystem::basename(bag.getFileName());
    response.duration = (ros::Time::now()-bagStartTime).toSec();
  }
  
  return true;
}

bool record(Record::Request& request, Record::Response& response) {
  if ((state == recording) || (state == playing)) {
    ROS_WARN("Failed to start recording: Already recording or playing.");
    return false;
  }
  
  std::string fileName = libraryPath+"/"+request.name+".bag";
  try {
    bag.open(fileName, rosbag::bagmode::Write);
  }
  catch (const std::runtime_error& error) {
    ROS_WARN("Failed to open %s for recording: %s", fileName.c_str(),
      error.what());
    return false;
  }
  
  subscriber = ros::NodeHandle("~").subscribe("/"+subscriberTopic,
    subscriberQueueSize, receiveJoy);
  
  bagStartTime = ros::Time::now();
  state = recording;
  
  if (request.duration <  std::numeric_limits<float>::infinity())
    bagStopTimer = ros::NodeHandle("~").createTimer(
      ros::Duration(request.duration), stop, true);

  ROS_INFO("Started recording to %s.", bag.getFileName().c_str());
    
  return true;
}

bool play(Play::Request& request, Play::Response& response) {
  if ((state == recording) || (state == playing)) {
    ROS_WARN("Failed to start playing: Already recording or playing.");
    return false;
  }
  
  std::string fileName = libraryPath+"/"+request.name+".bag";
  try {
    bag.open(fileName, rosbag::bagmode::Read);
  }
  catch (const std::runtime_error& error) {
    ROS_WARN("Failed to open %s for playing: %s", fileName.c_str(),
      error.what());
    return false;
  }

  publisher = ros::NodeHandle("~").advertise<Joy>("/"+publisherTopic,
    publisherQueueSize);
  
  std::vector<std::string> topics;
  topics.push_back("/"+subscriberTopic);
  bagView.reset(new rosbag::View(bag, rosbag::TopicQuery(topics)));  
  bagIterator = bagView->begin();
  
  if (bagIterator != bagView->end()) {
    bagStartTime = ros::Time::now()+ros::Duration(fmax(request.delay, 0.0f));
    state = playing;
  
    bagPlayTimer = ros::NodeHandle("~").createTimer(ros::Duration(
      (bagIterator->getTime()-bagView->getBeginTime()).toSec()+
      fmax(request.delay, 0.0f)), playNext, true);   
    
    if (request.duration <  std::numeric_limits<float>::infinity())
      bagStopTimer = ros::NodeHandle("~").createTimer(
        ros::Duration(request.duration), stop, true);   
  }
  else {
    ROS_WARN("Failed to start playing from %s: Bag is empty.",
      bag.getFileName().c_str());
    bag.close();
    
    return false;
  }
    
  ROS_INFO("Started playing from %s.", bag.getFileName().c_str());
  
  return true;
}

bool stop(Stop::Request& request, Stop::Response& response) {
  response.duration = stop();
  return true;
}

bool list(List::Request& request, List::Response& response) {
  boost::filesystem::directory_iterator itEnd;
  for (boost::filesystem::directory_iterator it(libraryPath);
      it != itEnd; ++it) {
    if (!boost::filesystem::is_regular_file(it->status()))
      continue;

    if (it->path().extension() == ".bag")
      response.names.push_back(boost::filesystem::basename(it->path()));
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_recorder");
  ros::NodeHandle node("~");

  getParameters(node);

  getStateService = node.advertiseService("get_state", getState);
  recordService = node.advertiseService("record", record);
  playService = node.advertiseService("play", play);
  stopService = node.advertiseService("stop", stop);
  listService = node.advertiseService("list", list);

  ros::spin();

  return 0;
}
