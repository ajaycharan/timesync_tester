/*
 * timesync_server.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: ffontana
 */


#include "ros/ros.h"
#include "timesync_tester/TimeMsg.h"
#include <thread>
#include <memory>

class TimeSyncClient
{
public:
  TimeSyncClient();
  ~TimeSyncClient();
  void msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg);

private:
  ros::NodeHandle nh_;

  ros::Subscriber ping_sub_;
  ros::Publisher  pong_pub_;
};

TimeSyncClient::TimeSyncClient()
 : nh_()
{
  ping_sub_ = nh_.subscribe("ping", 1, &TimeSyncClient::msgCallback, this);
  pong_pub_ = nh_.advertise<timesync_tester::TimeMsg>("pong", 1);
}

TimeSyncClient::~TimeSyncClient()
{
}


void TimeSyncClient::msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg)
{
  timesync_tester::TimeMsg local_msg(*msg);
  local_msg.pong_stamp = ros::Time::now();
  pong_pub_.publish(local_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timesync_client");
  TimeSyncClient client;
  ros::spin();
  return 0;
}


