/*
 * timesync_server.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: ffontana
 */


#include "ros/ros.h"
#include "timesync_tester/TimeMsg.h"

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
  ping_sub_ = nh_.subscribe("ping", 1, &TimeSyncClient::msgCallback, this, ros::TransportHints().tcpNoDelay());
  pong_pub_ = nh_.advertise<timesync_tester::TimeMsg>("pong", 1);
}

TimeSyncClient::~TimeSyncClient()
{
}


void TimeSyncClient::msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg)
{
  static int prev_seqence_number = -1;
  timesync_tester::TimeMsg local_msg(*msg);
  local_msg.pong_stamp = ros::Time::now();
  pong_pub_.publish(local_msg);
  //printf("id = %d\n", msg->seqence_number);
  if(prev_seqence_number+1 != msg->seqence_number)
  {
	printf("dropped package: expected: %d received: %d\n", prev_seqence_number+1, msg->seqence_number);
  }
  prev_seqence_number = msg->seqence_number;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timesync_client");
  TimeSyncClient client;
  ros::spin();
  return 0;
}


