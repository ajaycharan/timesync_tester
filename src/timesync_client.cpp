/*
 * timesync_server.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: ffontana
 */


#include "ros/ros.h"
#include "timesync_tester/TimeMsg.h"
#include <ctime>

class TimeSyncClient
{
public:
  TimeSyncClient();
  ~TimeSyncClient();
  void msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg);

private:
  ros::Time TimeSyncClient::getTime();

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

ros::Time TimeSyncClient::getTime()
{

    uint32_t start_sec = 0;
      uint32_t start_nsec = 0;
		ros::Time time;
		FILETIME ft;
         GetSystemTimeAsFileTime(&ft);
         LARGE_INTEGER start_li;
         start_li.LowPart = ft.dwLowDateTime;
         start_li.HighPart = ft.dwHighDateTime;
         // why did they choose 1601 as the time zero, instead of 1970?
         // there were no outstanding hard rock bands in 1601.

		 
         start_li.QuadPart -= 116444736000000000Ui64;
         //start_li.QuadPart -= 116444736000000000ULL;
         start_sec = (uint32_t)(start_li.QuadPart / 10000000); // 100-ns units. odd.
         start_nsec = (start_li.LowPart % 10000000) * 100;

		 time.sec = start_sec;
		 time.nsec = start_nsec;
		 return time;
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
  printf("%.4f %.4f\n",ros::Time::now().toSec(), getTime().toSec());
  prev_seqence_number = msg->seqence_number;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timesync_client");
  TimeSyncClient client;
  ros::spin();
  return 0;
}


