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
#include <atomic>

class TimeSyncServer
{
public:
  TimeSyncServer();
  ~TimeSyncServer();
  void msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg);

  ros::NodeHandle nh_;

  ros::Publisher ping_pub_;
  ros::Subscriber pong_sub_;

  std::list<timesync_tester::TimeMsg> msg_buffer_;

  std::unique_ptr<std::thread> spin_thread_;

  std::atomic_bool thread_running_;

  void spinThread();
  void recordData();
  void evalData();
};

TimeSyncServer::TimeSyncServer()
 : nh_()
 , thread_running_(false)
{
  ping_pub_ = nh_.advertise<timesync_tester::TimeMsg>("ping", 1);
  pong_sub_ = nh_.subscribe("pong", 1, &TimeSyncServer::msgCallback, this);

  thread_running_ = true;
  spin_thread_ = std::unique_ptr<std::thread>(new std::thread(&TimeSyncServer::spinThread, this));
}

TimeSyncServer::~TimeSyncServer()
{
  if(thread_running_ && spin_thread_ )
  {
    thread_running_ = false;
    spin_thread_->join();
  }
}

void TimeSyncServer::spinThread()
{
  ros::spin();
}

void TimeSyncServer::msgCallback(const timesync_tester::TimeMsg::ConstPtr &msg)
{
  timesync_tester::TimeMsg local_msg(*msg);
  local_msg.received_stamp = ros::Time::now();
  msg_buffer_.push_back(local_msg);
}

void TimeSyncServer::recordData()
{
  ros::Duration(1.0).sleep();

  int seqence_number;

  while(seqence_number<10 && ros::ok())
  {
    timesync_tester::TimeMsg msg;
    msg.seqence_number = seqence_number++;
    msg.outgoing_stamp = ros::Time::now();
    ping_pub_.publish(msg);
    ros::Duration(1.0).sleep();
  }
}

void TimeSyncServer::evalData()
{
  ROS_INFO("received %d messages", (int)msg_buffer_.size());

  for(auto it = msg_buffer_.begin(); it != msg_buffer_.end(); it++)
  {
    double ping_pong_time = (it->received_stamp - it->outgoing_stamp).toSec();
    double estimated_receive_time = (it->received_stamp.toSec() + it->outgoing_stamp.toSec())/2.0;
    double slave_offset = it->outgoing_stamp.toSec() + ping_pong_time*0.5 - it->pong_stamp.toSec();
    printf("%d [ %.2fms, %.2fms ]\n", it->seqence_number, ping_pong_time*1000.0, slave_offset*1000.0);
  }
  printf("negative number means client stamps to early\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timesync_server");
  TimeSyncServer server;
  server.recordData();
  server.evalData();
  return 0;
}


