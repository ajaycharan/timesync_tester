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

  double calculateMean(const std::list<double> &input);
  double calculateVariane(const std::list<double> &input);

};

TimeSyncServer::TimeSyncServer() :
    nh_(), thread_running_(false)
{
  ping_pub_ = nh_.advertise<timesync_tester::TimeMsg>("ping", 1);
  pong_sub_ = nh_.subscribe("pong", 1, &TimeSyncServer::msgCallback, this);

  thread_running_ = true;
  spin_thread_ = std::unique_ptr < std::thread > (new std::thread(&TimeSyncServer::spinThread, this));
}

TimeSyncServer::~TimeSyncServer()
{
  if (thread_running_ && spin_thread_)
  {
    thread_running_ = false;
    spin_thread_->join();
  }
}

double TimeSyncServer::calculateMean(const std::list<double> &input)
{
  if (input.size() == 0)
    return 0.0;

  double sum = 0;

  for (auto it = input.begin(); it != input.end(); it++)
  {
    sum += *it;
  }
  return (sum / (double)input.size());
}

double TimeSyncServer::calculateVariane(const std::list<double> &input)
{
  if (input.size() == 0)
    return 0.0;

  double mean = calculateMean(input);
  double temp = 0;

  for (auto it = input.begin(); it != input.end(); it++)
  {
    temp += (*it - mean) * (*it - mean);
  }
  return temp / ((double)input.size()-1.0);
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

  while (seqence_number < 10 && ros::ok())
  {
    timesync_tester::TimeMsg msg;
    msg.seqence_number = seqence_number++;
    msg.outgoing_stamp = ros::Time::now();
    ping_pub_.publish(msg);
    ros::Duration(0.5).sleep();
  }
}

void TimeSyncServer::evalData()
{
  ROS_INFO("received %d messages", (int )msg_buffer_.size());

  std::list<double> ping_pong_times;
  std::list<double> slave_offsets;

  for (auto it = msg_buffer_.begin(); it != msg_buffer_.end(); it++)
  {
    double ping_pong_time = (it->received_stamp - it->outgoing_stamp).toSec();
    double estimated_receive_time = (it->received_stamp.toSec() + it->outgoing_stamp.toSec()) / 2.0;
    double slave_offset = it->outgoing_stamp.toSec() + ping_pong_time * 0.5 - it->pong_stamp.toSec();
    printf("%d [ %.2fms, %.2fms ]\n", it->seqence_number, ping_pong_time * 1000.0, slave_offset * 1000.0);

    printf("%.4f %.4f %.4f\n", it->outgoing_stamp.toSec(), it->pong_stamp.toSec(), it->received_stamp.toSec() );


    ping_pong_times.push_back(ping_pong_time);
    slave_offsets.push_back(slave_offset);
  }
  printf("ping times [ %.2f ms, %e ms ]\n", calculateMean(ping_pong_times) * 1000.0, calculateVariane(ping_pong_times) * 1000.0);
  printf("slave offset [ %.2f ms, %e ms ]\n", calculateMean(slave_offsets) * 1000.0, calculateVariane(slave_offsets) * 1000.0);

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

