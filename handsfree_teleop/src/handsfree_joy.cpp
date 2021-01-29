#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class HandsFreeTeleop
{
public:
  HandsFreeTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_x, linear_y, angular_, deadman_axis_, channel1, channel2;
  double l_scale_, a_scale_, c_scale_1, c_scale_2;
  ros::Publisher vel_pub_, channel1_pub_, channel2_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  std_msgs::Float64 channel1_data, channel2_data;

  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;

};

HandsFreeTeleop::HandsFreeTeleop():
  ph_("~"),
  linear_x(1),
  angular_(0),
  deadman_axis_(4),
  l_scale_(0.3),
  a_scale_(0.9)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("scale_channel1", c_scale_1, c_scale_1);
  ph_.param("scale_channel2", c_scale_2, c_scale_2);

  //ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_linear_x", linear_x, linear_x);
  ph_.param("axis_linear_y", linear_y, linear_y);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_channel1", channel1, channel1);
  ph_.param("axis_channel2", channel2, channel2);

  deadman_pressed_ = true;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  channel1_pub_ = ph_.advertise<std_msgs::Float64>("/handsfree_teleop/channel1", 1, true);
  channel2_pub_ = ph_.advertise<std_msgs::Float64>("/handsfree_teleop/channel2", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &HandsFreeTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&HandsFreeTeleop::publish, this));
}

void HandsFreeTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.linear.x = l_scale_*joy->axes[linear_x];
  vel.linear.y = l_scale_*joy->axes[linear_y];
  vel.angular.z = a_scale_*joy->axes[angular_];
  last_published_ = vel;

  std_msgs::Float64 data1,data2;
  data1.data = c_scale_1*joy->axes[channel1];
  data2.data = c_scale_2*joy->axes[channel2];

  channel1_data = data1;
  channel2_data = data2;
  //deadman_pressed_ = joy->buttons[deadman_axis_];
}

void HandsFreeTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_)
  {
    vel_pub_.publish(last_published_);
    channel1_pub_.publish(channel1_data);
    channel2_pub_.publish(channel2_data);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    channel1_pub_.publish(*new std_msgs::Float64);
    channel2_pub_.publish(*new std_msgs::Float64);
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handsfree_teleop");
  HandsFreeTeleop handsfree_teleop;

  ros::spin();
}
