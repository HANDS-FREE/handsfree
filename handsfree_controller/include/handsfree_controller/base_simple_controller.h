/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are required to be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2015.10.1   V1.0           creat this file
*
* Description: send the robot speed and publish odom directly through specific hardware interface
***********************************************************************************************************************/

#include <controller_interface/controller.h>
#include <handsfree_hw/base_cmd_interface.h>
#include <pluginlib/class_list_macros.h>

#include <pluginlib/class_list_macros.h>

#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace base_simple_controller {

class BaseSimpleController : public controller_interface::Controller<hardware_interface::BaseVelocityInterface>
{
public:
	BaseSimpleController();

	bool init(hardware_interface::BaseVelocityInterface* hw,
				ros::NodeHandle& root_nh,
				ros::NodeHandle &controller_nh);

	void update(const ros::Time &time, const ros::Duration& period);

	void starting(const ros::Time& time);

	void stopping(const ros::Time& time);

private:
	ros::Duration publish_period_;
	ros::Time last_state_publish_time_, last_vel_get_;

	ros::Subscriber cmd_vel_sub_;
	geometry_msgs::Twist cmd_;
	realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer_;
	hardware_interface::BaseVelocityHandle handle_;

	std::string base_name_, base_frame_id_, name_;
	double cmd_vel_timeout_;
	double last_x_, last_y_, last_theta_;

	boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
	boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

	void cmdvelCallBack(const geometry_msgs::Twist::ConstPtr &msg);
};
PLUGINLIB_EXPORT_CLASS(base_simple_controller::BaseSimpleController, controller_interface::ControllerBase)

}
