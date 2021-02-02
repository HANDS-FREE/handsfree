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

#include <base_simple_controller.h>

#include <boost/assign.hpp>

#include <tf/transform_datatypes.h>

namespace base_simple_controller {

BaseSimpleController::BaseSimpleController()
: cmd_()
, base_frame_id_("base_link")
, cmd_vel_timeout_(0.5)
, base_name_("mobile_base")
{}

bool BaseSimpleController::init(hardware_interface::BaseVelocityInterface* hw,
		ros::NodeHandle& root_nh,
		ros::NodeHandle &controller_nh)
{
	last_x_ = last_y_ = last_theta_ = 0.0;
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	double publish_rate = 20.0;
	controller_nh.getParam("publish_rate", publish_rate);
	publish_period_ = ros::Duration(1.0 / publish_rate);
	last_vel_get_ = ros::Time::now();

	controller_nh.getParam("cmd_vel_timeout", cmd_vel_timeout_);
	controller_nh.getParam("base_frame_id", base_frame_id_);
	controller_nh.getParam("base_name", base_name_);

	handle_ = hw->getHandle(base_name_);

	odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
	odom_pub_->msg_.header.frame_id = "odom";
	odom_pub_->msg_.child_frame_id = base_frame_id_;
	odom_pub_->msg_.pose.pose.position.z = 0;
	// TODO: fulfill the true parameters add the experience parameters
	odom_pub_->msg_.pose.covariance = boost::assign::list_of
		(0.001) (0)  (0)  (0)  (0)  (0)
		(0)  (0.001) (0)  (0)  (0)  (0)
		(0)  (0)  (1000000.0) (0)  (0)  (0)
		(0)  (0)  (0)  (1000000.0) (0)  (0)
		(0)  (0)  (0)  (0)  (1000000.0) (0)
		(0)  (0)  (0)  (0)  (0)  (1000.0);
	odom_pub_->msg_.twist.twist.linear.z  = 0;
	odom_pub_->msg_.twist.twist.angular.x = 0;
	odom_pub_->msg_.twist.twist.angular.y = 0;
	odom_pub_->msg_.pose.covariance = boost::assign::list_of
			(0.001) (0)  (0)  (0)  (0)  (0)
			(0)  (0.001) (0)  (0)  (0)  (0)
			(0)  (0)  (1000000.0) (0)  (0)  (0)
			(0)  (0)  (0)  (1000000.0) (0)  (0)
			(0)  (0)  (0)  (0)  (1000000.0) (0)
			(0)  (0)  (0)  (0)  (0)  (1000.0);
	//TODO: add the convarience matrix to the omni base
	tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
	tf_odom_pub_->msg_.transforms.resize(1);
	tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
	tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
	tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";

	cmd_vel_sub_ = controller_nh.subscribe("cmd_vel",1 ,&BaseSimpleController::cmdvelCallBack, this);
	return true;
}

void BaseSimpleController::cmdvelCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
	if (isRunning())
	{
		command_buffer_.writeFromNonRT(*msg);
		last_vel_get_ = ros::Time::now();
	} else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

void BaseSimpleController::update(const ros::Time& time, const ros::Duration& period)
{
	if (last_state_publish_time_ + publish_period_ < time)
	{
		last_state_publish_time_ += publish_period_;

		const geometry_msgs::Quaternion orientation(
				tf::createQuaternionMsgFromYaw(handle_.getTheta()));
		if(odom_pub_->trylock())
		{
			odom_pub_->msg_.header.stamp = time;
			odom_pub_->msg_.pose.pose.position.x = handle_.getX();
			odom_pub_->msg_.pose.pose.position.y = handle_.getY();
			odom_pub_->msg_.pose.pose.orientation = orientation;
			odom_pub_->msg_.twist.twist.linear.x  = handle_.getXvel();
			odom_pub_->msg_.twist.twist.linear.y  = handle_.getYvel();
			odom_pub_->msg_.twist.twist.angular.z = handle_.getThetavel();
			odom_pub_->unlockAndPublish();

		}

		// Publish tf /odom frame
		if (tf_odom_pub_->trylock())
		{
			geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
			odom_frame.header.stamp = time;
			odom_frame.transform.translation.x = handle_.getX();
			odom_frame.transform.translation.y = handle_.getY();
			odom_frame.transform.rotation = orientation;
			tf_odom_pub_->unlockAndPublish();
		}
	}

	geometry_msgs::Twist temp;
	temp = *(command_buffer_.readFromRT());
	if ((time - last_vel_get_).toSec() > cmd_vel_timeout_)
	{
		// stop the robot first
		handle_.setXcmd(0.0);
		handle_.setYcmd(0.0);
		handle_.setThetacmd(0.0);
	} else
	{
		handle_.setXcmd(temp.linear.x);
		handle_.setYcmd(temp.linear.y);
		handle_.setThetacmd(temp.angular.z);
	}
}

void BaseSimpleController::starting(const ros::Time& time)
{
	// stop the robot first
	handle_.setXcmd(0.0);
	handle_.setYcmd(0.0);
	handle_.setThetacmd(0.0);

	last_state_publish_time_ = time;

#if ROS_VERSION_MINOR < 15
	state_ = RUNNING;
#else
    state_ = ControllerState::RUNNING;
#endif

}

void BaseSimpleController::stopping(const ros::Time& time)
{
	// stop the robot
	handle_.setXcmd(0.0);
	handle_.setYcmd(0.0);
	handle_.setThetacmd(0.0);
}

}
