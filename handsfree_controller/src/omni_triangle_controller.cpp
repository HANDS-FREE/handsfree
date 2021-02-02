/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *	Copyright (c) 20135, NWPU
 *	ALL rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author:Luke Liao
 */

#include <tf/transform_datatypes.h>

//#include <urdf_parser/urdf_parser.h>

#include <boost/assign.hpp>

#include <omni_triangle_controller.h>
/*
static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
	return std::sqrt(std::pow(vec1.x-vec2.x,2) +
			std::pow(vec1.y-vec2.y,2) +
			std::pow(vec1.z-vec2.z,2));
}
*/
/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
//static bool isCylinder(const boost::shared_ptr<const urdf::Link>& link)
//{
//	if(!link)
//	{
//		ROS_ERROR("Link == NULL.");
//		return false;
//	}
//
//	if(!link->collision)
//	{
//		ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
//		return false;
//	}
//
//	if(!link->collision->geometry)
//	{
//		ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
//		return false;
//	}
//
//	if(link->collision->geometry->type != urdf::Geometry::CYLINDER)
//	{
//		ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
//		return false;
//	}
//
//	return true;
//}
//
///*
// * \brief Get the wheel radius
// * \param [in]  wheel_link   Wheel link
// * \param [out] wheel_radius Wheel radius [m]
// * \return true if the wheel radius was found; false otherwise
// */
//static bool getWheelRadius(const boost::shared_ptr<const urdf::Link>& wheel_link, double& wheel_radius)
//{
//	if(!isCylinder(wheel_link))
//	{
//		ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
//		return false;
//	}
//
//	wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
//	return true;
//}

namespace omni_triangle_controller{

OmniDriveController::OmniDriveController()
: open_loop_(false)
, command_struct_()
, wheel_center_(0.0)
, wheel_radius_(0.0)
, cmd_vel_timeout_(0.5)
, base_frame_id_("base_link")
, enable_odom_tf_(true)
{
}

bool OmniDriveController::init(hardware_interface::VelocityJointInterface* hw,
		ros::NodeHandle& root_nh,
		ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	// Get joint names from the parameter server
	if (!getWheelNames(controller_nh, "wheel1", wheel1_name) ||
			!getWheelNames(controller_nh, "wheel2", wheel2_name) ||
			!getWheelNames(controller_nh, "wheel3", wheel3_name))
	{
		return false;
	}

	// Odometry related:
	double publish_rate;
	controller_nh.param("publish_rate", publish_rate, 20.0);
	ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
			<< publish_rate << "Hz.");
	publish_period_ = ros::Duration(1.0 / publish_rate);

	// Twist command related:
	controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
	ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
			<< cmd_vel_timeout_ << "s.");

	controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
	ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

	controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
	ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

	// Velocity and acceleration limits:
	controller_nh.param("linear/x/has_velocity_limits"    , limiter_linear_x_.has_velocity_limits    , limiter_linear_x_.has_velocity_limits    );
	controller_nh.param("linear/x/has_acceleration_limits", limiter_linear_x_.has_acceleration_limits, limiter_linear_x_.has_acceleration_limits);
	controller_nh.param("linear/x/max_velocity"           , limiter_linear_x_.max_velocity           ,  limiter_linear_x_.max_velocity          );
	controller_nh.param("linear/x/min_velocity"           , limiter_linear_x_.min_velocity           , -limiter_linear_x_.max_velocity          );
	controller_nh.param("linear/x/max_acceleration"       , limiter_linear_x_.max_acceleration       ,  limiter_linear_x_.max_acceleration      );
	controller_nh.param("linear/x/min_acceleration"       , limiter_linear_x_.min_acceleration       , -limiter_linear_x_.max_acceleration      );

	controller_nh.param("linear/y/has_velocity_limits"    , limiter_linear_y_.has_velocity_limits    , limiter_linear_y_.has_velocity_limits    );
	controller_nh.param("linear/y/has_acceleration_limits", limiter_linear_y_.has_acceleration_limits, limiter_linear_y_.has_acceleration_limits);
	controller_nh.param("linear/y/max_velocity"           , limiter_linear_y_.max_velocity           ,  limiter_linear_y_.max_velocity          );
	controller_nh.param("linear/y/min_velocity"           , limiter_linear_y_.min_velocity           , -limiter_linear_y_.max_velocity          );
	controller_nh.param("linear/y/max_acceleration"       , limiter_linear_y_.max_acceleration       ,  limiter_linear_y_.max_acceleration      );
	controller_nh.param("linear/y/min_acceleration"       , limiter_linear_y_.min_acceleration       , -limiter_linear_y_.max_acceleration      );

	controller_nh.param("angular/z/has_velocity_limits"    , limiter_angular_.has_velocity_limits    , limiter_angular_.has_velocity_limits    );
	controller_nh.param("angular/z/has_acceleration_limits", limiter_angular_.has_acceleration_limits, limiter_angular_.has_acceleration_limits);
	controller_nh.param("angular/z/max_velocity"           , limiter_angular_.max_velocity           ,  limiter_angular_.max_velocity          );
	controller_nh.param("angular/z/min_velocity"           , limiter_angular_.min_velocity           , -limiter_angular_.max_velocity          );
	controller_nh.param("angular/z/max_acceleration"       , limiter_angular_.max_acceleration       ,  limiter_angular_.max_acceleration      );
	controller_nh.param("angular/z/min_acceleration"       , limiter_angular_.min_acceleration       , -limiter_angular_.max_acceleration      );

	if (!setOdomParamsFromUrdf(root_nh, wheel1_name, wheel2_name, wheel3_name))
		return false;

	setOdomPubFields(root_nh, controller_nh);

	// Get the joint object to use in the realtime loop
	wheel1_joint = hw->getHandle(wheel1_name);  // throws on failure
	wheel2_joint = hw->getHandle(wheel2_name);  // throws on failure
	wheel3_joint = hw->getHandle(wheel3_name);

	sub_command_ = controller_nh.subscribe("cmd_vel", 1, &OmniDriveController::cmdVelCallback, this);
	ROS_INFO("wocao");
	return true;
}

void OmniDriveController::update(const ros::Time& time, const ros::Duration& period)
{
	//get the wheel velocity from the handle
	double wheel1_vel = wheel1_joint.getVelocity();
	double wheel2_vel = wheel2_joint.getVelocity();
	double wheel3_vel = wheel3_joint.getVelocity();

	// Estimate linear and angular velocity using joint information
			odometry_.update(wheel1_vel, wheel2_vel, wheel3_vel, time);

			// Publish odometry message
			if(last_state_publish_time_ + publish_period_ < time)
			{
				last_state_publish_time_ += publish_period_;
				// Compute and store orientation info
				const geometry_msgs::Quaternion orientation(
						tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

				// Populate odom message and publish
				if(odom_pub_->trylock())
				{
					odom_pub_->msg_.header.stamp = time;
					odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
					odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
					odom_pub_->msg_.pose.pose.orientation = orientation;
					odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
					odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
					odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
					odom_pub_->unlockAndPublish();
				}

				// Publish tf /odom frame
				if (enable_odom_tf_ && tf_odom_pub_->trylock())
				{
					geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
					odom_frame.header.stamp = time;
					odom_frame.transform.translation.x = odometry_.getX();
					odom_frame.transform.translation.y = odometry_.getY();
					odom_frame.transform.rotation = orientation;
					tf_odom_pub_->unlockAndPublish();
				}
			}

			// MOVE ROBOT
			// Retreive current velocity command and time step:
			Commands curr_cmd = *(command_.readFromRT());
			const double dt = (time - curr_cmd.stamp).toSec();

			// Brake if cmd_vel has timeout:
			if (dt > cmd_vel_timeout_)
			{
				ROS_INFO("wocao nima");
				curr_cmd.linear_x = 0.0;
				curr_cmd.linear_y = 0.0;
				curr_cmd.angular = 0.0;
			}

			// Limit velocities and accelerations:
			const double cmd_dt(period.toSec());
			limiter_linear_x_.limit(curr_cmd.linear_x, last_cmd_.linear_x, cmd_dt);
			limiter_linear_y_.limit(curr_cmd.linear_y, last_cmd_.linear_y, cmd_dt);
			limiter_angular_. limit(curr_cmd.angular,  last_cmd_.angular,  cmd_dt);
			last_cmd_ = curr_cmd;

			// Compute wheels velocities:
			const double vth = curr_cmd.angular, vx = - curr_cmd.linear_x, vy = curr_cmd.linear_y;
			const double psi = 2 * 3.14159265 / 3;
			const double L = wheel_center_;


			const double wheel1_cmd  = (L*vth - vy) / wheel_radius_;
			const double wheel2_cmd  = (L*vth + vy*cos(psi/2) - vx*sin(psi/2)) / wheel_radius_;
			const double wheel3_cmd  = (L*vth + vy*cos(psi/2) + vx*sin(psi/2)) / wheel_radius_;

			ROS_INFO("send velocity is %f %f %f", wheel1_cmd,wheel2_cmd,wheel3_cmd);

			//ROS_INFO("the velocity is %f  %f  %f", wheel1_cmd, wheel2_cmd, wheel3_cmd);

			wheel1_joint.setCommand(wheel1_cmd);
			wheel2_joint.setCommand(wheel2_cmd);
			wheel3_joint.setCommand(wheel3_cmd);
}

void OmniDriveController::starting(const ros::Time& time)
{
	brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

#if ROS_VERSION_MINOR < 15
    state_ = RUNNING;
#else
    state_ = ControllerState::RUNNING;
#endif

    odometry_.init(time);
}

void OmniDriveController::stopping(const ros::Time& time)
{
	brake();
}

void OmniDriveController::brake()
{
	const double vel = 0.0;
	ROS_INFO("wocao");
	wheel1_joint.setCommand(vel);
	wheel2_joint.setCommand(vel);
	wheel3_joint.setCommand(vel);
}

void OmniDriveController::cmdVelCallback(const geometry_msgs::Twist& command)
{
	if(isRunning())
	{
		command_struct_.angular    = command.angular.z;
		command_struct_.linear_x   = command.linear.x;
		command_struct_.linear_y   = command.linear.y;
		command_struct_.stamp = ros::Time::now();
		command_.writeFromNonRT (command_struct_);
		ROS_DEBUG_STREAM_NAMED(name_,
				"Added values to command. "
				<< "Ang: "   << command_struct_.angular << ", "
				<< "Lin x: "   << command_struct_.linear_x << ", "
				<< "Lin y: "   << command_struct_.linear_y << ", "
				<< "Stamp: " << command_struct_.stamp);
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

bool OmniDriveController::getWheelNames(ros::NodeHandle& controller_nh,
		const std::string& wheel_param,
		std::string& wheel_name)
{
	XmlRpc::XmlRpcValue wheel_list;
	if (!controller_nh.getParam(wheel_param, wheel_list))
	{
		ROS_ERROR_STREAM_NAMED(name_,
				"Couldn't retrieve wheel param '" << wheel_param << "'.");
		return false;
	}

	if (wheel_list.getType() != XmlRpc::XmlRpcValue::TypeString)
	{
		ROS_ERROR_STREAM_NAMED(name_,
				"Wheel param '" << wheel_param << "' #" <<
				" isn't a string.");
		return false;
	}

	wheel_name = static_cast<std::string>(wheel_list);
	return true;
}

bool OmniDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
		const std::string& wheel1_name,
		const std::string& wheel2_name,
		const std::string& wheel3_name)
{
	// Parse robot description
//	const std::string model_param_name = "robot_description";
//	bool res = root_nh.hasParam(model_param_name);
//	std::string robot_model_str="";
//	if(!res || !root_nh.getParam(model_param_name,robot_model_str))
//	{
//		ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
//		return false;
//	}
//
//	boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));
//
//	//check the wheel information in the urdf
//	boost::shared_ptr<const urdf::Joint> wheel1(model->getJoint(wheel1_name));
//	if(!wheel1)
//	{
//		ROS_ERROR_STREAM_NAMED(name_, wheel1
//				<< " couldn't be retrieved from model description");
//		return false;
//	}
//
//	boost::shared_ptr<const urdf::Joint> wheel2(model->getJoint(wheel2_name));
//	if(!wheel2)
//	{
//		ROS_ERROR_STREAM_NAMED(name_, wheel2
//				<< " couldn't be retrieved from model description");
//		return false;
//	}
//
//	boost::shared_ptr<const urdf::Joint> wheel3(model->getJoint(wheel3_name));
//	if(!wheel3)
//	{
//		ROS_ERROR_STREAM_NAMED(name_, wheel3
//				<< " couldn't be retrieved from model description");
//		return false;
//	}
//
//	ROS_INFO_STREAM("wheel1 to origin: " << wheel1->parent_to_joint_origin_transform.position.x << ","
//			<< wheel1->parent_to_joint_origin_transform.position.y << ", "
//			<< wheel1->parent_to_joint_origin_transform.position.z);
//	ROS_INFO_STREAM("wheel2 to origin: " << wheel2->parent_to_joint_origin_transform.position.x << ","
//			<< wheel2->parent_to_joint_origin_transform.position.y << ", "
//			<< wheel2->parent_to_joint_origin_transform.position.z);
//	ROS_INFO_STREAM("wheel3 to origin: " << wheel3->parent_to_joint_origin_transform.position.x << ","
//			<< wheel3->parent_to_joint_origin_transform.position.y << ", "
//			<< wheel3->parent_to_joint_origin_transform.position.z);
//
//	wheel_center_  = sqrt(wheel1->parent_to_joint_origin_transform.position.x * wheel1->parent_to_joint_origin_transform.position.x +
//			wheel1->parent_to_joint_origin_transform.position.y * wheel1->parent_to_joint_origin_transform.position.y +
//			wheel1->parent_to_joint_origin_transform.position.z * wheel1->parent_to_joint_origin_transform.position.z);

	// Get wheel radius
//	if(!getWheelRadius(model->getLink(wheel1->child_link_name), wheel_radius_))
//	{
//		ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << wheel1_name << " wheel radius");
//		return false;
//	}

	wheel_radius_ = 0.1016 / 2;
	wheel_center_ = 0.1924;

	// Set wheel params for the odometry computation
	odometry_.setWheelParams(wheel_center_, wheel_radius_);
	ROS_INFO_STREAM_NAMED(name_,
			"Odometry params : wheel center " << wheel_center_
			<< ", wheel radius " << wheel_radius_);
	return true;
}

void OmniDriveController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
	/*
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);*/

	// Setup odometry realtime publisher + odom message constant fields
	odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
	odom_pub_->msg_.header.frame_id = "odom";
	odom_pub_->msg_.child_frame_id = base_frame_id_;
	odom_pub_->msg_.pose.pose.position.z = 0;
	/*odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));*/
	odom_pub_->msg_.twist.twist.linear.z  = 0;
	odom_pub_->msg_.twist.twist.angular.x = 0;
	odom_pub_->msg_.twist.twist.angular.y = 0;
	/* odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));*/
	//TODO: add the convarience matrix to the omni base
	tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
	tf_odom_pub_->msg_.transforms.resize(1);
	tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
	tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
	tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

} // namespace diff_drive_controller
