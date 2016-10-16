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
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: handsfree ros ros_control framework
***********************************************************************************************************************/

#ifndef HF_HW_ROS_
#define HF_HW_ROS_

#include <vector>

#include <handsfree_hw/base_cmd_interface.h>
#include <handsfree_hw/base_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <handsfree_msgs/robot_state.h>

#include <controller_manager/controller_manager.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for hf link and transport
#include <handsfree_hw/transport.h>
#include <handsfree_hw/transport_serial.h>
#include <hf_link.h>
#include <handsfree_hw/hf_hw.h>

namespace handsfree_hw {

class HF_HW_ros : public  hardware_interface::RobotHW{

public:
    HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr);

    double getFreq()const
    {
        return controller_freq_;
    }

    void mainloop();

private:
    //communication with embeded system
    HF_HW hf_hw_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    // publish the robot state for diagnose system
    ros::Publisher robot_state_publisher_;
    ros::ServiceServer getparam_srv_;
    ros::ServiceServer setparam_srv_;

    //parameter list
    std::string base_mode_;
    bool with_arm_;
    double controller_freq_;

    //hardware resource
    handsfree_msgs::robot_state robot_state;

    std::vector<double> wheel_pos_, wheel_vel_, wheel_eff_, wheel_cmd_;
    std::vector<double> arm_pos_  , arm_vel_  , arm_eff_, arm_cmd_;
    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double x_vel_, y_vel_, theta_vel_;

    double head1_servo1_pos_, head1_servo1_vel_, head1_servo1_eff_;
    double head2_servo1_pos_, head2_servo1_vel_, head2_servo1_eff_;
    double head1_servo1_cmd_, head1_servo2_cmd_;

    double head1_servo2_pos_, head1_servo2_vel_, head1_servo2_eff_;
    double head2_servo2_pos_, head2_servo2_vel_, head2_servo2_eff_;
    double head2_servo1_cmd_, head2_servo2_cmd_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface servo_pos_interface_;
    hardware_interface::VelocityJointInterface base_vel_interface_;

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;

    inline void writeBufferUpdate()
    {
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
        hf_hw_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];

        hf_hw_.getRobotAbstract()->expect_robot_speed.x = x_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.y = y_cmd_;
        hf_hw_.getRobotAbstract()->expect_robot_speed.z = theta_cmd_;

        if (with_arm_)
        {
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo1 = arm_cmd_[0];
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo2 = arm_cmd_[1];
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo3 = arm_cmd_[2];
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo4 = arm_cmd_[3];
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo5 = arm_cmd_[4];
            hf_hw_.getRobotAbstract()->expect_arm1_state.servo6 = arm_cmd_[5];
        }
        // the servo num is different
        hf_hw_.getRobotAbstract()->expect_head1_state.pitch  = head1_servo1_cmd_;
        hf_hw_.getRobotAbstract()->expect_head1_state.yaw  = head1_servo2_cmd_;
        hf_hw_.getRobotAbstract()->expect_head2_state.pitch  = head2_servo1_cmd_;
        hf_hw_.getRobotAbstract()->expect_head2_state.yaw  = head2_servo2_cmd_;
    }

    inline void readBufferUpdate()
    {
        x_     = hf_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_     = hf_hw_.getRobotAbstract()->measure_global_coordinate.y;
        theta_ = hf_hw_.getRobotAbstract()->measure_global_coordinate.z;

        x_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.x;
        y_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.y;
        theta_vel_ = hf_hw_.getRobotAbstract()->measure_robot_speed.z;

        wheel_pos_[0] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo1;
        wheel_pos_[1] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo2;
        wheel_pos_[2] = hf_hw_.getRobotAbstract()->measure_motor_mileage.servo3;

        robot_state.battery_voltage = hf_hw_.getRobotAbstract()->robot_parameters.robot_body_radius;
        robot_state.cpu_temperature = hf_hw_.getRobotAbstract()->robot_system_info.cpu_temperature;
        robot_state.cpu_usage = hf_hw_.getRobotAbstract()->robot_system_info.cpu_usage;
        robot_state.system_time = hf_hw_.getRobotAbstract()->robot_system_info.system_time;

        if (with_arm_)
        {
            arm_pos_[0] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo1;
            arm_pos_[1] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo2;
            arm_pos_[2] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo3;
            arm_pos_[3] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo4;
            arm_pos_[4] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo5;
            arm_pos_[5] = hf_hw_.getRobotAbstract()->measure_arm1_state.servo6;
        }

        wheel_vel_[0] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo1;
        wheel_vel_[1] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo2;
        wheel_vel_[2] = hf_hw_.getRobotAbstract()->measure_motor_speed.servo3;

        head1_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.pitch ;
        head1_servo1_vel_ = 0 ;
        head1_servo1_eff_ = 0 ;

        head1_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head1_state.yaw ;
        head1_servo2_vel_ = 0 ;
        head1_servo2_eff_ = 0 ;

        head2_servo1_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.pitch ;
        head2_servo1_vel_ = 0 ;
        head2_servo1_eff_ = 0 ;

        head2_servo2_pos_ = hf_hw_.getRobotAbstract()->measure_head2_state.yaw ;
        head2_servo2_vel_ = 0 ;
        head2_servo2_eff_ = 0 ;
    }
};

}


#endif
