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
#include <handsfree_hw/hf_hw_ros.h>

namespace handsfree_hw {

HF_HW_ros::HF_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr) :
    hf_hw_(url, config_addr),
    nh_(nh)
{
    //get the parameter
    nh_.setCallbackQueue(&queue_);
    base_mode_ = "3omni-wheel";
    with_arm_ = false;
    controller_freq_ = 100;
    nh_.getParam("base_mode", base_mode_);
    nh_.getParam("with_arm", with_arm_);
    nh_.getParam("freq", controller_freq_);
    robot_state_publisher_ = nh_.advertise<handsfree_msgs::robot_state>("robot_state", 10);

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;
    x_vel_ = y_vel_ = theta_vel_ = 0.0;
    head1_servo1_cmd_ = head1_servo2_cmd_  = head2_servo1_cmd_ = head2_servo2_cmd_ = 0.0;
    head1_servo1_pos_ = head1_servo2_pos_ = head1_servo1_vel_ = head2_servo1_vel_ = head1_servo1_eff_ = head2_servo1_eff_ = 0;

    //register the hardware interface on the robothw
    hardware_interface::BaseStateHandle base_state_handle("mobile_base", &x_, &y_, &theta_, &x_vel_, &y_vel_, &theta_vel_);
    base_state_interface_.registerHandle(base_state_handle);
    registerInterface(&base_state_interface_);
    hardware_interface::BaseVelocityHandle base_handle(base_state_handle, &x_cmd_, &y_cmd_, &theta_cmd_);
    base_velocity_interface_.registerHandle(base_handle);
    registerInterface(&base_velocity_interface_);

    if (base_mode_ == "3omni-wheel")
    {
        wheel_pos_.resize(3,0);
        wheel_vel_.resize(3.0);
        wheel_eff_.resize(3,0);
        wheel_cmd_.resize(3,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel_1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel_2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        hardware_interface::JointStateHandle wheel3_state_handle("wheel_3", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
        jnt_state_interface_.registerHandle(wheel3_state_handle);
        hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
        base_vel_interface_.registerHandle(wheel3_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);
    } else if (base_mode_ == "2diff-wheel")
    {
        wheel_pos_.resize(2,0);
        wheel_vel_.resize(2.0);
        wheel_eff_.resize(2,0);
        wheel_cmd_.resize(2,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);

    } else if (base_mode_ == "4omni-wheel")
    {
        wheel_pos_.resize(4,0);
        wheel_vel_.resize(4.0);
        wheel_eff_.resize(4,0);
        wheel_cmd_.resize(4,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        hardware_interface::JointStateHandle wheel3_state_handle("wheel3", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
        jnt_state_interface_.registerHandle(wheel3_state_handle);
        hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
        base_vel_interface_.registerHandle(wheel3_handle);

        hardware_interface::JointStateHandle wheel4_state_handle("wheel4", &wheel_pos_[3], &wheel_vel_[3], &wheel_eff_[3]);
        jnt_state_interface_.registerHandle(wheel4_state_handle);
        hardware_interface::JointHandle wheel4_handle(wheel4_state_handle, &wheel_cmd_[3]);
        base_vel_interface_.registerHandle(wheel4_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);
    }

    if (with_arm_)
    {
        for (int i = 0;i < 6;i++)
        {
            //get the joint name
            std::stringstream ss;
            ss << "arm" << (i + 1)<<std::endl;
            hardware_interface::JointStateHandle arm_state_handle(ss.str(), &arm_pos_[i], &arm_pos_[i], &arm_pos_[i]);
            jnt_state_interface_.registerHandle(arm_state_handle);
            hardware_interface::JointHandle arm_handle(arm_state_handle , &arm_cmd_[i]);
            servo_pos_interface_.registerHandle(arm_handle);
        }
    }

    hardware_interface::JointStateHandle head1_servo1_state_handle("servo_1", &head1_servo1_pos_, &head1_servo1_vel_, &head1_servo1_eff_);
    jnt_state_interface_.registerHandle(head1_servo1_state_handle);
    hardware_interface::JointHandle head1_servo1_handle(head1_servo1_state_handle, &head1_servo1_cmd_);
    servo_pos_interface_.registerHandle(head1_servo1_handle);

    hardware_interface::JointStateHandle head1_servo2_state_handle("servo_2", &head1_servo2_pos_, &head1_servo2_vel_, &head1_servo2_eff_);
    jnt_state_interface_.registerHandle(head1_servo2_state_handle);
    hardware_interface::JointHandle head1_servo2_handle(head1_servo2_state_handle, &head1_servo2_cmd_);
    servo_pos_interface_.registerHandle(head1_servo2_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&servo_pos_interface_);

    if (hf_hw_.initialize_ok())
    {
        ROS_INFO("system initialized succeed, ready for communication");
    } else
    {
        ROS_ERROR("hf link initialized failed, please check the hardware");
    }
}

void HF_HW_ros::mainloop()
{
    ros::CallbackQueue cm_callback_queue;
    ros::NodeHandle cm_nh("mobile_base");
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager cm(this, cm_nh);

    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
    ros::AsyncSpinner hw_spinner(1, &queue_);
    ros::Rate loop(controller_freq_);
    cm_spinner.start();
    hw_spinner.start();

    int count = 0;
    ros::Time currentTime = ros::Time::now();
    while (ros::ok())
    {
        hf_hw_.checkHandshake();
        if (hf_hw_.updateCommand(READ_ROBOT_SYSTEM_INFO, count))
        {
            std::cout<< "spend time is  "<< (ros::Time::now() - currentTime).toSec()<<std::endl;
            currentTime = ros::Time::now();
            robot_state_publisher_.publish(robot_state);
        }
        hf_hw_.updateCommand(READ_MOTOR_SPEED, count);
        hf_hw_.updateCommand(READ_GLOBAL_COORDINATE, count);
        hf_hw_.updateCommand(READ_ROBOT_SPEED, count);
        hf_hw_.updateCommand(READ_HEAD_1, count);
        readBufferUpdate();

        cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));

        ROS_INFO("head1_servo1_cmd_ = %.4f  head1_servo2_cmd_=%.4f" , head1_servo1_cmd_ ,head1_servo2_cmd_);
        writeBufferUpdate();
        hf_hw_.updateCommand(SET_ROBOT_SPEED, count);
        hf_hw_.updateCommand(SET_HEAD_1, count);

        loop.sleep();
        count++;
    }

    cm_spinner.stop();
    hw_spinner.stop();
}

}
