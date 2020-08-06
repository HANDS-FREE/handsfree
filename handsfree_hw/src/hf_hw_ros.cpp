/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: 
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
    transport_url = url;

    ros::NodeHandle nh_private("~");
    nh_private.param<bool>("with_chassis", with_chassis_, true);
    nh_private.param<std::string>("chassis_type", chassis_type_, "stone_v3");
    nh_private.param<std::string>("chassis_base_mode", chassis_base_mode_, "4omni-wheel");

    nh_private.param<bool>("with_lift", with_lift_, false);
    nh_private.param<std::string>("lift_type", lift_type_, "lift_e120");

    nh_private.param<bool>("with_arm", with_arm_, false);
    nh_private.param<std::string>("arm_type", arm_type_, "arm_e6");
    nh_private.param<int>("arm_type", arm_dof_, 8);
    if(arm_dof_ < 0) arm_dof_ = 0;
    if(arm_dof_ > 8) arm_dof_ = 8;

    nh_private.param<bool>("with_head", with_head_, true);
    nh_private.param<std::string>("head_type", head_type_, "head_e2");

    nh_private.param<double>("freq", controller_freq_, 100);

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;
    x_vel_ = y_vel_ = theta_vel_ = 0.0;
    head_servo1_cmd_ = head_servo2_cmd_  =  0.0;
    head_servo1_pos_ = head_servo1_vel_ = head_servo1_eff_ = 0;
    head_servo2_pos_ = head_servo2_vel_ = head_servo2_eff_ = 0;
    button1_long_press_enable = false;
    button2_long_press_enable = false;
    last_thermal_infrared_status = false;

    if(with_chassis_)
    {
        robot_state_publisher_ = nh_.advertise<handsfree_msgs::robot_state>("robot_state", 10);
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);

        //register the hardware interface on the robothw
        hardware_interface::BaseStateHandle base_state_handle("mobile_base", &x_, &y_, &theta_, &x_vel_, &y_vel_, &theta_vel_);
        base_state_interface_.registerHandle(base_state_handle);
        registerInterface(&base_state_interface_);
        hardware_interface::BaseVelocityHandle base_handle(base_state_handle, &x_cmd_, &y_cmd_, &theta_cmd_);
        base_velocity_interface_.registerHandle(base_handle);
        registerInterface(&base_velocity_interface_);

        wheel_pos_.resize(6,0);
        wheel_vel_.resize(6.0);
        wheel_eff_.resize(6,0);
        wheel_cmd_.resize(6,0);

        if (chassis_base_mode_ == "3omni-wheel")
        {
            hardware_interface::JointStateHandle wheel1_state_handle("wheel1_joint", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
            jnt_state_interface_.registerHandle(wheel1_state_handle);
            hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
            base_vel_interface_.registerHandle(wheel1_handle);

            hardware_interface::JointStateHandle wheel2_state_handle("wheel2_joint", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
            jnt_state_interface_.registerHandle(wheel2_state_handle);
            hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
            base_vel_interface_.registerHandle(wheel2_handle);

            hardware_interface::JointStateHandle wheel3_state_handle("wheel3_joint", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
            jnt_state_interface_.registerHandle(wheel3_state_handle);
            hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
            base_vel_interface_.registerHandle(wheel3_handle);

            registerInterface(&jnt_state_interface_);
            registerInterface(&base_vel_interface_);
        }
        else if (chassis_base_mode_ == "2diff-wheel")
        {
            hardware_interface::JointStateHandle wheel1_state_handle("wheel1_joint", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
            jnt_state_interface_.registerHandle(wheel1_state_handle);
            hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
            base_vel_interface_.registerHandle(wheel1_handle);

            hardware_interface::JointStateHandle wheel2_state_handle("wheel2_joint", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
            jnt_state_interface_.registerHandle(wheel2_state_handle);
            hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
            base_vel_interface_.registerHandle(wheel2_handle);

            registerInterface(&jnt_state_interface_);
            registerInterface(&base_vel_interface_);
        }
        else if (chassis_base_mode_ == "4omni-wheel")
        {
            hardware_interface::JointStateHandle wheel1_state_handle("wheel1_joint", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
            jnt_state_interface_.registerHandle(wheel1_state_handle);
            hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
            base_vel_interface_.registerHandle(wheel1_handle);

            hardware_interface::JointStateHandle wheel2_state_handle("wheel2_joint", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
            jnt_state_interface_.registerHandle(wheel2_state_handle);
            hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
            base_vel_interface_.registerHandle(wheel2_handle);

            hardware_interface::JointStateHandle wheel3_state_handle("wheel3_joint", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
            jnt_state_interface_.registerHandle(wheel3_state_handle);
            hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
            base_vel_interface_.registerHandle(wheel3_handle);

            hardware_interface::JointStateHandle wheel4_state_handle("wheel4_joint", &wheel_pos_[3], &wheel_vel_[3], &wheel_eff_[3]);
            jnt_state_interface_.registerHandle(wheel4_state_handle);
            hardware_interface::JointHandle wheel4_handle(wheel4_state_handle, &wheel_cmd_[3]);
            base_vel_interface_.registerHandle(wheel4_handle);

            registerInterface(&jnt_state_interface_);
            registerInterface(&base_vel_interface_);
        }
    }

    if(with_arm_)
    {
        arm_pos_.resize(arm_dof_,0);
        arm_vel_.resize(arm_dof_,0);
        arm_eff_.resize(arm_dof_,0);
        arm_cmd_.resize(arm_dof_,0);

        arm_state_publisher_ = nh_.advertise<handsfree_msgs::arm_state>("arm/arm_state", 10);
        arm_joints_pos_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>("/handsfree/arm/set_arm_joints_pos", 5, boost::bind(&HF_HW_ros::callBackSetArmJointsPos, this, _1));
        //arm_end_pos_subscriber_ = nh_.subscribe<handsfree_msgs::PoseEuler>("/handsfree/arm/set_arm_end_pos", 5, boost::bind(&HF_HW_ros::callBackSetArmEndPos, this, _1));
        //arm_griper_pose_subscriber_ = nh_.subscribe<handsfree_msgs::PoseEuler>("/handsfree/arm/set_arm_griper_pos", 5, boost::bind(&HF_HW_ros::callBackSetArmGriperPos, this, _1));

        for (int i = 0;i < arm_dof_;i++)
        {
            std::stringstream ss;
            ss << "arm" << (i + 1)<<"_joint"; //get the joint name
            hardware_interface::JointStateHandle arm_state_handle(ss.str(), &arm_pos_[i], &arm_vel_[i], &arm_eff_[i]);
            jnt_state_interface_.registerHandle(arm_state_handle);

            //hardware_interface::JointHandle arm_handle(arm_state_handle , &arm_cmd_[i]);
            //servo_pos_interface_.registerHandle(arm_handle);
        }
    }

    if(with_head_)
    {
        hardware_interface::JointStateHandle head_servo1_state_handle("pitch_joint", &head_servo1_pos_, &head_servo1_vel_, &head_servo1_eff_);
        jnt_state_interface_.registerHandle(head_servo1_state_handle);
        hardware_interface::JointHandle head_servo1_handle(head_servo1_state_handle, &head_servo1_cmd_);
        servo_pos_interface_.registerHandle(head_servo1_handle);

        hardware_interface::JointStateHandle head_servo2_state_handle("yaw_joint", &head_servo2_pos_, &head_servo2_vel_, &head_servo2_eff_);
        jnt_state_interface_.registerHandle(head_servo2_state_handle);
        hardware_interface::JointHandle head_servo2_handle(head_servo2_state_handle, &head_servo2_cmd_);
        servo_pos_interface_.registerHandle(head_servo2_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&servo_pos_interface_);

    if (hf_hw_.initialize_ok())
    {
        ROS_INFO("system initialized succeed, ready for communication");
    }
    else
    {
        ROS_ERROR("robolink initialized failed, please check the serial port of the openre board,for details,please see: http://wiki.hfreetech.org/");
    }
}


void HF_HW_ros::imuDataUpdatePub(void)
{
    sensor_msgs::Imu imu_data;

    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "base_link";
    imu_data.orientation.x = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_quaternion.x;
    imu_data.orientation.y = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_quaternion.y;
    imu_data.orientation.z = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_quaternion.z;
    imu_data.orientation.w = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_quaternion.w;
    imu_data.linear_acceleration.x = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.linear_acceleration.x;
    imu_data.linear_acceleration.y = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.linear_acceleration.y;
    imu_data.linear_acceleration.z = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.linear_acceleration.z;
    imu_data.angular_velocity.x = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.angular_velocity.x;
    imu_data.angular_velocity.y = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.angular_velocity.y;
    imu_data.angular_velocity.z = hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.angular_velocity.z;

    std::cout << "orientation_euler_rpy:" << " Pitch = " << hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_euler_rpy.pitch
              << " Roll = " << hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_euler_rpy.roll
              << " Yaw = " << hf_hw_.getRobotAbstract()->sensors.imu_data.imu1.orientation_euler_rpy.yaw << std::endl;

    imu_publisher_.publish(imu_data);
}

void HF_HW_ros::armDataUpdatePub(void)
{
    if(with_arm_)
    {
        handsfree_msgs::arm_state arm_data;
        arm_data.header.stamp = ros::Time::now();
        arm_data.status = hf_hw_.getRobotAbstract()->arm.measure_arm_state.status;
        arm_data.voltage = hf_hw_.getRobotAbstract()->arm.measure_arm_state.voltage * 0.1;
        arm_data.current = hf_hw_.getRobotAbstract()->arm.measure_arm_state.current * 0.001;

        for(unsigned char i=0; i<arm_dof_ ; i++)
        {
            arm_data.joints.id.push_back(i+1);
            arm_data.joints.status.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].status);
            arm_data.joints.voltage.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].voltage * 0.1);
            arm_data.joints.current.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].current * 0.001);
            arm_data.joints.load.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].load);
            arm_data.joints.temperature.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].temperature);
            arm_data.joints.position.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].position * 0.001);
            arm_data.joints.speed.push_back(hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].speed * 0.001);
        }

        arm_data.end_pose.x = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.x * 0.0001; //m
        arm_data.end_pose.y = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.y * 0.0001;
        arm_data.end_pose.z = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.y * 0.0001;
        arm_data.end_pose.roll = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.roll * 0.001; //radian
        arm_data.end_pose.pitch = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.pitch * 0.001;
        arm_data.end_pose.yaw = hf_hw_.getRobotAbstract()->arm.measure_arm_state.end_pose.yaw * 0.001;

        arm_data.griper_pose.x = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.x * 0.0001; //m
        arm_data.griper_pose.y = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.y * 0.0001;
        arm_data.griper_pose.z = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.y * 0.0001;
        arm_data.griper_pose.roll = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.roll * 0.001; //radian
        arm_data.griper_pose.pitch = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.pitch * 0.001;
        arm_data.griper_pose.yaw = hf_hw_.getRobotAbstract()->arm.measure_arm_state.griper_pose.yaw * 0.001;

        arm_state_publisher_.publish(arm_data);

        for(unsigned char i=0; i<arm_dof_ ; i++)
        {
            arm_pos_[i] = hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].position / 1000.0;
            arm_vel_[i] = hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].speed / 1000.0;
            arm_eff_[i] = hf_hw_.getRobotAbstract()->arm.measure_arm_state.servo[i].load;
        }
    }
}

void HF_HW_ros::callBackSetArmJointsPos(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size() < arm_dof_) return;

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.command = 1;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[0] = msg->data[0] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[1] = msg->data[1] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[2] = msg->data[2] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[3] = msg->data[3] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[4] = msg->data[4] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[5] = msg->data[5] * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[6] = (short int)msg->data[6];
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[7] = (short int)msg->data[7];

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[0] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[1] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[2] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[3] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[4] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[5] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[6] = 0;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[7] = 0;

    hf_hw_.hwSendCommand(SET_ARM_STATE);
}

void HF_HW_ros::callBackSetArmEndPos(const handsfree_msgs::PoseEuler::ConstPtr &msg)
{
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.command = 2;

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.x = msg->x * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.y = msg->y * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.z = msg->z * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.roll = msg->roll * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.pitch = msg->pitch * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.end_pose.yaw = msg->yaw * 1000;

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[0] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[1] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[2] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[3] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[4] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[5] = 0.25*1000;

    hf_hw_.hwSendCommand(SET_ARM_STATE);
}

void HF_HW_ros::callBackSetArmGriperPos(const handsfree_msgs::PoseEuler::ConstPtr &msg)
{
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.command = 3;

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.x = msg->x * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.y = msg->y * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.z = msg->z * 10000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.roll = msg->roll * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.pitch = msg->pitch * 1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.griper_pose.yaw = msg->yaw * 1000;

    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[0] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[1] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[2] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[3] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[4] = 0.25*1000;
    hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[5] = 0.25*1000;

    hf_hw_.hwSendCommand(SET_ARM_STATE);
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

    while (ros::ok())
    {
        hf_hw_.checkHandshake();

        ros::Time currentTime = ros::Time::now();
        if (hf_hw_.updateCommand(GET_SYSTEM_INFO, count))
        {
            std::cout<< "spend time is  "<< (ros::Time::now() - currentTime).toSec()<<std::endl;
            robot_state_publisher_.publish(robot_state);
        }

        hf_hw_.updateCommand(GET_MOTOR_SPEED, count);
        hf_hw_.updateCommand(GET_GLOBAL_COORDINATE, count);
        hf_hw_.updateCommand(GET_ROBOT_SPEED, count);

        if(hf_hw_.updateCommand(GET_ARM_STATE, count))
        {
            armDataUpdatePub();
        }

        hf_hw_.updateCommand(GET_HEAD_STATE, count);

        if(hf_hw_.updateCommand(GET_SENSOR_IMU_DATA, count))
        {
            imuDataUpdatePub();
        }

        readBufferUpdate();

        cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));

        //ROS_INFO("head_servo1_cmd_ = %.4f  head_servo2_cmd_=%.4f" , head_servo1_cmd_ ,head_servo2_cmd_);
        writeBufferUpdate();

        hf_hw_.updateCommand(SET_ROBOT_SPEED, count);
        //hf_hw_.updateCommand(SET_ARM_STATE, count);
        hf_hw_.updateCommand(SET_HEAD_STATE , count);

        loop.sleep();
        count++;
    }
    cm_spinner.stop();
    hw_spinner.stop();
}

}
