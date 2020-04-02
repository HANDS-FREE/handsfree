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
    base_mode_ = "4omni-wheel";
    with_arm_ = false;
    controller_freq_ = 100;
    nh_.getParam("base_mode", base_mode_);
    nh_.getParam("with_arm", with_arm_);
    nh_.getParam("freq", controller_freq_);
    robot_state_publisher_ = nh_.advertise<handsfree_msgs::robot_state>("robot_state", 10);
    robot_time_publisher_ = nh_.advertise<handsfree_msgs::robot_time>("robot_time", 10);
    hw_control_cmd_publisher_ = nh_.advertise<std_msgs::String>("hw_control_cmd", 10);

    dissensor_publisher_ = nh_.advertise<handsfree_msgs::dissensor>("dissensor", 10);
    dissensor_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("dissensor_scan", 10);

    charge_distance_publisher_ = nh_.advertise<std_msgs::Float32>("charge_sonar_distance", 10);
    atuo_changer_state_publisher_ = nh_.advertise<std_msgs::Int32>("atuo_changer_state", 10);
    hand_changer_state_publisher_ = nh_.advertise<std_msgs::Int32>("hand_changer_state", 10);

    uwb_distance_publisher_ = nh_.advertise<std_msgs::Float32>("uwb_distance", 10);

    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;
    x_vel_ = y_vel_ = theta_vel_ = 0.0;
    head_servo1_cmd_ = head_servo2_cmd_  =  0.0;
    head_servo1_pos_ = head_servo1_vel_ = head_servo1_eff_ = 0;
    head_servo2_pos_ = head_servo2_vel_ = head_servo2_eff_ = 0;
    button1_long_press_enable = false;
    button2_long_press_enable = false;
    last_thermal_infrared_status = false;

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
    } else if (base_mode_ == "2diff-wheel")
    {
        wheel_pos_.resize(2,0);
        wheel_vel_.resize(2.0);
        wheel_eff_.resize(2,0);
        wheel_cmd_.resize(2,0);

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

    } else if (base_mode_ == "4omni-wheel")
    {
        wheel_pos_.resize(4,0);
        wheel_vel_.resize(4.0);
        wheel_eff_.resize(4,0);
        wheel_cmd_.resize(4,0);

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

    hardware_interface::JointStateHandle head_servo1_state_handle("pitch_joint", &head_servo1_pos_, &head_servo1_vel_, &head_servo1_eff_);
    jnt_state_interface_.registerHandle(head_servo1_state_handle);
    hardware_interface::JointHandle head_servo1_handle(head_servo1_state_handle, &head_servo1_cmd_);
    servo_pos_interface_.registerHandle(head_servo1_handle);

    hardware_interface::JointStateHandle head_servo2_state_handle("yaw_joint", &head_servo2_pos_, &head_servo2_vel_, &head_servo2_eff_);
    jnt_state_interface_.registerHandle(head_servo2_state_handle);
    hardware_interface::JointHandle head_servo2_handle(head_servo2_state_handle, &head_servo2_cmd_);
    servo_pos_interface_.registerHandle(head_servo2_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&servo_pos_interface_);

    m_subWorktime = nh_.subscribe<std_msgs::Int32MultiArray>("/handsfree/set_worktime_array", 5, boost::bind(&HF_HW_ros::callBackPatrolSetWorktime, this, _1));

    if (hf_hw_.initialize_ok())
    {
        ROS_INFO("system initialized succeed, ready for communication");
    } else
    {
        ROS_ERROR("robolink initialized failed, please check the serial port of the openre board,for details,please see: http://wiki.hfreetech.org/");
    }
}

#define PI 3.14159265
#define DEG_TO_RAD(x) (x * PI / 180.0)
#define MAX_RANGE 1.0 // meters
#define MIN_RANGE 0.02 // meters
#define ROBOTRADIUS 0.25 // meters
#define MIN_ANGLE (-180)    // degrees
#define MAX_ANGLE 180  // degrees
#define DEG_INCREMENT 0.25 // degrees
#define DISSENSORREGION 5 // degrees
#define NUM_READINGS  (int)((MAX_ANGLE-MIN_ANGLE) / DEG_INCREMENT + 1)
#define NUMBER_OF_SONAR_SENSORS 12
#define SONAR_FREQUENCY 10                                                      

void HF_HW_ros::dissensorScanPub(void)
{
    static float sonar_directions[NUMBER_OF_SONAR_SENSORS] = {0, 30, 60, 90, 120, 150, 180,-150,-120,-90,-60,-30};
    float sonar_data[NUMBER_OF_SONAR_SENSORS];
    float ranges[NUM_READINGS];
    float intensities[NUM_READINGS];

    for(unsigned int i = 0; i < NUMBER_OF_SONAR_SENSORS; i++){
        if(dissensor.ult.size() < NUMBER_OF_SONAR_SENSORS) return;
        //std::cerr<<"dissensor.ult.at(i)"<<dissensor.ult.at(i)<<std::endl;
        if(dissensor.ult.at(i)/1000.0 > MIN_RANGE && dissensor.ult.at(i)/1000.0 < MAX_RANGE) sonar_data[i] = dissensor.ult.at(i)/1000.0;
        else sonar_data[i] = std::numeric_limits<float>::infinity();
    }
    for(unsigned int i = 0; i < NUM_READINGS; i++){
        ranges[i] = std::numeric_limits<float>::infinity();;
        intensities[i] = 0.5;
    }

    // Generate actual sonar data in correct positions
    for (unsigned int i = 0; i < NUMBER_OF_SONAR_SENSORS; i++)
    {
        // Place sonar data into correct position in ranges array
        int index = (sonar_directions[i] - MIN_ANGLE)/DEG_INCREMENT;
        //std::cerr<<"index"<<index<<std::endl;
        int index_i = index - DISSENSORREGION/DEG_INCREMENT;
        int index_j = index + DISSENSORREGION/DEG_INCREMENT;
        for(int index = index_i;index<=index_j;index++)
        {
            int p=index;
            if(index < 0) p = index + NUM_READINGS;
            if(index >= NUM_READINGS) p = index - NUM_READINGS;
            ranges[p] = ROBOTRADIUS + sonar_data[i];
            intensities[p] = 0.5;
        }
    }

    // Populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "/base_link";
    scan.angle_min = DEG_TO_RAD(MIN_ANGLE);
    scan.angle_max = DEG_TO_RAD(MAX_ANGLE);
    scan.angle_increment = DEG_TO_RAD(DEG_INCREMENT);
    scan.time_increment = (1.0 / SONAR_FREQUENCY) / (NUM_READINGS);
    scan.range_min = MIN_RANGE;
    scan.range_max = 1001;

    scan.ranges.resize(NUM_READINGS);
    scan.intensities.resize(NUM_READINGS);
    for(unsigned int i = 0; i < NUM_READINGS; ++i){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
    }
    dissensor_scan_publisher_.publish(scan);
}  

void HF_HW_ros::chargerStatePub(void)
{

    std_msgs::Float32 charge_dis_;
    std_msgs::Int32 atuo_changer_state_,hand_changer_state_;

    charge_dis_.data =  dissensor.charger_distance;
    atuo_changer_state_.data = dissensor.atuo_charger_state;
    hand_changer_state_.data = dissensor.hand_charger_state;
    charge_distance_publisher_.publish(charge_dis_);
    atuo_changer_state_publisher_.publish(atuo_changer_state_);
    hand_changer_state_publisher_.publish(hand_changer_state_);
}

void HF_HW_ros::hwIOStatePub(void)
{
    std_msgs::String msg;

    if(dissensor.button1 == 0)
    {
        button1_long_press_enable = true;
    }
    else if (dissensor.button1 == 1)
    {
        dissensor.button1 = 0;
        msg.data = "HW_Button1_Click";
        hw_control_cmd_publisher_.publish(msg);
    }
    else if(dissensor.button1 == 2)
    {
        dissensor.button1 = 0;
        msg.data = "HW_Button1_Double_Click";
        hw_control_cmd_publisher_.publish(msg);
    }
    else if(dissensor.button1 == 3 && button1_long_press_enable)
    {
        msg.data = "HW_Button1_Long_Press";
        hw_control_cmd_publisher_.publish(msg);
    }
    else{}

    if(dissensor.button2 == 0)
    {
        button2_long_press_enable = true;
    }
    else if (dissensor.button2 == 1)
    {
        dissensor.button2 = 0;
        msg.data = "HW_Button2_Click";
        hw_control_cmd_publisher_.publish(msg);
    }
    else if(dissensor.button2 == 2)
    {
        dissensor.button2 = 0;
        msg.data = "HW_Button2_Double_Click";
        hw_control_cmd_publisher_.publish(msg);
    }
    else if(dissensor.button2 == 3 && button2_long_press_enable)
    {
        msg.data = "HW_Button2_Long_Press";
        hw_control_cmd_publisher_.publish(msg);
    }
    else{}

    if(last_thermal_infrared_status && dissensor.thermal_infrared == 0)
    {
        last_thermal_infrared_status = false;
        msg.data = "HW_Thermal_Infrared_False";
        hw_control_cmd_publisher_.publish(msg);
    }
    if(!last_thermal_infrared_status && dissensor.thermal_infrared == 1)
    {
        last_thermal_infrared_status = true;
        msg.data = "HW_Thermal_Infrared_True";
        hw_control_cmd_publisher_.publish(msg);
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

void HF_HW_ros::uwb_ibeacon_pub(void)
{
    std_msgs::Float32 uwb_dis_;
    uwb_dis_.data = dissensor.uwb_distance;
    uwb_distance_publisher_.publish(uwb_dis_);
}

void HF_HW_ros::callBackPatrolSetWorktime(const std_msgs::Int32MultiArray::ConstPtr &msg)
{

    if(msg->data.size() != 4)
    {
        std::cerr<<"HF_HW_ros::callBackPatrolSetWorktime Error: worktime data error"<<std::endl;
    }
    else
    {
        int hours = msg->data.at(0)/100;
        int minutes = msg->data.at(0)%100;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time1.valid=1;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time1.sec=0;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time1.min=minutes;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time1.hour=hours;

        hours = msg->data.at(3)/100;
        minutes = msg->data.at(3)%100;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time2.valid=1;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time2.sec=0;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time2.min=minutes;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time2.hour=hours;
        std::cerr<<"HF_HW_ros::callBackPatrolSetWorktime Successful: set worktime"<<std::endl;
        hf_hw_.hwSendCommand(SET_SYSTEM_INFO);
        hf_hw_.getRobotAbstract()->expect_system_info.work_time1.valid=0;
        hf_hw_.getRobotAbstract()->expect_system_info.work_time2.valid=0;
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

    while (ros::ok())
    {
        hf_hw_.checkHandshake();

        ros::Time currentTime = ros::Time::now();
        if (hf_hw_.updateCommand(GET_SYSTEM_INFO, count))
        {
            std::cout<< "spend time is  "<< (ros::Time::now() - currentTime).toSec()<<std::endl;
            robot_state_publisher_.publish(robot_state);
            robot_time_publisher_.publish(robot_time);
        }

        if(hf_hw_.updateCommand(GET_SENSOR_DIS_DATA,count))
        {
            dissensor_publisher_.publish(dissensor);
            dissensorScanPub();
            chargerStatePub();
            hwIOStatePub();
            uwb_ibeacon_pub();
        }

        hf_hw_.updateCommand(GET_MOTOR_SPEED, count);
        hf_hw_.updateCommand(GET_GLOBAL_COORDINATE, count);
        hf_hw_.updateCommand(GET_ROBOT_SPEED, count);
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
        hf_hw_.updateCommand(SET_HEAD_STATE , count);

        loop.sleep();
        count++;

        if(hf_hw_.time_out_cnt_ >= 40) //20s
        {
            hf_hw_.time_out_cnt_ = 0;
            std_msgs::String msg;
            if((access(transport_url.substr(transport_url.find("://")+3).c_str(),0)) != -1)
            {
                msg.data = "Error: " + transport_url.substr(transport_url.find("://")+3) + "_Timeout , Find_Serial_And_Restart";
                hf_hw_.restartTransportSerial(transport_url);
            }
            else
            {
                msg.data = "Error:" + transport_url.substr(transport_url.find("://")+3) + "_Timeout , Not_Find_Serial";
            }
            std::cout << msg.data << std::endl;
            hw_control_cmd_publisher_.publish(msg);
            //break;
        }
    }
    cm_spinner.stop();
    hw_spinner.stop();
}

}
