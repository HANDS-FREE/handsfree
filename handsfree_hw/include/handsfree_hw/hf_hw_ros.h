/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robolink.cpp
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
#include <handsfree_msgs/robot_time.h>
#include <handsfree_msgs/dissensor.h>
#include <handsfree_msgs/arm_state.h>
#include <handsfree_msgs/PoseEuler.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>

#include <controller_manager/controller_manager.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>

// for robolink and transport
#include <handsfree_hw/transport.h>
#include <handsfree_hw/transport_serial.h>
#include <handsfree_hw/hf_hw.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

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
    void imuDataUpdatePub(void);
    void armDataUpdatePub(void);

    bool button1_long_press_enable,button2_long_press_enable;
    bool last_thermal_infrared_status;

    //communication with embeded system
    HF_HW hf_hw_;
    std::string transport_url;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;

    // publish the robot state for diagnose system
    ros::Publisher robot_state_publisher_;
    ros::Publisher imu_publisher_;

    ros::Publisher arm_state_publisher_;
    // Subscriber the robot state for diagnose system
    ros::Subscriber arm_joints_pos_subscriber_, arm_end_pos_subscriber_, arm_griper_pose_subscriber_;
    void callBackSetArmJointsPos(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void callBackSetArmEndPos(const handsfree_msgs::PoseEuler::ConstPtr &msg);
    void callBackSetArmGriperPos(const handsfree_msgs::PoseEuler::ConstPtr &msg);

    //hardware resource
    handsfree_msgs::robot_state robot_state;
    handsfree_msgs::robot_time robot_time;
    handsfree_msgs::dissensor dissensor;

    ros::ServiceServer getparam_srv_;
    ros::ServiceServer setparam_srv_;

    //parameter list
    std::string chassis_type_, chassis_base_mode_, lift_type_, arm_type_, head_type_;
    bool with_chassis_, with_lift_, with_arm_, with_head_;
    int arm_dof_;
    double controller_freq_;

    std::vector<double> wheel_pos_, wheel_vel_, wheel_eff_, wheel_cmd_;
    std::vector<double> arm_pos_  , arm_vel_  , arm_eff_, arm_cmd_;
    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double x_vel_, y_vel_, theta_vel_;

    double head_servo1_pos_, head_servo1_vel_, head_servo1_eff_;
    double head_servo2_pos_, head_servo2_vel_, head_servo2_eff_;
    double head_servo1_cmd_, head_servo2_cmd_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface servo_pos_interface_;
    hardware_interface::VelocityJointInterface base_vel_interface_;

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;

    inline void writeBufferUpdate()
    {
        if(with_chassis_)
        {
            hf_hw_.getRobotAbstract()->chassis.expect_motor_speed.servo1 = wheel_cmd_[0];
            hf_hw_.getRobotAbstract()->chassis.expect_motor_speed.servo2 = wheel_cmd_[1];
            hf_hw_.getRobotAbstract()->chassis.expect_motor_speed.servo3 = wheel_cmd_[2];
            hf_hw_.getRobotAbstract()->chassis.expect_motor_speed.servo4 = wheel_cmd_[3];

            hf_hw_.getRobotAbstract()->chassis.expect_robot_speed.x = x_cmd_;
            hf_hw_.getRobotAbstract()->chassis.expect_robot_speed.y = y_cmd_;
            hf_hw_.getRobotAbstract()->chassis.expect_robot_speed.z = theta_cmd_;
        }

        if (with_arm_)
        {
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.command = 1;

            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[0] = arm_cmd_[0] * 1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[1] = arm_cmd_[1] * 1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[2] = arm_cmd_[2] * 1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[3] = arm_cmd_[3] * 1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[4] = arm_cmd_[4] * 1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_position[5] = arm_cmd_[5] * 1000;

            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[0] = 0.25*1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[1] = 0.25*1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[2] = 0.25*1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[3] = 0.25*1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[4] = 0.25*1000;
            //hf_hw_.getRobotAbstract()->arm.expect_arm_state.joints_speed[5] = 0.25*1000;

            //std::cerr<<" arm: " << "cmd[0]=" <<  arm_cmd_[0] << " cmd_[1]=" <<  arm_cmd_[1] << " cmd_[2]=" << arm_cmd_[1]
            //        << "cmd[3]=" <<  arm_cmd_[3] << " cmd_[4]=" <<  arm_cmd_[4] << " cmd_[5]=" << arm_cmd_[5] << std::endl;
        }

        if(with_head_)
        {
            hf_hw_.getRobotAbstract()->head.expect_head_state.pitch  = head_servo1_cmd_;
            hf_hw_.getRobotAbstract()->head.expect_head_state.yaw  = head_servo2_cmd_;
        }
    }

    inline void readBufferUpdate()
    {
        robot_state.battery_voltage = hf_hw_.getRobotAbstract()->system_info.battery_voltage;
        robot_state.cpu_temperature = hf_hw_.getRobotAbstract()->system_info.cpu_temperature;
        robot_state.cpu_usage = hf_hw_.getRobotAbstract()->system_info.cpu_usage;
        robot_state.system_time = hf_hw_.getRobotAbstract()->system_info.system_time;
        robot_state.power_remain = hf_hw_.getRobotAbstract()->system_info.power_remain;

        if(with_chassis_)
        {
            x_     = hf_hw_.getRobotAbstract()->chassis.measure_global_coordinate.x;
            y_     = hf_hw_.getRobotAbstract()->chassis.measure_global_coordinate.y;
            theta_ = hf_hw_.getRobotAbstract()->chassis.measure_global_coordinate.z;

            x_vel_ = hf_hw_.getRobotAbstract()->chassis.measure_robot_speed.x;
            y_vel_ = hf_hw_.getRobotAbstract()->chassis.measure_robot_speed.y;
            theta_vel_ = hf_hw_.getRobotAbstract()->chassis.measure_robot_speed.z;

            wheel_pos_[0] = hf_hw_.getRobotAbstract()->chassis.measure_motor_mileage.servo1;
            wheel_pos_[1] = hf_hw_.getRobotAbstract()->chassis.measure_motor_mileage.servo2;
            wheel_pos_[2] = hf_hw_.getRobotAbstract()->chassis.measure_motor_mileage.servo3;
            wheel_pos_[3] = hf_hw_.getRobotAbstract()->chassis.measure_motor_mileage.servo4;

            wheel_vel_[0] = hf_hw_.getRobotAbstract()->chassis.measure_motor_speed.servo1;
            wheel_vel_[1] = hf_hw_.getRobotAbstract()->chassis.measure_motor_speed.servo2;
            wheel_vel_[2] = hf_hw_.getRobotAbstract()->chassis.measure_motor_speed.servo3;
            wheel_vel_[3] = hf_hw_.getRobotAbstract()->chassis.measure_motor_speed.servo3;

            robot_time.local_time_valid = hf_hw_.getRobotAbstract()->system_info.local_time.valid;
            robot_time.local_time_year = hf_hw_.getRobotAbstract()->system_info.local_time.year;
            robot_time.local_time_month = hf_hw_.getRobotAbstract()->system_info.local_time.month;
            robot_time.local_time_date = hf_hw_.getRobotAbstract()->system_info.local_time.date;
            robot_time.local_time_week = hf_hw_.getRobotAbstract()->system_info.local_time.week;
            robot_time.local_time_hour = hf_hw_.getRobotAbstract()->system_info.local_time.hour;
            robot_time.local_time_min = hf_hw_.getRobotAbstract()->system_info.local_time.min;
            robot_time.local_time_sec = hf_hw_.getRobotAbstract()->system_info.local_time.sec;
            robot_time.work_time1_sec = hf_hw_.getRobotAbstract()->system_info.work_time1.sec;
            robot_time.work_time1_min = hf_hw_.getRobotAbstract()->system_info.work_time1.min;
            robot_time.work_time1_hour = hf_hw_.getRobotAbstract()->system_info.work_time1.hour;
            robot_time.work_time2_sec = hf_hw_.getRobotAbstract()->system_info.work_time2.sec;
            robot_time.work_time2_min = hf_hw_.getRobotAbstract()->system_info.work_time2.min;
            robot_time.work_time2_hour = hf_hw_.getRobotAbstract()->system_info.work_time2.hour;

            dissensor.ult.clear();
            dissensor.laser.clear();
            dissensor.drop.clear();
            for(unsigned char i = 0 ; i < 12 ; i++)
            {
                dissensor.ult.push_back(hf_hw_.getRobotAbstract()->sensors.disio_data.ult[i]);
            }
            for(unsigned char i = 0 ; i < 12 ; i++)
            {
                dissensor.laser.push_back(hf_hw_.getRobotAbstract()->sensors.disio_data.laser[i]);
            }
            for(unsigned char i = 0 ; i < 4 ; i++)
            {
                dissensor.drop.push_back(hf_hw_.getRobotAbstract()->sensors.disio_data.drop[i]);
            }
            dissensor.collision = hf_hw_.getRobotAbstract()->sensors.disio_data.collision;

            dissensor.uwb_rssi = hf_hw_.getRobotAbstract()->sensors.disio_data.uwb_rssi;
            dissensor.uwb_distance = hf_hw_.getRobotAbstract()->sensors.disio_data.uwb_distance;
            dissensor.ibeacon_rssi = hf_hw_.getRobotAbstract()->sensors.disio_data.ibeacon_rssi;
            dissensor.ibeacon_distance = hf_hw_.getRobotAbstract()->sensors.disio_data.ibeacon_distance;

            dissensor.button1 = hf_hw_.getRobotAbstract()->sensors.disio_data.button1;
            dissensor.button2 = hf_hw_.getRobotAbstract()->sensors.disio_data.button2;
            dissensor.atuo_charger_state = hf_hw_.getRobotAbstract()->sensors.disio_data.atuo_charger_state;
            dissensor.hand_charger_state = hf_hw_.getRobotAbstract()->sensors.disio_data.hand_charger_state;
            dissensor.charger_distance = hf_hw_.getRobotAbstract()->sensors.disio_data.charger_distance;

            dissensor.thermal_infrared = hf_hw_.getRobotAbstract()->sensors.disio_data.thermal_infrared;

            dissensor.vcc_motor_state = hf_hw_.getRobotAbstract()->sensors.disio_data.vcc_motor_state;
            dissensor.vcc_pc_state = hf_hw_.getRobotAbstract()->sensors.disio_data.vcc_pc_state;
            dissensor.pc_boot_up_state = hf_hw_.getRobotAbstract()->sensors.disio_data.pc_boot_up_state;
            dissensor.break_stop_state = hf_hw_.getRobotAbstract()->sensors.disio_data.break_stop_state;

            dissensor.control_quality = hf_hw_.getRobotAbstract()->sensors.disio_data.control_quality;
            dissensor.chassis_online = hf_hw_.getRobotAbstract()->sensors.disio_data.chassis_online;
            dissensor.col_drop_alarm = hf_hw_.getRobotAbstract()->sensors.disio_data.col_drop_alarm;
            dissensor.over_speed_state = hf_hw_.getRobotAbstract()->sensors.disio_data.over_speed_state;
            dissensor.motor1_online_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor1_online_state;
            dissensor.motor2_online_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor2_online_state;
            dissensor.motor1_mode_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor1_mode_state;
            dissensor.motor2_mode_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor2_mode_state;
            dissensor.motor1_fault_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor1_fault_state;
            dissensor.motor2_fault_state = hf_hw_.getRobotAbstract()->sensors.disio_data.motor2_fault_state;

            dissensor.mqtt_online = hf_hw_.getRobotAbstract()->sensors.disio_data.mqtt_online;
            dissensor.mqtt_get_topic_state = hf_hw_.getRobotAbstract()->sensors.disio_data.mqtt_get_topic_state;
            dissensor.mqtt_command = hf_hw_.getRobotAbstract()->sensors.disio_data.mqtt_command;
        }

        //if (with_arm_)
        //{
        //}

        if(with_head_)
        {
            head_servo1_pos_ = hf_hw_.getRobotAbstract()->head.measure_head_state.pitch ;
            head_servo1_vel_ = 0 ;
            head_servo1_eff_ = 0 ;

            head_servo2_pos_ = hf_hw_.getRobotAbstract()->head.measure_head_state.yaw ;
            head_servo2_vel_ = 0 ;
            head_servo2_eff_ = 0 ;
        }
    }
};

}


#endif
