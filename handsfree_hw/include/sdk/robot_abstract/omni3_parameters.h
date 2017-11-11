#ifndef OMNI3_PARAMETERS_H
#define OMNI3_PARAMETERS_H

#include "robot_abstract.h"

class Omni3Parameters : public RobotParameters
{
public:
    Omni3Parameters() : RobotParameters()
    {
        motor_para.motor_id = 0;
        motor_para.encoder_num  = 28000 ;
        motor_para.pwm_max = 5000;
        motor_para.pwm_dead_zone = 10;
        motor_para.speed_low_filter = 0.3;
        motor_para.protect_current = 200;  // 200A means disable current  protect
        motor_para.pid =  {0.0f  , 0.0f , 0.0f , 6.0f , 3.0f ,0.01f};

        chassis_para.type =  OMNI3;
        chassis_para.wheel_radius = 0.0320;
        chassis_para.body_radius = 0.1592;
        chassis_para.speed_low_filter = 0.4;
        chassis_para.motor_pid_t = 0.02;
        chassis_para.dof = 3;
        chassis_para.simulation_model = 0;
        chassis_para.imu_fusion_enalbe = 0;
        chassis_para.control_enable = 1;

        head_para.type = HFANALOG;
        head_para.speed_low_filter = 0.3;
        head_para.range.pitch = 50 * degree_to_radian;
        head_para.range.yaw = 70 * degree_to_radian;
        head_para.offset.pitch = -5 *degree_to_radian;
        head_para.offset.yaw = 60 * degree_to_radian;
        head_para.id.pitch = 7;
        head_para.id.yaw = 8;
        head_para.imu_fusion_enalbe = 0;
        head_para.control_enable = 1;

        arm_para.type = DOBOT2;
        arm_para.speed_low_filter = 1;
        arm_para.dof = 4;
        arm_para.imu_fusion_enalbe = 0;
        arm_para.control_enable = 1;
    }
};

#endif // OMNI3_PARAMETERS_H
