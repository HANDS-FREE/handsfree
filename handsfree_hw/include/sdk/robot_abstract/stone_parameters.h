/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: stone_parameters.h
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
*
* Description:
***********************************************************************************************************************/

#ifndef STONE_PARAMETERS_H
#define STONE_PARAMETERS_H

#include "robot_abstract.h"

class StoneParameters : public RobotParameters
{
public:
    StoneParameters() : RobotParameters()
    {
        motor_para.motor_id = 0;
        motor_para.encoder_num  = 1560 ;
        motor_para.pwm_max = 5000;
        motor_para.pwm_dead_zone = 10;
        motor_para.speed_low_filter = 0.3;
        motor_para.protect_current = 200;  // 200A means disable current  protect
        motor_para.pid =  {0.0f  , 0.0f , 0.0f , 60.0f , 500.0f ,0.2f};

        chassis_para.type =  DIFFERENTIAL2;
        chassis_para.wheel_radius = 0.0320;
        chassis_para.body_radius = 0.1592;
        chassis_para.speed_low_filter = 0.4;
        chassis_para.motor_pid_t = 0.02;
        chassis_para.dof = 2;
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

#endif // STONE_PARAMETERS_H
