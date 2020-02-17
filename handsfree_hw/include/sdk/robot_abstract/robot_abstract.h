/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* Contact:  QQ Exchange Group -- 521037187
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke
* Description:
***********************************************************************************************************************/

#ifndef ROBOT_ABSTRACT_H
#define ROBOT_ABSTRACT_H

#include "motor_parameters.h"
#include "chassis_parameters.h"
#include "head_parameters.h"
#include "arm_parameters.h"
#include "sensor_data.h"
#include "system_info.h"
#include "string.h"

class RobotParameters
{
public:
    RobotParameters(){
        degree_to_radian = 0.017453f;
        radian_to_degree = 57.2958f;
        strcpy(robot_info.robot_name, "robot_name");
        strcpy(robot_info.robot_description ,  "this is a (name) robot of handsfree");
        memset(&motor_para , 0 , sizeof(motor_para));
        memset(&chassis_para , 0 , sizeof(chassis_para));
        memset(&head_para , 0 , sizeof(head_para));
        memset(&arm_para , 0 , sizeof(arm_para));
    }

public:    
    RobotInfo robot_info;
    SystemParameters system_para;
    MotorParameters motor_para;
    ChassisParameters chassis_para;
    HeadParameters head_para;
    ArmParameters arm_para;
    float degree_to_radian , radian_to_degree;
};

class RobotAbstract
{
public:
    RobotAbstract()
    {
        para = RobotParameters();

        memset(&motors , 0 , sizeof(motors));
        memset(&chassis , 0 , sizeof(chassis));
        memset(&head , 0 , sizeof(head));
        memset(&arm , 0 , sizeof(arm));
        memset(&sensors , 0 , sizeof(sensors));
        memset(&system_info , 0 , sizeof(system_info));
        memset(&expect_system_info , 0 , sizeof(expect_system_info));
    }

    RobotParameters para;
    SystemInfo system_info,expect_system_info;
    MotorsControlData motors;
    ChassisControlData chassis;
    HeadControlData head;
    ArmControlData arm;
    SensorsData sensors;
};

#endif // ROBOT_ABSTRACT_H

