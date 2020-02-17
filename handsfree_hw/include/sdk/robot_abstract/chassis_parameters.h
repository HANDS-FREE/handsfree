#ifndef CHASSIS_PARAMETERS_H
#define CHASSIS_PARAMETERS_H

enum ChassisTFType : unsigned char
{
    DIFFERENTIAL2,
    DIFFERENTIAL4,
    OMNI3,
    OMINI4,
    MECANUM4,
    CARLIKE
};

typedef struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
}__attribute__((packed)) ChassisDOFVector;

typedef struct {
    float  x;
    float  y;
    float  z;
}__attribute__((packed)) ChassisCoord;

/*****************************************************************************************/
typedef struct{    
    unsigned char control_quality;
    unsigned char chassis_online;
    unsigned char col_drop_alarm;
    unsigned char over_speed_state;
    unsigned char motor_power_state;
}__attribute__((packed)) ChassisSystem;

/*****************************************************************************************/

typedef struct{
    ChassisTFType type;
    float wheel_radius;
    float body_radius;
    float speed_low_filter;
    unsigned char imu_fusion_enalbe;
    unsigned char control_enable;
    ChassisCoord max_speed_limit; //maximum speed limit
    ChassisCoord max_acc_lim; //maximum acceleration limit
}__attribute__((packed)) ChassisParameters;

//unit distances : metres
//angle： radian

typedef struct{
    ChassisDOFVector expect_motor_speed;   //(x1,x2,x3)(radian/s,radian/s,radian/s)
    ChassisDOFVector measure_motor_speed;
    ChassisCoord  expect_robot_speed;   //(x,y,w)(meter/s,meter/s,radian/s) reference system:robot
    ChassisCoord  measure_robot_speed;
    ChassisCoord  expect_global_speed;  //(x,y,w)(meter/s,meter/s,radian/s) reference system:global such as /map /odmo ;
    ChassisCoord  measure_global_speed;

    ChassisCoord  measure_robot_coordinate;
    ChassisCoord  measure_global_coordinate;  //(x,y,w)(meter,meter,radian)
    ChassisDOFVector measure_motor_mileage; //(x1,x2,x3)(radian,radian,radian)

    ChassisSystem system_info;
}__attribute__((packed)) ChassisControlData;

#endif // CHASSIS_PARAMETERS_H
