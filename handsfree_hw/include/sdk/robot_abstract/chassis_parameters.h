#ifndef CHASSIS_PARAMETERS_H
#define CHASSIS_PARAMETERS_H

enum ChassisTFType{
    DIFFERENTIAL2,
    DIFFERENTIAL4,
    OMNI3,
    OMINI4,
    MECANUM4,
    CARLIKE };

typedef struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
}ChassisDOFVector;

typedef struct {
    float  x;
    float  y;
    float  z;
}ChassisCoord;

typedef struct{
    ChassisTFType type;
    float wheel_radius;
    float body_radius;
    float speed_low_filter;
    float motor_pid_t;
    unsigned char dof;
    unsigned char simulation_model;
    unsigned char  imu_fusion_enalbe;
    unsigned char control_enable;
}ChassisParameters;

#endif // CHASSIS_PARAMETERS_H
