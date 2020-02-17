#ifndef ARM_PARAMETERS_H
#define ARM_PARAMETERS_H

enum ArmType : unsigned char
{
    DOBOT1,
    DOBOT2,
    OTHERS_ARM
};

typedef struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
    float  servo5;
    float  servo6;
    float  servo7;
    float  servo8;
}__attribute__((packed)) ArmDOFVector;

typedef struct{
    float  x;
    float  y;
    float  z;
    float  pitch;
    float  roll;
    float  yaw;
}__attribute__((packed)) ArmPose;

typedef struct{

}__attribute__((packed)) ArmSystem;

/*****************************************************************************************/

typedef struct{
    ArmType type;
    float speed_low_filter;
    unsigned char dof;
    unsigned char control_enable;
}__attribute__((packed)) ArmParameters;

typedef struct{
    ArmDOFVector expect_arm_state;
    ArmDOFVector measure_arm_state;
    ArmSystem system_info;
}__attribute__((packed)) ArmControlData;

#endif // ARM_PARAMETERS_H
