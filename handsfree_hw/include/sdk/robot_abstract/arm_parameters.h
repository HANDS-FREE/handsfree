#ifndef ARM_PARAMETERS_H
#define ARM_PARAMETERS_H

enum ArmType{
    DOBOT1,
    DOBOT2,
    OTHERS_ARM };

typedef  struct{
    float  servo1;
    float  servo2;
    float  servo3;
    float  servo4;
    float  servo5;
    float  servo6;
    float  servo7;
    float  servo8;
}ArmDOFVector;

typedef  struct{
    ArmType type;
    float speed_low_filter;
    unsigned char dof;
    unsigned char imu_fusion_enalbe;
    unsigned char control_enable;
}ArmParameters;

#endif // ARM_PARAMETERS_H
