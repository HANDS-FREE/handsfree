#ifndef HEAD_PARAMETERS_H
#define HEAD_PARAMETERS_H

enum HeadType{
    HFANALOG,
    HFDIGITAL,
    OTHERS_HEAD};

typedef  struct{
    float  servo1;
    float  servo2;
    float  servo3;
}HeadDOFVector;

typedef  struct{
    float  pitch;
    float  roll;
    float  yaw;
}HeadPose;

typedef  struct{
    HeadType type;
    float speed_low_filter;
    HeadPose range;  // radian
    HeadPose offset;  // radian
    HeadPose id;
    unsigned char  imu_fusion_enalbe;
    unsigned char control_enable;
}HeadParameters;

#endif // HEAD_PARAMETERS_H
