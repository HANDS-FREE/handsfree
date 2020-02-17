#ifndef HEAD_PARAMETERS_H
#define HEAD_PARAMETERS_H

enum HeadType : unsigned char
{
    HFANALOG,
    HFDIGITAL,
    OTHERS_HEAD
};

typedef struct{
    float  servo1;
    float  servo2;
    float  servo3;
}__attribute__((packed)) HeadDOFVector;

typedef struct{
    float  pitch;
    float  roll;
    float  yaw;
}__attribute__((packed)) HeadPose;

typedef struct{

}__attribute__((packed)) HeadSystem;

/*****************************************************************************************/

typedef struct{
    HeadType type;
    float speed_low_filter;
    HeadPose range;   //radian
    HeadPose offset;  //radian
    HeadPose id;
    unsigned char control_enable;
}__attribute__((packed)) HeadParameters;

typedef struct{
    HeadPose expect_head_state;    //(pitch,roll,yaw)(radian,radian,radian)
    HeadPose measure_head_state;
    HeadSystem system_info;
}__attribute__((packed)) HeadControlData;

#endif //HEAD_PARAMETERS_H
