#ifndef ARM_PARAMETERS_H
#define ARM_PARAMETERS_H

enum ArmType : unsigned char
{
    DOBOT2,
    HANDSFREE_ARM_E6,
    OTHERS_ARM
};

typedef struct{
    unsigned char status; //0~7bit , 0: online/offline
    unsigned short int voltage; //unit: 0.1v
    short int current;  //-10000 ~ 10000 , mA
    short int load;
    unsigned char temperature; //unit: degree
    short int position; //-PI ~ PI unit: 0.001 radian
    short int speed; //unit: 0.001 radian/s
}__attribute__((packed)) ArmJointData;

typedef struct{
    short int  x;  //unit: 0.1mm
    short int  y;
    short int  z;
    short int  pitch; //unit: 0.001 radian
    short int  roll;
    short int  yaw;
}__attribute__((packed)) ArmEndPose;

/*****************************************************************************************/

#define ARM_MAX_DOF_NUM 8

typedef struct{
    unsigned char command;
    short int joints_position[ARM_MAX_DOF_NUM]; //-PI ~ PI unit: 0.001 radian
    short int joints_speed[ARM_MAX_DOF_NUM]; //unit: 0.001 radian/s
    ArmEndPose end_pose;
    ArmEndPose griper_pose;
}__attribute__((packed)) ArmDOFControl;

typedef struct{
    unsigned char status; //0~7bit , 0: online/offline
    unsigned short int voltage; //unit: 0.1v
    short int current;  //-10000 ~ 10000 , mA
    ArmJointData servo[ARM_MAX_DOF_NUM];
    ArmEndPose end_pose;
    ArmEndPose griper_pose;
}__attribute__((packed)) ArmDOFFeedBack;

/*****************************************************************************************/

typedef struct{
    ArmType type;
    float speed_low_filter;
    unsigned char dof;
    unsigned char control_enable;
}__attribute__((packed)) ArmParameters;

typedef struct{
    ArmDOFControl expect_arm_state;
    ArmDOFFeedBack measure_arm_state;
}__attribute__((packed)) ArmControlData;

#endif // ARM_PARAMETERS_H
