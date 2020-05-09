#ifndef ROBOLINK_H
#define ROBOLINK_H

#include <robot_abstract.h>
#include <state_machine.h>

//comand type
enum Command : unsigned char
{
    SHAKING_HANDS,
    GET_SYSTEM_INFO,
    //SET_SYSTEM_INFO
    //READ_MOTRO_PARAMETERS,
    WRITE_MOTOR_PARAMETERS,
    SAVE_MOTOR_PARAMETERS,
    //READ_CHASSIS_PARAMETERS,
    WRITE_CHASSIS_PARAMETERS,
    SAVE_CHASSIS_PARAMETERS,
    //READ_HEAD_PARAMETERS,
    WRITE_HEAD_PARAMETERS,
    SAVE_HEAD_PARAMETERS,
    //READ_ARM_PARAMETERS,
    WRITE_ARM_PARAMETERS,
    SAVE_ARM_PARAMETERS,
    SET_GLOBAL_SPEED,
    GET_GLOBAL_SPEED,
    SET_ROBOT_SPEED,
    GET_ROBOT_SPEED,
    SET_MOTOR_SPEED,
    GET_MOTOR_SPEED,
    GET_MOTOR_MILEAGE,
    GET_GLOBAL_COORDINATE,
    GET_ROBOT_COORDINATE,
    CLEAR_COORDINATE_DATA,
    SET_HEAD_STATE,
    GET_HEAD_STATE,
    SET_ARM_STATE,
    GET_ARM_STATE,
    GET_SENSOR_IMU_DATA,
    GET_SENSOR_DIS_DATA,
    GET_SENSOR_GPS_DATA,
    READ_MOTRO_PARAMETERS,
    READ_CHASSIS_PARAMETERS,
    READ_HEAD_PARAMETERS,
    READ_ARM_PARAMETERS,
    SET_SYSTEM_INFO,
    SET_IOCONTROL_DATA,
    LAST_COMMAND_FLAG
};

class RoboLink : public StateMachine
{
public:
    RoboLink(RobotAbstract *robot_ , unsigned char my_id_=0x11 , unsigned char friend_id_=0x01 , unsigned char port_num_=0 , unsigned int baudrate=921600) :
        StateMachine(my_id_ , friend_id_ , port_num_ , baudrate)
    {
        robolink_node_model = ROBOLINK_NODE_MODEL ;
        robot=robot_;
        shaking_hands_state = 0;
        analysis_package_count  = 0;
        command_state_ = SHAKING_HANDS;
    }

public:  
    //only for master
    //the master can use masterSendCommand function to send data to slave
    //like SET_GLOBAL_SPEED , READ_ROBOT_SYSTEM_INFO, READ_ROBOT_SPEED...
    unsigned char masterSendCommand(const Command command_state);
    inline unsigned char getReceiveRenewFlag(const Command command_state) const
    {
        return receive_package_renew[command_state];
    }
    inline unsigned char* getSerializedData(void)
    {
        return tx_buffer;
    }
    inline int getSerializedLength(void)
    {
        return tx_buffer_length;
    }

public: 
    //only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];

public:  
    //common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);

private:
    unsigned char robolink_node_model;      // 0 slave , 1 master
    unsigned char shaking_hands_state;     //1 Success   0 Failed
    float analysis_package_count;
    RobotAbstract* robot;      //robot abstract pointer to RoboLink
    Command    command_state_;

    unsigned char packageAnalysis(void);
    unsigned char readCommandAnalysis(const Command command_state , unsigned char *p , const unsigned short int len);
    unsigned char setCommandAnalysis(const Command command_state , unsigned char *p , const unsigned short int len);
    void sendStruct(const Command command_state , unsigned char *p , const unsigned short int len);
};

#endif  // #ifndef ROBOLINK_H

