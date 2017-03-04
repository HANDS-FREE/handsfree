#ifndef HF_LINK_H
#define HF_LINK_H

#include "robot_abstract.h"
#include "hf_link_state_machine.h"

//comand type
enum Command{
    SHAKING_HANDS,
    READ_SYSTEM_INFO,
    SET_MOTOR_PARAMETERS,
    SAVE_MOTOR_PARAMETERS,
    SET_CHASSIS_PARAMETERS,
    SAVE_CHASSIS_PARAMETERS,
    SET_HEAD_PARAMETERS,
    SAVE_HEAD_PARAMETERS,
    SET_ARM_PARAMETERS,
    SAVE_ARM_PARAMETERS,
    SET_GLOBAL_SPEED,
    READ_GLOBAL_SPEED,
    SET_ROBOT_SPEED,
    READ_ROBOT_SPEED,
    SET_MOTOR_SPEED,
    READ_MOTOR_SPEED,
    READ_MOTOR_MILEAGE,
    READ_GLOBAL_COORDINATE,
    READ_ROBOT_COORDINATE,
    CLEAR_COORDINATE_DATA,
    SET_HEAD_STATE,
    READ_HEAD_STATE,
    SET_ARM_STATE,
    READ_ARM_STATE,
    READ_IMU_BASE_DATA,
    READ_IMU_FUSION_DATA,
    READ_GPS_DATA,
    LAST_COMMAND_FLAG};

class HFLink : public StateMachine
{
public:
    HFLink(RobotAbstract* robot_  , unsigned char my_id_= 0x11 , unsigned char friend_id_= 0x01 , unsigned char port_num_ = 1) :
        StateMachine(my_id_ , friend_id_ , port_num_)
    {
        hf_link_node_model = HF_LINK_NODE_MODEL ;
        //enable hflink ack , generally, master disable and slave enable
        //and slave also can disable to reduce communication burden
        hf_link_ack_en = 0;
        if(hf_link_node_model == 0) hf_link_ack_en = 1;

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

public: 
    //only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];

public:  
    //common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);
    inline void enable_ack(void){if(hf_link_ack_en != 1) hf_link_ack_en=1;}
    inline void disable_ack(void){hf_link_ack_en=0;}

private:
    unsigned char hf_link_node_model;      // 0 slave , 1 master
    unsigned char hf_link_ack_en;                //enable hflink ack
    unsigned char shaking_hands_state;     //1 Success   0 Failed
    float analysis_package_count;
    RobotAbstract* robot;      //robot abstract pointer to hflink
    Command    command_state_;

    unsigned char packageAnalysis(void);
    unsigned char readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    unsigned char setCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
};

#endif  // #ifndef HF_LINK_H

