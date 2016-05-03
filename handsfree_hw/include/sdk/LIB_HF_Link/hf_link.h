/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: hf_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* mawenke       2015.10.1   V1.0           creat this file
*
* Description: This file defined hands_free_robot simple communications protocol
*              please read Hands Free Link Manua.doc for detail
***********************************************************************************************************************/


#ifndef HF_LINK_H
#define HF_LINK_H

#include "robot_abstract.h"

#define HF_LINK_NODE_MODEL  1    //1master  0slave

#if HF_LINK_NODE_MODEL==0
#include "hf_link_port.h"
#endif

static const unsigned short int MESSAGE_BUFER_SIZE = 100;  //limite one message's size
/****structure for communications protocol , read Hands Free Link Manua.doc for detail****/
typedef struct HFMessage{
    unsigned char sender_id;
    unsigned char receiver_id;
    unsigned short int length;
    unsigned char data[MESSAGE_BUFER_SIZE];
}HFMessage;

//communication status
enum Recstate{
    WAITING_FF1,
    WAITING_FF2,
    SENDER_ID,
    RECEIVER_ID,
    RECEIVE_LEN_H,
    RECEIVE_LEN_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};

//comand type
enum Command{
    SHAKING_HANDS,
    READ_ROBOT_SYSTEM_INFO,
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
    SET_ARM_1,
    READ_ARM_1,
    SET_ARM_2,
    READ_ARM_2,
    SET_HEAD_1,
    READ_HEAD_1,
    SET_HEAD_2,
    READ_HEAD_2,
    READ_IMU_DATA,
    SET_ROBOY_PARAMETERS,
    SAVE_ROBOY_PARAMETERS,
    SET_MOTOR_PARAMETERS,
    SAVE_MOTOR_PARAMETERS,
    LAST_COMMAND_FLAG};

class HFLink
{
public:
    HFLink(unsigned char my_id_=0x11 , unsigned char friend_id_=0x01 , RobotAbstract* my_robot_=0)
    {
        hf_link_node_model = HF_LINK_NODE_MODEL ;
        port_num = 1 ;
        my_id = my_id_;         //0x11 means slave ,  read Hands Free Link Manua.doc for detail
        friend_id = friend_id_;
        hf_link_ack_en = 0;
        my_robot=my_robot_;
        //enable hflink ack , generally, master disable and slave enable
        //and slave also can disable to reduce communication burden
        if(hf_link_node_model==0) hf_link_ack_en = 1;
        shaking_hands_state=0;

        receive_state_=WAITING_FF1;
        command_state_=SHAKING_HANDS;
        rx_message_.sender_id=0;
        rx_message_.receiver_id=0;
        rx_message_.length=0;
        tx_message_.sender_id=0;
        tx_message_.receiver_id=0;
        tx_message_.length=0;

        receive_package_count=0;
        package_update_frequency=0;

        send_packag_count=0;
        tx_buffer[0]=0;
        tx_buffer_length=0;
    }

public:   // only for master
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

public: // only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];
    unsigned char shaking_hands_state;    //1 Success   0 Failed

public:  // common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);

    inline void set_my_id(unsigned char id){my_id =id ;}
    inline void set_friend_id(unsigned char id){friend_id =id ;}
    inline void set_port_num(unsigned char num){port_num =num ;}
    inline void enable_ack(void){if(hf_link_ack_en != 1) hf_link_ack_en=1;}
    inline void disable_ack(void){hf_link_ack_en=0;}

private:

    unsigned char hf_link_node_model;   // 0 slave , 1 master
    unsigned char port_num;
    unsigned char my_id;
    unsigned char friend_id;
    unsigned char hf_link_ack_en;         //enable hflink ack

    // robot abstract pointer to hflink
    RobotAbstract* my_robot;
    Recstate   receive_state_;
    Command    command_state_;
    HFMessage rx_message_ , tx_message_;

    float receive_package_count;
    float package_update_frequency;   // how many message receive in one second


    float send_packag_count;
    unsigned char tx_buffer[MESSAGE_BUFER_SIZE + 20];
    unsigned tx_buffer_length;

    unsigned int receive_check_sum_;
    short int receive_message_length_;
    short int byte_count_;
    unsigned char receiveFiniteStates(const unsigned char rx_data);
    unsigned char packageAnalysis(void);
    unsigned char readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    unsigned char setCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendMessage(void);
};


#endif  // #ifndef HF_LINK_H

