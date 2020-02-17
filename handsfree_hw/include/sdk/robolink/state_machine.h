#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdio.h>
#include <platform.h>

//limite one message's size
static const unsigned short int MESSAGE_BUFER_SIZE = 1024;
typedef struct RobotMessage{
    unsigned char sender_id;
    unsigned char receiver_id;
    unsigned short int length;
    unsigned char data[MESSAGE_BUFER_SIZE];
}RobotMessage;

//communication status
enum Recstate : unsigned char
{
    WAITING_FF1,
    WAITING_FF2,
    SENDER_ID,
    RECEIVER_ID,
    RECEIVE_LEN_H,
    RECEIVE_LEN_L,
    RECEIVE_PACKAGE,
    RECEIVE_CHECK
};

class StateMachine
{
public:
    StateMachine(unsigned char my_id_ , unsigned char friend_id_  , unsigned char port_num_ , unsigned int baudrate)
    {
        my_id = my_id_;   //0x11 means slave ,  read Hands Free Link Manua.doc for detail
        friend_id = friend_id_;   // 0x01 means master
        port_num = port_num_;
#if ROBOLINK_NODE_MODEL==0
        Board::getInstance()->usartDeviceInit((DeviceType)port_num , baudrate);
#endif
    }

public:
    unsigned char port_num;

protected:
    unsigned char receiveStates(unsigned char rx_data);
    void sendMessage(const RobotMessage* tx_message_);
    RobotMessage rx_message ,  tx_message;
    unsigned char tx_buffer[MESSAGE_BUFER_SIZE];
    unsigned tx_buffer_length;
    unsigned char my_id , friend_id;

private:
    Recstate receive_state_;
    float receive_message_count  , send_message_count;
    unsigned int receive_check_sum_;
    short int receive_message_length_;
    short int byte_count_;
};

#endif // STATE_MACHINE_H
