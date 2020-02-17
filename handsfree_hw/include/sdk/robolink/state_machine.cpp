/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* Contact:  QQ Exchange Group -- 521037187
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
*              please read HandsFree Robolink manua for detail
***********************************************************************************************************************/

#include <state_machine.h>

unsigned char StateMachine::receiveStates(const unsigned char rx_data)
{

    switch (receive_state_)
    {
    case WAITING_FF1:
        if (rx_data == 0xff)
        {
            receive_state_ = WAITING_FF2;
            receive_check_sum_ =0;
            receive_message_length_ = 0;
            byte_count_=0;
            receive_check_sum_ += rx_data;
        }
        break;

    case WAITING_FF2:
        if (rx_data == 0xff)
        {
            receive_state_ = SENDER_ID;
            receive_check_sum_ += rx_data;
        }
        else
            receive_state_ = WAITING_FF1;
        break;

    case SENDER_ID:
        rx_message.sender_id = rx_data ;
        if (rx_message.sender_id == friend_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVER_ID;
        }
        else
        {
#if ROBOLINK_NODE_MODEL == 1
            printf("master_warning : the sender_id is not my friend \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVER_ID:
        rx_message.receiver_id = rx_data ;
        if (rx_message.receiver_id == my_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVE_LEN_H;
        }
        else
        {
#if ROBOLINK_NODE_MODEL == 1
            printf("master_warning : the reciver_id is not my_id \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVE_LEN_H:
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data<<8;
        receive_state_ = RECEIVE_LEN_L;
        break;

    case RECEIVE_LEN_L:
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data;
        rx_message.length = receive_message_length_;
        receive_state_ = RECEIVE_PACKAGE;
        break;

    case RECEIVE_PACKAGE:
        receive_check_sum_ += rx_data;
        rx_message.data[byte_count_++] = rx_data;
        if(byte_count_ >= receive_message_length_)
        {
            receive_state_ = RECEIVE_CHECK;
            receive_check_sum_=receive_check_sum_ % 255;
        }
        break;

    case RECEIVE_CHECK:
        if(rx_data == (unsigned char)receive_check_sum_)
        {
            receive_check_sum_=0;
            receive_state_ = WAITING_FF1;
#if ROBOLINK_NODE_MODEL == 1
            printf("master_info : receive a message \n");
#endif
            receive_message_count ++ ;
            return 1 ;
        }
        else
        {
#if ROBOLINK_NODE_MODEL == 1
            printf("master_error : check sum error \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;
    default:
        receive_state_ = WAITING_FF1;
    }

    return 0;
}

/***********************************************************************************************************************
* Function:    void StateMachine::sendMessage(void)
*
* Scope:
*
* Description:  send a message to robolink node
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
void StateMachine::sendMessage(const RobotMessage* tx_message_)
{

    unsigned int check_sum_=0;

    tx_buffer[0]=0xff;
    check_sum_ += 0xff;

    tx_buffer[1]=0xff;
    check_sum_ += 0xff;

    tx_buffer[2]=tx_message_->sender_id;
    check_sum_ += tx_buffer[2];

    tx_buffer[3]=tx_message_->receiver_id;
    check_sum_ += tx_buffer[3];

    tx_buffer[4]=(unsigned char)( tx_message_->length >> 8);  //LEN_H
    check_sum_ += tx_buffer[4];

    tx_buffer[5]=(unsigned char)tx_message_->length;   //LEN_L
    check_sum_ += tx_buffer[5];

    unsigned short int tx_i  = 0;
    for(tx_i = 0 ;  tx_i < tx_message_->length ; tx_i++)   //package
    {
        tx_buffer[ 6 + tx_i] = tx_message_->data[ tx_i ];
        check_sum_ += tx_buffer[6+tx_i];
    }

    check_sum_=check_sum_%255;
    tx_buffer[6+tx_i] = check_sum_;

    tx_buffer_length = 6 + tx_message_->length + 1;

#if ROBOLINK_NODE_MODEL==0
    RoboLinkSendBuffer(port_num , tx_buffer , tx_buffer_length);
#endif

    send_message_count++;
}
