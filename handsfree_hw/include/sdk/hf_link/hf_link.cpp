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

#include "hf_link.h"
#include "stdio.h"
#include <string.h>

unsigned char HFLink::byteAnalysisCall(const unsigned char rx_byte)
{
    if( receiveStates(rx_byte) )
    {
        //receive a new message
        unsigned char package_update = packageAnalysis();
        if(package_update == 1) analysis_package_count++;
        return package_update;
    }
    return 0;
}

/***********************************************************************************************************************
* Function:
*
* Scope:
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
unsigned char HFLink::packageAnalysis(void)
{

    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }

    command_state_ = (Command)rx_message.data[0];

    if (hf_link_node_model == 0)  //the slave need to check the SHAKING_HANDS"s state
    {
        if(shaking_hands_state==0 && command_state_ != SHAKING_HANDS) //if not  shaking hands
        {
            sendStruct(SHAKING_HANDS  , NULL , 0);
            return 1;
        }
    }

    unsigned char analysis_state =0;

    switch (command_state_)
    {
    case SHAKING_HANDS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case READ_SYSTEM_INFO :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->system_info , sizeof(robot->system_info));
        break;

    case SET_MOTOR_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->para.motor_para  , sizeof(robot->para.motor_para));
        break;

    case SAVE_MOTOR_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_CHASSIS_PARAMETERS :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->para.chassis_para , sizeof(robot->para.chassis_para));
        break;

    case SAVE_CHASSIS_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_HEAD_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->para.head_para , sizeof(robot->para.head_para));
        break;

    case SAVE_HEAD_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_ARM_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case SAVE_ARM_PARAMETERS :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_GLOBAL_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_global_speed , sizeof(robot->expect_global_speed));
        break;

    case READ_GLOBAL_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_speed , sizeof(robot->measure_global_speed));
        break;

    case SET_ROBOT_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_robot_speed , sizeof(robot->expect_robot_speed));
        break;

    case READ_ROBOT_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_robot_speed , sizeof(robot->measure_robot_speed));
        break;

    case SET_MOTOR_SPEED :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_motor_speed, sizeof(robot->expect_motor_speed));
        break;

    case READ_MOTOR_SPEED :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_motor_speed , sizeof(robot->measure_motor_speed));
        break;

    case READ_MOTOR_MILEAGE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_motor_mileage , sizeof(robot->measure_motor_mileage));
        break;

    case READ_GLOBAL_COORDINATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate));
        break;

    case READ_ROBOT_COORDINATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_robot_coordinate , sizeof(robot->measure_robot_coordinate));
        break;

    case CLEAR_COORDINATE_DATA :
        analysis_state=setCommandAnalysis(command_state_ , NULL  , 0);
        break;

    case SET_HEAD_STATE :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_head_state, sizeof(robot->expect_head_state));
        break;

    case READ_HEAD_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_head_state , sizeof(robot->measure_head_state));
        break;

    case SET_ARM_STATE :
        analysis_state=setCommandAnalysis(command_state_ , (unsigned char *)&robot->expect_arm_state, sizeof(robot->expect_arm_state));
        break;

    case READ_ARM_STATE :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->measure_arm_state , sizeof(robot->measure_arm_state));
        break;

    case READ_IMU_BASE_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->gyro_acc , sizeof(robot->gyro_acc));
        break;

    case READ_IMU_FUSION_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->magnetic_fusion , sizeof(robot->magnetic_fusion));
        break;

    case READ_GPS_DATA :
        analysis_state=readCommandAnalysis(command_state_ , (unsigned char *)&robot->gps_data, sizeof(robot->gps_data));
        break;

    default :
        analysis_state = 0;
        break;

    }

    rx_message.sender_id=0;    //clear flag
    rx_message.receiver_id=0;
    rx_message.length=0;
    rx_message.data[0]=0;

    return analysis_state;
}

/***********************************************************************************************************************
* Function:    void HFLink::masterSendCommand(Command command)
*
* Scope:       public
*
* Description: send a command or data to the friend_id
*              this function is olny belongs to master
*
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
unsigned char HFLink::masterSendCommand(const Command command_state)
{

    if(robot == NULL){
        printf(" error , the robot is NULL  \n");
        return 0;
    }

    if(hf_link_node_model == 0){  //slave
        printf("error , I'm not a master");
        return 0;
    }
    if(command_state >= LAST_COMMAND_FLAG)  return 0; //error

    receive_package_renew[(unsigned char)command_state] = 0 ;

    switch (command_state)
    {
    case SHAKING_HANDS :
        sendStruct(command_state , (unsigned char *)&robot->measure_global_coordinate , sizeof(robot->measure_global_coordinate) );
        break;

    case READ_SYSTEM_INFO :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_MOTOR_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.motor_para , sizeof(robot->para.motor_para) );
        break;

    case SAVE_MOTOR_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_CHASSIS_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.chassis_para , sizeof(robot->para.chassis_para) );
        break;

    case SAVE_CHASSIS_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_HEAD_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.head_para , sizeof(robot->para.head_para) );
        break;

    case SAVE_HEAD_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ARM_PARAMETERS :
        sendStruct(command_state , (unsigned char *)&robot->para.arm_para , sizeof(robot->para.arm_para) );
        break;

    case SAVE_ARM_PARAMETERS :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_GLOBAL_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_global_speed , sizeof(robot->expect_global_speed));
        break;

    case READ_GLOBAL_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ROBOT_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_robot_speed , sizeof(robot->expect_robot_speed));
        break;

    case READ_ROBOT_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_MOTOR_SPEED :
        sendStruct(command_state , (unsigned char *)&robot->expect_motor_speed, sizeof(robot->expect_motor_speed));
        break;

    case READ_MOTOR_SPEED :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_MOTOR_MILEAGE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_GLOBAL_COORDINATE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_ROBOT_COORDINATE :
        sendStruct(command_state , NULL , 0);
        break;

    case CLEAR_COORDINATE_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_HEAD_STATE :
        sendStruct(command_state , (unsigned char *)&robot->expect_head_state, sizeof(robot->expect_head_state));
        break;

    case READ_HEAD_STATE :
        sendStruct(command_state , NULL , 0);
        break;

    case SET_ARM_STATE :
        sendStruct(command_state , (unsigned char *)&robot->expect_arm_state , sizeof(robot->expect_arm_state ));
        break;

    case READ_ARM_STATE :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_IMU_BASE_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_IMU_FUSION_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    case READ_GPS_DATA :
        sendStruct(command_state , NULL , 0);
        break;

    default :
        return 0;
        break;
    }
    return 1;
}

/***********************************************************************************************************************
* Function:
*
* Scope:       public
*
* Description:
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
unsigned char HFLink::readCommandAnalysis(Command command_state , unsigned char* p ,  unsigned short int len)
{
    if (hf_link_node_model == 1)
    { // master   , means the slave feedback a package to master , and the master save this package
        if((rx_message.length-1) != len)
        {
            printf("I'm a master , can not read the message from slave , the length is not mathcing to struct \n");
            return 0;
        }
        memcpy(p , &rx_message.data[1] , len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    else if(hf_link_node_model == 0)
    { // slave   , means the master pub a read command to slave ,and the slave feedback the a specific info to him
        sendStruct(command_state  , p , len);
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    return 1;
}

unsigned char HFLink::setCommandAnalysis( Command command_state , unsigned char* p ,  unsigned short int len)
{

    if (hf_link_node_model == 1)
    { // master  , the slave can set the master's data ,so this code means received the slave's ack
        if(command_state == SHAKING_HANDS)
        {
            shaking_hands_state = 1;   //wait he master send SHAKING_HANDS
            printf("received a SHAKING_HANDS commmand and the slave is waiting master send SHAKING_HANDS data ");
        }
        else
        {
            printf("I'm master , received a ack ");
        }
        receive_package_renew[(unsigned char)command_state] = 1 ;
    }
    else if(hf_link_node_model == 0)
    { // slave  , means the master pub a set command to slave ,and the slave save this package then feed back a ack

        receive_package_renew[(unsigned char)command_state] = 1 ;   //update receive flag , and wait the cpu to deal
        if(len > 0)
        {
            if( (rx_message.length-1) != len)
            {
                printf("I'm a slave , can not read the message from master , the length is not mathcing \n");
            }
            else memcpy(p , &rx_message.data[1] , len);
        }

        if(command_state == SHAKING_HANDS) shaking_hands_state=1;   //SHAKING_HANDS not need ack to master
        else sendStruct(command_state  , NULL , 0);   // returns a ack to master , i receive your set package
    }
    return 1;
}

/***********************************************************************************************************************
* Function:    void HFLink::sendStruct(const Command command_state , unsigned char* p , const unsigned short int len)
*
* Scope:       private
*
* Description:
* len =0       send a Single command to the friend
*              if i am slave , it can be  feed back a ack to master or request instructions  like SHAKING_HANDS
*              if i am master , it can be some request instructions like READ_ROBOT_SYSTEM_INFO READ_xxx
*
*
* len>0 :      send a Struct command to the friend hf_link nodeif
*              if i am slave , then means feed back  a  struc(valid data) to master
*              if i am master , then means set a a  struc(valid data)to slave
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/
void HFLink::sendStruct(const Command command_type , unsigned char* p ,  unsigned short int len)
{
    tx_message.sender_id = my_id;
    tx_message.receiver_id = friend_id;
    tx_message.length=len+1;
    tx_message.data[0] = (unsigned char)command_type;
    if(len > 0)
    {
        memcpy(&tx_message.data[1] , p , len);
    }
    sendMessage(&tx_message);
}

