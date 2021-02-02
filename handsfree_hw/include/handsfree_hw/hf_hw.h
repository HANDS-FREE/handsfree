/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: robolink.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: define the handfree pc software interface
***********************************************************************************************************************/

#ifndef HF_HW_H_
#define HF_HW_H_

#include <fstream>
#include <handsfree_hw/transport_serial.h>
#include <robolink.h>
#include <cstdlib>
#include <time.h>

namespace handsfree_hw {

class HF_HW{
public:
    HF_HW(std::string url, std::string config_addr);

    void restartTransportSerial(std::string url);

    bool updateCommand(const Command &command, int count);

    void updateRobot();

    inline RobotAbstract* getRobotAbstract()
    {
        return &my_robot_;
    }

    inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
    {
        return port_->getIOinstace();
    }
/*
    bool reconfig()
    {

    }
*/
    inline bool initialize_ok () const
    {
        return initialize_ok_;
    }

    inline void checkHandshake()
    {

        if (robolink_->getReceiveRenewFlag(SHAKING_HANDS)==1 || shaking_hands_flag_ == 0)
        {
            sendCommand(SHAKING_HANDS);

            time_t tt;
            time( &tt );
            tm* t= localtime( &tt );
            getRobotAbstract()->expect_system_info.local_time.valid=1;
            getRobotAbstract()->expect_system_info.local_time.year=t->tm_year + 1900;
            getRobotAbstract()->expect_system_info.local_time.month=t->tm_mon + 1;
            getRobotAbstract()->expect_system_info.local_time.date=t->tm_mday;
            getRobotAbstract()->expect_system_info.local_time.hour=t->tm_hour;
            getRobotAbstract()->expect_system_info.local_time.min=t->tm_min;
            getRobotAbstract()->expect_system_info.local_time.sec=t->tm_sec;
            sendCommand(SET_SYSTEM_INFO);
            getRobotAbstract()->expect_system_info.local_time.valid=0;
            shaking_hands_flag_ = 1;
            std::cerr<<"send shake hands local_time="<< t->tm_year + 1900 <<"-"<< t->tm_mon + 1<<"-"
                    << t->tm_mday<<" "<< t->tm_hour <<":"<< t->tm_min <<":"<< t->tm_sec<<std::endl;
        }
    }

    inline void hwSendCommand(const Command command_state)
    {
        sendCommand(command_state);
    }

    unsigned short int time_out_cnt_;
    unsigned short int shaking_hands_flag_;

private:
    boost::shared_ptr<Transport> port_;
    boost::shared_ptr<RoboLink> robolink_;
    boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for reading config file
    std::fstream file_;
    bool initialize_ok_;
    //for updating data
    int robolink_command_set_[LAST_COMMAND_FLAG];
    int robolink_freq_[LAST_COMMAND_FLAG];
    int robolink_count_[LAST_COMMAND_FLAG];
    int robolink_command_set_current_[LAST_COMMAND_FLAG];

    int time_out_;
    bool time_out_flag_;
    boost::mutex wait_mutex_;
    bool ack_ready_;
    void timeoutHandler(const boost::system::error_code &ec);

    inline uint8_t checkUpdate(const Command command_state)
    {
        if (robolink_command_set_current_[command_state] & robolink_->getReceiveRenewFlag(command_state))
        {
            return 1;
        }
        if (robolink_command_set_current_[command_state] == 0 ) return 1;
        return 0;
    }

    inline void sendCommand(const Command command_state)
    {
        std::cout<<"hf_hw: send message  "<<command_state <<std::endl;
        robolink_->masterSendCommand(command_state);
        Buffer data(robolink_->getSerializedData(), robolink_->getSerializedLength() + robolink_->getSerializedData());
        port_->writeBuffer(data);
    }

    // a single object for robot
    RobotAbstract my_robot_;
};

}


#endif /* HF_HW_H_ */
