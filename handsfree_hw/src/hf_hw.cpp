/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: 
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

#include <handsfree_hw/hf_hw.h>

namespace handsfree_hw {

HF_HW::HF_HW(std::string url, std::string config_addr)
{
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ = 500;
        robolink_ = boost::make_shared<RoboLink>(&my_robot_ , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
                                                     boost::posix_time::milliseconds(time_out_)));
    }else if (transport_method == "udp")
    {
    }else if (transport_method == "tcp")
    {
    }

    //process the config file
    file_.open(config_addr.c_str(), std::fstream::in);
    if (file_.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG; i++)
        {
            std::string temp;
            file_ >> temp >> robolink_command_set_[i] >> robolink_freq_[i];
            std::cout<< temp << robolink_command_set_[i] << robolink_freq_[i]<<std::endl;
        }
        file_.close();
        std::cerr << "config file opened succeed" <<std::endl;
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        initialize_ok_ = false;
    }

    time_out_cnt_ = 0;
    shaking_hands_flag_ = 0;
}

void HF_HW::restartTransportSerial(std::string url)
{
    port_ = boost::make_shared<TransportSerial>(url);
    usleep(10000);
}

void HF_HW::timeoutHandler(const boost::system::error_code &ec)
{
    if (!ec)
    {
        std::cerr << "Time Out" <<std::endl;
        boost::mutex::scoped_lock lock(wait_mutex_);
        time_out_flag_ = true;
    }
}

// for QT client , not for ros update
void HF_HW::updateRobot()
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));

    memset(robolink_count_, 0 ,sizeof robolink_count_);
    int count = 0;
    while (true)
    {
        cicle_timer_.expires_from_now(boost::posix_time::millisec(10)); // 100 hz
        //std::cout<< "start a write" <<std::endl;
        //  check hand shake
        checkHandshake();
        //  uodate normal data
        memset(robolink_command_set_current_, 0, sizeof robolink_command_set_current_);
        for (int i = 1; i < LAST_COMMAND_FLAG; i++)
        {
            if (robolink_command_set_[i] != 0)
            {
                int cnt = count % 100;
                if (cnt %  (100 / robolink_freq_[i]) == 0)
                {
                    sendCommand((Command)i);
                    robolink_command_set_current_[i] = 1;
                }
            }
        }

        //std::cout<< "start a read" <<std::endl;
        Buffer data = port_->readBuffer();
        ack_ready_ = false;
        while (!ack_ready_)
        {
            for (int i = 0; i < data.size(); i++)
            {
                //std::cout<<"get byte   :"<< data[i]<< std::endl;
                if (robolink_->byteAnalysisCall(data[i]))
                {
                    // all receive package ack arrived
                    uint8_t temp = 1;
                    for (int i = 1; i < LAST_COMMAND_FLAG; i++)
                        temp = temp & checkUpdate((Command)i);
                    std::cout<< temp <<std::endl;
                    if (temp)
                    {
                        ack_ready_ = true;
                        std::cout<< "all package received" <<std::endl;
                    }
                }
            }
            if (cicle_timer_.expires_from_now().is_negative())
            {
                //std::cout<<"Timeout continue next circle"<<std::endl;
                break;
            }
            data = port_->readBuffer();
        }
        count++;
        cicle_timer_.wait();
    }
}

bool HF_HW::updateCommand(const Command &command, int count)
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
    // update command set  data from embedded system
    if (robolink_command_set_[command] != 0)
    {
        int cnt = count % 100;
        if (cnt %  (100 / robolink_freq_[command]) == 0)
        {
            sendCommand(command);
        } else
        {
            // skip this package
            return false;
        }
    }
    else return false;

    Buffer data = port_->readBuffer();
    ack_ready_ = false;
    while (!ack_ready_)
    {
        for (int i = 0; i < data.size(); i++)
        {
            if (robolink_->byteAnalysisCall(data[i]))
            {
                // one package ack arrived
                ack_ready_ = true;
                time_out_cnt_ = 0;
            }
        }
        data = port_->readBuffer();
        if (cicle_timer_.expires_from_now().is_negative())
        {
            time_out_cnt_++;
            std::cerr<<"Timeout continue skip this package time_out_cnt_= " << time_out_cnt_ <<std::endl;
            return false;
        }
        usleep(50);
        //sleep 10us  robolink max speed: 800 package/S (stm32f103rct6) , x86_timeout_cpu: 22%
        //sleep 50us  robolink max speed: 700 package/S (stm32f103rct6) , x86_timeout_cpu: 18%
        //sleep 100us robolink max speed: 300 package/S (stm32f103rct6) , x86_timeout_cpu: 15%
        //sleep 500us robolink max speed: 120 package/S (stm32f103rct6) , x86_timeout_cpu: 14%
    }
    return true;
}
}


