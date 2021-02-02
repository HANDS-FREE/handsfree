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
* Description: define handfree transport serial method
***********************************************************************************************************************/


#ifndef TRANSPORT_SERIAL_H_
#define TRANSPORT_SERIAL_H_

#include <handsfree_hw/transport.h>

namespace handsfree_hw {

class SerialParams {
public:
    std::string serialPort;
    unsigned int baudRate;
    unsigned int flowControl;
    unsigned int parity;
    unsigned int stopBits;
    SerialParams() :
        serialPort(), baudRate(921600), flowControl(0), parity(0), stopBits(0)
    {
    }
    SerialParams(
            std::string _serialPort,
            unsigned int _baudRate,
            unsigned int _flowControl,
            unsigned int _parity,
            unsigned int _stopBits
            ) :
        serialPort(_serialPort),
        baudRate(_baudRate),
        flowControl(_flowControl),
        parity(_parity),
        stopBits(_stopBits)
    {
    }
};

class TransportSerial : public Transport {

public:
    TransportSerial ();
    TransportSerial (std::string url);
    ~TransportSerial ()
    {
        if(thread_.joinable())
        {
            ios_->stop();
            thread_.join();
        }
    }

    virtual Buffer readBuffer();

    virtual void writeBuffer(Buffer &data);

private:
    boost::shared_ptr<boost::asio::serial_port> port_;
    SerialParams params_;
    // for async read
    Buffer temp_read_buf_;

    boost::thread thread_;
    // locks
    boost::mutex port_mutex_;
    boost::mutex write_mutex_;
    boost::mutex read_mutex_;

    bool initializeSerial();
    void mainRun();

    void start_a_read();
    void start_a_write();
    void readHandler(const boost::system::error_code &ec, size_t bytesTransferred);
    void writeHandler(const boost::system::error_code &ec);
};

}



#endif /* TRANSPORT_SERIAL_H_ */
