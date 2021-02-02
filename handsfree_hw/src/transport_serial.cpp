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
* Description: define handfree transport serial method
***********************************************************************************************************************/
#include <handsfree_hw/transport_serial.h>

namespace handsfree_hw {

TransportSerial::TransportSerial() :
    Transport("serial:///dev/ttyUSB0")
{
    params_.serialPort = "/dev/ttyUSB0";
    initializeSerial();
}

TransportSerial::TransportSerial(std::string url) :
    Transport(url)
{
    if (comm_url_.substr(0, comm_url_.find("://")) != "serial")
    {
        std::cerr << "url error, please correct your config" <<std::endl;
        return ;
    }
    params_.serialPort = comm_url_.substr(comm_url_.find("://")+ 3, comm_url_.length() - comm_url_.find("://"));
    if (!initializeSerial())
    {
        std::cerr << "serial Transport initialize failed ,please check your system" <<std::endl;
    } else
    {
        std::cout << "transport initialize ready" <<std::endl;
    }
}

void TransportSerial::mainRun()
{
    std::cout << "Transport main read/write started" <<std::endl;
    start_a_read();
    ios_->run();
}

void TransportSerial::start_a_read()
{
    boost::mutex::scoped_lock lock(port_mutex_);

    port_->async_read_some(boost::asio::buffer(temp_read_buf_),
                           boost::bind(&TransportSerial::readHandler,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred
                                       ));
}

void TransportSerial::readHandler(const boost::system::error_code &ec, size_t bytesTransferred)
{
    if (ec)
    {
        std::cerr << "Transport Serial read Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(read_mutex_);
    Buffer data(temp_read_buf_.begin(), temp_read_buf_.begin() + bytesTransferred);
    read_buffer_.push(data);
    start_a_read();
}

void TransportSerial::start_a_write()
{
    boost::mutex::scoped_lock lock(port_mutex_);

    if (!write_buffer_.empty())
    {
        boost::asio::async_write(*port_, boost::asio::buffer((write_buffer_.front())),
                                 boost::bind(&TransportSerial::writeHandler, this, boost::asio::placeholders::error));
        write_buffer_.pop();
    }


}

void TransportSerial::writeHandler(const boost::system::error_code &ec)
{
    if (ec)
    {
        std::cerr << "Transport Serial write Error "<< std::endl;
        return;
    }

    boost::mutex::scoped_lock lock(write_mutex_);

    if (!write_buffer_.empty())	start_a_write();
}

Buffer TransportSerial::readBuffer()
{
    boost::mutex::scoped_lock lock(read_mutex_);

    if (!read_buffer_.empty())
    {
        Buffer data(read_buffer_.front());
        read_buffer_.pop();
        return data;
    }
    Buffer data;
    return data;
}

void TransportSerial::writeBuffer(Buffer &data)
{
    boost::mutex::scoped_lock lock(write_mutex_);

    write_buffer_.push(data);
    start_a_write();
}

bool TransportSerial::initializeSerial()
{
    try
    {
        std::cout<<params_.serialPort <<std::endl;
        port_ = boost::make_shared<boost::asio::serial_port>(*ios_, params_.serialPort);
        //port_ = boost::make_shared<boost::asio::serial_port>(boost::ref(*ios_), params_.serialPort);
        port_->set_option(
                    boost::asio::serial_port::baud_rate(params_.baudRate));
        port_->set_option(
                    boost::asio::serial_port::flow_control((boost::asio::serial_port::flow_control::type)params_.flowControl));
        port_->set_option(
                    boost::asio::serial_port::parity((boost::asio::serial_port::parity::type)params_.parity));
        port_->set_option(
                    boost::asio::serial_port::stop_bits((boost::asio::serial_port::stop_bits::type)params_.stopBits));
        port_->set_option(boost::asio::serial_port::character_size(8));

    }
    catch(std::exception &e)
    {
        std::cerr << "Failed to open the serial port " << std::endl;
        std::cerr << "Error info is "<< e.what() << std::endl;
        initialize_ok_ = false;
        return false;
    }

    std::cerr << "transport initialize ready" <<std::endl;
    initialize_ok_ = true;

    temp_read_buf_.resize(1024, 0);
    try
    {
        thread_ = boost::thread(boost::bind(&TransportSerial::mainRun, this));
    }
    catch(std::exception &e)
    {
        std::cerr << "Transport Serial thread create failed " << std::endl;
        std::cerr << "Error Info: " << e.what() <<std::endl;
        return false;
    }

    return true;
}

}
