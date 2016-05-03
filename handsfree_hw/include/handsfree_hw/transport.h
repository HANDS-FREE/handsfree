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
* luke liao       2016.3.29   V1.0           creat this file
*
* Description: define handfree transport base class
***********************************************************************************************************************/



#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

namespace handsfree_hw {

typedef std::vector<uint8_t> Buffer;

class Transport {
public:
	Transport(std::string url) :
		comm_url_(url),
		write_buffer_(),
		read_buffer_()
	{
		ios_ = boost::make_shared<boost::asio::io_service>();
	}

	virtual Buffer readBuffer() = 0;

	virtual void writeBuffer(Buffer &data) = 0;

	inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
	{
		return ios_;
	}

	bool initialize_ok()
	{
		return initialize_ok_;
	}

protected:
	// for communication location
	std::string comm_url_;
	std::queue<Buffer> write_buffer_;
	std::queue<Buffer> read_buffer_;

	bool initialize_ok_;

	// for boost asio service
	boost::shared_ptr<boost::asio::io_service> ios_;
};

}



#endif /* TRANSPORT_BASE_H_ */
