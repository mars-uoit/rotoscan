/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2010, 2011, DFKI GmbH
 *  Copyright (c) 2011, Felix Kolbe
 *  Copyright (c) 2012, Universität Bremen
 *  All rights reserved.
 *
 *  Authors: Rene Wagner <rene.wagner@dfki.de>
 *           Felix Kolbe <felix.kolbe@informatik.haw-hamburg.de>
 *           Rene Wagner <rwagner@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the DFKI GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ROTOSCAN_TTY_CLIENT_HPP__
#define __ROTOSCAN_TTY_CLIENT_HPP__

#include <string>
using namespace std;

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include "logging_defaults.hpp"
#include "parser.hpp"

#include <rotoscan/base_client.hpp>

namespace rotoscan {

class tty_client : public base_client<boost::asio::serial_port>
{
	typedef base_client<boost::asio::serial_port> base;
	typedef tty_client self;

public:
	tty_client(boost::asio::io_service& io_service,
			   const std::string &device, const int &baudrate)
		: base(io_service)
  	{
		using boost::asio::serial_port;
		
		// Try to connect
		boost::system::error_code error = boost::asio::error::operation_aborted;

		ROTOSCAN_LOG_INFO("trying to connect to: " << device);

		socket_.open(device, error);
		socket_.set_option(serial_port::baud_rate(baudrate));
		socket_.set_option(serial_port::flow_control(serial_port::flow_control::none));
		socket_.set_option(serial_port::parity(serial_port::parity::none));
		socket_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
		socket_.set_option(serial_port::character_size(8));

//		// hint: how to read an option
//		serial_port::baud_rate br;
//		socket_.get_option(br);
//		cout << "baudrate was: " << br.value() << endl;
//		br.~baud_rate();	// needed before succeeding read

		if (error)
			throw boost::system::system_error(error);

//		if (!socket_.is_open()) { 		probably not needed
//			ROTOSCAN_LOG_FATAL("could not connect to "+port);
//		}
		ROTOSCAN_LOG_INFO("connection established");
		connected_ = true;

		start();
	}
};

} // namespace rotoscan

#endif // __ROTOSCAN_TCP_CLIENT_HPP__
