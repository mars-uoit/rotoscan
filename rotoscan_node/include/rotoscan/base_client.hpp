/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2010, DFKI GmbH
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

#ifndef __BASE_CLIENT_HPP__
#define __BASE_CLIENT_HPP__

#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include <timeutils/time.hpp>

#include "logging_defaults.hpp"
#include "parser.hpp"

namespace rotoscan {

template<typename SocketType>
class base_client
{
	typedef base_client self;
	typedef SocketType socket_type;
	typedef boost::function<void (const boost::system::error_code &)> error_handler_function;

public:
	base_client(boost::asio::io_service& io_service)
		: io_service_(io_service),
		  socket_(io_service),
		  connected_(false)
  	{
	}

	template<typename ErrorHandler>
	void register_io_error_handler(ErrorHandler h)
	{
		error_handler_ = error_handler_function(h);
	}
	
	template <typename ScanHandler>
	void register_scan_handler(ScanHandler h)
	{
		parser_.register_scan_handler<ScanHandler>(h);
	}

	bool connected() const
	{
		return connected_;
	}

	void close()
	{
		io_service_.post(boost::bind(&self::do_close, this));
	}
	
	int socket_fd()
	{
		return socket_.native();
	}

protected:
	void start()
	{
		parser_.reset();
		socket_.async_read_some(boost::asio::buffer(raw_buf_),
								boost::bind(&self::handle_inbound,
											this,
											boost::asio::placeholders::error,
											boost::asio::placeholders::bytes_transferred));

	}

private:
	void printHex(const uint8_t *data, int len) {
		while (len--)
			fprintf(stdout, "0x%x ", *(data++));
		fprintf(stdout, "\n");
	}

	void handle_inbound(const boost::system::error_code &error, std::size_t bytes_read) {
		if (!error) {
			ROTOSCAN_LOG_TRACE("got data " << bytes_read);
			//printHex((const uint8_t *) raw_buf_.data(), bytes_read);
			for (const char *i = raw_buf_.data(); bytes_read; ++i, --bytes_read) {
				parser_.parse((uint8_t)*i);
			}
			socket_.async_read_some(boost::asio::buffer(raw_buf_),
									boost::bind(&self::handle_inbound,
												this,
												boost::asio::placeholders::error,
												boost::asio::placeholders::bytes_transferred));
		} else {
			close(error);
		}
	}
	
	void close(const boost::system::error_code &error)
	{
		ROTOSCAN_LOG_FATAL("IO error: " << error.message());
		do_close();
		
		if (!error_handler_.empty())
			error_handler_(error);
	}
	
	void do_close()
	{
		socket_.close();
		connected_ = false;
	}

protected:
	boost::asio::io_service& io_service_;
    socket_type socket_;

	bool connected_;

private:
	error_handler_function error_handler_;

	boost::array<char, 512> raw_buf_;

	parser parser_;
};

} // namespace rotoscan

#endif // __BASE_CLIENT_HPP__
