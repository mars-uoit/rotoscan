/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2010, 2011, DFKI GmbH
 *  Copyright (c) 2012, Universität Bremen
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rene.wagner@dfki.de>
 *          Rene Wagner <rwagner@informatik.uni-bremen.de>
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

#ifndef __ROTOSCAN_TCP_CLIENT_HPP__
#define __ROTOSCAN_TCP_CLIENT_HPP__

#include <string>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include <timeutils/time.hpp>

#include "logging_defaults.hpp"
#include "parser.hpp"

#include <rotoscan/base_client.hpp>

namespace rotoscan {

class tcp_client : public base_client<boost::asio::ip::tcp::socket>
{
	typedef base_client<boost::asio::ip::tcp::socket> base;
	typedef tcp_client self;

public:
	tcp_client(boost::asio::io_service& io_service,
			   const std::string &server, const std::string &port)
		: base(io_service)
  	{
		using boost::asio::ip::tcp;

		// Get a list of endpoints corresponding to the server name.
		tcp::resolver resolver(io_service);
		tcp::resolver::query query(server, port);
		tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
		tcp::resolver::iterator end;

		// Try each endpoint until we successfully establish a connection.
		boost::system::error_code error = boost::asio::error::host_not_found;
		while (error && endpoint_iterator != end)
		{
			socket_.close();
			socket_.connect(*endpoint_iterator++, error);
		}
		if (error)
			throw boost::system::system_error(error);

		ROTOSCAN_LOG_INFO("connection established");
		connected_ = true;

		start();
	}
};

} // namespace rotoscan

#endif // __ROTOSCAN_TCP_CLIENT_HPP__
