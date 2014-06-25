/*
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

#include <sstream>

#include <ros/ros.h>
#include <ros/poll_manager.h>
#include <ros/poll_set.h>
#include <sensor_msgs/LaserScan.h>

#include "rotoscan_ros_logging.hpp"
#include <rotoscan/base_client.hpp>
#include <rotoscan/tcp_client.hpp>
#include <rotoscan/tty_client.hpp>

class rotoscan_node
{
	typedef rotoscan_node self;
	
public:
	template<typename RotoscanClient>
	rotoscan_node(ros::NodeHandle &nh,
				  ros::NodeHandle &nh_ns,
				  boost::asio::io_service& io_service,
				  RotoscanClient &client)
		: io_service_(io_service),
		  scan_pub_(nh.advertise<sensor_msgs::LaserScan>("scan", 1))
	{
		nh_ns.param<std::string>("frame_id", frame_id_, "laser");
		nh_ns.param("range_max", range_max_, 40.);

		int fd = client.socket_fd();
		ros::PollSet &poll_set = ros::PollManager::instance()->getPollSet();
		poll_set.addSocket(fd, boost::bind(&self::handle_socket_io, this));
		poll_set.addEvents(fd, POLLIN|POLLPRI|POLLERR|POLLHUP);

		client.register_io_error_handler(boost::bind(&self::handle_asio_error, this, _1));
		client.register_scan_handler(boost::bind(&self::handle_scan, this, _1));
	}

private:
	void handle_socket_io()
	{
		boost::system::error_code ec;
		io_service_.poll(ec);
		if (ec)
		{
			ROS_FATAL_STREAM("poll error: " << ec.message());
			ros::shutdown();
		}
	}

	void handle_asio_error(const boost::system::error_code &error)
	{
		ros::shutdown();
	}
	
	void handle_scan(const rotoscan::scan &scan)
	{
		if (scan.error())
		{
			ROS_ERROR("Got scan with error flag set");
		}
		else
		{
			if (scan.warning())
				ROS_WARN("Got scan with warning flag set");

			// min/max/inc values of scanner hardware
			const double hw_angle_min = 95.04 * M_PI/180.;
			const double hw_angle_max = hw_angle_min - 190.0 * M_PI/180.;
			const double hw_angle_inc = -(hw_angle_min - hw_angle_max) / double(rotoscan::scan::MAX_MEASUREMENTS-1);

			sensor_msgs::LaserScan scan_msg;
			scan_msg.header.stamp.fromNSec(1000*scan.time_received); // scan.time_received is in usec
			scan_msg.header.frame_id = frame_id_;

			// adjust parameters according to current firmware configuration
			//ROS_DEBUG("start %u, stop %u, step_size %u", scan.start_angle, scan.stop_angle, scan.angular_step_size);
			scan_msg.angle_min = hw_angle_min + (scan.start_angle - 1) * hw_angle_inc;
			scan_msg.angle_max = hw_angle_min + (scan.stop_angle - 1) * hw_angle_inc;
			scan_msg.angle_increment = hw_angle_inc * scan.angular_step_size;
			scan_msg.scan_time = 0.0399; // 25.06.. Hz (because of 40 ms per scan)
			scan_msg.time_increment = scan_msg.scan_time / double(rotoscan::scan::MAX_MEASUREMENTS-1) * scan.angular_step_size;

			scan_msg.range_min = 0.08; // smaller values would be inside or on the sensor case
			scan_msg.range_max = range_max_;
		
			//ROS_DEBUG("min: %f max: %f inc: %f", scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment);

			scan_msg.ranges.reserve(scan.measurements.size());
			for (rotoscan::scan::measurement_iterator i = scan.measurements.begin();
				 i != scan.measurements.end(); ++i)
			{
				scan_msg.ranges.push_back(*i / 1000.);
			}
			scan_pub_.publish(scan_msg);
		}
	}
	
	boost::asio::io_service& io_service_;
	std::string frame_id_;
	double range_max_;
	ros::Publisher scan_pub_;
};


template<typename RotoscanClient>
static
int spin(ros::NodeHandle &nh,
		 ros::NodeHandle &nh_ns,
		 boost::asio::io_service &io_service,
		 RotoscanClient &client)
{
	rotoscan_node node(nh, nh_ns, io_service, client);
	
	ros::spin();
	
	return 0;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rotoscan_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	
	std::string connection;
	if(!nh_ns.getParam("connection", connection))
	{
		ROS_FATAL("need connection parameter");
		return 1;
	}

	std::string frame_id;

	try
	{
		boost::asio::io_service io_service;
		
		if (connection == "tcp")
		{
			std::string host, port;
			nh_ns.param("host", host, std::string("192.168.1.1"));
			nh_ns.param("port", port, std::string("9008"));

			rotoscan::tcp_client c(io_service, host, port);

			return spin(nh, nh_ns, io_service, c);
		}
		else if (connection == "tty")
		{
			std::string device;
			int baudrate;
			nh_ns.param("device", device, std::string("/dev/ttyUSB0"));
			nh_ns.param("baudrate", baudrate, 57600);

			rotoscan::tty_client c(io_service, device, baudrate);

			return spin(nh, nh_ns, io_service, c);
		}
		else
		{
			ROS_FATAL_STREAM("unknown connection type: " << connection);
			return 1;
		}
	}
	catch (std::exception &e)
	{
		ROS_FATAL_STREAM("exception: " << e.what());
	}
	catch (const char *e)
	{
		ROS_FATAL_STREAM("exception: " << e);
	}
	
	return 1;
}
