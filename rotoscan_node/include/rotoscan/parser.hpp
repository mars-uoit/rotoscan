/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2010, DFKI GmbH
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

#ifndef __ROTOSCAN_PARSER_HPP__
#define __ROTOSCAN_PARSER_HPP__

#include <string>

#include <boost/bind.hpp>
#include <boost/signal.hpp>

#include <timeutils/time.hpp>

#include "scan.hpp"
#include "logging_defaults.hpp"

#define ROTOSCAN_AS_HEX(x) "0x" << std::hex << (x) << std::dec
#define ROTOSCAN_CHAR_AS_HEX(x) ROTOSCAN_AS_HEX(int((x)) & 0xff)

namespace rotoscan {

class parser
{
	typedef parser self;
	
	typedef enum
	{
		START,
		OPERATION,
		OPTION_1,
		OPTION_2,
		OPTION_3,
		SCAN_NUMBER_1,
		SCAN_NUMBER_2,
		SCAN_NUMBER_3,
		SCAN_NUMBER_4,
		SCAN_NUMBER_5,
		SCAN_NUMBER_6,
		SCAN_NUMBER_7,
		SCAN_NUMBER_8,
		ANGULAR_STEP_SIZE,
		START_ANGLE_1,
		START_ANGLE_2,
		STOP_ANGLE_1,
		STOP_ANGLE_2,
		DISTANCE_MEASUREMENT_1,
		DISTANCE_MEASUREMENT_2,
		CHECKSUM,
		END
	} expected_byte_t;

	enum
	{
		FRAME_MARKER = 0x0
	};

public:
	parser()
	{
		reset();
	}

	template <typename ScanHandler>
	void register_scan_handler(ScanHandler h)
	{
		deliver_scan_.connect(h);
	}

	void reset()
	{
		expected_ = START;
		frame_markers_ = 0;
	}

	void parse(const uint8_t &byte)
	{
		handle_char(byte);
	}

private:
	void handle_scan_number_fill_byte(uint8_t c, expected_byte_t next)
	{
		if (c != (scan::SCAN_NUMBER_FILL_BYTE & 0xff))
		{
			ROTOSCAN_LOG_ERROR("expected fill byte but got: " << ROTOSCAN_CHAR_AS_HEX(c));
			expected_ = START;
		}
		else
			expected_ = next;
	}

	void handle_intra_frame(uint8_t c)
	{
		switch(expected_)
		{
		case OPERATION:
			{
				// reset stuff
				checksum_ = c;
				scan_.measurements.clear();

				scan_.operation = c;
				expected_ = OPTION_1;
				ROTOSCAN_LOG_TRACE("operation: " << ROTOSCAN_CHAR_AS_HEX(scan_.operation));
			}
			break;
		case OPTION_1:
			{
				scan_.option_1 = c;
				ROTOSCAN_LOG_TRACE("option 1");
				uint8_t other_options = c & 0x3;
				switch(other_options)
				{
				case scan::OPTION_1_ONLY:
					scan_.option_2 = 0;
					scan_.option_3 = 0;
					expected_ = SCAN_NUMBER_1;
					ROTOSCAN_LOG_TRACE("no other options");
					break;
				case scan::OPTION_1_AND_2:
				case scan::OPTION_1_2_3:
					expected_ = OPTION_2;
					break;
				default:
					ROTOSCAN_LOG_ERROR("invalid value for other options: " << ROTOSCAN_AS_HEX(int(other_options) & 0x3));
					expected_ = START;
					break;
				}
			}
			break;
		case OPTION_2:			
			{
				ROTOSCAN_LOG_TRACE("option 2");
				scan_.option_2 = c;
				if (c & scan::OPTION_2_OPTION_3_ENABLED)
				{
					expected_ = OPTION_3;
				}
				else
				{
					scan_.option_3 = 0;
					expected_ = SCAN_NUMBER_1;
					ROTOSCAN_LOG_TRACE("no other options");
				}
			}
			break;
		case OPTION_3:
			{
				ROTOSCAN_LOG_TRACE("option 3");
				scan_.option_3 = c;
				expected_ = SCAN_NUMBER_1;
			}
			break;
		case SCAN_NUMBER_1:
			{
				ROTOSCAN_LOG_TRACE("scan number 1");
				scan_.scan_number = c << 24;
				expected_ = SCAN_NUMBER_2;
			}
			break;
		case SCAN_NUMBER_2:
			{
				ROTOSCAN_LOG_TRACE("scan number 2");
				handle_scan_number_fill_byte(c, SCAN_NUMBER_3);
			}
			break;
		case SCAN_NUMBER_3:
			{
				ROTOSCAN_LOG_TRACE("scan number 3");
				scan_.scan_number |= c << 16;
				expected_ = SCAN_NUMBER_4;
			}
			break;
		case SCAN_NUMBER_4:
			{
				ROTOSCAN_LOG_TRACE("scan number 4");
				handle_scan_number_fill_byte(c, SCAN_NUMBER_5);
			}
			break;
		case SCAN_NUMBER_5:
			{
				ROTOSCAN_LOG_TRACE("scan number 5");
				scan_.scan_number |= c << 8;
				expected_ = SCAN_NUMBER_6;
			}
			break;
		case SCAN_NUMBER_6:
			{
				ROTOSCAN_LOG_TRACE("scan number 6");
				handle_scan_number_fill_byte(c, SCAN_NUMBER_7);
			}
			break;
		case SCAN_NUMBER_7:
			{
				ROTOSCAN_LOG_TRACE("scan number 7");
				scan_.scan_number |= c;
				expected_ = SCAN_NUMBER_8;
			}
			break;
		case SCAN_NUMBER_8:
			{
				ROTOSCAN_LOG_TRACE("scan number 8");
				handle_scan_number_fill_byte(c, ANGULAR_STEP_SIZE);
			}
			break;
		case ANGULAR_STEP_SIZE:
			{
				ROTOSCAN_LOG_TRACE("angular step size");
				scan_.angular_step_size = c;
				expected_ = START_ANGLE_1;
			}
			break;
		case START_ANGLE_1:
			{
				ROTOSCAN_LOG_TRACE("start angle 1");
				scan_.start_angle = c << 8;
				expected_ = START_ANGLE_2;
			}
			break;
		case START_ANGLE_2:
			{
				ROTOSCAN_LOG_TRACE("start angle 2");
				scan_.start_angle |= c;
				expected_ = STOP_ANGLE_1;
			}
			break;
		case STOP_ANGLE_1:
			{
				ROTOSCAN_LOG_TRACE("stop angle 1");
				scan_.stop_angle = c << 8;
				expected_ = STOP_ANGLE_2;
			}
			break;
		case STOP_ANGLE_2:
			{
				ROTOSCAN_LOG_TRACE("stop angle 2");
				scan_.stop_angle |= c;
				read_measurements_ = (scan_.stop_angle - scan_.start_angle) / (uint16_t) scan_.angular_step_size + 1;

				if (scan_.stop_angle < scan_.start_angle)
				{
					ROTOSCAN_LOG_ERROR("stop angle < start angle");
					expected_ = START;
				}
				else if (read_measurements_ > scan::MAX_MEASUREMENTS)
				{
					ROTOSCAN_LOG_ERROR("(stop angle - start angle) too large: " << read_measurements_);
					expected_ = START;
				}				
				else if (read_measurements_)
					expected_ = DISTANCE_MEASUREMENT_1;
				else
					expected_ = CHECKSUM;
			}
			break;
		case DISTANCE_MEASUREMENT_1:
			{
				ROTOSCAN_LOG_TRACE("distance measurement 1");
				current_measurement_ = (uint16_t(c) & 0xff) << 8;
				expected_ = DISTANCE_MEASUREMENT_2;
			}
			break;
		case DISTANCE_MEASUREMENT_2:
			{
				ROTOSCAN_LOG_TRACE("distance measurement 2");
				current_measurement_ |= uint16_t(c) & 0xfe;
				scan_.measurements.push_back(current_measurement_);
				if(--read_measurements_)
					expected_ = DISTANCE_MEASUREMENT_1;
				else
					expected_ = CHECKSUM;
			}
			break;
		default:
			throw "shouldn't have got here";
			break;
		}
	}

	void handle_char(uint8_t c)
	{
		if (frame_markers_ > 1)
		{
			switch(c)
			{
			case FRAME_MARKER:
				// end mark
				ROTOSCAN_LOG_TRACE("end of frame");
				expected_ = START;
				break;
			case 0xff:
				// escape like character
				ROTOSCAN_LOG_TRACE("dropping escape");
				checksum_ ^= c;
				break;
			default:
				// start
				timeutils::get_current_time(&scan_.time_received);
				ROTOSCAN_LOG_TRACE("frame start");
				expected_ = OPERATION;
				handle_intra_frame(c);
				break;
			}
			frame_markers_ = 0;
			return;
		}
		
		if (c == FRAME_MARKER)
			++frame_markers_;
		else
			frame_markers_ = 0;
		
		switch(expected_)
		{
		case START:
		case END:
			break;
		case CHECKSUM:
			{
				if (checksum_ == 0x0)
					// 0x0 is replaced with 0xff
					checksum_ = 0xff;
				if (checksum_ != uint8_t(c))
					// not fatal for now. protocol doesn't make sense here:
					// 0xff is both a valid regular value
					// and a replacement for 0x00
					ROTOSCAN_LOG_ERROR("checksum mismatch. calculated " << ROTOSCAN_CHAR_AS_HEX(checksum_) << " but got " << ROTOSCAN_CHAR_AS_HEX(c));

				deliver_scan_(scan_);
				
				expected_ = END;
			}
			break;
		default:
			{
				checksum_ ^= c;		
				handle_intra_frame(c);
			}
			break;
		}
	}
	
	expected_byte_t expected_;
	size_t frame_markers_;

	uint8_t checksum_;
	scan::size_type read_measurements_;
	uint16_t current_measurement_;
	scan scan_;

	boost::signal<void (const scan&)> deliver_scan_;	
};

} // namespace rotoscan

#endif // __ROTOSCAN_PARSER_HPP__
