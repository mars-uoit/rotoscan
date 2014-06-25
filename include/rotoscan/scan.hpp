/*
 *  Copyright (c) 2009, Rene Wagner
 *  Copyright (c) 2012, Universität Bremen
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rw@nelianur.org>
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
 *   * Neither the name of Rene Wagner nor the names of any
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

#ifndef __ROTOSCAN_SCAN_HPP__
#define __ROTOSCAN_SCAN_HPP__

#include <stdint.h>

#include <vector>

namespace rotoscan {
	
/**
 * Represents a binary packet containing a single range scan from the sensor.
 *
 * Based on Leuze electronic manual
 *   "RODplussoft - Software and protocol description
 *    for rotoScan ROD 4plus / ROD 4-08plus
 *    D/GB 229/0 - 03/07"
 *
 * Some corrections have been applied. Where possible, all constants
 * and field names use the same nomenclature as in the manual.
 *
 */
struct scan
{
	typedef std::vector<uint16_t> measurement_vector;
	typedef measurement_vector::const_iterator measurement_iterator;
	typedef measurement_vector::size_type size_type;

	enum
	{
		MAX_MEASUREMENTS = 529
	};
	
	scan()
		: operation(0),
		  option_1(0),
		  option_2(0),
		  option_3(0),
		  scan_number(0),
		  angular_step_size(0),
		  start_angle(0),
		  stop_angle(0)
	{
		measurements.reserve(MAX_MEASUREMENTS);
	}

	// time since epoch in usec. filled in by the parser as soon as the frame marker is seen.
	uint64_t time_received;
		
	enum
	{
		OPERATION_MEASUREMENT = 0x23,
		OPERATION_ERROR = 0x53,
		OPERATION_WARNING = 0x54
	};

	// current operation
	uint8_t operation;

	// convenience functions to access operation field
	bool measuring() const
	{
		return operation == OPERATION_MEASUREMENT;
	}

	bool error() const
	{
		return operation == OPERATION_ERROR;
	}

	bool warning() const
	{
		return operation == OPERATION_WARNING;
	}
		
	enum
	{
		OPTION_1_ONLY = 1 << 0,             // only option 1 is present
		OPTION_1_AND_2 = (1 << 1),          // option 1 and 2 are present
		OPTION_1_2_3 = (1 << 1) | (1 << 0), // option 1, 2 and 3 are present

		// (redundant) operation info
		OPTION_1_INIT = 1 << 2,
		OPTION_1_MEASUREMENT = 1 << 3,
		OPTION_1_ERROR = 1 << 4
	};

	uint8_t option_1;

	enum
	{
		OPTION_2_NEAR_1_OCCUPIED  = 1 << 0,
		OPTION_2_FAR_1_OCCUPIED   = 1 << 1,
		OPTION_2_WARNING          = 1 << 2,
		OPTION_2_FAULT            = 1 << 3,
		OPTION_2_RESTART_DISABLE  = 1 << 4,
		OPTION_2_NEAR_2_OCCUPIED  = 1 << 5,
		OPTION_2_FAR_2_OCCUPIED   = 1 << 6,
		OPTION_2_OPTION_3_ENABLED = 1 << 7
	};

	uint8_t option_2;

	enum
	{
		OPTION_3_ALWAYS_SET = 1 << 7
	};
		
	uint8_t option_3;

	static
	uint8_t option_3_detection_field_1(uint8_t option_3)
	{
		return option_3 & 0x7;
	}

	static
	uint8_t option_3_detection_field_2(uint8_t option_3)
	{
		return (option_3 >> 3) & 0x7;
	}

	enum
	{
		SCAN_NUMBER_FILL_BYTE = 0xfe
	};
		
	// sequence number
	uint32_t scan_number;

	enum
	{
		ANGULAR_STEPS_PER_360_DEG = 1000
	};

	// angular separation between two consecutive range measurements
	// multiply this by ~0.36° to get the angle
	uint8_t angular_step_size;

	// index of start angle [1 .. 529]
	uint16_t start_angle;

	// index of stop angle [1 .. 529]
	uint16_t stop_angle;

	// the actual range measurements
	measurement_vector measurements;
};
	
} // namespace rotoscan

#endif // __ROTOSCAN_SCAN_HPP__
