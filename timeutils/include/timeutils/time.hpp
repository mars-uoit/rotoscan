/*
 *  Copyright (c) 2009, Rene Wagner
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rw@nelianur.org>
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

#ifndef __TIMEUTILS_TIME_HPP__
#define __TIMEUTILS_TIME_HPP__

#include <stdint.h>
#include <sys/time.h>

#include <cstddef>

namespace timeutils {

enum {
	USEC_PER_SECOND = 1000000
};

static
void get_current_time(uint64_t *t)
{
	struct timeval tv;

	if (0 != gettimeofday(&tv, NULL))
		throw "error calling gettimeofday()";

	*t = uint64_t(tv.tv_sec) * USEC_PER_SECOND + tv.tv_usec;

	//std::cout << tv.tv_sec << " * " << USEC_PER_SECOND << " + " << tv.tv_usec << " = " << *t << std::endl;
}

} // namespace timeutils

#endif // __TIMEUTILS_TIME_HPP__
