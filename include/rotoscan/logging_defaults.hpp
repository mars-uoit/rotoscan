/*
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

#ifndef __ROTOSCAN_LOGGING_DEFAULTS_HPP__
#define __ROTOSCAN_LOGGING_DEFAULTS_HPP__

#include <iostream>

#ifndef ROTOSCAN_LOG_TRACE
#define ROTOSCAN_LOG_TRACE(msg) (std::cout << "rotoscan: trace: " << msg << std::endl)
#endif

#ifndef ROTOSCAN_LOG_DEBUG
#define ROTOSCAN_LOG_DEBUG(msg) (std::cout << "rotoscan: debug: " << msg << std::endl)
#endif

#ifndef ROTOSCAN_LOG_INFO
#define ROTOSCAN_LOG_INFO(msg) (std::cout << "rotoscan: info: " << msg << std::endl)
#endif

#ifndef ROTOSCAN_LOG_WARN
#define ROTOSCAN_LOG_WARN(msg) (std::cerr << "rotoscan: warn: " << msg << std::endl)
#endif

#ifndef ROTOSCAN_LOG_ERROR
#define ROTOSCAN_LOG_ERROR(msg) (std::cerr << "rotoscan: error: " << msg << std::endl)
#endif

#ifndef ROTOSCAN_LOG_FATAL
#define ROTOSCAN_LOG_FATAL(msg) (std::cerr << "rotoscan: fatal: " << msg << std::endl)
#endif

#endif // __ROTOSCAN_LOGGING_DEFAULTS_HPP__
