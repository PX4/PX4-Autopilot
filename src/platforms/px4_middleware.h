/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_middleware.h
 *
 * PX4 generic middleware wrapper
 */

#pragma once

#include <stdint.h>
#include <unistd.h>

namespace px4
{

__EXPORT void init(int argc, char *argv[], const char *process_name);

__EXPORT uint64_t get_time_micros();

#if defined(__PX4_ROS)
/**
 * Returns true if the app/task should continue to run
 */
inline bool ok() { return ros::ok(); }
#else
extern bool task_should_exit;
/**
 * Returns true if the app/task should continue to run
 */
__EXPORT inline bool ok() { return !task_should_exit; }
#endif

class Rate
{
public:
	/**
	 * Construct the Rate object and set rate
	 * @param rate_hz rate from which sleep time is calculated in Hz
	 */
	explicit Rate(unsigned rate_hz) { sleep_interval = 1e6 / rate_hz; }

	/**
	 * Sleep for 1/rate_hz s
	 */
	void sleep() { usleep(sleep_interval); }

private:
	uint64_t sleep_interval;

};

} // namespace px4
