/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#include "state_sharing/utils.hpp"

void setParameter(const ArgParser &args,
		  const char *name, const float defaultValue)
{
	if (args.hasArgument(name)) {
		param_t param = param_find_no_notification(name);
		auto value = args.getFloat(name, defaultValue);

		if (param != PARAM_INVALID) {
			PX4_DEBUG("setting argument %s with value %f", name, (double) value);
			param_set(param, &value);
		}
	}
}

void setParameter(const ArgParser &args,
		  const char *name, const int defaultValue)
{
	if (args.hasArgument(name)) {
		param_t param = param_find_no_notification(name);
		auto value = args.getInt(name, defaultValue);

		if (param != PARAM_INVALID) {
			PX4_DEBUG("setting argument %s with value %d", name, value);
			param_set(param, &value);
		}
	}
}

uint64_t getRealTimeNs()
{
	timespec tv = {};
	px4_clock_gettime(CLOCK_REALTIME, &tv);
	return tv.tv_sec * 1e9 + tv.tv_nsec;
}
