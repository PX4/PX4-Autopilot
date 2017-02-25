/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file param.cpp
 *
 * Global parameter store c++ wrapper.
 *
 */

#include "param.hpp"

#include <px4_defines.h>

bool
param_get(const param_t param, float &val)
{
	if (param_type(param) != PARAM_TYPE_FLOAT) {
		PX4_WARN("param_get wrong type %s", param_name(param));
		return false;
	}

	float tmp_val = 0.0f;

	if (param_get(param, &tmp_val) == PX4_OK) {
		val = tmp_val;
		return true;

	} else {
		PX4_WARN("param_get %s failed", param_name(param));
		return false;
	}
}

bool
param_get(const param_t param, int32_t &val)
{
	if (param_type(param) != PARAM_TYPE_INT32) {
		PX4_WARN("param_get wrong type %s", param_name(param));
		return false;
	}

	int32_t tmp_val = 0;

	if (param_get(param, &tmp_val) == PX4_OK) {
		val = tmp_val;
		return true;

	} else {
		PX4_WARN("param_get %s failed", param_name(param));
		return false;
	}
}
