/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "px4_test.h"

#include <float.h>
#include <parameters/param.h>

#include <zephyr/ztest.h>

ZTEST(px4_param, test_int32)
{
	const param_t param = param_find("SYS_AUTOSTART");
	zassert_not_equal(param, PARAM_INVALID);
	zassert_equal(param_type(param), PARAM_TYPE_INT32);

	int32_t value = -1;
	zassert_ok(param_get(param, &value));
	zassert_equal(value, 0);

	value = 4001;
	zassert_ok(param_set_no_notification(param, &value));
	value = 0;
	zassert_ok(param_get(param, &value));
	zassert_equal(value, 4001);

	param_reset_no_notification(param);
	zassert_ok(param_get(param, &value));
	zassert_equal(value, 0);
}

ZTEST(px4_param, test_float)
{
	const param_t param = param_find("BAT_LOW_THR");
	zassert_not_equal(param, PARAM_INVALID);
	zassert_equal(param_type(param), PARAM_TYPE_FLOAT);

	float default_value = 0.f;
	zassert_ok(param_get(param, &default_value));

	float value = default_value + 0.1f;
	zassert_ok(param_set_no_notification(param, &value));
	value = 0.f;
	zassert_ok(param_get(param, &value));
	zassert_within(value, default_value + 0.1f, FLT_EPSILON);

	param_reset_no_notification(param);
	zassert_ok(param_get(param, &value));
	zassert_within(value, default_value, FLT_EPSILON);
}

ZTEST_SUITE(px4_param, NULL, px4_test_setup, NULL, NULL, NULL);
