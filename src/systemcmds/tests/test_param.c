/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file test_param.c
 *
 * Tests related to the parameter system.
 */

#include <stdio.h>
#include "systemlib/err.h"

#include "systemlib/param/param.h"
#include "tests.h"

PARAM_DEFINE_INT32(test, 0x12345678);

int
test_param(int argc, char *argv[])
{
	param_t		p;

	p = param_find("test");
	if (p == PARAM_INVALID)
		errx(1, "test parameter not found");

	param_type_t t = param_type(p);
	if (t != PARAM_TYPE_INT32)
		errx(1, "test parameter type mismatch (got %u)", (unsigned)t);

	int32_t	val;
	if (param_get(p, &val) != 0)
		errx(1, "failed to read test parameter");
	if (val != 0x12345678)
		errx(1, "parameter value mismatch");

	val = 0xa5a5a5a5;
	if (param_set(p, &val) != 0)
		errx(1, "failed to write test parameter");
	if (param_get(p, &val) != 0)
		errx(1, "failed to re-read test parameter");
	if ((uint32_t)val != 0xa5a5a5a5)
		errx(1, "parameter value mismatch after write");

	warnx("parameter test PASS");

	return 0;
}
