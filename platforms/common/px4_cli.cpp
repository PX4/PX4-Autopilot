/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <parameters/param.h>
#include <px4_platform_common/cli.h>

#ifndef MODULE_NAME
#define MODULE_NAME "cli"
#endif

#include <px4_platform_common/log.h>

#include <cstring>
#include <errno.h>
#include <cstdlib>

int px4_get_parameter_value(const char *option, int &value)
{
	value = 0;

	/* check if this is a param name */
	if (strncmp("p:", option, 2) == 0) {

		const char *param_name = option + 2;

		/* user wants to use a param name */
		param_t param_handle = param_find(param_name);

		if (param_handle != PARAM_INVALID) {

			if (param_type(param_handle) != PARAM_TYPE_INT32) {
				return -EINVAL;
			}

			int32_t pwm_parm;
			int ret = param_get(param_handle, &pwm_parm);

			if (ret != 0) {
				return ret;
			}

			value = pwm_parm;

		} else {
			PX4_ERR("param name '%s' not found", param_name);
			return -ENXIO;
		}

	} else {
		char *ep;
		value = strtol(option, &ep, 0);

		if (*ep != '\0') {
			return -EINVAL;
		}
	}

	return 0;
}
