/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file armv7-m_dcache.cpp
 *
 * Driver for the armv7 m_dcache.
 *
 */

#include <px4_config.h>
#include <px4_log.h>
#include <board_config.h>
#include <stdint.h>
#include <string.h>

#include <parameters/param.h>

#include "cache.h"

#if defined(CONFIG_ARMV7M_DCACHE) && defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)

extern "C" __EXPORT int dcache_main(int argc, char *argv[]);
extern "C" __EXPORT int board_get_dcache_setting();

/************************************************************************************
 * Name: board_get_dcache_setting
 *
 * Description:
 *  Called to retrieve the parameter setting to enable/disable
 *  the dcache.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *  -1 -  Not set - if Eratta exits turn dcache off else leave it on
 *   0 -  if Eratta exits turn dcache off else leave it on
 *   1 -  Force it off
 *   2 -  Force it on
 *
 ************************************************************************************/

int board_get_dcache_setting()
{
	param_t ph = param_find("SYS_FORCE_F7DC");
	int32_t dcache_setting = -1;

	if (ph != PARAM_INVALID) {
		param_get(ph, &dcache_setting);
	}

	return dcache_setting;
}

int dcache_main(int argc, char *argv[])
{
	int action = -1;
	char *pmesg = nullptr;
	bool state = false;

	if (argc > 1) {
		if (!strcmp(argv[1], "on") || !strcmp(argv[1], "1")) {
			action = 1;
		}

		if (!strcmp(argv[1], "off") || !strcmp(argv[1], "0")) {
			action = 0;
		}
	}

	board_dcache_info(action, &pmesg, &state);
	PX4_INFO("M7 cpuid %s dcache %s", pmesg, state ? "On" : "Off");
	return 0;
}
#endif
