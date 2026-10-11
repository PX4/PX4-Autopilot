/****************************************************************************
 *
 *   Copyright (c) 2025 Lectron. All rights reserved.
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
 * @file board_hw_info.c
 *
 * Lectron Pi5 Autopilot Hardware Configuration
 * - Fixed hardware revision and version (V6X000)
 * - Custom sensor configuration for Lectron hardware
 */

#include <px4_platform_common/px4_config.h>
#include <syslog.h>
#include "board_config.h"

#define MODULE_NAME "board_hw"
#define HW_TYPE_NAME "V6X000"
#define HW_VERSION   0
#define HW_REVISION  0

__EXPORT int board_determine_hw_info(void)
{
	syslog(LOG_INFO, "[board_hw] Lectron Pi5 Autopilot: Hardware fixed to V6X000\n");
	return 0; // Success
}

__EXPORT const char *board_get_hw_type_name(void)
{
	return HW_TYPE_NAME;
}

__EXPORT int board_get_hw_version(void)
{
	return HW_VERSION;
}

__EXPORT int board_get_hw_revision(void)
{
	return HW_REVISION;
}

__EXPORT const char *board_get_hw_base_type_name(void)
{
	return "LECTRONPI5AUTOPILOT";
}
