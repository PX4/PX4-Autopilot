/****************************************************************************
 *
 *   Copyright (c) 2025 Lectron FMU-V6X. All rights reserved.
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
 * Lectron FMU-V6X Hardware Configuration
 * - Fixed hardware revision and version (V6X000)
 * - Custom sensor configuration for Lectron hardware
 */

#include <px4_platform_common/px4_config.h>
#include <syslog.h>
#include "board_config.h"

#define MODULE_NAME "board_hw"

static int hw_version = 0;  // V6X000
static int hw_revision = 0;
static char hw_type_name[] = "V6X000";  // Lectron FMU-V6X

__EXPORT int board_determine_hw_info(void)
{
	syslog(LOG_INFO, "[board_hw] Lectron FMU-V6X: Hardware fixed to V6X000\n");
	return 0; // Success
}

__EXPORT const char *board_get_hw_type_name(void)
{
	return hw_type_name;
}

__EXPORT int board_get_hw_version(void)
{
	return hw_version;
}

__EXPORT int board_get_hw_revision(void)
{
	return hw_revision;
}

__EXPORT const char *board_get_hw_base_type_name(void)
{
	return "LECTRONV6X";
}
