/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * @file hw_rev_ver_canhubk3.c
 * CANHUBK3 Hardware Revision and Version ID API
 */
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/board_determine_hw_info.h>
#include <stdio.h>
#include <board_config.h>

#include <systemlib/px4_macros.h>

#if defined(BOARD_HAS_HW_VERSIONING)


#define HW_INFO_SIZE HW_INFO_VER_DIGITS + HW_INFO_REV_DIGITS


/****************************************************************************
 * Private Data
 ****************************************************************************/
static int is_adap_connected = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_get_hw_type
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_type_name()
{
	if (is_adap_connected) {
		return (const char *)"MR-CANHUBK3-ADAP";

	} else {
		return (const char *)"MR-CANHUBK344";
	}
}

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_version()
{
	return  0;
}

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_revision()
{
	return  0;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses GPIO to detect MR-CANHUBK3-ADAP
 *
 ************************************************************************************/

int board_determine_hw_info()
{
	s32k3xx_pinconfig(CANHUBK3_ADAP_DETECT);
	is_adap_connected = !s32k3xx_gpioread(CANHUBK3_ADAP_DETECT);
	return 0;
}
#endif
