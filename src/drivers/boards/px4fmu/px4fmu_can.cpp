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
 * @file px4fmu_can.c
 *
 * Board-specific CAN functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>
#include <arch/board/board.h>

#include <chip.h>
#include <up_arch.h>

#include <stm32.h>
#include <stm32_can.h>
#include "px4fmu_internal.h"

#include <drivers/device/can.h>

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(CONFIG_STM32_CAN1) && defined(CONFIG_STM32_CAN2)
#  warning "Both CAN1 and CAN2 are enabled.  Assuming only CAN1."
#  undef CONFIG_STM32_CAN2
#endif

#ifdef CONFIG_STM32_CAN1
#  define CAN_PORT 1
#else
#  define CAN_PORT 2
#endif

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   Modified CAN initialisation to work with the PX4 CAN multiplexer architecture.
 *
 ************************************************************************************/

extern "C" int can_devinit();

int can_devinit()
{
	static bool initialized = false;
	
	/* Check if we have already initialized */

	if (!initialized)
		return OK;

	/* Call stm32_caninitialize() to get an instance of the CAN interface */

	can_dev_s *can = stm32_caninitialize(CAN_PORT);

	if (can == NULL) {
		candbg("ERROR:  Failed to get CAN interface\n");
		return -ENODEV;
	}

	/* Register the CAN driver */

	int ret = device::CAN::connect(can, CAN_PORT, 16);	/* XXX hardcoded TX queue size */

	if (ret < 0) {
		candbg("ERROR: can_register failed: %d\n", ret);
		return ret;
	}

	/* Now we are initialized */

	initialized = true;

	return OK;
}

#endif
