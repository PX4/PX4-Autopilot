/****************************************************************************
 *
 *   Copyright (C) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file px4same70xplained_can.c
 *
 * Board-specific CAN functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>
#include "board_config.h"

#include "chip.h"
#include "up_arch.h"

#include "chip.h"
#include "sam_mcan.h"

#ifdef CONFIG_CAN

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if (defined(CONFIG_SAMV7_MCAN0) || defined(CONFIG_SAMV7_MCAN1))
#  warning "Both CAN1 and CAN2 are enabled.  Assuming only CAN1."
#  undef CONFIG_SAMV7_MCAN0
#endif

#ifdef CONFIG_SAMV7_MCAN0
#  define CAN_PORT 0
#else
#  define CAN_PORT 1
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
 *   All SAM architectures must provide the following interface to work with
 *   examples/can.
 *
 ************************************************************************************/

int board_can_initialize(void)
{
	static bool initialized = false;
	struct can_dev_s *can;
	int ret;

	/* Check if we have already initialized */

	if (!initialized) {
		/* Call sam_mcan_initialize() to get an instance of the CAN interface */

		can = sam_mcan_initialize(CAN_PORT);

		if (can == NULL) {
			candbg("ERROR:  Failed to get CAN interface\n");
			return -ENODEV;
		}

		/* Register the CAN driver at "/dev/can0" */

		ret = can_register("/dev/can0", can);

		if (ret < 0) {
			candbg("ERROR: can_register failed: %d\n", ret);
			return ret;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif
