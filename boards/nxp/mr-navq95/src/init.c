/****************************************************************************
 * boards/nxp/mr-navq95/src/init.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/board.h>

#include "mr-navq95.h"
#include <px4_platform_common/init.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

__BEGIN_DECLS
extern const uint64_t _fitcmfuncs;   /* Copy source address in FLASH */
extern uint64_t _sitcmfuncs;         /* Copy destination start address in ITCM */
extern uint64_t _eitcmfuncs;         /* Copy destination end address in ITCM */
extern uint64_t _itcm_start;         /* ITCM bank base (from linker) */
extern uint64_t _itcm_end;           /* ITCM bank end  (from linker) */
__END_DECLS

/****************************************************************************
 * Name: imx9_init_isram_functions
 *
 * Description:
 *   Called off reset vector to reconfigure the ITCM
 *   and finish the FLASH to RAM Copy.
 *
 *   Runs before arm_fpuconfig(), so it must not emit any FPU/NEON
 *   instructions (this file is built -mgeneral-regs-only): the 64-bit
 *   stores stay integer strd instead of vstr, which would fault while
 *   the FPU is still disabled.
 *
 ****************************************************************************/

__EXPORT void imx9_init_isram_functions(void)
{
	volatile uint64_t *src;
	volatile uint64_t *dest;

	/* ITCM is ECC protected with read-modify-write (NVIC_TCMCR_RMW), so any
	 * read of a 64-bit granule never written since power-on faults. Seed the
	 * whole bank with aligned 64-bit writes before copying/executing code. */
	for (dest = &_itcm_start; dest < &_itcm_end;) {
		*dest++ = 0;
	}

	/* Copy any necessary code sections from FLASH to ITCM. The process is the
	* same as the code copying from FLASH to RAM above. */
	for (src = (uint64_t *)&_fitcmfuncs, dest = (uint64_t *)&_sitcmfuncs;
	     dest < (uint64_t *)&_eitcmfuncs;) {
		*dest++ = *src++;
	}

}

#ifdef CONFIG_BOARDCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
	int ret = OK;
	UNUSED(arg);

	px4_platform_init();

#ifndef CONFIG_BOARD_LATE_INITIALIZE
	/* Perform board initialization */

	ret = imx95_bringup();
#endif

	px4_platform_configure();

	imx9_spiinitialize();

	return ret;
}

#endif /* CONFIG_BOARDCTL */
