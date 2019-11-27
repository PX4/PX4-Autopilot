/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_dcache_control.c
 * Support for parameter based control of dcache for the
 * ARM 1259864 Data corruption eratta
 */

#include <px4_config.h>
#include <stdint.h>
#include <stdbool.h>

#include "nvic.h"
#include "cache.h"
#include "up_arch.h"

#include <parameters/param.h>

#define CPUID_REVISION_SHIFT           0
#define CPUID_REVISION_MASK            (0xf << CPUID_REVISION_SHIFT)
#define CPUID_REVISION(cpuid)          (((cpuid) & CPUID_REVISION_MASK) >> CPUID_REVISION_SHIFT)
#define CPUID_PARTNO_SHIFT             4
#define CPUID_PARTNO_MASK              (0xfff << CPUID_PARTNO_SHIFT)
#define CPUID_PARTNO(cpuid)            (((cpuid) & CPUID_PARTNO_MASK) >> CPUID_PARTNO_SHIFT)
# define CPUID_CORTEX_M7               0xc27
#define CPUID_VARIANT_SHIFT            20
#define CPUID_VARIANT_MASK             (0xf << CPUID_VARIANT_SHIFT)
#define CPUID_VARIANT(cpuid)            (((cpuid) & CPUID_VARIANT_MASK) >> CPUID_VARIANT_SHIFT)
#define CPUID_IMPLEMENTER_SHIFT        24
#define CPUID_IMPLEMENTER_MASK         (0xff << CPUID_IMPLEMENTER_SHIFT)
#define CPUID_IMPLEMENTER(cpuid)       (((cpuid) & CPUID_IMPLEMENTER_MASK) >> CPUID_IMPLEMENTER_SHIFT)

#if defined(CONFIG_ARMV7M_DCACHE) && defined(CONFIG_ARMV7M_DCACHE_WRITETHROUGH)
/************************************************************************************
 * Name: board_configure_dcache
 *
 * Description:
 *  Called at various points in start up to disable the dcache if the
 *  1259864 Data corruption in a sequence of Write-Through stores and loads
 *  errata is preset.
 *
 * Input Parameters:
 *   stage - 0 - early init no OS;
 *           1  - OS and file system are runnting;
 *
 * Returned Value:
 ************************************************************************************/

void board_configure_dcache(int stage)
{
	/* 1259864 Data corruption in a sequence of Write-Through stores and loads
	 * Fault Status: Present in r0p1, r0p2, r1p0 and r1p1. Fixed in r1p2
	 */

	uint32_t cpuid = getreg32(NVIC_CPUID_BASE);


	bool erratta = CPUID_PARTNO(cpuid) == CPUID_CORTEX_M7 && (CPUID_VARIANT(cpuid) == 0 || (CPUID_VARIANT(cpuid) == 1
			&& CPUID_REVISION(cpuid) < 2));

	/* On boot we should default to disabled on effected HW */

	if (erratta && stage == 0) {
		arch_disable_dcache();
		return;
	}

	/* Based on a param We can enable the dcache */

	if (stage != 0) {

		int32_t dcache = board_get_dcache_setting();

		switch (dcache) {
		default:
		case 0:
			erratta ? arch_disable_dcache() : arch_enable_dcache();
			break;

		case 1:
			arch_disable_dcache();
			break;

		case 2:
			arch_enable_dcache();
			break;
			return;
		}

	}
}

/************************************************************************************
 * Name: board_dcache_info
 *
 * Description:
 *  Called to retrieve dcache info and optionally set dcache to on or off.
 *
 * Input Parameters:
 *  action  - -1 Provide info only.
 *  pmesg   - if non null return the chipid revision and patch level
 *            will indicate if the dcache eratta is present.
 *  state   - if non null return the state of the dcache
 *            true on, false is off.
 *
 * Returned Value:
 *   0 - success
 *
 ************************************************************************************/

int board_dcache_info(int action, char **pmesg, bool *pstate)
{
	uint32_t cpuid = getreg32(NVIC_CPUID_BASE);
	static char mesg[] = "r?p? has dcache eratta!";
	bool erratta = (CPUID_PARTNO(cpuid) == CPUID_CORTEX_M7 && (CPUID_VARIANT(cpuid) == 0 || (CPUID_VARIANT(cpuid) == 1
			&& CPUID_REVISION(cpuid) < 2)));

	mesg[1] = '0' + CPUID_VARIANT(cpuid);
	mesg[3] = '0' + CPUID_REVISION(cpuid);

	if (!erratta) {
		mesg[5] = 'O';
		mesg[6] = 'K';
		mesg[7] = '\0';
	}

	if (action == 0) {
		arch_disable_dcache();
	}

	if (action == 1) {
		arch_enable_dcache();
	}

	if (pmesg) {
		*pmesg = mesg;
	}

	if (pstate) {
		*pstate = getreg32(NVIC_CFGCON) & NVIC_CFGCON_DC ? true : false;
	}

	return 0;

}
#endif
