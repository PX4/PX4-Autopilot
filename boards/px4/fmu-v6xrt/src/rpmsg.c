/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file rpmsg.c
 *
 * CM4 remote description for the rptun transport and the board half of the
 * remote_core command. The CM7 loads the CM4 ELF from BOARD_CM4_FIRMWARE, so
 * the resource table lives in that image; the memory map is cm4/memmap.h.
 */

#include <nuttx/config.h>
#include <nuttx/rptun/rptun.h>

#include <inttypes.h>
#include <stdio.h>

#include "board_config.h"
#include "arm_internal.h"
#include "imxrt_rptun.h"
#include "mpu.h"
#include "hardware/imxrt_memorymap.h"
#include "hardware/rt117x/imxrt117x_src.h"
#include "hardware/rt117x/imxrt117x_ccm.h"
#include "../cm4/status.h"

/* CM4 device address -> CM7 view through the LMEM backdoor. */
static const struct rptun_addrenv_s g_cm4_addrenv[] = {
	{ .pa = CM4_CODE_TCM_PA, .da = CM4_CODE_TCM_DA, .size = CM4_TCM_SIZE },
	{ .pa = CM4_SYS_TCM_PA,  .da = CM4_SYS_TCM_DA,  .size = CM4_TCM_SIZE },
	{ .size = 0 },
};

static const struct imxrt_rptun_config_s g_cm4_config = {
	.cpuname    = BOARD_RPMSG_CPUNAME,
	.firmware   = BOARD_CM4_FIRMWARE,
	.addrenv    = g_cm4_addrenv,
	.boot_addr  = CM4_CODE_TCM_PA,
	.autostart  = false,
};

int board_rpmsg_initialize(void)
{
	/* The CM4 writes the vrings and rpmsg buffers; the CM7 must not read
	 * them through its D-cache. A later region wins over the cacheable
	 * OCRAM_M4 region from imxrt_mpu_initialize().
	 */
	mpu_configure_region(CM4_SHM_PA, CM4_SHM_SIZE,
			     MPU_RASR_AP_RWRW | MPU_RASR_TEX_NOR | MPU_RASR_S | MPU_RASR_XN);

	return imxrt_rptun_init(&g_cm4_config);
}

#define CCM_M4_ROOT             1

int board_rpmsg_status(void)
{
	const volatile uint32_t *st = (const volatile uint32_t *)CM4_STATUS_PA;
	const volatile uint32_t *vec = (const volatile uint32_t *)CM4_CODE_TCM_PA;
	const volatile char *app_pa = (const volatile char *)CM4_APP_NAME_PA;
	uint32_t srsr = getreg32(IMXRT_SRC_SRSR);
	uint32_t m4_sta = getreg32(IMXRT_CCM_BASE + IMXRT_CCM_CR_STAT0_OFFSET(CCM_M4_ROOT));
	uint32_t gpr0 = getreg32(IMXRT_IOMUXC_LPSR_GPR_GPR0);
	uint32_t gpr1 = getreg32(IMXRT_IOMUXC_LPSR_GPR_GPR1);
	uint32_t state = st[0];
	const char *name = "unknown";
	char app[CM4_APP_NAME_LEN + 1] = {0};

	switch (state) {
	case CM4_STATE_BOOT:  name = "boot"; break;

	case CM4_STATE_MU:    name = "mu ready"; break;

	case CM4_STATE_READY: name = "rpmsg ready"; break;

	default:
		if ((state & 0xFFFF0000) == CM4_STATE_FAULT) { name = "FAULT"; }

		if ((state & 0xFFFF0000) == CM4_STATE_BADRSC) { name = "BAD RESOURCE TABLE"; }

		break;
	}

	for (int i = 0; i < CM4_APP_NAME_LEN; i++) { app[i] = app_pa[i]; }

	printf("SRC SRSR 0x%08" PRIx32 " (%s%s%s) SRMR 0x%08" PRIx32 "\n", srsr,
	       (srsr & SRC_SRSR_M4_LOCKUP) ? "m4-lockup " : "", (srsr & SRC_SRSR_M7_LOCKUP) ? "m7-lockup " : "",
	       (srsr & SRC_SRSR_WDOG) ? "wdog " : "", getreg32(IMXRT_SRC_SRMR));
	printf("M4 clock root control 0x%08" PRIx32 " status 0x%08" PRIx32 " (%s)\n",
	       getreg32(IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_OFFSET(CCM_M4_ROOT)), m4_sta,
	       (m4_sta & CCM_CR_STAT0_OFF) ? "OFF" : "on");
	printf("CM4 VTOR from LPSR GPR0/1: 0x%08" PRIx32 " (GPR0 0x%08" PRIx32 " GPR1 0x%08" PRIx32 ")\n",
	       ((gpr1 & 0xffff) << 16) | (gpr0 & 0xfff8), gpr0, gpr1);
	printf("cm4 slice %s reset, SCR 0x%08" PRIx32 ", vectors sp 0x%08" PRIx32 " pc 0x%08" PRIx32 "\n",
	       (getreg32(IMXRT_SRC_STAT_M4CORE) & 1) ? "in" : "out of", getreg32(IMXRT_SRC_SCR), vec[0], vec[1]);
	printf("cm4 state 0x%08" PRIx32 " (%s), ipsr %" PRIu32 ", kicks rx %" PRIu32 ", msgs tx %" PRIu32 "\n",
	       state, name, st[1], st[2], st[3]);
	printf("cm4 application \"%s\"\n", app);
	return 0;
}
