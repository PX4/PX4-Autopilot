/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * @file bootloader_main.c
 *
 * FMU-specific early startup code for bootloader
*/

#include "board_config.h"
#include "bl.h"

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <arch/board/board.h>
#include <px4_platform_common/init.h>
#include <debug.h>
#include "riscv_internal.h"

extern int mpfs_dma_alloc_init(void);
extern int sercon_main(int c, char **argv);
__EXPORT void board_on_reset(int status) {}

static void configure_pmp(void)
{
#if 0
	// SDCARD DMA: NB! sizes must be power of 2
	const uint64_t mode_bits = 0x1Full << 56;

	// 1MB of LIM ram (bootloader RAM_START & SIZE)
	uint64_t base_range = (0x08000000ull | (0x100000ull - 1ull)) >> 2;
	putreg64(mode_bits | base_range, 0x20005700);

	// 4MB from the start of DDR
	base_range = (0x80000000ull | (0x400000ull - 1ull)) >> 2;
	putreg64(mode_bits | base_range, 0x20005708);

#else

#define MPFS_PMPCFG_MMC_0             (MPFS_MPUCFG_BASE + 0x700)
#define MPFS_PMPCFG_MMC_1             (MPFS_MPUCFG_BASE + 0x708)
#define MPFS_PMPCFG_MMC_2             (MPFS_MPUCFG_BASE + 0x710)
#define MPFS_PMPCFG_MMC_3             (MPFS_MPUCFG_BASE + 0x718)

#define MPFS_PMPCFG_ETH0_0   (MPFS_MPUCFG_BASE + 0x400)
#define MPFS_PMPCFG_ETH0_1   (MPFS_MPUCFG_BASE + 0x408)
#define MPFS_PMPCFG_ETH0_2   (MPFS_MPUCFG_BASE + 0x410)
#define MPFS_PMPCFG_ETH0_3   (MPFS_MPUCFG_BASE + 0x418)
#define MPFS_PMPCFG_ETH1_0   (MPFS_MPUCFG_BASE + 0x500)
#define MPFS_PMPCFG_ETH1_1   (MPFS_MPUCFG_BASE + 0x508)
#define MPFS_PMPCFG_ETH1_2   (MPFS_MPUCFG_BASE + 0x510)
#define MPFS_PMPCFG_ETH1_3   (MPFS_MPUCFG_BASE + 0x518)

	putreg64(0x1f00000fffffffff, MPFS_PMPCFG_MMC_0);
	putreg64(0x1f00000fffffffff, MPFS_PMPCFG_MMC_1);
	putreg64(0x1f00000fffffffff, MPFS_PMPCFG_MMC_2);
	putreg64(0x1f00000fffffffff, MPFS_PMPCFG_MMC_3);

	// Ethernet MPU, MAC 0
	putreg64(0x1f00000fffffffff, MPFS_PMPCFG_ETH0_0);

#endif

}

__EXPORT void mpfs_boardinitialize(void)
{
	_alert("Icicle bootloader\n");

	/* this call exists to fix a weird linking issue */
	up_udelay(0);

	/* Enable clocks, TIMER is used for PX4 HRT, others are common
	   for many peripherals */
	modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET, 0,
		    (
			    SYSREG_SUBBLK_CLOCK_CR_TIMER
			    | SYSREG_SUBBLK_CLOCK_CR_GPIO2
			    | SYSREG_SUBBLK_CLOCK_CR_CFM
			    | SYSREG_SUBBLK_CLOCK_CR_FIC3
		    )
		   );

	/* Take peripheral out of reset */

	modifyreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_SOFT_RESET_CR_OFFSET,
		    (
			    SYSREG_SOFT_RESET_CR_TIMER
			    | SYSREG_SOFT_RESET_CR_GPIO2
			    | SYSREG_SOFT_RESET_CR_CFM
			    | SYSREG_SOFT_RESET_CR_FIC3
			    | SYSREG_SOFT_RESET_CR_FPGA
		    ), 0);

	/* configure PMP */

	configure_pmp();
}

void board_late_initialize(void)
{

#if defined(CONFIG_FAT_DMAMEMORY) && defined(CONFIG_GRAN)
	/* configure the DMA allocator */

	if (mpfs_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#endif
}

int board_app_initialize(uintptr_t arg)
{
	return 0;
}

extern void sys_tick_handler(void);

void board_timerhook(void)
{
	sys_tick_handler();
}
