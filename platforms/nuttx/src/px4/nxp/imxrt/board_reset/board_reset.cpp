/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
 * @file board_reset.cpp
 * Implementation of IMXRT based Board RESET API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/shutdown.h>
#include <errno.h>
#include <nuttx/board.h>
#include <arm_internal.h>

#ifdef CONFIG_ARCH_FAMILY_IMXRT117x
#include <hardware/rt117x/imxrt117x_snvs.h>
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#endif

#define BOOT_RTC_SIGNATURE                0xb007b007
#define PX4_IMXRT_RTC_REBOOT_REG          3
#define PX4_IMXRT_RTC_REBOOT_REG_ADDRESS  IMXRT_SNVS_LPGPR3

#if CONFIG_IMXRT_RTC_MAGIC_REG == PX4_IMXRT_RTC_REBOOT_REG
#  error CONFIG_IMXRT_RTC_MAGIC_REG can nt have the save value as PX4_IMXRT_RTC_REBOOT_REG
#endif

static int board_reset_enter_bootloader()
{
#ifdef CONFIG_ARCH_FAMILY_IMXRT117x
	uint32_t regvalue = BOOT_RTC_SIGNATURE;
	modifyreg32(IMXRT_SNVS_LPCR, 0, SNVS_LPCR_GPR_Z_DIS);
	putreg32(regvalue, PX4_IMXRT_RTC_REBOOT_REG_ADDRESS);
#elif defined(BOARD_HAS_TEENSY_BOOTLOADER)
	asm("BKPT #251"); /* Enter Teensy MKL02 bootloader */
#endif
	return OK;
}

int board_reset(int status)
{
	if (status == REBOOT_TO_BOOTLOADER) {
		board_reset_enter_bootloader();
	}

#if defined(BOARD_HAS_ISP_BOOTLOADER)

	else if (status == REBOOT_TO_ISP) {
		uint32_t arg = 0xeb100000;
		ROM_API_Init();
		ROM_RunBootloader(&arg);
	}

#endif

#if defined(BOARD_HAS_ON_RESET)
	board_on_reset(status);
#endif

	up_systemreset();
	return 0;
}
