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
 * @file board_mcu_version.c
 * Implementation of STM32 based SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#define RP2040_CPUID_BASE	(RP2040_PPB_BASE + 0xed00)

/* magic numbers from reference manual */

enum MCU_REV {
	MCU_REV_RP2040_REV_1 = 0x1
};

/* Define any issues with the Silicon as lines separated by \n
 * omitting the last \n
 */
#define RP2040_ERRATA "This device does not have a unique id!"


//STM DocID018909 Rev 8 Sect 38.18 and DocID026670 Rev 5 40.6.1 (MCU device ID code)
# define REVID_MASK    0xF
# define DEVID_MASK    0xFFFFFFF0

# define RP2040_DEVICE_ID	0x410CC60


int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	uint32_t abc = getreg32(RP2040_CPUID_BASE);

	int32_t chip_version = (abc & DEVID_MASK) > 4;
	enum MCU_REV revid = abc & REVID_MASK;
	const char *chip_errata = NULL;

	switch (chip_version) {


	case RP2040_DEVICE_ID:
		*revstr = "RP2040";
		chip_errata = RP2040_ERRATA;
		break;
	default:
		*revstr = "RPI???";
		break;
	}

	switch (revid) {

	case MCU_REV_RP2040_REV_1:
		*rev = '1';
		break;
	default:
		*rev = '?';
		revid = -1;
		break;
	}

	if (errata) {
		*errata = chip_errata;
	}

	return revid;
}
