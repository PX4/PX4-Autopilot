/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Implementation of ESP32 based SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>


/* Define any issues with the Silicon as lines separated by \n
 * omitting the last \n
 */
#define ESP32_ERRATA "No unique CPU ID, using MAC address."
#define DR_REG_EFUSE_BASE 0x3ff5a000
#define DR_REG_SYSCON_BASE 0x60026000
#define EFUSE_BLK0_RDATA3_REG (DR_REG_EFUSE_BASE + 0x00c)
#define EFUSE_BLK0_RDATA5_REG (DR_REG_EFUSE_BASE + 0x014)
#define SYSCON_DATE_REG       (DR_REG_SYSCON_BASE + 0x3FC)

#define REG_READ(_r) (*(volatile uint32_t *)(_r))


int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	/*
	 comes from esp-idf in efuse_hal_get_major_chip_version
	 and efuse_hal_get_minor_chip_version
	*/
	uint8_t eco_bit0 = (REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15) & 0x1;
	uint8_t eco_bit1 = (REG_READ(EFUSE_BLK0_RDATA5_REG) >> 20) & 0x1;
	uint8_t eco_bit2 = (REG_READ(SYSCON_DATE_REG) & 0x80000000) >> 31;
	uint32_t combine_value = (eco_bit2 << 2) | (eco_bit1 << 1) | eco_bit0;
	uint8_t minor_revision = (REG_READ(EFUSE_BLK0_RDATA5_REG) >> 24) & 0x3;

	int revid;

	switch (combine_value) {
	case 0:
		*revstr = "ESP32 v0";
		*rev = minor_revision + 48;
		revid = minor_revision;
		break;

	case 1:
		*revstr = "ESP32 v1";
		*rev = minor_revision + 48;
		revid = minor_revision;
		break;

	case 3:
		*revstr = "ESP32 v2";
		*rev = minor_revision + 48;
		revid = minor_revision;
		break;

	case 7:
		*revstr = "ESP32 v3";
		*rev = minor_revision + 48;
		revid = minor_revision;
		break;

	default:
		*revstr = "ESP32 v?";
		*rev = '?';
		revid = 0;
		break;
	}

	return revid;
}
