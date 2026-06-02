/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>
#include <board_config.h>

#include <nuttx/spi/spi.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/mtd_manifest.h>

/* Physical bus devices */

static const px4_mft_device_t spi4_fram = {        // MB85RS1MT on FMUM native: 1Mbit, emulated as (1024 Blocks of 32)
	.bus_type   = px4_mft_device_t::SPI,
	.spi_driver = px4_mft_device_t::SPI_DRIVER_DEFAULT,
	.devid      = SPIDEV_FLASH(0)
};
static const px4_mft_device_t spi4_nor = {         // MX25L1606EZUI-12G-TR: 16Mbit NOR flash (512 sectors x 4KB, 8192 pages x 256B)
	.bus_type   = px4_mft_device_t::SPI,
	.spi_driver = px4_mft_device_t::SPI_DRIVER_MX25L,
	.devid      = SPIDEV_FLASH(0)
};
static const px4_mft_device_t i2c4 = {             // 24LC64T   8K 32 X 256 or 16K for EEPROM only boards
	.bus_type =  px4_mft_device_t::I2C,
	.devid    =  PX4_MK_I2C_DEVID(4, 0x50)
};

/* MTD partition entries */

static const px4_mtd_entry_t fmum_nor = {          // MX25L1606E: 2MB / 256-byte page = 8192 blocks
	.device = &spi4_nor,
	.npart = 1,
	.bulk_erase = true,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/mnt/microsd",
			.nblocks = (2097152 / 256),
			.bypass_ftl = true
		}
	},
};

static const px4_mtd_entry_t fmum_fram = {
	.device = &spi4_fram,
	.npart = 1,
	.partd = {
		{
			.type = MTD_PARAMETERS,
			.path = "/mnt/microsd",
			.nblocks = (131072 / (1 << CONFIG_RAMTRON_EMULATE_SECTOR_SHIFT))
		}
	},
};

/* EEPROM layout for EEPROM-only boards (128Kbit, 512 pages x 32B = 16KB). */
static constexpr uint32_t kEepromParts = 3;

static const px4_mtd_entry_t fmum_eeprom_large = {
	.device = &i2c4,
	.npart = kEepromParts,
	.partd = {
		{
			.type = MTD_MFT_VER,
			.path = "/fs/mtd_mft_ver",
			.nblocks = 1
		},
		{
			.type = MTD_MFT_REV,
			.path = "/fs/mtd_mft_rev",
			.nblocks = 1
		},
		{
			.type = MTD_NET,
			.path = "/fs/mtd_net",
			.nblocks = 1
		}
	},
};
static_assert(kEepromParts == 3,
	      "EEPROM partition count changed: update init.c accordingly");

/* EEPROM layout for FRAM/NOR boards (64Kbit, 256 pages x 32B = 8KB). */
static const px4_mtd_entry_t fmum_eeprom_small = {
	.device = &i2c4,
	.npart = 5,
	.partd = {
		{
			.type = MTD_CALDATA,
			.path = "/fs/mtd_caldata",
			.nblocks = 224
		},
		{
			.type = MTD_MFT_VER,
			.path = "/fs/mtd_mft_ver",
			.nblocks = 8
		},
		{
			.type = MTD_MFT_REV,
			.path = "/fs/mtd_mft_rev",
			.nblocks = 8
		},
		{
			.type = MTD_ID,
			.path = "/fs/mtd_id",
			.nblocks = 8
		},
		{
			.type = MTD_NET,
			.path = "/fs/mtd_net",
			.nblocks = 8
		}
	},
};

/* MTD manifests */

static const px4_mtd_manifest_t board_mtd_config_eeprom_small = {
	.nconfigs = 1,
	.entries = { &fmum_eeprom_small }
};

static const px4_mtd_manifest_t board_mtd_config_eeprom_large = {
	.nconfigs = 1,
	.entries = { &fmum_eeprom_large }
};

static const px4_mtd_manifest_t board_mtd_config_spi_fram = {
	.nconfigs = 1,
	.entries = { &fmum_fram }
};

static const px4_mtd_manifest_t board_mtd_config_spi_nor = {
	.nconfigs = 1,
	.entries = { &fmum_nor }
};

/* Full description FRAM manifest, set after complete init. */
static const px4_mtd_manifest_t board_mtd_config_fram = {
	.nconfigs = 2,
	.entries = { &fmum_eeprom_small, &fmum_fram }
};

/* Full description NOR manifest, set after complete init. */
static const px4_mtd_manifest_t board_mtd_config_nor = {
	.nconfigs = 2,
	.entries = { &fmum_eeprom_small, &fmum_nor }
};

/* MFT entries */

static const px4_mft_entry_s mft_mft = {
	.type = MFT,
	.pmft = (void *) system_query_manifest,
};

static const px4_mft_entry_s mtd_mft_eeprom_small = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config_eeprom_small,
};

static const px4_mft_entry_s mtd_mft_eeprom_large = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config_eeprom_large,
};

static const px4_mft_entry_s mtd_mft_fram = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config_fram,
};

static const px4_mft_entry_s mtd_mft_nor = {
	.type = MTD,
	.pmft = (void *) &board_mtd_config_nor,
};

/* MFT manifests */

/* Intermediate manifest while SPI storage is not yet detected (FRAM/NOR boards) */
static const px4_mft_s mft_eeprom_small = {
	.nmft = 2,
	.mfts = { &mtd_mft_eeprom_small, &mft_mft }
};

/* Final manifest for EEPROM-only boards */
static const px4_mft_s mft_eeprom_large = {
	.nmft = 2,
	.mfts = { &mtd_mft_eeprom_large, &mft_mft }
};

/* Final manifest for FRAM boards */
static const px4_mft_s mft_fram = {
	.nmft = 2,
	.mfts = { &mtd_mft_fram, &mft_mft }
};

/* Final manifest for NOR flash boards */
static const px4_mft_s mft_nor = {
	.nmft = 2,
	.mfts = { &mtd_mft_nor, &mft_mft }
};

static const px4_mft_s *g_manifest = &mft_eeprom_small;

const px4_mft_s *board_get_manifest(void)
{
	return g_manifest;
}

void board_set_eeprom_manifest(bool small_eeprom)
{
	g_manifest = small_eeprom ? &mft_eeprom_small : &mft_eeprom_large;
}

int board_configure_fram()
{
	/* Configure the FRAM (EEPROM already initialized) */
	int ret = px4_mtd_config(&board_mtd_config_spi_fram);

	/* Set complete manifest (EEPROM + FRAM)*/
	if (ret == OK) {
		g_manifest = &mft_fram;
	}

	return ret;
}

int board_configure_nor()
{
	/* Configure the NOR (EEPROM already initialized) */
	int ret = px4_mtd_config(&board_mtd_config_spi_nor);

	/* Set complete manifest (EEPROM + NOR)*/
	if (ret == OK) {
		g_manifest = &mft_nor;
	}

	return ret;
}
