/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_flexspi_nor_flash.c
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

#include "imxrt_flexspi_nor_flash.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

enum {
	/* SPI instructions */

	READ_FAST = 0,
	READ_STATUS_REG = 1,
	WRITE_ENABLE = 3,
	SECTOR_ERASE_4K = 5,
	READ_FAST_QUAD_OUTPUT = 6,
	PAGE_PROGRAM_QUAD_INPUT = 7,
	ERASE_BLOCK = 8,
	PAGE_PROGRAM = 9,
	CHIP_ERASE = 11,
};

locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config = {
	.mem_config =
	{
		.tag                    = FLEXSPI_CFG_BLK_TAG,
		.version                = FLEXSPI_CFG_BLK_VERSION,
		.read_sample_clksrc     =  FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD,
		.cs_hold_time           = 1u,
		.cs_setup_time          = 1u,
		.column_address_width   = 0u,
		.device_type            = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
		.sflash_pad_type        = SERIAL_FLASH_4PADS,
		.serial_clk_freq        = FLEXSPI_SERIAL_CLKFREQ_133MHz,
		.sflash_a1size          = 8u * 1024u * 1024u,
		.lookup_table           =
		{
			[4 * READ_FAST] =       FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0xeb,
								RADDR_SDR, FLEXSPI_4PAD, 0x18),
			[4 * READ_FAST + 1] =   FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06,
								READ_SDR,  FLEXSPI_4PAD, 0x04),

			[4 * READ_STATUS_REG] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05,
								READ_SDR,  FLEXSPI_1PAD, 0x04),

			[4 * WRITE_ENABLE] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06,
							     STOP, FLEXSPI_1PAD, 0),

			[4 * SECTOR_ERASE_4K] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20,
								RADDR_SDR, FLEXSPI_1PAD, 0x18),

			[4 * CHIP_ERASE] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60,
							   STOP, FLEXSPI_1PAD, 0),

			[4 * ERASE_BLOCK] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xd8,
							    RADDR_SDR, FLEXSPI_1PAD, 0x18),

			[4 * PAGE_PROGRAM] =     FLEXSPI_LUT_SEQ(CMD_SDR,   FLEXSPI_1PAD, 0x02,
					RADDR_SDR, FLEXSPI_1PAD, 0x18),
			[4 * PAGE_PROGRAM + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04,
					STOP,      FLEXSPI_1PAD, 0x0),

			[4 * READ_FAST_QUAD_OUTPUT] =     FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x6b,
					RADDR_SDR, FLEXSPI_1PAD, 0x18),
			[4 * READ_FAST_QUAD_OUTPUT + 1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x08,
					READ_SDR,  FLEXSPI_4PAD, 0x04),

			[4 * PAGE_PROGRAM_QUAD_INPUT] =     FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32,
					RADDR_SDR, FLEXSPI_1PAD, 0x18),
			[4 * PAGE_PROGRAM_QUAD_INPUT + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_4PAD, 0x04,
					STOP,      FLEXSPI_1PAD, 0),

		},
	},

	.page_size = 256u,
	.sector_size = 4u * 1024u,
	.blocksize = 64u * 1024u,
	.is_uniform_blocksize = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
