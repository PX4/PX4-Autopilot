/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_flexspi_nor_flash.c
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


locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config = {
	.mem_config =
	{
		.tag                    = FLEXSPI_CFG_BLK_TAG,
		.version                = FLEXSPI_CFG_BLK_VERSION,
		.read_sample_clksrc     = FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD,
		.cs_hold_time           = 3u,
		.cs_setup_time          = 3u,
		/* Enable DDR mode, Wordaddassable, Safe configuration, Differential clock */
		.controller_misc_option = 0x10,
		.device_type            = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
		.sflash_pad_type        = SERIAL_FLASH_4PADS,
		.serial_clk_freq        = FLEXSPI_SERIAL_CLKFREQ_133MHz,
		.sflash_a1size          = 16u * 1024u * 1024u,
		.lookup_table           =
		{
			/* LUTs */
			// Read LUTs
			[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
			[1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),

			// Read Status LUTs
			[4 * 1 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),

			// Write Enable LUTs
			[4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x0),

			// Erase Sector LUTs
			[4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18),

			// Erase Block LUTs
			[4 * 8 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD8, RADDR_SDR, FLEXSPI_1PAD, 0x18),

			// Pape Program LUTs
			[4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18),
			[4 * 9 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0),

			// Erase Chip LUTs
			[4 * 11 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x0),
		},
	},

	.page_size                = 256u,
	.sector_size              = 4u * 1024u,
	.ipcmd_serial_clkfreq     = 0x1,
	.blocksize                = 64u * 1024u,
	.is_uniform_blocksize     = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
