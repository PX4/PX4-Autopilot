/****************************************************************************
 * boards/px4/fmu-v6xrt/src/imxrt_flexspi_nor_flash.c
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

#include <px4_arch/imxrt_flexspi_nor_flash.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

locate_data(".boot_hdr.conf")
const struct flexspi_nor_config_s g_flash_config = {
	.memConfig =
	{
#if !defined(CONFIG_BOARD_BOOTLOADER_INVALID_FCB)
		.tag                 = FLEXSPI_CFG_BLK_TAG,
#else
		.tag                 = 0xffffffffL,
#endif
		.version             = FLEXSPI_CFG_BLK_VERSION,
		.readSampleClkSrc    = kFlexSPIReadSampleClk_LoopbackInternally,
		.csHoldTime          = 1,
		.csSetupTime         = 1,
		.deviceModeCfgEnable = 1,
		.deviceModeType      = kDeviceConfigCmdType_Generic,
		.waitTimeCfgCommands = 1,
		.controllerMiscOption =
		(1u << kFlexSpiMiscOffset_SafeConfigFreqEnable),
		.deviceType    = kFlexSpiDeviceType_SerialNOR,
		.sflashPadType = kSerialFlash_1Pad,
		.serialClkFreq = kFlexSpiSerialClk_30MHz,
		.sflashA1Size  = 64ul * 1024u * 1024u,
		.dataValidTime =
		{
			[0] = {.time_100ps = 0},
		},
		.busyOffset      = 0u,
		.busyBitPolarity = 0u,
		.lookupTable =
		{
			/* Read Dedicated 3Byte Address Read(0x03), 24bit address */
			[0 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03, RADDR_SDR, FLEXSPI_1PAD, 0x18),    //0x871187ee,
			[0 + 1] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP_EXE, FLEXSPI_1PAD, 0),//0xb3048b20
		},
	},
	.pageSize           = 256u,
	.sectorSize         = 4u * 1024u,
	.blockSize          = 64u * 1024u,
	.isUniformBlockSize = false,
	.ipcmdSerialClkFreq = 1,
	.serialNorType = 2,
	.reserve2[0] = 0x7008200,
};

const struct flexspi_nor_config_s g_flash_fast_config = {
	.memConfig =
	{
		.tag                 = FLEXSPI_CFG_BLK_TAG,
		.version             = FLEXSPI_CFG_BLK_VERSION,
		.readSampleClkSrc    = kFlexSPIReadSampleClk_ExternalInputFromDqsPad,
		.csHoldTime          = 1,
		.csSetupTime         = 1,
		.deviceModeCfgEnable = 1,
		.deviceModeType      = kDeviceConfigCmdType_Spi2Xpi,
		.waitTimeCfgCommands = 1,
		.deviceModeSeq =
		{
			.seqNum   = 1,
			.seqId    = 6, /* See Lookup table for more details */
			.reserved = 0,
		},
		.deviceModeArg = 2, /* Enable OPI DDR mode */
		.controllerMiscOption =
		(1u << kFlexSpiMiscOffset_SafeConfigFreqEnable) | (1u << kFlexSpiMiscOffset_DdrModeEnable),
		.deviceType    = kFlexSpiDeviceType_SerialNOR,
		.sflashPadType = kSerialFlash_8Pads,
		.serialClkFreq = kFlexSpiSerialClk_200MHz,
		.sflashA1Size  = 64ul * 1024u * 1024u,
		.dataValidTime =
		{
			[0] = {.time_100ps = 0},
		},
		.busyOffset      = 0u,
		.busyBitPolarity = 0u,
		.lookupTable =
		{
			/* Read */// EEH+11H+32bit addr+20dummy cycles+ 4Bytes read data
			/* Macronix manual says 20 dummy cycles @ 200Mhz, FlexSPI peripheral Operand value needs to be 2N in DDR mode hence 0x28 */
			[0 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xEE, CMD_DDR, FLEXSPI_8PAD, 0x11),    //0x871187ee,
			[0 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, DUMMY_DDR, FLEXSPI_8PAD, 0x28),//0xb3288b20,
			[0 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP_EXE, FLEXSPI_1PAD, 0x00), //0xa704,

			/* Read status */
			[4 * 2 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x05, CMD_DDR, FLEXSPI_8PAD, 0xfa),
			[4 * 2 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, DUMMY_DDR, FLEXSPI_8PAD, 0x04),
			[4 * 2 + 2] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP_EXE, FLEXSPI_1PAD, 0x00),

			/* Write enable SPI *///06h
			[4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP_EXE, FLEXSPI_1PAD, 0x00),//0x00000406,

			/* Write enable OPI SPI *///06h
			[4 * 4 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x06, CMD_DDR, FLEXSPI_8PAD, 0xF9),

			/* Erase sector */
			[4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x21, CMD_DDR, FLEXSPI_8PAD, 0xDE),
			[4 * 5 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, STOP_EXE, FLEXSPI_1PAD, 0x00),

			/*Write Configuration Register 2 =01, Enable OPI DDR mode*/ //72H +32bit address + CR20x00000000 = 0x01
			[4 * 6 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72, CMD_SDR, FLEXSPI_1PAD, 0x00),//0x04000472,
			[4 * 6 + 1] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x00),//0x04000400,
			[4 * 6 + 2] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, WRITE_SDR, FLEXSPI_1PAD, 0x01),//0x20010400,

			/*Page program*/
			[4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x12, CMD_DDR, FLEXSPI_8PAD, 0xED),//0x87ed8712,
			[4 * 9 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, WRITE_DDR, FLEXSPI_8PAD, 0x04),//0xa3048b20,
		},
	},
	.pageSize           = 256u,
	.sectorSize         = 4u * 1024u,
	.blockSize          = 64u * 1024u,
	.isUniformBlockSize = false,
	.ipcmdSerialClkFreq = 1,
	.serialNorType = 2,
	.reserve2[0] = 0x7008200,
};
