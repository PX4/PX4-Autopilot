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
	.memConfig =
	{
		.tag                 = FLEXSPI_CFG_BLK_TAG,
		.version             = FLEXSPI_CFG_BLK_VERSION,
		.readSampleClkSrc    = kFlexSPIReadSampleClk_ExternalInputFromDqsPad,
		.csHoldTime          = 3,
		.csSetupTime         = 3,
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
		.serialClkFreq = kFlexSpiSerialClk_133MHz,
		.sflashA1Size  = 64ul * 1024u * 1024u,
		.dataValidTime = {16, 0},
		.busyOffset      = 0u,
		.busyBitPolarity = 0u,
		.lookupTable =
		{
			/* Read */// EEH+11H+32bit addr+20dummy cycles+ 4Bytes read data    //133Mhz 20 dummy=10+10
			[0 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xEE, CMD_DDR, FLEXSPI_8PAD, 0x11),  //0x871187ee,
			[0 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, DUMMY_DDR, FLEXSPI_8PAD, 0x0A),//0xb30a8b20,
			[0 + 2] = FLEXSPI_LUT_SEQ(DUMMY_DDR, FLEXSPI_8PAD, 0x0A, READ_DDR, FLEXSPI_8PAD, 0x04),//0xa704b30a,

			/* Read Status SPI */// SPI 05h+ status data 0X24 maybe 0X04
			[4 * 1 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x24),//0x24040405,

			/* Read Status OPI *///05H+FAH+ 4byte 00H(addr)+4Byte read
			[4 * 2 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x05, CMD_DDR, FLEXSPI_8PAD, 0xFA),//0x87fa8705,
			[4 * 2 + 1] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00),//0x87008700,
			[4 * 2 + 2] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00),//0x87008700,
			[4 * 2 + 3] = FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP_EXE, FLEXSPI_1PAD, 0x00),//0x0000a704,

			/* Write enable SPI *///06h
			[4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP_EXE, FLEXSPI_1PAD, 0x00),//0x00000406,

			/* Write enable OPI *///06h+F9H
			[4 * 4 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x06, CMD_DDR, FLEXSPI_8PAD, 0xF9),//0x87f98706,

			/* Erase sector */ //21H+DEH + 32bit address
			[4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x21, CMD_DDR, FLEXSPI_8PAD, 0xDE),//0x87de8721,
			[4 * 5 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, STOP_EXE, FLEXSPI_1PAD, 0x00),//0x00008b20,

			/*Write Configuration Register 2 =01, Enable OPI DDR mode*/ //72H +32bit address + CR20x00000000 = 0x01
			[4 * 6 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x72, CMD_SDR, FLEXSPI_1PAD, 0x00),//0x04000472,
			[4 * 6 + 1] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, CMD_SDR, FLEXSPI_1PAD, 0x00),//0x04000400,
			[4 * 6 + 2] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x00, WRITE_SDR, FLEXSPI_1PAD, 0x01),//0x20010400,

			/*block erase*/ //DCH+23H+32bit address
			[4 * 8 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xDC, CMD_DDR, FLEXSPI_8PAD, 0x23),//0x872387dc,
			[4 * 8 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, STOP_EXE, FLEXSPI_1PAD, 0x00), //0x00008b20,

			/*page program*/ //12H+EDH+32bit address+ write data 4bytes
			[4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x12, CMD_DDR, FLEXSPI_8PAD, 0xED),//0x87ed8712,
			[4 * 9 + 1] = FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, WRITE_DDR, FLEXSPI_8PAD, 0x04), //0xa3048b20,

			/* Chip Erase (CE) Sequence *///60H+9FH
			[4 * 11 + 0] = FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x60, CMD_DDR, FLEXSPI_8PAD, 0x9F),//0x879f8760,
		},
	},
	.pageSize           = 256u,
	.sectorSize         = 4u * 1024u,
	.blockSize          = 64u * 1024u,
	.isUniformBlockSize = false,
};
