/****************************************************************************
 * boards/px4/fmu-v6xrt/src/imxrt_flexspi_storage.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include <px4_platform_common/px4_mtd.h>
#include "px4_log.h"

#include "imxrt_flexspi.h"
#include "board_config.h"
#include "hardware/imxrt_pinmux.h"

#ifdef CONFIG_IMXRT_FLEXSPI

/* Used sectors must be multiple of the flash block size
 * i.e. W25Q32JV has a block size of 64KB
*/

#define NOR_USED_SECTORS  (0x20U)   /* 32 * 4KB = 128KB */
#define NOR_TOTAL_SECTORS (0x0800U)
#define NOR_PAGE_SIZE     (0x0100U) /* 256 bytes */
#define NOR_SECTOR_SIZE   (0x1000U) /* 4KB */
#define NOR_START_SECTOR  (NOR_TOTAL_SECTORS - NOR_USED_SECTORS)
#define NOR_START_PAGE    ((NOR_START_SECTOR * NOR_SECTOR_SIZE) / NOR_PAGE_SIZE)
#define NOR_STORAGE_ADDR  (IMXRT_FLEXCIPHER_BASE + NOR_START_SECTOR * NOR_SECTOR_SIZE)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

enum {
	/* SPI instructions */

	READ_FAST = 0,
	READ_STATUS_REG = 1,
	WRITE_STATUS_REG = 3,
	WRITE_ENABLE = 4,
	SECTOR_ERASE_4K = 5,
	READ_FAST_QUAD_OUTPUT = 6,
	PAGE_PROGRAM_QUAD_INPUT = 7,
	PAGE_PROGRAM = 9,
	CHIP_ERASE = 11,
};

static const uint32_t g_flexspi_nor_lut[][4] = {
	[READ_FAST] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0xeb,
				FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_4PAD, 0x18),
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_4PAD, 0x06,
				FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_4PAD, 0x04),
	},

	[READ_STATUS_REG] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x05,
				FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_1PAD, 0x04),
	},

	[WRITE_STATUS_REG] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x01,
				FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x04),
	},

	[WRITE_ENABLE] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x06,
				FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
	},

	[SECTOR_ERASE_4K] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x20,
				FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
	},

	[CHIP_ERASE] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0xc7,
				FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
	},

	[PAGE_PROGRAM] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x02,
				FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_1PAD, 0x04,
				FLEXSPI_COMMAND_STOP,  FLEXSPI_1PAD, 0x0),
	},

	[READ_FAST_QUAD_OUTPUT] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x6b,
				FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_DUMMY_SDR, FLEXSPI_4PAD, 0x08,
				FLEXSPI_COMMAND_READ_SDR,  FLEXSPI_4PAD, 0x04),
	},

	[PAGE_PROGRAM_QUAD_INPUT] =
	{
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_SDR,       FLEXSPI_1PAD, 0x32,
				FLEXSPI_COMMAND_RADDR_SDR, FLEXSPI_1PAD, 0x18),
		FLEXSPI_LUT_SEQ(FLEXSPI_COMMAND_WRITE_SDR, FLEXSPI_4PAD, 0x04,
				FLEXSPI_COMMAND_STOP,      FLEXSPI_1PAD, 0),
	},

};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* FlexSPI NOR device private data */

struct imxrt_flexspi_storage_dev_s {
	struct mtd_dev_s mtd;
	struct flexspi_dev_s *flexspi;   /* Saved FlexSPI interface instance */
	uint8_t *ahb_base;
	enum flexspi_port_e port;
	struct flexspi_device_config_s *config;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int imxrt_flexspi_storage_erase(struct mtd_dev_s *dev,
				       off_t startblock,
				       size_t nblocks);
static ssize_t imxrt_flexspi_storage_read(struct mtd_dev_s *dev,
		off_t offset,
		size_t nbytes,
		uint8_t *buffer);
static ssize_t imxrt_flexspi_storage_bread(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		uint8_t *buffer);
static ssize_t imxrt_flexspi_storage_bwrite(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		const uint8_t *buffer);
static int imxrt_flexspi_storage_ioctl(struct mtd_dev_s *dev,
				       int cmd,
				       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct flexspi_device_config_s g_flexspi_device_config = {
	.flexspi_root_clk = 4000000,
	.is_sck2_enabled = 0,
	.flash_size = NOR_USED_SECTORS * NOR_SECTOR_SIZE / 4,
	.cs_interval_unit = FLEXSPI_CS_INTERVAL_UNIT1_SCK_CYCLE,
	.cs_interval = 0,
	.cs_hold_time = 3,
	.cs_setup_time = 3,
	.data_valid_time = 0,
	.columnspace = 0,
	.enable_word_address = 0,
	.awr_seq_index = 0,
	.awr_seq_number = 0,
	.ard_seq_index = READ_FAST,
	.ard_seq_number = 1,
	.ahb_write_wait_unit = FLEXSPI_AHB_WRITE_WAIT_UNIT2_AHB_CYCLE,
	.ahb_write_wait_interval = 0,
	.rx_sample_clock = FLEXSPI_READ_SAMPLE_CLK_LOOPBACK_FROM_DQS_PAD,
};

static struct imxrt_flexspi_storage_dev_s g_flexspi_nor = {
	.mtd =
	{
		.erase  = imxrt_flexspi_storage_erase,
		.bread  = imxrt_flexspi_storage_bread,
		.bwrite = imxrt_flexspi_storage_bwrite,
		.read   = imxrt_flexspi_storage_read,
		.ioctl  = imxrt_flexspi_storage_ioctl,
#ifdef CONFIG_MTD_BYTE_WRITE
		.write  = NULL,
#endif
		.name   = "imxrt_flexspi_storage"
	},
	.flexspi = (void *)0,
	.ahb_base = (uint8_t *) NOR_STORAGE_ADDR,
	.port = FLEXSPI_PORT_A1,
	.config = &g_flexspi_device_config
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int imxrt_flexspi_storage_read_status(
	const struct imxrt_flexspi_storage_dev_s *dev,
	uint32_t *status)
{
	int stat;

	struct flexspi_transfer_s transfer = {
		.device_address = 0,
		.port = dev->port,
		.cmd_type = FLEXSPI_READ,
		.seq_number = 1,
		.seq_index = READ_STATUS_REG,
		.data = status,
		.data_size = 1,
	};

	stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);

	if (stat != 0) {
		return -EIO;
	}

	return 0;
}

static int imxrt_flexspi_storage_write_enable(
	const struct imxrt_flexspi_storage_dev_s *dev)
{
	int stat;

	struct flexspi_transfer_s transfer = {
		.device_address = 0,
		.port = dev->port,
		.cmd_type = FLEXSPI_COMMAND,
		.seq_number = 1,
		.seq_index = WRITE_ENABLE,
		.data = NULL,
		.data_size = 0,
	};

	stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);

	if (stat != 0) {
		return -EIO;
	}

	return 0;
}

static int imxrt_flexspi_storage_erase_sector(
	const struct imxrt_flexspi_storage_dev_s *dev,
	off_t offset)
{
	int stat;
	struct flexspi_transfer_s transfer = {
		.device_address = offset,
		.port = dev->port,
		.cmd_type = FLEXSPI_COMMAND,
		.seq_number = 1,
		.seq_index = SECTOR_ERASE_4K,
		.data = NULL,
		.data_size = 0,
	};

	stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);

	if (stat != 0) {
		return -EIO;
	}


	return 0;
}

static int imxrt_flexspi_storage_erase_chip(
	const struct imxrt_flexspi_storage_dev_s *dev)
{
	/* We can't erase the chip we're executing from */
	return -EINVAL;
}

static int imxrt_flexspi_storage_page_program(
	const struct imxrt_flexspi_storage_dev_s *dev,
	off_t offset,
	const void *buffer,
	size_t len)
{
	int stat;

	struct flexspi_transfer_s transfer = {
		.device_address = offset,
		.port = dev->port,
		.cmd_type = FLEXSPI_WRITE,
		.seq_number = 1,
		.seq_index = PAGE_PROGRAM_QUAD_INPUT,
		.data = (uint32_t *) buffer,
		.data_size = len,
	};

	stat = FLEXSPI_TRANSFER(dev->flexspi, &transfer);

	if (stat != 0) {
		return -EIO;
	}

	return 0;
}

static int imxrt_flexspi_storage_wait_bus_busy(
	const struct imxrt_flexspi_storage_dev_s *dev)
{
	uint32_t status = 0;
	int ret;

	do {
		ret = imxrt_flexspi_storage_read_status(dev, &status);

		if (ret) {
			return ret;
		}
	} while (status & 1);

	return 0;
}

static ssize_t imxrt_flexspi_storage_read(struct mtd_dev_s *dev,
		off_t offset,
		size_t nbytes,
		uint8_t *buffer)
{
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	uint8_t *src;

	finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

	if (priv->port >= FLEXSPI_PORT_COUNT) {
		return -EIO;
	}

	src = priv->ahb_base + offset;

	memcpy(buffer, src, nbytes);

	finfo("return nbytes: %d\n", (int)nbytes);
	return (ssize_t)nbytes;
}

static ssize_t imxrt_flexspi_storage_bread(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		uint8_t *buffer)
{
	ssize_t nbytes;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* On this device, we can handle the block read just like the byte-oriented
	 * read
	 */

	nbytes = imxrt_flexspi_storage_read(dev, startblock * NOR_PAGE_SIZE,
					    nblocks * NOR_PAGE_SIZE, buffer);

	if (nbytes > 0) {
		nbytes /= NOR_PAGE_SIZE;
	}

	return nbytes;
}

static ssize_t imxrt_flexspi_storage_bwrite(struct mtd_dev_s *dev,
		off_t startblock,
		size_t nblocks,
		const uint8_t *buffer)
{
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	size_t len = nblocks * NOR_PAGE_SIZE;
	off_t offset = (startblock + NOR_START_PAGE) * NOR_PAGE_SIZE;
	uint8_t *src = (uint8_t *) buffer;
#ifdef CONFIG_ARMV7M_DCACHE
	uint8_t *dst = priv->ahb_base + startblock * NOR_PAGE_SIZE;
#endif
	int i;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	FLEXSPI_CONFIGURE_PREFETCH(priv->flexspi, false);

	irqstate_t flags = enter_critical_section();

	while (len) {
		i = MIN(NOR_PAGE_SIZE, len);
		imxrt_flexspi_storage_write_enable(priv);
		imxrt_flexspi_storage_page_program(priv, offset, src, i);
		imxrt_flexspi_storage_wait_bus_busy(priv);
		offset += i;
		src += i;
		len -= i;
	}

	FLEXSPI_CONFIGURE_PREFETCH(priv->flexspi, true);

	leave_critical_section(flags);

#ifdef CONFIG_ARMV7M_DCACHE
	up_invalidate_dcache((uintptr_t)dst,
			     (uintptr_t)dst + nblocks * NOR_PAGE_SIZE);
#endif

	return nblocks;
}

static int imxrt_flexspi_storage_erase(struct mtd_dev_s *dev,
				       off_t startblock,
				       size_t nblocks)
{
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	size_t blocksleft = nblocks;
#ifdef CONFIG_ARMV7M_DCACHE
	uint8_t *dst = priv->ahb_base + startblock * NOR_SECTOR_SIZE;
#endif

	startblock += NOR_START_SECTOR;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	FLEXSPI_CONFIGURE_PREFETCH(priv->flexspi, false);

	irqstate_t flags = enter_critical_section();

	while (blocksleft-- > 0) {
		/* Erase each sector */

		imxrt_flexspi_storage_write_enable(priv);
		imxrt_flexspi_storage_erase_sector(priv, startblock * NOR_SECTOR_SIZE);
		imxrt_flexspi_storage_wait_bus_busy(priv);
		startblock++;
	}

	FLEXSPI_CONFIGURE_PREFETCH(priv->flexspi, true);

	leave_critical_section(flags);

#ifdef CONFIG_ARMV7M_DCACHE
	up_invalidate_dcache((uintptr_t)dst,
			     (uintptr_t)dst + nblocks * NOR_SECTOR_SIZE);
#endif

	return (int)nblocks;
}

static int imxrt_flexspi_storage_ioctl(struct mtd_dev_s *dev,
				       int cmd,
				       unsigned long arg)
{
	struct imxrt_flexspi_storage_dev_s *priv =
		(struct imxrt_flexspi_storage_dev_s *)dev;
	int ret = -EINVAL; /* Assume good command with bad parameters */

	finfo("cmd: %d\n", cmd);

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			struct mtd_geometry_s *geo =
				(struct mtd_geometry_s *)((uintptr_t)arg);

			if (geo) {
				/* Populate the geometry structure with information need to
				 * know the capacity and how to access the device.
				 *
				 * NOTE:
				 * that the device is treated as though it where just an array
				 * of fixed size blocks.  That is most likely not true, but the
				 * client will expect the device logic to do whatever is
				 * necessary to make it appear so.
				 */

				geo->blocksize    = (NOR_PAGE_SIZE);
				geo->erasesize    = (NOR_SECTOR_SIZE);
				geo->neraseblocks = (NOR_USED_SECTORS);

				ret               = OK;

				finfo("blocksize: %lu erasesize: %lu neraseblocks: %lu\n",
				      geo->blocksize, geo->erasesize, geo->neraseblocks);
			}
		}
		break;

	case BIOC_PARTINFO: {
			struct partition_info_s *info =
				(struct partition_info_s *)arg;

			if (info != NULL) {
				info->numsectors  = (NOR_USED_SECTORS * NOR_SECTOR_SIZE) / NOR_PAGE_SIZE;
				info->sectorsize  = NOR_PAGE_SIZE;
				info->startsector = 0;
				info->parent[0]   = '\0';
				ret               = OK;
			}
		}
		break;

	case MTDIOC_BULKERASE: {
			/* Erase the entire device */

			imxrt_flexspi_storage_write_enable(priv);
			imxrt_flexspi_storage_erase_chip(priv);
			imxrt_flexspi_storage_wait_bus_busy(priv);
			ret               = OK;
		}
		break;

	default:
		ret = -ENOTTY; /* Bad/unsupported command */
		break;
	}

	finfo("return %d\n", ret);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: imxrt_flexspi_storage_initialize
 *
 * Description:
 *  This function is called by board-bringup logic to configure the
 *  flash device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int imxrt_flexspi_storage_initialize(void)
{
	struct mtd_dev_s *mtd_dev = &g_flexspi_nor.mtd;
	int ret = -ENODEV;

	/* Select FlexSPI1 */

	g_flexspi_nor.flexspi = imxrt_flexspi_initialize(0);

	if (g_flexspi_nor.flexspi) {
		ret = OK;


		FLEXSPI_UPDATE_LUT(g_flexspi_nor.flexspi,
				   0,
				   (const uint32_t *)g_flexspi_nor_lut,
				   sizeof(g_flexspi_nor_lut) / 4);
	}

	/* Register the MTD driver so that it can be accessed from the
	 * VFS.
	 */

	ret = register_mtddriver("/dev/nor", mtd_dev, 0755, NULL);

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
		       ret);
	}

#ifdef CONFIG_FS_LITTLEFS

	/* Mount the LittleFS file system */

	ret = nx_mount("/dev/nor", "/fs/nor", "littlefs", 0,
		       "autoformat");

	if (ret < 0) {
		syslog(LOG_ERR,
		       "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
		       ret);
	}

#endif

	return ret;
}
#endif /* CONFIG_IMXRT_FLEXSPI */
