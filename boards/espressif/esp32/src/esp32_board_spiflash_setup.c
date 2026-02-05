/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_spiflash.c
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
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/spi.h>
#ifdef CONFIG_ESP32_SPIFLASH_NXFFS
#include <nuttx/fs/nxffs.h>
#endif
#ifdef CONFIG_BCH
#include <nuttx/drivers/drivers.h>
#endif

#include <px4_platform_common/log.h>
#define MODULE_NAME "spiflash"

#include "esp32_spiflash.h"
#include "esp32_partition.h"
// #include "esp32_board_spiflash_setup.h"

#include "board_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x)                (sizeof((x)) / sizeof((x)[0]))

#ifdef CONFIG_ESP32_OTA_PARTITION_ENCRYPT
#  define OTA_ENCRYPT true
#else
#  define OTA_ENCRYPT false
#endif

#ifdef CONFIG_ESP32_WIFI_MTD_ENCRYPT
#  define WIFI_ENCRYPT true
#else
#  define WIFI_ENCRYPT false
#endif

#ifdef CONFIG_ESP32_STORAGE_MTD_ENCRYPT
#  define STORAGE_ENCRYPT true
#else
#  define STORAGE_ENCRYPT false
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
int esp32_spiflash_init(void);
int esp32_partition_init(void);

static int init_ota_partitions(void)
{
	struct mtd_dev_s *mtd;
	char blockdev[18];
	int ret = OK;

	mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_STORAGE_MTD_OFFSET, CONFIG_ESP32_STORAGE_MTD_SIZE, false);

	ret = ftl_initialize(0, mtd);

	if (ret < 0) {
		PX4_INFO("ERROR: Failed to initialize the FTL layer: %d\n", ret);
		return ret;
	}

	snprintf(blockdev, sizeof(blockdev), "/dev/mtdblock%d", 0);

	ret = bchdev_register(blockdev, "/fs/mtd_params", false);

	if (ret < 0) {
		PX4_INFO("ERROR: bchdev_register %s failed: %d\n", "/fs/mtd_params", ret);
		return ret;
	}

	return ret;
}

/****************************************************************************
 * Name: setup_smartfs
 *
 * Description:
 *   Provide a block driver wrapper around MTD partition and mount a
 *   SMART FS over it.
 *
 * Parameters:
 *   smartn - Number used to register the mtd partition: /dev/smartx, where
 *            x = smartn.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: setup_littlefs
 *
 * Description:
 *   Register a mtd driver and mount a Little FS over it.
 *
 * Parameters:
 *   path   - Path name used to register the mtd driver.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *   priv   - Privileges
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int setup_littlefs(const char *path, struct mtd_dev_s *mtd,
			  const char *mnt_pt, int priv)
{
	int ret = OK;

	ret = register_mtddriver(path, mtd, priv, NULL);

	if (ret < 0) {
		PX4_INFO("ERROR: Failed to register MTD: %d\n", ret);
		return -ENOMEM;
	}

	// if (mnt_pt != NULL)
	//   {
	//     ret = nx_mount(path, "/mnt/lfs", "littlefs", 0, "");
	//     if (ret < 0)
	//       {
	//         ret = nx_mount(path, "/fs/lfs", "littlefs", 0, "forceformat");
	//         if (ret < 0)
	//           {
	//             PX4_INFO("ERROR: Failed to mount the FS volume: %d\n", ret);
	//             return ret;
	//           }
	//       }
	//   }

	return OK;
}

/****************************************************************************
 * Name: setup_spiffs
 *
 * Description:
 *   Register a mtd driver and mount a SPIFFS over it.
 *
 * Parameters:
 *   path   - Path name used to register the mtd driver.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *   priv   - Privileges
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int setup_spiffs(const char *path, struct mtd_dev_s *mtd,
			const char *mnt_pt, int priv)
{
	int ret = OK;

	ret = register_mtddriver(path, mtd, priv, NULL);

	if (ret < 0) {
		ferr("ERROR: Failed to register MTD: %d\n", ret);
		return -ENOMEM;
	}

	if (mnt_pt != NULL) {
		ret = nx_mount(path, mnt_pt, "spiffs", 0, NULL);

		if (ret < 0) {
			ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
			return ret;
		}
	}

	return ret;
}

/****************************************************************************
 * Name: setup_nxffs
 *
 * Description:
 *   Register a mtd driver and mount a SPIFFS over it.
 *
 * Parameters:
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPIFLASH_NXFFS
static int setup_nxffs(struct mtd_dev_s *mtd, const char *mnt_pt)
{
	int ret = OK;

	ret = nxffs_initialize(mtd);

	if (ret < 0) {
		ferr("ERROR: NXFFS init failed: %d\n", ret);
		return ret;
	}

	if (mnt_pt != NULL) {
		ret = nx_mount(NULL, mnt_pt, "nxffs", 0, NULL);

		if (ret < 0) {
			ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
			return ret;
		}
	}

	return ret;
}
#endif


/****************************************************************************
 * Name: init_storage_partition
 *
 * Description:
 *   Initialize partition that is dedicated to general use.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int init_storage_partition(void)
{
	int ret = OK;
	struct mtd_dev_s *mtd;


	mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_STORAGE_MTD_OFFSET,
					   CONFIG_ESP32_STORAGE_MTD_SIZE,
					   STORAGE_ENCRYPT);

	if (!mtd) {
		PX4_INFO("ERROR: Failed to alloc MTD partition of SPI Flash\n");
		return -ENOMEM;
	}

	const char *path = "/dev/esp32flash";
	ret = setup_littlefs(path, mtd, "/mnt/esp32", 0755);

	if (ret < 0) {
		PX4_INFO("ERROR: Failed to setup littlefs\n");
		return ret;
	}

	// const char *path = "/dev/esp32flash";
	// ret = setup_spiffs(path, mtd, "/mnt/spiffs/", 0755);
	// if (ret < 0)
	//   {
	//     ferr("ERROR: Failed to setup spiffs\n");
	//     return ret;
	//   }


	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spiflash_init
 *
 * Description:
 *   Initialize the SPI Flash and register the MTD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_spiflash_init(void)
{
	int ret = OK;

	ret = init_ota_partitions();
	PX4_INFO("ret = %d = init_ota_paritions()\n", ret);

	if (ret < 0) {
		return ret;
	}

	// ret = esp32_partition_init();
	// if (ret < 0)
	//   {
	//     syslog(LOG_ERR, "ERROR: Failed to initialize partition error=%d\n",
	//            ret);
	//   }
//
	// ret = init_storage_partition();
	// PX4_INFO("ret = %d = init_storage_paritions()\n", ret);

	// if (ret < 0)
	//   {
	//     return ret;
	//   }

	return ret;
}
