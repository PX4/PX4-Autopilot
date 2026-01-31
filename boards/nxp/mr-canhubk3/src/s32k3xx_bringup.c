/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_bringup.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_S32K3XX_FLEXCAN
#  include "s32k3xx_flexcan.h"
#endif

#ifdef CONFIG_S32K3XX_ENET
#  include "s32k3xx_emac.h"
#endif

#ifdef CONFIG_S32K3XX_QSPI
#  include <stdio.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/spi/qspi.h>
#  include "s32k3xx_qspi.h"
#endif

#include <arch/board/board.h>
#include <px4_platform_common/log.h>
#include "board_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODULE_NAME "bringup"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef HAVE_MX25L
struct qspi_dev_s *g_qspi;
struct mtd_dev_s *g_mtd_fs;
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int s32k3xx_bringup(void)
{
	int ret = OK;
#if defined(CONFIG_BCH) || defined(HAVE_MX25L_LITTLEFS)
	char blockdev[32];
#  if !defined(HAVE_MX25L_LITTLEFS) && !defined(HAVE_MX25L_NXFFS)
	char chardev[32];
#  endif /* !HAVE_MX25L_LITTLEFS && !HAVE_MX25L_NXFFS */
#endif /* CONFIG_BCH || HAVE_MX25L_LITTLEFS */

#ifdef CONFIG_S32K3XX_LPSPI
	/* Initialize SPI driver */

	s32k3xx_spidev_initialize();
#endif

	s32k3xx_pinconfig(GPIO_LED_SAFETY);
	s32k3xx_pinconfig(GPIO_BTN_SAFETY);

#ifdef CONFIG_INPUT_BUTTONS
	/* Register the BUTTON driver */

	ret = btn_lower_initialize("/dev/buttons");

	if (ret < 0) {
		PX4_ERR("btn_lower_initialize() failed: %d", ret);

	} else {
		PX4_INFO("btn_lower_initialize() successful");
	}

#endif

#ifdef CONFIG_USERLED
	/* Register the LED driver */

	ret = userled_lower_initialize("/dev/userleds");

	if (ret < 0) {
		PX4_ERR("userled_lower_initialize() failed: %d", ret);

	} else {
		PX4_INFO("userled_lower_initialize() successful");
	}

#endif

#ifdef CONFIG_S32K3XX_PROGMEM
	struct mtd_dev_s *mtd;

	mtd = progmem_initialize();

	if (mtd == NULL) {
		PX4_ERR("progmem_initialize() failed");
	}

	ret = register_mtddriver("/dev/progmem0", mtd, 0755, NULL);

	if (ret != OK) {
		PX4_ERR("register_mtddriver() failed: %d", ret);

	} else {
		ret = nx_mount("/dev/progmem0", "/mnt/progmem", "littlefs", 0, NULL);

		if (ret < 0) {
			ret = nx_mount("/dev/progmem0", "/mnt/progmem", "littlefs", 0,
				       "forceformat");

			if (ret < 0) {
				PX4_ERR("progmem mount failed: %d", ret);

			} else {
				PX4_INFO("progmem forceformat");
			}
		}
	}

#endif

#ifdef HAVE_MX25L
	/* Create an instance of the S32K3XX QSPI device driver */

	g_qspi = s32k3xx_qspi_initialize(0);

	if (!g_qspi) {
		PX4_ERR("s32k3xx_qspi_initialize() failed");

	} else {
		/* Use the QSPI device instance to initialize the MX25 device */

		g_mtd_fs = mx25rxx_initialize(g_qspi, true);

		if (!g_mtd_fs) {
			PX4_ERR("mx25rxx_initialize() failed");

		} else {

#  ifdef HAVE_MX25L_LITTLEFS
			/* Configure the device with no partition support */

			snprintf(blockdev, sizeof(blockdev), "/dev/mtdqspi%d",
				 MX25L_MTD_MINOR);

			ret = register_mtddriver(blockdev, g_mtd_fs, 0755, NULL);

			if (ret != OK) {
				PX4_ERR("register_mtddriver() failed: %d", ret);

			} else {
				ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0, NULL);

				if (ret < 0) {
					ret = nx_mount(blockdev, "/mnt/qspi", "littlefs", 0,
						       "forceformat");

					if (ret < 0) {
						PX4_ERR("MX25L mount: %d", ret);

					} else {
						PX4_INFO("MX25L forceformat");
					}
				}
			}

#  elif defined(HAVE_MX25L_NXFFS)
			/* Initialize to provide NXFFS on the N25QXXX MTD interface */

			ret = nxffs_initialize(g_mtd_fs);

			if (ret < 0) {
				PX4_ERR("nxffs_initialize() failed: %d", ret);

			} else {
				PX4_INFO("nxffs_initialize() successful");

				/* Mount the file system at /mnt/qspi */

				ret = nx_mount(NULL, "/mnt/qspi", "nxffs", 0, NULL);

				if (ret < 0) {
					PX4_ERR("nx_mount() failed: %d", ret);

				} else {
					PX4_INFO("nx_mount() successful");
				}
			}

#  else /* if defined(HAVE_MX25L_CHARDEV) */
			/* Use the FTL layer to wrap the MTD driver as a block driver */

			ret = ftl_initialize(MX25L_MTD_MINOR, g_mtd_fs);

			if (ret < 0) {
				PX4_ERR("ftl_initialize() failed: %d", ret);
			}

#    ifdef CONFIG_BCH

			else {
				PX4_INFO("ftl_initialize() successful");

				/* Use the minor number to create device paths */

				snprintf(blockdev, sizeof(blockdev), "/dev/mtdblock%d",
					 MX25L_MTD_MINOR);
				snprintf(chardev, sizeof(chardev), "/dev/mtd%d",
					 MX25L_MTD_MINOR);

				/* Now create a character device on the block device */

				ret = bchdev_register(blockdev, chardev, false);

				if (ret < 0) {
					PX4_ERR("bchdev_register %s failed: %d", chardev, ret);

				} else {
					PX4_INFO("bchdev_register %s successful", chardev);
				}
			}

#    endif /* CONFIG_BCH */
#  endif
		}
	}

#endif

#ifdef CONFIG_S32K3XX_SELFTEST
	s32k3xx_selftest();
#endif /* CONFIG_S32K3XX_SELFTEST */

#ifdef CONFIG_NETDEV_LATEINIT
#  ifdef CONFIG_S32K3XX_ENET
	s32k3xx_netinitialize(0);
#  endif /* CONFIG_S32K3XX_ENET */
#  ifdef CONFIG_S32K3XX_FLEXCAN0
	s32k3xx_caninitialize(0);
#  endif /* CONFIG_S32K3XX_FLEXCAN0 */
#  ifdef CONFIG_S32K3XX_FLEXCAN1
	s32k3xx_caninitialize(1);
#  endif /* CONFIG_S32K3XX_FLEXCAN1 */
#  ifdef CONFIG_S32K3XX_FLEXCAN2
	s32k3xx_caninitialize(2);
#  endif /* CONFIG_S32K3XX_FLEXCAN2 */
#  ifdef CONFIG_S32K3XX_FLEXCAN3
	s32k3xx_caninitialize(3);
#  endif /* CONFIG_S32K3XX_FLEXCAN3 */
#  ifdef CONFIG_S32K3XX_FLEXCAN4
	s32k3xx_caninitialize(4);
#    ifdef CONFIG_S32K3XX_TJA1153
	s32k3xx_tja1153_initialize(4);
#    endif /* CONFIG_S32K3XX_TJA1153 */
#  endif /* CONFIG_S32K3XX_FLEXCAN4 */
#  ifdef CONFIG_S32K3XX_FLEXCAN5
	s32k3xx_caninitialize(5);
#    ifdef CONFIG_S32K3XX_TJA1153
	s32k3xx_tja1153_initialize(5);
#    endif /* CONFIG_S32K3XX_TJA1153 */
#  endif /* CONFIG_S32K3XX_FLEXCAN5 */
#endif /* CONFIG_NETDEV_LATEINIT */

	board_hardfault_init(2, true);

	return ret;
}
