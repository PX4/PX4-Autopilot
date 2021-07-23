/****************************************************************************
 * boards/risc-v/mpfs/icicle/src/mpfs_emmcsd.c
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

#include <debug.h>
#include <errno.h>
#include <nuttx/mmcsd.h>

#include "mpfs_emmcsd.h"
#include "board_config.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_emmcsd_init
 *
 * Description:
 *   Configure the eMMCSD driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int mpfs_board_emmcsd_init(void)
{
	int ret;

	/* Mount the SDIO-based MMC/SD block driver */

	/* First, get an instance of the SDIO interface */

	finfo("Initializing SDIO slot %d\n", SDIO_SLOTNO);

	g_sdio_dev = sdio_initialize(SDIO_SLOTNO);

	if (!g_sdio_dev) {
		ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */

	finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);

	ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);

	if (ret != OK) {
		ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	sdio_mediachange(g_sdio_dev, true);

	return OK;
}
