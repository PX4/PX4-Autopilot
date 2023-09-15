/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/mmcsd.h>
#include <nuttx/fs/partition.h>

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

static void partition_handler(FAR struct partition_s *part, FAR void *arg)
{
	unsigned partition = *(int *)arg;
	char devname[] = "/dev/mmcsd0p0";

	if (partition < 10 && part->index == partition) {
		devname[sizeof(devname) - 2] = partition + 48;
		register_blockpartition(devname, 0, "/dev/mmcsd0", part->firstblock, part->nblocks);
	}
}

int mpfs_board_register_partition(unsigned partition)
{
	return parse_block_partition("/dev/mmcsd0", partition_handler, &partition);
}

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
