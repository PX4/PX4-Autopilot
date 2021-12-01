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
#include <nuttx/mtd/mtd.h>

#include "board_config.h"
#include "hw_config.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_spinor_init
 *
 * Description:
 *   Configure the SPI NOR flash driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int mpfs_board_spinor_init(struct spi_dev_s *spinor)
{
	struct mtd_dev_s *mtd, *mtd_p0, *mtd_p1;

	const char *block0 = "/dev/mtdblock0";
	const char *block1 = "/dev/mtdblock1";
	struct mtd_geometry_s geo;
	int ret;

	if (!spinor) {
		syslog(LOG_ERR, "EROR: FAILED to initialize SPI port 1\n");
		return -ENODEV;
	}

	mtd = m25p_initialize(spinor);

	if (!mtd) {
		syslog(LOG_ERR, "ERROR: Failed to bind SPI port 1 to the SPI NOR driver\n");
		return -ENODEV;
	}

	// Find the MTD geometry and calculate partition sizes
	if (mtd->ioctl(mtd, MTDIOC_GEOMETRY,
		       (unsigned long)((uintptr_t)&geo))) {
		_alert("ERROR: MTD geometry unknown\n");
		return -ENODEV;
	}

	unsigned all_pages = (geo.neraseblocks * geo.erasesize) /  geo.blocksize;
	unsigned boot_pages = BOARD_FLASH_SIZE / geo.blocksize;

	// Allocate boot partition
	mtd_p0 = mtd_partition(mtd, 0, boot_pages);

	if (!mtd_p0) {
		syslog(LOG_ERR, "ERROR: Failed to create boot partition\n");
		return -ENODEV;
	}

	// Allocate all the rest for lfs partition
	mtd_p1 = mtd_partition(mtd, boot_pages, all_pages - boot_pages);

	if (!mtd_p1) {
		syslog(LOG_ERR, "ERROR: Failed to create lfs partition\n");
		return -ENODEV;
	}

	ret = register_mtddriver(block0, mtd_p0, 0777, NULL);

	if (ret != 0) {
		syslog(LOG_ERR, "ERROR: Failed to register MTD driver for boot partition: %d\n", ret);
		return ret;
	}

	ret = register_mtddriver(block1, mtd_p1, 0777, NULL);

	if (ret != 0) {
		syslog(LOG_ERR, "ERROR: Failed to register MTD driver for lfs partition: %d\n", ret);
		return ret;
	}

	return ret;
}
