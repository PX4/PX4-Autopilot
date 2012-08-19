/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/**
 * @file eeprom.c
 *
 * EEPROM service and utility app.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <sys/mount.h>

#include <nuttx/i2c.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#ifndef PX4_I2C_BUS_ONBOARD
#  error PX4_I2C_BUS_ONBOARD not defined, cannot locate onboard EEPROM
#endif
#if !defined(CONFIG_MTD_AT24XX) || !CONFIG_MTD_AT24XX
#  error CONFIG_MTD_AT24XX not defined, no supported EEPROM available
#endif

__EXPORT int eeprom_main(int argc, char *argv[]);

static void	eeprom_start(void);


int eeprom_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strcmp(argv[1], "start"))
			eeprom_start();
	}

	errx(1, "expected a command, try 'start'");
}


static void
eeprom_start(void)
{
	static bool started = false;
	int ret;

	if (started)
		errx(1, "EEPROM service already started");

	/* find the right I2C */
	struct i2c_s *i2c = up_i2cinitialize(PX4_I2C_BUS_ONBOARD);
	if (i2c == NULL)
		errx(1, "failed to locate I2C bus");

	/* start the MTD driver */
	struct mtd_dev_s *mtd = at24c_initialize(i2c);
	if (mtd == NULL)
		errx(1, "failed to initialize EEPROM driver");

	/* start NXFFS */
	ret = nxffs_initialize(mtd);
	if (ret < 0)
		err(1, "failed to initialize NXFFS");

	/* mount the EEPROM */
	ret = mount(NULL, "/eeprom", "nxffs", 0, NULL);
	if (ret < 0)
		err(1, "failed to mount EEPROM");

	errx(0, "mounted EEPROM at /eeprom");
}
