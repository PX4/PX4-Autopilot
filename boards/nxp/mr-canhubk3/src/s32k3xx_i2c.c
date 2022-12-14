/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_i2c.c
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "s32k3xx_lpi2c.h"

#include "board_config.h"

#ifdef CONFIG_S32K3XX_LPI2C

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_i2cdev_initialize
 *
 * Description:
 *   Initialize I2C driver and register /dev/i2cN devices.
 *
 ****************************************************************************/

int weak_function s32k3xx_i2cdev_initialize(void)
{
	int ret = OK;

#if defined(CONFIG_S32K3XX_LPI2C0) && defined(CONFIG_I2C_DRIVER)
	/* LPI2C0 *****************************************************************/

	/* Initialize the I2C driver for LPI2C0 */

	struct i2c_master_s *lpi2c0 = s32k3xx_i2cbus_initialize(0);

	if (lpi2c0 == NULL) {
		i2cerr("ERROR: FAILED to initialize LPI2C0\n");
		return -ENODEV;
	}

	ret = i2c_register(lpi2c0, 0);

	if (ret < 0) {
		i2cerr("ERROR: FAILED to register LPI2C0 driver\n");
		s32k3xx_i2cbus_uninitialize(lpi2c0);
		return ret;
	}

#endif /* CONFIG_S32K3XX_LPI2C0 && CONFIG_I2C_DRIVER */

#if defined(CONFIG_S32K3XX_LPI2C1) && defined(CONFIG_I2C_DRIVER)
	/* LPI2C1 *****************************************************************/

	/* Initialize the I2C driver for LPI2C1 */

	struct i2c_master_s *lpi2c1 = s32k3xx_i2cbus_initialize(1);

	if (lpi2c1 == NULL) {
		i2cerr("ERROR: FAILED to initialize LPI2C1\n");
		return -ENODEV;
	}

	ret = i2c_register(lpi2c1, 1);

	if (ret < 0) {
		i2cerr("ERROR: FAILED to register LPI2C1 driver\n");
		s32k3xx_i2cbus_uninitialize(lpi2c1);
		return ret;
	}

#endif /* CONFIG_S32K3XX_LPI2C1 && CONFIG_I2C_DRIVER */

	return ret;
}

#endif /* CONFIG_S32K3XX_LPI2C */
