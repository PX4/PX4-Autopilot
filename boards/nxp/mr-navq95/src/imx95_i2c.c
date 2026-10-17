/****************************************************************************
 * boards/nxp/mr-navq95/src/imx95_i2c.c
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
#include <sys/types.h>

#include <nuttx/i2c/i2c_master.h>

#include "imx9_lpi2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int imx95_i2c_initialize(void)
{
	int ret = OK;

#ifdef CONFIG_IMX9_LPI2C6
	struct i2c_master_s *i2c;

	i2c = imx9_i2cbus_initialize(6);

	if (i2c == NULL) {
		i2cerr("ERROR: Failed to init I2C6 interface\n");
		return -ENODEV;
	}

#ifdef CONFIG_I2C_DRIVER
	ret = i2c_register(i2c, 0);

	if (ret < 0) {
		i2cerr("ERROR: Failed to register I2C6 driver: %d\n", ret);
		imx9_i2cbus_uninitialize(i2c);
		return ret;
	}

#endif

#endif
	return ret;
}
