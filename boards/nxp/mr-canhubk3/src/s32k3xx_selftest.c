/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_selftest.c
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
 *  Unless required by applicable law or agreed to in writing, software
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
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include "s32k3xx_lpi2c.h"
#include "s32k3xx_pin.h"

#include <arch/board/board.h>
#include "board_config.h"

#ifdef CONFIG_S32K3XX_SELFTEST

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPI2C
static int s32k3xx_selftest_se050(void);
#endif /* CONFIG_S32K3XX_LPI2C */
#ifdef CONFIG_S32K3XX_FLEXCAN
static int s32k3xx_selftest_can(void);
#  if !defined(CONFIG_S32K3XX_TJA1153)
static int s32k3xx_selftest_sct(void);
#  endif /* !CONFIG_S32K3XX_TJA1153 */
#endif /* CONFIG_S32K3XX_FLEXCAN */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPI2C
/****************************************************************************
 * Name: s32k3xx_selftest_se050
 *
 * Description:
 *   Basic check to see if the SE050 is alive by having it ACK a I2C write.
 *
 ****************************************************************************/

static int s32k3xx_selftest_se050(void)
{
	struct i2c_master_s *lpi2c1;
	struct i2c_msg_s se050_msg;
	uint8_t buf = 0;
	int ret = 0;
	bool error = false;

#if !defined(CONFIG_S32K3XX_LPI2C1)
#  error CONFIG_S32K3XX_LPI2C1 needs to be enabled to perform SE050 self-test
#endif

	/* Initialize LPI2C1 to which the SE050 is connected */

	lpi2c1 = s32k3xx_i2cbus_initialize(1);

	if (lpi2c1 != NULL) {
		_info("s32k3xx_i2cbus_initialize() successful\n");

	} else {
		error = true;
		_err("s32k3xx_i2cbus_initialize() failed\n");

		return -1; /* Return immediately, no cleanup needed */
	}

	/* Verify SE050 by checking for ACK on I2C write */

	se050_msg.frequency = I2C_SPEED_STANDARD;
	se050_msg.addr      = 0x48;
	se050_msg.flags     = 0;
	se050_msg.buffer    = &buf;
	se050_msg.length    = 1;

	ret = I2C_TRANSFER(lpi2c1, &se050_msg, 1);

	if (ret == 0) {
		_info("SE050 ACK successful\n");

	} else {
		error = true;
		_err("SE050 ACK failed: %d\n", ret);

		/* Don't return yet, we still need to cleanup */
	}

	/* Let the LPI2C driver know we won't be using it anymore */

	ret = s32k3xx_i2cbus_uninitialize(lpi2c1);

	if (ret == 0) {
		_info("s32k3xx_i2cbus_uninitialize() successful\n");

		/* Return error if we had any earlier, otherwise return result of
		 * s32k3xx_i2cbus_uninitialize()
		 */

		return (error ? -1 : ret);

	} else {
		error = true;
		_err("s32k3xx_i2cbus_uninitialize() failed: %d\n", ret);

		return -1;
	}
}
#endif /* CONFIG_S32K3XX_LPI2C */

#ifdef CONFIG_S32K3XX_FLEXCAN
/****************************************************************************
 * Name: s32k3xx_selftest_can
 *
 * Description:
 *   Basic check of the local failure flags of the CAN transceivers (0-3).
 *
 ****************************************************************************/

static int s32k3xx_selftest_can(void)
{
	uint32_t errn_pins[4] = {
		PIN_CAN0_ERRN,
		PIN_CAN1_ERRN,
		PIN_CAN2_ERRN,
		PIN_CAN3_ERRN
	};

	uint32_t stbn_pins[4] = {
		PIN_CAN0_STB,
		PIN_CAN1_STB,
		PIN_CAN2_STB,
		PIN_CAN3_STB
	};

	uint32_t en_pins[4] = {
		PIN_CAN0_ENABLE,
		PIN_CAN1_ENABLE,
		PIN_CAN2_ENABLE,
		PIN_CAN3_ENABLE
	};

	int i;
	int ret = 0;
	bool error = false;

#if !defined(CONFIG_S32K3XX_FLEXCAN0) || !defined(CONFIG_S32K3XX_FLEXCAN1) || \
    !defined(CONFIG_S32K3XX_FLEXCAN2) || !defined(CONFIG_S32K3XX_FLEXCAN3)
#  error CONFIG_S32K3XX_FLEXCAN0-3 need to be enabled to perform CAN self-test
#endif

	/* Initialize pins, go into normal mode (EN high, STB_N high) to clear Pwon
	 * and Wake flags.
	 */

	for (i = 0; i < 4; i++) {
		ret = s32k3xx_pinconfig(errn_pins[i]);

		if (ret != 0) {
			error = true;
			_err("CAN%d ERR_N pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}

		ret = s32k3xx_pinconfig(stbn_pins[i] | GPIO_OUTPUT_ONE);

		if (ret != 0) {
			error = true;
			_err("CAN%d STB_N pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}

		ret = s32k3xx_pinconfig(en_pins[i] | GPIO_OUTPUT_ONE);

		if (ret != 0) {
			error = true;
			_err("CAN%d EN pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}
	}

	/* Wait for the transition to normal mode to finish and previous status
	 * flags to be cleared before switching modes again.  This is longer
	 */

	up_udelay(3000);

	/* Go into listen-only mode (EN low, STB_N still high) to read local
	 * failure flags.
	 */

	for (i = 0; i < 4; i++) {
		s32k3xx_gpiowrite(en_pins[i], 0);
	}

	/* Wait for transition to listen-only mode to finish */

	up_udelay(200);

	/* Check for local failure flags and then go back to normal mode */

	for (i = 0; i < 4; i++) {
		if (s32k3xx_gpioread(errn_pins[i])) {
			_info("CAN%d flag check successful\n", i);

		} else {
			error = true;
			_err("CAN%d flag check failed\n", i);

			/* Don't return yet, we still need to cleanup */
		}

		s32k3xx_gpiowrite(en_pins[i], 1);
	}

	return (error ? -1 : 0);
}

#  if !defined(CONFIG_S32K3XX_TJA1153)
/****************************************************************************
 * Name: s32k3xx_selftest_sct
 *
 * Description:
 *   Basic check of the SCTs (4-5).
 *
 ****************************************************************************/

static int s32k3xx_selftest_sct(void)
{
	uint32_t stbn_pins[2] = {
		PIN_CAN4_STB,
		PIN_CAN5_STB
	};

	uint32_t en_pins[2] = {
		PIN_CAN4_ENABLE,
		PIN_CAN5_ENABLE
	};

	uint32_t txd_pins[2] = {
		PIN_CAN4_TX,
		PIN_CAN5_TX
	};

	uint32_t rxd_pins[2] = {
		PIN_CAN4_RX,
		PIN_CAN5_RX
	};

	int i;
	int ret = 0;
	bool error = false;

	/* Configure pins and enable CAN PHY.  CAN_TXD will be temporarily changed
	 * to a GPIO output to be able to control its logic level.
	 */

	for (i = 4; i < 6; i++) {
		ret = s32k3xx_pinconfig(stbn_pins[i - 4] | GPIO_OUTPUT_ZERO);

		if (ret != 0) {
			error = true;
			_err("CAN%d STB_N pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}

		ret = s32k3xx_pinconfig(en_pins[i - 4] | GPIO_OUTPUT_ONE);

		if (ret != 0) {
			error = true;
			_err("CAN%d EN pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}

		ret = s32k3xx_pinconfig((txd_pins[i - 4] & (_PIN_PORT_MASK |
					 _PIN_MASK)) | GPIO_OUTPUT | GPIO_OUTPUT_ONE);

		if (ret != 0) {
			error = true;
			_err("CAN%d TXD pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}

		ret = s32k3xx_pinconfig(rxd_pins[i - 4]);

		if (ret != 0) {
			error = true;
			_err("CAN%d RXD pin configuration failed\n", i);

			return -1; /* Return immediately, no cleanup needed */
		}
	}

	/* Wait for CAN PHY to detect change at TXD pin */

	up_udelay(5000);

	/* Check if CAN_RXD follows the high level of CAN_TXD */

	for (i = 4; i < 6; i++) {
		if (s32k3xx_gpioread(rxd_pins[i - 4])) {
			_info("CAN%d RXD high check successful\n", i);

		} else {
			error = true;
			_err("CAN%d RXD high check failed\n", i);

			/* Don't return yet, we still need to cleanup */
		}
	}

	for (i = 4; i < 6; i++) {
		s32k3xx_gpiowrite(txd_pins[i - 4], 0);
	}

	/* Wait for CAN PHY to detect change at TXD pin */

	up_udelay(5000);

	/* Check if CAN_RXD follows the low level of CAN_TXD */

	for (i = 4; i < 6; i++) {
		if (!s32k3xx_gpioread(rxd_pins[i - 4])) {
			_info("CAN%d RXD low check successful\n", i);

		} else {
			error = true;
			_err("CAN%d RXD low check failed\n", i);

			/* Don't return yet, we still need to cleanup */
		}
	}

	/* Restore CAN_TXD pinconfig */

	for (i = 4; i < 6; i++) {
		s32k3xx_pinconfig(txd_pins[i - 4]);

		if (ret != 0) {
			error = true;
			_err("CAN%d TXD restoring pin configuration failed\n", i);

			/* Don't return yet, we still need to cleanup */
		}
	}

	return (error ? -1 : 0);
}
#  endif /* !CONFIG_S32K3XX_TJA1153 */
#endif /* CONFIG_S32K3XX_FLEXCAN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_selftest
 *
 * Description:
 *   Runs basic routines to verify that all board components are up and
 *   running.  Results are send to the syslog, it is recommended to
 *   enable all output levels (error, warning and info).
 *
 ****************************************************************************/

void s32k3xx_selftest(void)
{
	int ret = 0;
	bool error = false;

#ifdef CONFIG_S32K3XX_LPI2C
	ret = s32k3xx_selftest_se050();

	if (ret != 0) {
		error = true;
		_err("s32k3xx_selftest_se050() failed\n");
	}

#endif

#ifdef CONFIG_S32K3XX_FLEXCAN
	ret = s32k3xx_selftest_can();

	if (ret != 0) {
		error = true;
		_err("s32k3xx_selftest_can() failed\n");
	}

#  if !defined(CONFIG_S32K3XX_TJA1153)
	ret = s32k3xx_selftest_sct();

	if (ret != 0) {
		error = true;
		_err("s32k3xx_selftest_sct() failed\n");
	}

#  endif /* !CONFIG_S32K3XX_TJA1153 */
#endif

	if (!error) {
		_info("s32k3xx_selftest() successful\n");
	}
}

#endif /* CONFIG_S32K3XX_SELFTEST */
