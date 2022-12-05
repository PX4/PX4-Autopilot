/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_boot.c
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
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "board_config.h"

#ifdef CONFIG_S32K3XX_FS26
#include "s32k3xx_fs26.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_board_initialize
 *
 * Description:
 *   All S32K3XX architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void s32k3xx_board_initialize(void)
{
#ifdef CONFIG_SEGGER_SYSVIEW
	up_perf_init((void *)MR_CANHUBK3_SYSCLK_FREQUENCY);
#endif

#ifdef CONFIG_S32K3XX_FS26
	/* Configure LPSPI3 & FS26 as quick as possible to avoid watchdog reset */

	s32k3xx_pinconfig(PIN_LPSPI3_PCS);

	/* Initialize the SPI driver for LPSPI3 */

	struct spi_dev_s *g_lpspi3 = s32k3xx_lpspibus_initialize(3);

	if (g_lpspi3 == NULL) {
		spierr("ERROR: FAILED to initialize LPSPI3\n");
	}

	fs26_initialize(g_lpspi3);
#endif

#ifdef CONFIG_ARCH_LEDS
	/* Configure on-board LEDs if LED support has been selected. */

	board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
	/* Perform board-specific initialization */

	s32k3xx_bringup();
}
#endif
