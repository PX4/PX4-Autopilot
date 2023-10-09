/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_periphclocks.c
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

#include <stdbool.h>

#include "s32k3xx_clocknames.h"
#include "s32k3xx_periphclocks.h"

#include "board_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K3XX board must provide the following initialized structure.
 * This is needed to establish the initial peripheral clocking.
 */

const struct peripheral_clock_config_s g_peripheral_clockconfig0[] = {
	{
		.clkname = FLEXCAN0_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN0
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN1_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN1
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN2_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN2
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN3_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN3
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN4_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN4
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = FLEXCAN5_CLK,
#ifdef CONFIG_S32K3XX_FLEXCAN5
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPI2C0_CLK,
#ifdef CONFIG_S32K3XX_LPI2C0
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPI2C1_CLK,
#ifdef CONFIG_S32K3XX_LPI2C1
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPSPI1_CLK,
#ifdef CONFIG_S32K3XX_LPSPI1
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPSPI2_CLK,
#ifdef CONFIG_S32K3XX_LPSPI2
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPSPI3_CLK,
#ifdef CONFIG_S32K3XX_LPSPI3
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPSPI4_CLK,
#ifdef CONFIG_S32K3XX_LPSPI4
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPSPI5_CLK,
#ifdef CONFIG_S32K3XX_LPSPI5
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART0_CLK,
#ifdef CONFIG_S32K3XX_LPUART0
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART1_CLK,
#ifdef CONFIG_S32K3XX_LPUART1
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART2_CLK,
#ifdef CONFIG_S32K3XX_LPUART2
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART4_CLK,
#ifdef CONFIG_S32K3XX_LPUART4
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART7_CLK,
#ifdef CONFIG_S32K3XX_LPUART7
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART9_CLK,
#ifdef CONFIG_S32K3XX_LPUART9
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART10_CLK,
#ifdef CONFIG_S32K3XX_LPUART10
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART13_CLK,
#ifdef CONFIG_S32K3XX_LPUART13
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = LPUART14_CLK,
#ifdef CONFIG_S32K3XX_LPUART14
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = WKPU_CLK,
#ifdef CONFIG_S32K3XX_WKPUINTS
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = EMAC_CLK,
#ifdef CONFIG_S32K3XX_ENET
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = QSPI_CLK,
#ifdef CONFIG_S32K3XX_QSPI
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = EDMA_CLK,
#ifdef CONFIG_S32K3XX_EDMA
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = DMAMUX0_CLK,
#ifdef CONFIG_S32K3XX_EDMA
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = DMAMUX1_CLK,
#ifdef CONFIG_S32K3XX_EDMA
		.clkgate = true,
#else
		.clkgate = false,
#endif
	},
	{
		.clkname = STM0_CLK,
		.clkgate = true,
	},
	{
		.clkname = EMIOS0_CLK,
		.clkgate = true,
	},
	{
		.clkname = ADC2_CLK,
		.clkgate = true,
	}
};

unsigned int const num_of_peripheral_clocks_0 =
	sizeof(g_peripheral_clockconfig0) /
	sizeof(g_peripheral_clockconfig0[0]);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
