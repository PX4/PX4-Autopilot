/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_clockconfig.c
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

#include "s32k3xx_clockconfig.h"
#include "s32k3xx_start.h"

#include "board_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Each S32K3XX board must provide the following initialized structure.
 * This is needed to establish the initial board clocking.
 */

const struct clock_configuration_s g_initial_clkconfig = {
	.cgm                 =
	{
		.sirc              =
		{
			.range           = CGM_FIRC_RANGE_32K,           /* Slow IRC is trimmed to 32 kHz */
		},
		.firc              =
		{
			.range           = CGM_FIRC_RANGE_HIGH,          /* RANGE */
			.div             = CGM_CLOCK_DIV_BY_1,           /* FIRCDIV1 */
		},
		.scs               =
		{
			.scs_source      = CGM_SCS_SOURCE_PLL_PHI0,
			.core_clk        =
			{
				.div           = CGM_MUX_DIV_BY_1,
				.trigger       = false,
			},
			.aips_plat_clk   =
			{
				.div           = CGM_MUX_DIV_BY_2,
				.trigger       = false,
			},
			.aips_slow_clk   =
			{
				.div           = CGM_MUX_DIV_SLOW_BY_4,
				.trigger       = false,
			},
			.hse_clk         =
			{
				.div           = CGM_MUX_DIV_BY_1,
				.trigger       = false,
			},
			.dcm_clk         =
			{
				.div           = CGM_MUX_DIV_BY_1,
				.trigger       = false,
			},
			.lbist_clk       =
			{
				.div           = CGM_MUX_DIV_BY_1,
				.trigger       = false,
			},
#ifdef CONFIG_S32K3XX_QSPI
			.qspi_mem_clk        =
			{
				.div           = CGM_MUX_DIV_BY_1,
				.trigger       = false,
			},
#endif
			.mux_1_stm0 =
			{
				.source        = CGM_CLK_SRC_FXOSC,
				.div           = CGM_MUX_DIV_BY_2,
			},
			.mux_3 =
			{
				.source        = CGM_CLK_SRC_AIPS_PLAT_CLK,
				.div           = CGM_MUX_DIV_BY_1,
			},
			.mux_4 =
			{
				.source        = CGM_CLK_SRC_AIPS_PLAT_CLK,
				.div           = CGM_MUX_DIV_BY_1,
			},
#ifdef CONFIG_S32K3XX_ENET
			.mux_7_emac_rx =
			{
				.source        = CGM_CLK_SRC_EMAC_RMII_TX_CLK,
				.div           = CGM_MUX_DIV_BY_2,
			},
			.mux_8_emac_tx =
			{
				.source        = CGM_CLK_SRC_EMAC_RMII_TX_CLK,
				.div           = CGM_MUX_DIV_BY_2,
			},
			.mux_9_emac_ts =
			{
				.source        = CGM_CLK_SRC_EMAC_RMII_TX_CLK,
				.div           = CGM_MUX_DIV_BY_2, /* FIXME check div value */
			},
#endif
#ifdef CONFIG_S32K3XX_QSPI
			.mux_10_qspi_sfck =
			{
				.source        = CGM_CLK_SRC_PLL_PHI1_CLK,
				.div           = CGM_MUX_DIV_BY_4,
			},
#endif
		},
		.pll =
		{
			.modul_freq      = 0,
			.modul_depth     = 0,
			.core_pll_power  = true,
			.modulation_type = false,
			.sigma_delta     = CGM_PLL_SIGMA_DELTA,
			.enable_dither   = false,
			.mode            = CGM_PLL_INTEGER_MODE,
			.prediv          = 2,
			.mult            = 120,
			.postdiv         = 2,
			.phi0            = CGM_PLL_PHI_DIV_BY_3,
			.phi1            = CGM_PLL_PHI_DIV_BY_3,
		},
		.clkout            =
		{
			.source          = CGM_CLK_SRC_AIPS_SLOW_CLK,
			.div             = CGM_CLKOUT_DIV_BY_1,
		}
	},
	.pcc                 =
	{
		.pclks             = g_peripheral_clockconfig0,    /* Peripheral clock configurations */
	},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
