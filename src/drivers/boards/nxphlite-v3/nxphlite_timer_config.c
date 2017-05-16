/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

/*
 * @file nxphlite_timer_config.c
 *
 * Configuration data for the kinetis pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

// TODO:Stubbed out for now
#include <stdint.h>

#include <kinetis.h>
#include "chip/kinetis_sim.h"
#include "chip/kinetis_ftm.h"

#include <drivers/drv_pwm_output.h>
#include <drivers/kinetis/drv_io_timer.h>

#include "board_config.h"

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = KINETIS_FTM0_BASE,
		.clock_register = KINETIS_SIM_SCGC6,
		.clock_bit = SIM_SCGC6_FTM0,
		.first_channel_index = 0,
		.last_channel_index = 5,
		.handler = io_timer_handler0,
		.vectorno =  KINETIS_IRQ_FTM0,

	},
	{
		.base = KINETIS_FTM3_BASE,
		.clock_register = KINETIS_SIM_SCGC3,
		.clock_bit = SIM_SCGC3_FTM3,
		.first_channel_index = 6,
		.last_channel_index = 13,
		.handler = io_timer_handler1,
		.vectorno =  KINETIS_IRQ_FTM3,
	}
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		.gpio_out = GPIO_FTM0_CH0OUT,
		.gpio_in  = GPIO_FTM0_CH0IN,
		.timer_index = 0,
		.timer_channel = 1,
	},
	{
		.gpio_out = GPIO_FTM0_CH3OUT,
		.gpio_in  = GPIO_FTM0_CH3IN,
		.timer_index = 0,
		.timer_channel = 4,
	},
	{
		.gpio_out = GPIO_FTM0_CH4OUT,
		.gpio_in  = GPIO_FTM0_CH4IN,
		.timer_index = 0,
		.timer_channel = 5,
	},
	{
		.gpio_out = GPIO_FTM0_CH5OUT,
		.gpio_in  = GPIO_FTM0_CH5IN,
		.timer_index = 0,
		.timer_channel = 6,
	},
	{
		.gpio_out = GPIO_FTM0_CH6OUT,
		.gpio_in  = GPIO_FTM0_CH6IN,
		.timer_index = 0,
		.timer_channel = 7,
	},
	{
		.gpio_out = GPIO_FTM0_CH7OUT,
		.gpio_in  = GPIO_FTM0_CH7IN,
		.timer_index = 0,
		.timer_channel = 8,
	},

	{
		.gpio_out = GPIO_FTM3_CH0OUT,
		.gpio_in  = GPIO_FTM3_CH0IN,
		.timer_index = 1,
		.timer_channel = 1,
	},
	{
		.gpio_out = GPIO_FTM3_CH1OUT,
		.gpio_in  = GPIO_FTM3_CH1IN,
		.timer_index = 1,
		.timer_channel = 2,
	},
	{
		.gpio_out = GPIO_FTM3_CH2OUT,
		.gpio_in  = GPIO_FTM3_CH2IN,
		.timer_index = 1,
		.timer_channel = 3,
	},
	{
		.gpio_out = GPIO_FTM3_CH3OUT,
		.gpio_in  = GPIO_FTM3_CH3IN,
		.timer_index = 1,
		.timer_channel = 4,
	},
	{
		.gpio_out = GPIO_FTM3_CH4OUT,
		.gpio_in  = GPIO_FTM3_CH4IN,
		.timer_index = 1,
		.timer_channel = 5,
	},
	{
		.gpio_out = GPIO_FTM3_CH5OUT,
		.gpio_in  = GPIO_FTM3_CH5IN,
		.timer_index = 1,
		.timer_channel = 6,
	},
	{
		.gpio_out = GPIO_FTM3_CH6OUT,
		.gpio_in  = GPIO_FTM3_CH6IN,
		.timer_index = 1,
		.timer_channel = 7,
	},
	{
		.gpio_out = GPIO_FTM3_CH7OUT,
		.gpio_in  = GPIO_FTM3_CH7IN,
		.timer_index = 1,
		.timer_channel = 8,
	},
};

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* Timer register accessors */

#define REG(t, _reg)	_REG(KINETIS_FTM##t##_BASE + (_reg))

#define rSC(t)         REG(t,KINETIS_FTM_SC_OFFSET)
#define rCNT(t)        REG(t,KINETIS_FTM_CNT_OFFSET)
#define rMOD(t)        REG(t,KINETIS_FTM_MOD_OFFSET)
#define rC0SC(t)       REG(t,KINETIS_FTM_C0SC_OFFSET)
#define rC0V(t)        REG(t,KINETIS_FTM_C0V_OFFSET)
#define rC1SC(t)       REG(t,KINETIS_FTM_C1SC_OFFSET)
#define rC1V(t)        REG(t,KINETIS_FTM_C1V_OFFSET)
#define rC2SC(t)       REG(t,KINETIS_FTM_C2SC_OFFSET)
#define rC2V(t)        REG(t,KINETIS_FTM_C2V_OFFSET)
#define rC3SC(t)       REG(t,KINETIS_FTM_C3SC_OFFSET)
#define rC3V(t)        REG(t,KINETIS_FTM_C3V_OFFSET)
#define rC4SC(t)       REG(t,KINETIS_FTM_C4SC_OFFSET)
#define rC4V(t)        REG(t,KINETIS_FTM_C4V_OFFSET)
#define rC5SC(t)       REG(t,KINETIS_FTM_C5SC_OFFSET)
#define rC5V(t)        REG(t,KINETIS_FTM_C5V_OFFSET)
#define rC6SC(t)       REG(t,KINETIS_FTM_C6SC_OFFSET)
#define rC6V(t)        REG(t,KINETIS_FTM_C6V_OFFSET)
#define rC7SC(t)       REG(t,KINETIS_FTM_C7SC_OFFSET)
#define rC7V(t)        REG(t,KINETIS_FTM_C7V_OFFSET)

#define rCNTIN(t)      REG(t,KINETIS_FTM_CNTIN_OFFSET)
#define rSTATUS(t)     REG(t,KINETIS_FTM_STATUS_OFFSET)
#define rMODE(t)       REG(t,KINETIS_FTM_MODE_OFFSET)
#define rSYNC(t)       REG(t,KINETIS_FTM_SYNC_OFFSET)
#define rOUTINIT(t)    REG(t,KINETIS_FTM_OUTINIT_OFFSET)
#define rOUTMASK(t)    REG(t,KINETIS_FTM_OUTMASK_OFFSET)
#define rCOMBINE(t)    REG(t,KINETIS_FTM_COMBINE_OFFSET)
#define rDEADTIME(t)   REG(t,KINETIS_FTM_DEADTIME_OFFSET)
#define rEXTTRIG(t)    REG(t,KINETIS_FTM_EXTTRIG_OFFSET)
#define rPOL(t)        REG(t,KINETIS_FTM_POL_OFFSET)
#define rFMS(t)        REG(t,KINETIS_FTM_FMS_OFFSET)
#define rFILTER(t)     REG(t,KINETIS_FTM_FILTER_OFFSET)
#define rFLTCTRL(t)    REG(t,KINETIS_FTM_FLTCTRL_OFFSET)
#define rQDCTRL(t)     REG(t,KINETIS_FTM_QDCTRL_OFFSET)
#define rCONF(t)       REG(t,KINETIS_FTM_CONF_OFFSET)
#define rFLTPOL(t)     REG(t,KINETIS_FTM_FLTPOL_OFFSET)
#define rSYNCONF(t)    REG(t,KINETIS_FTM_SYNCONF_OFFSET)
#define rINVCTRL(t)    REG(t,KINETIS_FTM_INVCTRL_OFFSET)
#define rSWOCTRL(t)    REG(t,KINETIS_FTM_SWOCTRL_OFFSET)
#define rPWMLOAD(t)    REG(t,KINETIS_FTM_PWMLOAD_OFFSET)

#if !defined(BOARD_PWM_SRC_CLOCK_FREQ)
#define BOARD_PWM_SRC_CLOCK_FREQ 4000000
#endif

__EXPORT void nxphlite_timer_initialize(void)
{

	/* We will uses FTM2 (CH0) on PTA10 drive FTM_CLKIN0 (PCT12)*/


	kinetis_pinconfig(PIN_FTM2_CH0_1);
	kinetis_pinconfig(PIN_FTM_CLKIN0_3);

	uint32_t regval = _REG(KINETIS_SIM_SOPT4);
	regval &= ~(SIM_SOPT4_FTM0CLKSEL | SIM_SOPT4_FTM3CLKSEL);
	_REG(KINETIS_SIM_SOPT4) = regval;


	/* Enabled System Clock Gating Control for FTM 0 and 2*/

	regval = _REG(KINETIS_SIM_SCGC6);
	regval |= SIM_SCGC6_FTM0 | SIM_SCGC6_FTM2;
	_REG(KINETIS_SIM_SCGC6) = regval;

	/* Enabled System Clock Gating Control for FTM 2 and 3 */

	regval = _REG(KINETIS_SIM_SCGC3);
	regval |= SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
	_REG(KINETIS_SIM_SCGC3) = regval;


	/* disable and configure the FTM2 as
	 * Bus Clock 56 Mhz
	 * PS 1
	 * MOD 7 for 4 Mhz
	 */

	rSC(2)    = FTM_SC_CLKS_NONE | FTM_SC_PS_1;
	rCNT(2)   = 0;

	/* Generate 2 times the Freq */

	rMOD(2)   = (BOARD_BUS_FREQ / (BOARD_PWM_SRC_CLOCK_FREQ * 2)) - 1;

	/* toggle on every compare adds a divide by 2 */

	rC0SC(2)  = (FTM_CSC_CHF | FTM_CSC_MSA | FTM_CSC_ELSA);

	/* Match on the wrap */

	rC0V(2)   = 0;

	/* Set to run in debug mode */

	rCONF(2)   |= FTM_CONF_BDMMODE_MASK;

	/* enable the timer */

	rSC(2) |= FTM_SC_CLKS_SYSCLK;


}
