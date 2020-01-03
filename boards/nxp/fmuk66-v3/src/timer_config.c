/****************************************************************************
 *
 *   Copyright (C) 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file timer_config.c
 *
 * Configuration data for the kinetis pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

// TODO:Stubbed out for now
#include <stdint.h>

#include <kinetis.h>
#include "hardware/kinetis_sim.h"
#include "hardware/kinetis_ftm.h"

#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>

#include "board_config.h"

__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	{
		.base = KINETIS_FTM0_BASE,
		.clock_register = KINETIS_SIM_SCGC6,
		.clock_bit = SIM_SCGC6_FTM0,
		.vectorno =  KINETIS_IRQ_FTM0,

	},
	{
		.base = KINETIS_FTM3_BASE,
		.clock_register = KINETIS_SIM_SCGC3,
		.clock_bit = SIM_SCGC3_FTM3,
		.vectorno =  KINETIS_IRQ_FTM3,
	},
	{
		.base = KINETIS_FTM2_BASE,
		.clock_register = KINETIS_SIM_SCGC3,
		.clock_bit = SIM_SCGC3_FTM2,
		.vectorno =  KINETIS_IRQ_FTM2,
	}
};

__EXPORT const io_timers_channel_mapping_t io_timers_channel_mapping = {
	.element = {
		{
			.first_channel_index = 0,
			.channel_count = 4,
		},
		{
			.first_channel_index = 4,
			.channel_count = 3,
		},
		{
			.first_channel_index = 6,
			.channel_count = 1,
		}
	}
};

__EXPORT const timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	{
		.gpio_out = GPIO_FTM0_CH0OUT, // FMU_CH1
		.gpio_in  = GPIO_FTM0_CH0IN,
		.timer_index = 0,
		.timer_channel = 1,           // physical channel number +1
	},
	{
		.gpio_out = GPIO_FTM0_CH3OUT, // FMU_CH2
		.gpio_in  = GPIO_FTM0_CH3IN,
		.timer_index = 0,
		.timer_channel = 4,
	},
	{
		.gpio_out = GPIO_FTM0_CH4OUT, // FMU_CH3
		.gpio_in  = GPIO_FTM0_CH4IN,
		.timer_index = 0,
		.timer_channel = 5,
	},
	{
		.gpio_out = GPIO_FTM0_CH5OUT, // FMU_CH4
		.gpio_in  = GPIO_FTM0_CH5IN,
		.timer_index = 0,
		.timer_channel = 6,
	},
	{
		.gpio_out = GPIO_FTM3_CH6OUT, // FMU_CH5
		.gpio_in  = GPIO_FTM3_CH6IN,
		.timer_index = 1,
		.timer_channel = 7,
	},
	{
		.gpio_out = GPIO_FTM3_CH7OUT, // FMU_CH6
		.gpio_in  = GPIO_FTM3_CH7IN,
		.timer_index = 1,
		.timer_channel = 8,
	},
	{
		.gpio_out = GPIO_FTM3_CH0OUT, // U_TRI
		.gpio_in  = GPIO_FTM3_CH0IN,
		.timer_index = 1,
		.timer_channel = 1,
	},
	{
		.gpio_out = GPIO_FTM2_CH0OUT,
		.gpio_in  = GPIO_FTM2_CH0IN, // U_ECH
		.timer_index = 2,
		.timer_channel = 1,
	},
};

__EXPORT const struct io_timers_t led_pwm_timers[MAX_LED_TIMERS] = {
	{
		.base = KINETIS_FTM3_BASE,
		.clock_register = KINETIS_SIM_SCGC3,
		.clock_bit = SIM_SCGC3_FTM3,
		.vectorno =  0,
	},
};

__EXPORT const struct timer_io_channels_t led_pwm_channels[MAX_TIMER_LED_CHANNELS] = {
	{
		.gpio_out = LED_TIM3_CH1OUT, // RGB_R
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 2,
	},
	{
		.gpio_out = LED_TIM3_CH5OUT, // RGB_G
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 6,
	},
	{
		.gpio_out = LED_TIM3_CH4OUT, // RGB_B
		.gpio_in  = 0,
		.timer_index = 0,
		.timer_channel = 5,
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
#define BOARD_PWM_SRC_CLOCK_FREQ 16000000
#endif


__EXPORT void fmuk66_timer_initialize(void)
{


	/* Y1 is 16 Mhz used to driver the FTM_CLKIN0 (PCT12) */


	/* Enable PCT12 as FTM_CLKIN0 */

	kinetis_pinconfig(PIN_FTM_CLKIN0_3);

	/* Select FTM_CLKIN0 as source for FTM0, FTM2, and FTM3 */

	uint32_t regval = _REG(KINETIS_SIM_SOPT4);
	regval &= ~(SIM_SOPT4_FTM0CLKSEL | SIM_SOPT4_FTM2CLKSEL | SIM_SOPT4_FTM3CLKSEL);
	_REG(KINETIS_SIM_SOPT4) = regval;


	/* Enabled System Clock Gating Control for FTM 0 and 2*/

	regval = _REG(KINETIS_SIM_SCGC6);
	regval |= SIM_SCGC6_FTM0 | SIM_SCGC6_FTM2;
	_REG(KINETIS_SIM_SCGC6) = regval;

	/* Enabled System Clock Gating Control for FTM 2 and 3 */

	regval = _REG(KINETIS_SIM_SCGC3);
	regval |= SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
	_REG(KINETIS_SIM_SCGC3) = regval;

}
