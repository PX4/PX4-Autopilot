/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file registers.c
 *
 * Implementation of the PX4IO register space.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/systemlib.h>
#include <stm32_pwr.h>

#include "aerocore_rc.h"
/**
 * PAGE 0
 *
 * Static configuration parameters.
 */

/**
 * PAGE 1
 *
 * Status values.
 */
uint16_t		r_page_status[] = {
	[PX4IO_P_STATUS_FREEMEM]		= 0,
	[PX4IO_P_STATUS_CPULOAD]		= 0,
	[PX4IO_P_STATUS_FLAGS]			= 0,
	[PX4IO_P_STATUS_ALARMS]			= 0,
	[PX4IO_P_STATUS_VBATT]			= 0,
	[PX4IO_P_STATUS_IBATT]			= 0,
	[PX4IO_P_STATUS_VSERVO]			= 0,
	[PX4IO_P_STATUS_VRSSI]			= 0,
	[PX4IO_P_STATUS_PRSSI]			= 0,
};

/**
 * PAGE 2
 *
 * Post-mixed actuator values.
 */

/**
 * PAGE 3
 *
 * Servo PWM values
 */

/**
 * PAGE 4
 *
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[] =
{
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_FLAGS]			= 0,
	[PX4IO_P_RAW_RC_NRSSI]			= 0,
	[PX4IO_P_RAW_RC_DATA]			= 0,
	[PX4IO_P_RAW_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_LOST_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_RC_BASE ... (PX4IO_P_RAW_RC_BASE + PX4IO_RC_INPUT_CHANNELS)] = 0
};

/**
 * PAGE 5
 *
 * Scaled/routed RC input
 */
uint16_t		r_page_rc_input[] = {
	[PX4IO_P_RC_VALID]			= 0,
	[PX4IO_P_RC_BASE ... (PX4IO_P_RC_BASE + PX4IO_RC_MAPPED_CONTROL_CHANNELS)] = 0
};

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.
 * PAGE 7 PWM rate maps.
 */

/**
 * PAGE 100
 *
 * Setup registers
 */
volatile uint16_t	r_page_setup[] =
{
#ifdef CONFIG_ARCH_BOARD_PX4IO_V2
	/* default to RSSI ADC functionality */
	[PX4IO_P_SETUP_FEATURES]		= PX4IO_P_SETUP_FEATURES_ADC_RSSI,
#else
	[PX4IO_P_SETUP_FEATURES]		= 0,
#endif
	[PX4IO_P_SETUP_ARMING]			= 0,
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[PX4IO_P_SETUP_PWM_ALTRATE]		= 200,
#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
	[PX4IO_P_SETUP_RELAYS]			= 0,
#endif
#ifdef ADC_VSERVO
	[PX4IO_P_SETUP_VSERVO_SCALE]		= 10000,
#else
	[PX4IO_P_SETUP_VBATT_SCALE]		= 10000,
#endif
	[PX4IO_P_SETUP_SET_DEBUG]		= 0,
	[PX4IO_P_SETUP_REBOOT_BL]		= 0,
	[PX4IO_P_SETUP_CRC ... (PX4IO_P_SETUP_CRC+1)] = 0,
	[PX4IO_P_SETUP_RC_THR_FAILSAFE_US] = 0,
};

#ifdef CONFIG_ARCH_BOARD_PX4IO_V2
#define PX4IO_P_SETUP_FEATURES_VALID	(PX4IO_P_SETUP_FEATURES_SBUS1_OUT | \
					 PX4IO_P_SETUP_FEATURES_SBUS2_OUT | \
					 PX4IO_P_SETUP_FEATURES_ADC_RSSI | \
					 PX4IO_P_SETUP_FEATURES_PWM_RSSI)
#else
#define PX4IO_P_SETUP_FEATURES_VALID	0
#endif
#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_FMU_ARMED | \
					 PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK | \
					 PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK | \
					 PX4IO_P_SETUP_ARMING_IO_ARM_OK | \
					 PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM | \
					 PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE | \
					 PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED | \
					 PX4IO_P_SETUP_ARMING_LOCKDOWN)
#define PX4IO_P_SETUP_RATES_VALID	((1 << PX4IO_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PX4IO_RELAY_CHANNELS) - 1)

/**
 * PAGE 101
 *
 * Control values from the FMU.
 */

/*
 * PAGE 102 does not have a buffer.
 */

/**
 * PAGE 103
 *
 * R/C channel input configuration.
 */
uint16_t		r_page_rc_input_config[PX4IO_RC_INPUT_CHANNELS * PX4IO_P_RC_CONFIG_STRIDE];

/* valid options */
#define PX4IO_P_RC_CONFIG_OPTIONS_VALID	(PX4IO_P_RC_CONFIG_OPTIONS_REVERSE | PX4IO_P_RC_CONFIG_OPTIONS_ENABLED)

/*
 * PAGE 104 uses r_page_servos.
 */

/**
 * PAGE 105
 *
 * Failsafe servo PWM values
 * 
 * Disable pulses as default.
 */

/**
 * PAGE 106
 *
 * minimum PWM values when armed
 *
 */

/**
 * PAGE 107
 *
 * maximum PWM values when armed
 *
 */

/**
 * PAGE 108
 *
 * disarmed PWM values for difficult ESCs
 *
 */


