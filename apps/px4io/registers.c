/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

#include "px4io.h"
#include "protocol.h"

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);

/**
 * Setup registers
 */
uint16_t		r_page_setup[] =
{
	[PX4IO_P_SETUP_ARMING]			= 0,
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_LOWRATE]		= 50,
	[PX4IO_P_SETUP_PWM_HIGHRATE]		= 200,
	[PX4IO_P_SETUP_RELAYS]			= 0,
};

#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_ARM_OK | PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE)
#define PX4IO_P_SETUP_RATES_VALID	((1 << IO_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PXIO_RELAY_CHANNELS) - 1)

/**
 * Control values from the FMU.
 */
uint16_t		r_page_controls[PX4IO_CONTROL_CHANNELS];

/**
 * Static configuration parameters.
 */
static const uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= 0,
	[PX4IO_P_CONFIG_SOFTWARE_VERSION]	= 0,
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 0,
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= 64,
	[PX4IO_P_CONFIG_CONTROL_COUNT]		= PX4IO_CONTROL_CHANNELS,
	[PX4IO_P_CONFIG_ACTUATOR_COUNT]		= IO_SERVO_COUNT,
	[PX4IO_P_CONFIG_RC_INPUT_COUNT]		= MAX_CONTROL_CHANNELS,
	[PX4IO_P_CONFIG_ADC_INPUT_COUNT]	= ADC_CHANNEL_COUNT,
	[PX4IO_P_CONFIG_RELAY_COUNT]		= PX4IO_RELAY_CHANNELS,
};

/**
 * Status values.
 */
uint16_t		r_page_status[] = {
	[PX4IO_P_STATUS_FREEMEM]		= 0,
	[PX4IO_P_STATUS_CPULOAD]		= 0,
	[PX4IO_P_STATUS_FLAGS]			= 0,
	[PX4IO_P_STATUS_ALARMS]			= 0,
	[PX4IO_P_STATUS_VBATT]			= 0,
	[PX4IO_P_STATUS_TEMPERATURE]		= 0
};

/**
 * ADC input buffer.
 */
uint16_t		r_page_adc[ADC_CHANNEL_COUNT];

/**
 * Post-mixed actuator values.
 */
uint16_t 		r_page_actuators[IO_SERVO_COUNT];

/**
 * Servo PWM values
 */
uint16_t		r_page_servos[IO_SERVO_COUNT];

/**
 * Scaled/routed RC input
 */
uint16_t		r_page_rc_input[MAX_CONTROL_CHANNELS];

/**
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[MAX_CONTROL_CHANNELS];


void
registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	switch (page) {

		/* handle bulk controls input */
	case PX4IO_PAGE_CONTROLS:

		/* copy channel data */
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX scaling - should be -10000..10000 */
			r_page_controls[offset] = *values;

			offset++;
			num_values--;
			values++;
		}

		/* XXX we should cause a mixer tick ASAP */
		system_state.mixer_fmu_available = true;
		break;

		/* handle text going to the mixer parser */
	case PX4IO_PAGE_MIXERLOAD:
		mixer_handle_text(values, num_values * sizeof(*values));
		break;

	default:
		/* avoid offset wrap */
		if ((offset + num_values) > 255)
			num_values = 255 - offset;

		/* iterate individual registers, set each in turn */
		while (num_values--) {
			if (registers_set_one(page, offset, *values))
				break;
			offset++;
			values++;
		}
	}
}

static int
registers_set_one(uint8_t page, uint8_t offset, uint16_t value)
{
	switch (page) {

	case PX4IO_PAGE_STATUS:
		switch (offset) {
		case PX4IO_P_STATUS_ALARMS:
			/* clear bits being written */
			r_page_status[PX4IO_P_STATUS_ALARMS] &= ~value;
			break;

		default:
			/* just ignore writes to other registers in this page */
			break;
		}
		break;

	case PX4IO_PAGE_SETUP:
		switch (offset) {
		case PX4IO_P_SETUP_ARMING:

			value &= PX4IO_P_SETUP_ARMING_VALID;
			r_page_setup[PX4IO_P_SETUP_ARMING] = value;

			/* update arming state - disarm if no longer OK */
			if (system_state.armed && !(value & PX4IO_P_SETUP_ARMING_ARM_OK))
				system_state.armed = false;

			break;

		case PX4IO_P_SETUP_PWM_RATES:
			value &= PX4IO_P_SETUP_RATES_VALID;
			r_page_setup[PX4IO_P_SETUP_PWM_RATES] = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_PWM_LOWRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			r_page_setup[PX4IO_P_SETUP_PWM_LOWRATE] = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_PWM_HIGHRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			r_page_setup[PX4IO_P_SETUP_PWM_HIGHRATE] = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_RELAYS:
			value &= PX4IO_P_SETUP_RELAYS_VALID;
			r_page_setup[PX4IO_P_SETUP_RELAYS] = value;
			POWER_RELAY1(value & (1 << 0) ? 1 : 0);
			POWER_RELAY2(value & (1 << 1) ? 1 : 0);
			POWER_ACC1(value & (1 << 2) ? 1 : 0);
			POWER_ACC2(value & (1 << 3) ? 1 : 0);
			break;

		default:
			return -1;
		}
		break;

	default:
		return -1;
	}
	return 0;
}

int
registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values)
{

	switch (page) {
	case PX4IO_PAGE_CONFIG:
		*values = r_page_config;
		*num_values = sizeof(r_page_config) / sizeof(r_page_config[0]);
		break;

	case PX4IO_PAGE_STATUS:
		{
			struct mallinfo minfo = mallinfo();
			r_page_status[PX4IO_P_STATUS_FREEMEM] = minfo.fordblks;
		}
		/* XXX PX4IO_P_STATUS_CPULOAD */
		r_page_status[PX4IO_P_STATUS_FLAGS] = 
			(system_state.armed ? PX4IO_P_STATUS_FLAGS_ARMED : 0) |
			(system_state.manual_override_ok ? PX4IO_P_STATUS_FLAGS_OVERRIDE : 0) |
			((system_state.rc_channels > 0) ? PX4IO_P_STATUS_FLAGS_RC_OK : 0))
			/* XXX specific receiver status */

		/* XXX PX4IO_P_STATUS_ALARMS] */

		{
			/*
			 * Coefficients here derived by measurement of the 5-16V
			 * range on one unit:
			 *
			 * V   counts
			 *  5  1001
			 *  6  1219
			 *  7  1436
			 *  8  1653
			 *  9  1870
			 * 10  2086
			 * 11  2303
			 * 12  2522
			 * 13  2738
			 * 14  2956
			 * 15  3172
			 * 16  3389
			 *
			 * slope = 0.0046067
			 * intercept = 0.3863
			 *
			 * Intercept corrected for best results @ 12V.
			 */
			unsigned counts = adc_measure(ADC_VBATT);
			r_page_status[PX4IO_P_STATUS_VBATT] = (4150 + (counts * 46)) / 10;
		}
		/* XXX PX4IO_P_STATUS_TEMPERATURE */

		*values = r_page_status;
		*num_values = sizeof(r_page_status) / sizeof(r_page_status[0]);
		break;

	case PX4IO_PAGE_ACTUATORS:
		*values = r_page_actuators;
		*num_values = sizeof(r_page_actuators) / sizeof(r_page_actuators[0]);
		break;

	case PX4IO_PAGE_SERVOS:
		*values = system_state.servos;
		*num_values = IO_SERVO_COUNT;
		break;

	case PX4IO_PAGE_RAW_RC_INPUT:
		*values = r_page_raw_rc_input;
		*num_values = sizeof(r_page_raw_rc_input) / sizeof(r_page_raw_rc_input[0]);
		break;

	case PX4IO_PAGE_RC_INPUT:
		*values = system_state.rc_channel_data;
		*num_values = system_state.rc_channels;
		return -1;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		r_page_adc[0] = adc_measure(ADC_VBATT);
		r_page_adc[1] = adc_measure(ADC_IN5);
		*values = r_page_adc;
		*num_values = ADC_CHANNEL_COUNT;
		break;

	default:
		return -1;
	}

	/* if the offset is beyond the end of the page, we have no data */
	if (*num_values <= offset)
		return -1;

	/* adjust value count and pointer for the page offset */
	*num_values -= offset;
	*values += offset;

	return 0;
}
