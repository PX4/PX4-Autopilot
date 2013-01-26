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

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"
#include "protocol.h"

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);

/**
 * Setup registers
 */
volatile uint16_t	r_page_setup[] =
{
	[PX4IO_P_SETUP_FEATURES]		= 0,
	[PX4IO_P_SETUP_ARMING]			= 0,
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_LOWRATE]		= 50,
	[PX4IO_P_SETUP_PWM_HIGHRATE]		= 200,
	[PX4IO_P_SETUP_RELAYS]			= 0,
	[PX4IO_P_SETUP_VBATT_SCALE]		= 10000,
	[PX4IO_P_SETUP_IBATT_SCALE]		= 0,
	[PX4IO_P_SETUP_IBATT_BIAS]		= 0
};

#define PX4IO_P_SETUP_FEATURES_VALID	(PX4IO_P_FEAT_ARMING_MANUAL_OVERRIDE_OK)
#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_ARM_OK | \
					 PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE)
#define PX4IO_P_SETUP_RATES_VALID	((1 << IO_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PX4IO_RELAY_CHANNELS) - 1)

/**
 * Control values from the FMU.
 */
volatile uint16_t	r_page_controls[PX4IO_CONTROL_CHANNELS];

/**
 * Static configuration parameters.
 */
static const uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= 1,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_SOFTWARE_VERSION]	= 1,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 3,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= 64,	/* XXX hardcoded magic number */
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
	[PX4IO_P_STATUS_IBATT]			= 0
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
 * Servo PWM values
 */
uint16_t		r_page_servo_failsafe[IO_SERVO_COUNT];

/**
 * Scaled/routed RC input
 */
uint16_t		r_page_rc_input[] = {
	[PX4IO_P_RC_VALID]			= 0,
	[PX4IO_P_RC_BASE ... (PX4IO_P_RC_BASE + MAX_CONTROL_CHANNELS)] = 0
};

/**
 * Raw RC input
 */
uint16_t		r_page_raw_rc_input[] =
{
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_BASE ... (PX4IO_P_RAW_RC_BASE + MAX_CONTROL_CHANNELS)] = 0
};

/**
 * R/C channel input configuration.
 */
uint16_t		r_page_rc_input_config[MAX_CONTROL_CHANNELS * PX4IO_P_RC_CONFIG_STRIDE];

#define PX4IO_P_RC_CONFIG_OPTIONS_VALID	PX4IO_P_RC_CONFIG_OPTIONS_REVERSE

void
registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
	system_state.fmu_data_received_time = hrt_absolute_time();

	switch (page) {

		/* handle bulk controls input */
	case PX4IO_PAGE_CONTROLS:

		/* copy channel data */
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX range-check value? */
			r_page_controls[offset] = *values;

			offset++;
			num_values--;
			values++;
		}

		/* XXX we should cause a mixer tick ASAP */
		system_state.fmu_data_received_time = hrt_absolute_time();
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK;
		r_status_flags &= ~PX4IO_P_STATUS_FLAGS_RAW_PWM;
		break;

		/* handle raw PWM input */
	case PX4IO_PAGE_DIRECT_PWM:

		/* copy channel data */
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX range-check value? */
			r_page_servos[offset] = *values;

			offset++;
			num_values--;
			values++;
		}

		/* XXX we should cause a mixer tick ASAP */
		system_state.fmu_data_received_time = hrt_absolute_time();
		r_status_flags |= PX4IO_P_STATUS_FLAGS_FMU_OK | PX4IO_P_STATUS_FLAGS_RAW_PWM;
		break;

		/* handle setup for servo failsafe values */
	case PX4IO_PAGE_FAILSAFE_PWM:

		/* copy channel data */
		while ((offset < PX4IO_CONTROL_CHANNELS) && (num_values > 0)) {

			/* XXX range-check value? */
			r_page_servo_failsafe[offset] = *values;

			offset++;
			num_values--;
			values++;
		}
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
			r_status_alarms &= ~value;
			break;

		default:
			/* just ignore writes to other registers in this page */
			break;
		}
		break;

	case PX4IO_PAGE_SETUP:
		switch (offset) {
		case PX4IO_P_SETUP_FEATURES:

			value &= PX4IO_P_SETUP_FEATURES_VALID;
			r_setup_features = value;

			/* update manual override state - disable if no longer OK */
			if ((r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) && !(value & PX4IO_P_FEAT_ARMING_MANUAL_OVERRIDE_OK))
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_OVERRIDE;

			break;

		case PX4IO_P_SETUP_ARMING:

			value &= PX4IO_P_SETUP_ARMING_VALID;
			r_setup_arming = value;

			/* update arming state - disarm if no longer OK */
			if ((r_status_flags & PX4IO_P_STATUS_FLAGS_ARMED) && !(value & PX4IO_P_SETUP_ARMING_ARM_OK))
				r_status_flags &= ~PX4IO_P_STATUS_FLAGS_ARMED;
			break;

		case PX4IO_P_SETUP_PWM_RATES:
			value &= PX4IO_P_SETUP_RATES_VALID;
			r_setup_pwm_rates = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_PWM_LOWRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			r_setup_pwm_lowrate = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_PWM_HIGHRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			r_setup_pwm_highrate = value;
			/* XXX re-configure timers */
			break;

		case PX4IO_P_SETUP_RELAYS:
			value &= PX4IO_P_SETUP_RELAYS_VALID;
			r_setup_relays = value;
			POWER_RELAY1(value & (1 << 0) ? 1 : 0);
			POWER_RELAY2(value & (1 << 1) ? 1 : 0);
			POWER_ACC1(value & (1 << 2) ? 1 : 0);
			POWER_ACC2(value & (1 << 3) ? 1 : 0);
			break;

		default:
			return -1;
		}
		break;

	case PX4IO_PAGE_RC_CONFIG: {
		unsigned channel = offset / PX4IO_P_RC_CONFIG_STRIDE;
		unsigned index = offset % PX4IO_P_RC_CONFIG_STRIDE;
		uint16_t *conf = &r_page_rc_input_config[channel * PX4IO_P_RC_CONFIG_STRIDE];

		if (channel >= MAX_CONTROL_CHANNELS)
			return -1;

		/* disable the channel until we have a chance to sanity-check it */
		conf[PX4IO_P_RC_CONFIG_OPTIONS] &= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

		switch (index) {

		case PX4IO_P_RC_CONFIG_MIN:
		case PX4IO_P_RC_CONFIG_CENTER:
		case PX4IO_P_RC_CONFIG_MAX:
		case PX4IO_P_RC_CONFIG_DEADZONE:
		case PX4IO_P_RC_CONFIG_ASSIGNMENT:
			conf[index] = value;
			break;

		case PX4IO_P_RC_CONFIG_OPTIONS:
			value &= PX4IO_P_RC_CONFIG_OPTIONS_VALID;

			/* set all options except the enabled option */
			conf[index] = value & ~PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

			/* should the channel be enabled? */
			/* this option is normally set last */
			if (value & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) {
				/* assert min..center..max ordering */
				if (conf[PX4IO_P_RC_CONFIG_MIN] < 500)
					break;
				if (conf[PX4IO_P_RC_CONFIG_MAX] < 2500)
					break;
				if (conf[PX4IO_P_RC_CONFIG_CENTER] < conf[PX4IO_P_RC_CONFIG_MIN])
					break;
				if (conf[PX4IO_P_RC_CONFIG_CENTER] > conf[PX4IO_P_RC_CONFIG_MAX])
					break;
				/* assert deadzone is sane */
				if (conf[PX4IO_P_RC_CONFIG_DEADZONE] > 500)
					break;
				if (conf[PX4IO_P_RC_CONFIG_MIN] > (conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE]))
					break;
				if (conf[PX4IO_P_RC_CONFIG_MAX] < (conf[PX4IO_P_RC_CONFIG_CENTER] + conf[PX4IO_P_RC_CONFIG_DEADZONE]))
					break;
				if (conf[PX4IO_P_RC_CONFIG_ASSIGNMENT] >= MAX_CONTROL_CHANNELS)
					break;

				/* sanity checks pass, enable channel */
				conf[index] |= PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
			}
			break;

		}
	}

	default:
		return -1;
	}
	return 0;
}

int
registers_get(uint8_t page, uint8_t offset, uint16_t **values, unsigned *num_values)
{
#define SELECT_PAGE(_page_name)	{ *values = (uint16_t *)_page_name; *num_values = sizeof(_page_name) / sizeof(_page_name[0]); }

	switch (page) {

	/*
	 * Handle pages that are updated dynamically at read time.
	 */
	case PX4IO_PAGE_STATUS:
		/* PX4IO_P_STATUS_FREEMEM */
		{
			struct mallinfo minfo = mallinfo();
			r_page_status[PX4IO_P_STATUS_FREEMEM] = minfo.fordblks;
		}

		/* XXX PX4IO_P_STATUS_CPULOAD */

		/* PX4IO_P_STATUS_FLAGS maintained externally */

		/* PX4IO_P_STATUS_ALARMS maintained externally */

		/* PX4IO_P_STATUS_VBATT */
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
			unsigned mV = (4150 + (counts * 46)) / 10;
			unsigned corrected = (mV * r_page_setup[PX4IO_P_SETUP_VBATT_SCALE]) / 10000;

			r_page_status[PX4IO_P_STATUS_VBATT] = corrected;
		}

		/* PX4IO_P_STATUS_IBATT */
		{
			unsigned counts = adc_measure(ADC_VBATT);
			unsigned scaled = (counts * r_page_setup[PX4IO_P_SETUP_IBATT_SCALE]) / 10000;
			int corrected = scaled + REG_TO_SIGNED(r_page_setup[PX4IO_P_SETUP_IBATT_BIAS]);
			if (corrected < 0)
				corrected = 0;
			r_page_status[PX4IO_P_STATUS_IBATT] = corrected;
		}

		SELECT_PAGE(r_page_status);
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		r_page_adc[0] = adc_measure(ADC_VBATT);
		r_page_adc[1] = adc_measure(ADC_IN5);

		SELECT_PAGE(r_page_adc);
		break;

	/*
	 * Pages that are just a straight read of the register state.
	 */
#define COPY_PAGE(_page_name, _page)	case _page_name: SELECT_PAGE(_page); break

	/* status pages */
	COPY_PAGE(PX4IO_PAGE_CONFIG,		r_page_config);
	COPY_PAGE(PX4IO_PAGE_ACTUATORS,		r_page_actuators);
	COPY_PAGE(PX4IO_PAGE_SERVOS,		r_page_servos);
	COPY_PAGE(PX4IO_PAGE_RAW_RC_INPUT,	r_page_raw_rc_input);
	COPY_PAGE(PX4IO_PAGE_RC_INPUT,		r_page_rc_input);

	/* readback of input pages */
	COPY_PAGE(PX4IO_PAGE_SETUP,		r_page_setup);
	COPY_PAGE(PX4IO_PAGE_CONTROLS,		r_page_controls);
	COPY_PAGE(PX4IO_PAGE_RC_CONFIG,		r_page_rc_input_config);
	COPY_PAGE(PX4IO_PAGE_FAILSAFE_PWM,	r_page_servo_failsafe);

	default:
		return -1;
	}

#undef SELECT_PAGE

	/* if the offset is beyond the end of the page, we have no data */
	if (*num_values <= offset)
		return -1;

	/* adjust value count and pointer for the page offset */
	*num_values -= offset;
	*values += offset;

	return 0;
}
