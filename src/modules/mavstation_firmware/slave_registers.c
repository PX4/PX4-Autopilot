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
#include <string.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include "protocol.h"
#include "slave_registers.h"
#include "adc.h"
#include "appdebug.h"

/*
 * Register aliases.
 *
 * Handy aliases for registers that are widely used.
 */
#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]

#define r_setup_features	r_page_setup[PX4IO_P_SETUP_FEATURES]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]
#define r_setup_pwm_rates	r_page_setup[PX4IO_P_SETUP_PWM_RATES]
#define r_setup_pwm_defaultrate	r_page_setup[PX4IO_P_SETUP_PWM_DEFAULTRATE]
#define r_setup_pwm_altrate	r_page_setup[PX4IO_P_SETUP_PWM_ALTRATE]
#define r_setup_relays		r_page_setup[PX4IO_P_SETUP_RELAYS]

#define r_control_values	(&r_page_controls[0])

// Temporarily define this in here until there is some servo interface to put it
// in: XXX
#define MAVSTATION_SERVO_COUNT		8

static int	registers_set_one(uint8_t page, uint8_t offset, uint16_t value);
static void	pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate);

/**
 * PAGE 0
 *
 * Static configuration parameters.
 */
static uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= 1,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_SOFTWARE_VERSION]	= 1,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 3,	/* XXX hardcoded magic number */
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= 64,	/* XXX hardcoded magic number */
};

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
	[PX4IO_P_STATUS_IBATT]			= 0
};

/**
 * PAGE 3
 *
 * Servo PWM values
 */
uint16_t		r_page_servos[MAVSTATION_SERVO_COUNT];

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.
 * PAGE 7 PWM rate maps.
 */
uint16_t		r_page_scratch[32];

/**
 * PAGE 100
 *
 * Setup registers
 */
volatile uint16_t	r_page_setup[] =
{
	[PX4IO_P_SETUP_FEATURES]		= 0,
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[PX4IO_P_SETUP_PWM_ALTRATE]		= 200,
	[PX4IO_P_SETUP_VBATT_SCALE]		= 10000,
	[PX4IO_P_SETUP_SET_DEBUG]		= 0,
};

#define PX4IO_P_SETUP_FEATURES_VALID	(0)
#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_FMU_ARMED | \
					 PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK | \
					 PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK | \
					 PX4IO_P_SETUP_ARMING_IO_ARM_OK)
#define PX4IO_P_SETUP_RATES_VALID	((1 << MAVSTATION_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PX4IO_RELAY_CHANNELS) - 1)

/*
 * PAGE 102 does not have a buffer.
 */


void
slave_registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{

	switch (page) {

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

		case PX4IO_P_STATUS_FLAGS:
			r_status_flags = value;
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

			/* no implemented feature selection at this point */

			break;

		case PX4IO_P_SETUP_PWM_RATES:
			value &= PX4IO_P_SETUP_RATES_VALID;
			pwm_configure_rates(value, r_setup_pwm_defaultrate, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_DEFAULTRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			pwm_configure_rates(r_setup_pwm_rates, value, r_setup_pwm_altrate);
			break;

		case PX4IO_P_SETUP_PWM_ALTRATE:
			if (value < 50)
				value = 50;
			if (value > 400)
				value = 400;
			pwm_configure_rates(r_setup_pwm_rates, r_setup_pwm_defaultrate, value);
			break;

		case PX4IO_P_SETUP_SET_DEBUG:
			r_page_setup[PX4IO_P_SETUP_SET_DEBUG] = value;
			isr_debug(0, "set debug %u\n", (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG]);
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

uint8_t last_page;
uint8_t last_offset;

int slave_registers_get(uint8_t page, uint8_t offset, volatile uint16_t **values, unsigned *num_values)
{
#define SELECT_PAGE(_page_name)							\
	do {									\
		*values = &_page_name[0];					\
		*num_values = sizeof(_page_name) / sizeof(_page_name[0]);	\
	} while(0)

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
			unsigned counts = adc_measure(ADC_CHANNEL_VBATT);
			if (counts != 0xffff) {
				unsigned mV = (4150 + (counts * 46)) / 10 - 200;
				unsigned corrected = (mV * r_page_setup[PX4IO_P_SETUP_VBATT_SCALE]) / 10000;

				r_page_status[PX4IO_P_STATUS_VBATT] = corrected;
			}
		}

		/* PX4IO_P_STATUS_IBATT */
		{
			/*
			  note that we have no idea what sort of
			  current sensor is attached, so we just
			  return the raw 12 bit ADC value and let the
			  FMU sort it out, with user selectable
			  configuration for their sensor
			 */
			unsigned counts = adc_measure(ADC_CHANNEL_IN5);
			if (counts != 0xffff) {
				r_page_status[PX4IO_P_STATUS_IBATT] = counts;
			}
		}

		SELECT_PAGE(r_page_status);
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));
		r_page_scratch[0] = adc_measure(ADC_CHANNEL_VBATT);
		r_page_scratch[1] = adc_measure(ADC_CHANNEL_IN5);

		SELECT_PAGE(r_page_scratch);
		break;

	case PX4IO_PAGE_PWM_INFO:
		memset(r_page_scratch, 0, sizeof(r_page_scratch));
		for (unsigned i = 0; i < MAVSTATION_SERVO_COUNT; i++)
			r_page_scratch[PX4IO_RATE_MAP_BASE + i] = up_pwm_servo_get_rate_group(i);

		SELECT_PAGE(r_page_scratch);
		break;

	/*
	 * Pages that are just a straight read of the register state.
	 */

	/* status pages */
	case PX4IO_PAGE_CONFIG:
		SELECT_PAGE(r_page_config);
		break;
	case PX4IO_PAGE_SERVOS:
		SELECT_PAGE(r_page_servos);
		break;

	/* readback of input pages */
	case PX4IO_PAGE_SETUP:
		SELECT_PAGE(r_page_setup);
		break;

	default:
		return -1;
	}

#undef SELECT_PAGE
#undef COPY_PAGE

last_page = page;
last_offset = offset;

	/* if the offset is at or beyond the end of the page, we have no data */
	if (offset >= *num_values)
		return -1;

	/* correct the data pointer and count for the offset */
	*values += offset;
	*num_values -= offset;

	return 0;
}

/*
 * Helper function to handle changes to the PWM rate control registers.
 */
static void
pwm_configure_rates(uint16_t map, uint16_t defaultrate, uint16_t altrate)
{
	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < MAVSTATION_SERVO_COUNT; group++) {

			/* get the channel mask for this rate group */
			uint32_t mask = up_pwm_servo_get_rate_group(group);
			if (mask == 0)
				continue;

			/* all channels in the group must be either default or alt-rate */
			uint32_t alt = map & mask;

			if (pass == 0) {
				/* preflight */
				if ((alt != 0) && (alt != mask)) {
					/* not a legal map, bail with an alarm */
					r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
					return;
				}
			} else {
				/* set it - errors here are unexpected */
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_altrate) != OK)
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
				} else {
					if (up_pwm_servo_set_rate_group_update(group, r_setup_pwm_defaultrate) != OK)
						r_status_alarms |= PX4IO_P_STATUS_ALARMS_PWM_ERROR;
				}
			}
		}
	}
	r_setup_pwm_rates = map;
	r_setup_pwm_defaultrate = defaultrate;
	r_setup_pwm_altrate = altrate;
}


/* --- ELEMENT GETTERS --------------------------------------------------- */

uint8_t slave_registers_get_debug_level(void) {
	return r_page_setup[PX4IO_P_SETUP_SET_DEBUG];
}

uint8_t slave_registers_get_status_flags(void) {
	return r_page_setup[PX4IO_P_STATUS_FLAGS];
}

uint8_t slave_registers_get_setup_features(void) {
	return r_page_setup[PX4IO_P_SETUP_FEATURES];
}
