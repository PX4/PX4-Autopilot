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
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include <drivers/drv_pwm_output.h>
#include <systemlib/mixer/mixer.h>

extern "C" {
//#define DEBUG
#include "px4io.h"
}

/*
 * Count of periodic calls in which we have no FMU input.
 */
static unsigned fmu_input_drops;
#define FMU_INPUT_DROP_LIMIT	20

/* current servo arm/disarm state */
bool mixer_servos_armed = false;

/* selected control values and count for mixing */
static uint16_t *control_values;
static int control_count;

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

static MixerGroup mixer_group(mixer_callback, 0);

void
mixer_tick(void)
{
	bool should_arm;

	/*
	 * Decide which set of inputs we're using.
	 */
	if (system_state.mixer_use_fmu) {
		/* we have recent control data from the FMU */
		control_count = PX4IO_CONTROL_CHANNELS;
		control_values = &system_state.fmu_channel_data[0];

		/* check that we are receiving fresh data from the FMU */
		if (!system_state.fmu_data_received) {
			fmu_input_drops++;

			/* too many frames without FMU input, time to go to failsafe */
			if (fmu_input_drops >= FMU_INPUT_DROP_LIMIT) {
				system_state.mixer_use_fmu = false;
			}

		} else {
			fmu_input_drops = 0;
			system_state.fmu_data_received = false;
		}

	} else if (system_state.rc_channels > 0) {
		/* we have control data from an R/C input */
		control_count = system_state.rc_channels;
		control_values = &system_state.rc_channel_data[0];

	} else {
		/* we have no control input */
		/* XXX builtin failsafe would activate here */
		control_count = 0;
	}

	/*
	 * Run the mixers if we have any control data at all.
	 */
	if (control_count > 0) {
		float	outputs[IO_SERVO_COUNT];
		unsigned mixed;

		/* mix */
		mixed = mixer_group.mix(&outputs[0], IO_SERVO_COUNT);

		/* scale to PWM and update the servo outputs as required */
		for (unsigned i = 0; i < IO_SERVO_COUNT; i++) {
			if (i < mixed) {
				/* scale to servo output */
				system_state.servos[i] = (outputs[i] * 500.0f) + 1500;

			} else {
				/* set to zero to inhibit PWM pulse output */
				system_state.servos[i] = 0;
			}

			/*
			 * If we are armed, update the servo output.
			 */
			if (system_state.armed && system_state.arm_ok)
				up_pwm_servo_set(i, system_state.servos[i]);
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 */
	should_arm = system_state.armed && system_state.arm_ok && (control_count > 0);

	if (should_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true);
		mixer_servos_armed = true;

	} else if (!should_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false);
		mixer_servos_armed = false;
	}
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	/* if the control index refers to an input that's not valid, we can't return it */
	if (control_index >= control_count)
		return -1;

	/* scale from current PWM units (1000-2000) to mixer input values */
	control = ((float)control_values[control_index] - 1500.0f) / 500.0f;

	return 0;
}

static char mixer_text[256];
static unsigned mixer_text_length = 0;

void
mixer_handle_text(const void *buffer, size_t length)
{

	px4io_mixdata	*msg = (px4io_mixdata *)buffer;

	debug("mixer text %u", length);

	if (length < sizeof(px4io_mixdata))
		return;

	unsigned	text_length = length - sizeof(px4io_mixdata);

	switch (msg->action) {
	case F2I_MIXER_ACTION_RESET:
		debug("reset");
		mixer_group.reset();
		mixer_text_length = 0;

		/* FALLTHROUGH */
	case F2I_MIXER_ACTION_APPEND:
		debug("append %d", length);

		/* check for overflow - this is really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text))
			return;

		/* append mixer text and nul-terminate */
		memcpy(&mixer_text[mixer_text_length], msg->text, text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		debug("buflen %u", mixer_text_length);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		unsigned resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {
			debug("used %u", mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0)
				memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);

			mixer_text_length = resid;
		}

		break;
	}
}