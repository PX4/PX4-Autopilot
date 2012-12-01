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
 * @file mixer.c
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

#include <systemlib/ppm_decode.h>

#include "px4io.h"

/*
 * Count of periodic calls in which we have no FMU input.
 */
static unsigned fmu_input_drops;
#define FMU_INPUT_DROP_LIMIT	20
/*
 * Collect RC input data from the controller source(s).
 */
static void	mixer_get_rc_input(void);

/*
 * Update a mixer based on the current control signals.
 */
static void	mixer_update(int mixer, uint16_t *inputs, int input_count);

/* current servo arm/disarm state */
bool mixer_servos_armed = false;

/*
 * Each mixer consumes a set of inputs and produces a single output.
 */
struct mixer {
	uint16_t current_value;
	/* XXX more config here */
} mixers[IO_SERVO_COUNT];

void
mixer_tick(void)
{
	uint16_t *control_values;
	int control_count;
	int i;
	bool should_arm;

	/*
	 * Start by looking for R/C control inputs.
	 * This updates system_state with any control inputs received.
	 */
	mixer_get_rc_input();

	/*
	 * Decide which set of inputs we're using.
	 */
	if (system_state.mixer_use_fmu) {
		/* we have recent control data from the FMU */
		control_count = PX4IO_OUTPUT_CHANNELS;
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
		control_count = 0;
	}
	/*
	 * Tickle each mixer, if we have control data.
	 */
	if (control_count > 0) {
		for (i = 0; i < IO_SERVO_COUNT; i++) {
			mixer_update(i, control_values, control_count);

			/*
			 * If we are armed, update the servo output.
			 */
			if (system_state.armed && system_state.arm_ok)
				up_pwm_servo_set(i, mixers[i].current_value);
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 */

	should_arm = system_state.armed && system_state.arm_ok && (control_count > 0) && system_state.mixer_use_fmu;
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

static void
mixer_update(int mixer, uint16_t *inputs, int input_count)
{
	/* simple passthrough for now */
	if (mixer < input_count) {
		mixers[mixer].current_value = inputs[mixer];
	} else {
		mixers[mixer].current_value = 0;
	}
}

static void
mixer_get_rc_input(void)
{
	/* if we haven't seen any new data in 200ms, assume we have lost input and tell FMU */
	if ((hrt_absolute_time() - ppm_last_valid_decode) > 200000) {

		/* input was ok and timed out, mark as update */
		if (system_state.ppm_input_ok) {
			system_state.ppm_input_ok = false;
			system_state.fmu_report_due = true;
		}
		return;
	}

	/* mark PPM as valid */
	system_state.ppm_input_ok = true;

	/* check if no DSM and S.BUS data is available */
	if (!system_state.sbus_input_ok && !system_state.dsm_input_ok) {
		/* otherwise, copy channel data */
		system_state.rc_channels = ppm_decoded_channels;
		for (unsigned i = 0; i < ppm_decoded_channels; i++)
			system_state.rc_channel_data[i] = ppm_buffer[i];
		system_state.fmu_report_due = true;
	}
}
