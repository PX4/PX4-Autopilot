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

#include "px4io.h"

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		200000

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
	 * Decide which set of inputs we're using.
	 */
	if (!system_state.mixer_manual_override && system_state.mixer_fmu_available) {
		/* we have recent control data from the FMU */
		control_count = PX4IO_OUTPUT_CHANNELS;
		control_values = &system_state.fmu_channel_data[0];

		/* check that we are receiving fresh data from the FMU */
		if ((hrt_absolute_time() - system_state.fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

			/* too many frames without FMU input, time to go to failsafe */
			system_state.mixer_manual_override = true;
			system_state.mixer_fmu_available = false;
		}

	} else if (system_state.rc_channels > 0 && system_state.manual_override_ok) {
		/* we have control data from an R/C input */
		control_count = system_state.rc_channels;
		control_values = &system_state.rc_channel_data[0];
	} else {
		/* we have no control input (no FMU, no RC) */

		// XXX builtin failsafe would activate here 
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
	 * A sufficient reason is armed state and either FMU or RC control inputs
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
