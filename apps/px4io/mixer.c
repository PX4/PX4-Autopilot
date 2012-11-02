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
 * @file Control channel input/output mixer and failsafe.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <arch/board/drv_ppm_input.h>
#include <arch/board/drv_pwm_servo.h>
#include <drivers/drv_hrt.h>

#include "px4io.h"

#ifdef CONFIG_DISABLE_MQUEUE
# error Mixer requires message queues - set CONFIG_DISABLE_MQUEUE=n and try again
#endif

static mqd_t	input_queue;

/*
 * Count of periodic calls in which we have no data.
 */
static unsigned mixer_input_drops;
#define MIXER_INPUT_DROP_LIMIT	10

/*
 * Count of periodic calls in which we have no FMU input.
 */
static unsigned fmu_input_drops;
#define FMU_INPUT_DROP_LIMIT	10

/*
 * HRT periodic call used to check for control input data.
 */
static struct hrt_call mixer_input_call;

/*
 * Mixer periodic tick.
 */
static void	mixer_tick(void *arg);

/*
 * Collect RC input data from the controller source(s).
 */
static void	mixer_get_rc_input(void);

/*
 * Update a mixer based on the current control signals.
 */
static void	mixer_update(int mixer, uint16_t *inputs, int input_count);

/* servo driver handle */
int mixer_servo_fd;

/* current servo arm/disarm state */
bool mixer_servos_armed;

/*
 * Each mixer consumes a set of inputs and produces a single output.
 */
struct mixer {
	uint16_t current_value;
	/* XXX more config here */
} mixers[IO_SERVO_COUNT];

int
mixer_init(const char *mq_name)
{
	/* open the control input queue; this should always exist */
	input_queue = mq_open(mq_name, O_RDONLY | O_NONBLOCK);
	ASSERTCODE((input_queue >= 0), A_INPUTQ_OPEN_FAIL);

	/* open the servo driver */
	mixer_servo_fd = open("/dev/pwm_servo", O_WRONLY);
	ASSERTCODE((mixer_servo_fd >= 0), A_SERVO_OPEN_FAIL);

	/* look for control data at 50Hz */
	hrt_call_every(&mixer_input_call, 1000, 20000, mixer_tick, NULL);

	return 0;
}

static void
mixer_tick(void *arg)
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
		for (i = 0; i < PX4IO_OUTPUT_CHANNELS; i++) {
			mixer_update(i, control_values, control_count);

			/*
			 * If we are armed, update the servo output.
			 */
			if (system_state.armed)
				ioctl(mixer_servo_fd, PWM_SERVO_SET(i), mixers[i].current_value);
		}

	}

	/*
	 * Decide whether the servos should be armed right now.
	 */
	should_arm = system_state.armed && (control_count > 0);
	if (should_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		ioctl(mixer_servo_fd, PWM_SERVO_ARM, 0);
		mixer_servos_armed = true;

	} else if (!should_arm && mixer_servos_armed) {
		/* armed but need to disarm*/
		ioctl(mixer_servo_fd, PWM_SERVO_DISARM, 0);		
		mixer_servos_armed = false;
	}
}

static void
mixer_get_rc_input(void)
{
	ssize_t len;

	/*
	 * Pull channel data from the message queue into the system state structure.
	 *
	 */
	len = mq_receive(input_queue, &system_state.rc_channel_data, sizeof(system_state.rc_channel_data), NULL);

	/*
	 * If we have data, update the count and status.
	 */
	if (len > 0) {
		system_state.rc_channels = len / sizeof(system_state.rc_channel_data[0]);
		mixer_input_drops = 0;

		system_state.fmu_report_due = true;
	} else {
		/*
		 * No data; count the 'frame drops' and once we hit the limit
		 * assume that we have lost input.
		 */
		if (mixer_input_drops < MIXER_INPUT_DROP_LIMIT) {
			mixer_input_drops++;

			/* if we hit the limit, stop pretending we have input and let the FMU know */
			if (mixer_input_drops == MIXER_INPUT_DROP_LIMIT) {
				system_state.rc_channels = 0;
				system_state.fmu_report_due = true;
			}
		}
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
