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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/ppm_decode.h>

#include "px4io.h"

/*
 * Count of periodic calls in which we have no data.
 */
static unsigned mixer_input_drops;
#define MIXER_INPUT_DROP_LIMIT	10

/*
 * Count of periodic calls in which we have no FMU input.
 */
static unsigned fmu_input_drops;
#define FMU_INPUT_DROP_LIMIT	20

/*
 * Serial port fd for serial RX protocols
 */
static int rx_port = -1;

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
mixer_init(void)
{
	/* open the serial port */
	rx_port = open("/dev/ttyS0", O_RDONLY | O_NONBLOCK);

	/* look for control data at 50Hz */
	hrt_call_every(&mixer_input_call, 1000, 20000, mixer_tick, NULL);

	return 0;
}

void
mixer_set_serial_mode(uint8_t serial_mode)
{

	if (serial_mode == system_state.serial_rx_mode)
		return;

	struct termios t;
	tcgetattr(rx_port, &t);

	switch (serial_mode) {
	case RX_MODE_PPM_ONLY:
		break;
	case RX_MODE_SPEKTRUM_6:
	case RX_MODE_SPEKTRUM_7:
		/* 115200, no parity, one stop bit */
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		break;
	case RX_MODE_FUTABA_SBUS:
		/* 100000, even parity, two stop bits */
		cfsetspeed(&t, 100000);
		t.c_cflag |= (CSTOPB | PARENB);
		break;
	default:
		return;
	}

	tcsetattr(rx_port, TCSANOW, &t);
	system_state.serial_rx_mode = serial_mode;
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
				up_pwm_servo_set(i, mixers[i].current_value);
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 */
	should_arm = system_state.armed && (control_count > 0);
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

static bool
mixer_get_spektrum_input(void)
{
	static uint8_t buf[16];
	static unsigned count;

	/* always read as much data as we can into the buffer */
	if (count >= sizeof(buf))
		count = 0;
	ssize_t result = read(rx_port, buf, sizeof(buf) - count);
	/* no data or an error */
	if (result <= 0)
		return false;
	count += result;

	/* if there are more than two bytes in the buffer, check for sync */
	if (count >= 2) {
		if ((buf[0] != 0x3) || (buf[1] != 0x1)) {
			/* not in sync; look for a possible sync marker */
			for (unsigned i = 1; i < count; i++) {
				if (buf[i] == 0x3) {
					/* could be a frame marker; move buffer bytes */
					count -= i;
					memmove(buf, buf + i, count);
					break;
				}
			}
		}
	}
	if (count < sizeof(buf))
		return false;

	/* we got a frame; decode it */
	const uint16_t *channels = (const uint16_t *)&buf[2];

	/*
	 * Channel assignment for DX6i vs. DX7 is different.
	 *
	 * DX7 etc. is:
	 *
	 * 0: Aileron
	 * 1: Flaps
	 * 2: Gear
	 * 3: Elevator
	 * 4: Aux2
	 * 5: Throttle
	 * 6: Rudder
	 *
	 * DX6i is:
	 *
	 * 0: Aileron	
	 * 1: Flaps
	 * 2: Elevator
	 * 3: Rudder
	 * 4: Throttle
	 * 5: Gear
	 * 6: <notused>
	 *
	 * We convert these to our standard Futaba-style assignment:
	 *
	 * 0: Throttle  (Throttle)
	 * 1: Roll      (Aileron)
	 * 2: Pitch     (Elevator)
	 * 3: Yaw       (Rudder)
	 * 4: Override  (Flaps)
	 * 5: FUNC_0    (Gear)
	 * 6: FUNC_1    (Aux2)
	 */
	if (system_state.serial_rx_mode == RX_MODE_SPEKTRUM_7) {
		system_state.rc_channel_data[0] = channels[5]; /* Throttle */
		system_state.rc_channel_data[1] = channels[0]; /* Roll */
		system_state.rc_channel_data[2] = channels[3]; /* Pitch */
		system_state.rc_channel_data[3] = channels[6]; /* Yaw */
		system_state.rc_channel_data[4] = channels[1]; /* Override */
		system_state.rc_channel_data[5] = channels[2]; /* FUNC_0 */
		system_state.rc_channel_data[6] = channels[4]; /* FUNC_1 */
		system_state.rc_channels = 7;
	} else {
		system_state.rc_channel_data[0] = channels[4]; /* Throttle */
		system_state.rc_channel_data[1] = channels[0]; /* Roll */
		system_state.rc_channel_data[2] = channels[2]; /* Pitch */
		system_state.rc_channel_data[3] = channels[3]; /* Yaw */
		system_state.rc_channel_data[4] = channels[1]; /* Override */
		system_state.rc_channel_data[5] = channels[5]; /* FUNC_0 */
		system_state.rc_channels = 6;
	}
	count = 0;
	return true;
}

static bool
mixer_get_sbus_input(void)
{
	/* XXX not implemented yet */
	return false;
}

static void
mixer_get_rc_input(void)
{
	bool got_input = false;

	switch (system_state.serial_rx_mode) {
	case RX_MODE_PPM_ONLY:
		if (ppm_decoded_channels > 0) {
			/* copy channel data */
			system_state.rc_channels = ppm_decoded_channels;
			for (unsigned i = 0; i < ppm_decoded_channels; i++)
				system_state.rc_channel_data[i] = ppm_buffer[i];
			got_input = true;
		}
		break;

	case RX_MODE_SPEKTRUM_6:
	case RX_MODE_SPEKTRUM_7:
		got_input = mixer_get_spektrum_input();
		break;

	case RX_MODE_FUTABA_SBUS:
		got_input = mixer_get_sbus_input();
		break;

	default:
		break;
	}

	if (got_input) {
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
