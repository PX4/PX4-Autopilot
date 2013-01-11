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
 * @file comms.c
 *
 * FMU communication for the PX4IO module.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <termios.h>

#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/hx_stream.h>
#include <systemlib/perf_counter.h>

#define DEBUG
#include "px4io.h"

#define FMU_MIN_REPORT_INTERVAL	  5000	/*  5ms */
#define FMU_MAX_REPORT_INTERVAL	100000	/* 100ms */

#define FMU_STATUS_INTERVAL	1000000	/* 100ms */

static int			fmu_fd;
static hx_stream_t		stream;
static struct px4io_report	report;

static void			comms_handle_frame(void *arg, const void *buffer, size_t length);

perf_counter_t			comms_rx_errors;

static void
comms_init(void)
{
	/* initialise the FMU interface */
	fmu_fd = open("/dev/ttyS1", O_RDWR);
	stream = hx_stream_init(fmu_fd, comms_handle_frame, NULL);

	comms_rx_errors = perf_alloc(PC_COUNT, "rx_err");
	hx_stream_set_counters(stream, 0, 0, comms_rx_errors);

	/* default state in the report to FMU */
	report.i2f_magic = I2F_MAGIC;

	struct termios t;

	/* 115200bps, no parity, one stop bit */
	tcgetattr(fmu_fd, &t);
	cfsetspeed(&t, 115200);
	t.c_cflag &= ~(CSTOPB | PARENB);
	tcsetattr(fmu_fd, TCSANOW, &t);

	/* init the ADC */
	adc_init();
}

void
comms_main(void)
{
	comms_init();

	struct pollfd fds;
	fds.fd = fmu_fd;
	fds.events = POLLIN;
	debug("FMU: ready");

	for (;;) {
		/* wait for serial data, but no more than 10ms */
		poll(&fds, 1, 10);

		/*
		 * Pull bytes from FMU and feed them to the HX engine.
		 * Limit the number of bytes we actually process on any one iteration.
		 */
		if (fds.revents & POLLIN) {
			char buf[32];
			ssize_t count = read(fmu_fd, buf, sizeof(buf));

			for (int i = 0; i < count; i++)
				hx_stream_rx(stream, buf[i]);
		}

		/*
		 * Decide if it's time to send an update to the FMU.
		 */
		static hrt_abstime last_report_time;
		hrt_abstime now, delta;

		/* should we send a report to the FMU? */
		now = hrt_absolute_time();
		delta = now - last_report_time;

		if ((delta > FMU_MIN_REPORT_INTERVAL) &&
		    (system_state.fmu_report_due || (delta > FMU_MAX_REPORT_INTERVAL))) {

			system_state.fmu_report_due = false;
			last_report_time = now;

			/* populate the report */
			for (unsigned i = 0; i < system_state.rc_channels; i++) {
				report.rc_channel[i] = system_state.rc_channel_data[i];
			}

			report.channel_count = system_state.rc_channels;
			report.armed = system_state.armed;

			report.battery_mv = system_state.battery_mv;
			report.adc_in = system_state.adc_in5;
			report.overcurrent = system_state.overcurrent;

			/* and send it */
			hx_stream_send(stream, &report, sizeof(report));
		}

		/*
		 * Fetch ADC values, check overcurrent flags, etc.
		 */
		static hrt_abstime last_status_time;

		if ((now - last_status_time) > FMU_STATUS_INTERVAL) {

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
			system_state.battery_mv = (4150 + (counts * 46)) / 10;

			system_state.adc_in5 = adc_measure(ADC_IN5);

			system_state.overcurrent =
				(OVERCURRENT_SERVO ? (1 << 0) : 0) |
				(OVERCURRENT_ACC   ? (1 << 1) : 0);

			last_status_time = now;
		}
	}
}

static void
comms_handle_config(const void *buffer, size_t length)
{
	const struct px4io_config *cfg = (struct px4io_config *)buffer;

	if (length != sizeof(*cfg))
		return;

	/* fetch the rc mappings */
	for (unsigned i = 0; i < 4; i++) {
		system_state.rc_map[i] = cfg->rc_map[i];
	}

	/* fetch the rc channel attributes */
	for (unsigned i = 0; i < 4; i++) {
		system_state.rc_min[i]  = cfg->rc_min[i];
		system_state.rc_trim[i] = cfg->rc_trim[i];
		system_state.rc_max[i]  = cfg->rc_max[i];
		system_state.rc_rev[i]  = cfg->rc_rev[i];
		system_state.rc_dz[i]   = cfg->rc_dz[i];
	}
}

static void
comms_handle_command(const void *buffer, size_t length)
{
	const struct px4io_command *cmd = (struct px4io_command *)buffer;

	if (length != sizeof(*cmd))
		return;

	irqstate_t flags = irqsave();

	/* fetch new PWM output values */
	for (unsigned i = 0; i < PX4IO_CONTROL_CHANNELS; i++)
		system_state.fmu_channel_data[i] = cmd->output_control[i];

	/* if the IO is armed and the FMU gets disarmed, the IO must also disarm */
	if (system_state.arm_ok && !cmd->arm_ok)
		system_state.armed = false;

	system_state.arm_ok = cmd->arm_ok;
	system_state.vector_flight_mode_ok = cmd->vector_flight_mode_ok;
	system_state.manual_override_ok = cmd->manual_override_ok;
	system_state.mixer_fmu_available = true;
	system_state.fmu_data_received_time = hrt_absolute_time();

	/* set PWM update rate if changed (after limiting) */
	uint16_t new_servo_rate = cmd->servo_rate;

	/* reject faster than 500 Hz updates */
	if (new_servo_rate > 500) {
		new_servo_rate = 500;
	}

	/* reject slower than 50 Hz updates */
	if (new_servo_rate < 50) {
		new_servo_rate = 50;
	}

	if (system_state.servo_rate != new_servo_rate) {
		up_pwm_servo_set_rate(new_servo_rate);
		system_state.servo_rate = new_servo_rate;
	}

	/*
	 * update servo values immediately.
	 * the updates are done in addition also
	 * in the mainloop, since this function will only
	 * update with a connected FMU.
	 */
	mixer_tick();

	/* handle relay state changes here */
	for (unsigned i = 0; i < PX4IO_RELAY_CHANNELS; i++) {
		if (system_state.relays[i] != cmd->relay_state[i]) {
			switch (i) {
			case 0:
				POWER_ACC1(cmd->relay_state[i]);
				break;

			case 1:
				POWER_ACC2(cmd->relay_state[i]);
				break;

			case 2:
				POWER_RELAY1(cmd->relay_state[i]);
				break;

			case 3:
				POWER_RELAY2(cmd->relay_state[i]);
				break;
			}
		}

		system_state.relays[i] = cmd->relay_state[i];
	}

	irqrestore(flags);
}

static void
comms_handle_frame(void *arg, const void *buffer, size_t length)
{
	const uint16_t *type = (const uint16_t *)buffer;


	/* make sure it's what we are expecting */
	if (length > 2) {
		switch (*type) {
		case F2I_MAGIC:
			comms_handle_command(buffer, length);
			break;

		case F2I_CONFIG_MAGIC:
			comms_handle_config(buffer, length);
			break;

		case F2I_MIXER_MAGIC:
			mixer_handle_text(buffer, length);
			break;

		default:
			break;
		}
	}
}

