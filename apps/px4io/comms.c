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
#include <systemlib/hx_stream.h>
#include <systemlib/perf_counter.h>

#define DEBUG
#include "px4io.h"

#define FMU_MIN_REPORT_INTERVAL	  5000	/*  5ms */
#define FMU_MAX_REPORT_INTERVAL	100000	/* 100ms */

int frame_rx;
int frame_bad;

static int			fmu_fd;
static hx_stream_t		stream;
static struct px4io_report	report;

static void			comms_handle_frame(void *arg, const void *buffer, size_t length);

static void
comms_init(void)
{
	/* initialise the FMU interface */
	fmu_fd = open("/dev/ttyS1", O_RDWR);
	stream = hx_stream_init(fmu_fd, comms_handle_frame, NULL);

	/* default state in the report to FMU */
	report.i2f_magic = I2F_MAGIC;

	struct termios t;

	/* 115200bps, no parity, one stop bit */
	tcgetattr(fmu_fd, &t);
	cfsetspeed(&t, 115200);
	t.c_cflag &= ~(CSTOPB | PARENB);
	tcsetattr(fmu_fd, TCSANOW, &t);
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
		/* wait for serial data, but no more than 100ms */
		poll(&fds, 1, 100);

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
			for (int i = 0; i < system_state.rc_channels; i++)
				report.rc_channel[i] = system_state.rc_channel_data[i];

			if (system_state.sbus_input_ok || system_state.dsm_input_ok || system_state.ppm_input_ok) {
				report.channel_count = system_state.rc_channels;
			} else {
				report.channel_count = 0;
			}
			
			report.armed = system_state.armed;

			/* and send it */
			hx_stream_send(stream, &report, sizeof(report));
		}
	}
}

static void
comms_handle_config(const void *buffer, size_t length)
{
	const struct px4io_config *cfg = (struct px4io_config *)buffer;

	if (length != sizeof(*cfg)) {
		frame_bad++;
		return;
	}

	frame_rx++;
}

static void
comms_handle_command(const void *buffer, size_t length)
{
	const struct px4io_command *cmd = (struct px4io_command *)buffer;

	if (length != sizeof(*cmd)) {
		frame_bad++;
		return;
	}

	frame_rx++;
	irqstate_t flags = irqsave();

	/* fetch new PWM output values */
	for (unsigned i = 0; i < PX4IO_OUTPUT_CHANNELS; i++)
		system_state.fmu_channel_data[i] = cmd->servo_command[i];

	/* if the IO is armed and the FMU gets disarmed, the IO must also disarm */
	if(system_state.arm_ok && !cmd->arm_ok) {
		system_state.armed = false;
	}

	system_state.arm_ok = cmd->arm_ok;
	system_state.mixer_use_fmu = true;
	system_state.fmu_data_received = true;


	/* handle changes signalled by FMU */
//	if (!system_state.arm_ok && system_state.armed)
//		system_state.armed = false;

	/* XXX do relay changes here */	
	for (unsigned i = 0; i < PX4IO_RELAY_CHANNELS; i++)
		system_state.relays[i] = cmd->relay_state[i];

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
		default:
		    	frame_bad++;
			break;
		}
	}
}

