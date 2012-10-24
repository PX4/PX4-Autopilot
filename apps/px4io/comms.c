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
 * @file FMU communication for the PX4IO module.
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

#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <systemlib/hx_stream.h>
#include <systemlib/perf_counter.h>

#include "px4io.h"

#define FMU_MIN_REPORT_INTERVAL	  5000	/*  5ms */
#define FMU_MAX_REPORT_INTERVAL	100000	/* 100ms */

static int			fmu_fd;
static hx_stream_t		stream;

static struct px4io_report	report;

static void			comms_handle_frame(void *arg, const void *buffer, size_t length);

void
comms_init(void)
{
	fmu_fd = open("/dev/ttyS1", O_RDWR | O_NONBLOCK);
	if (fmu_fd < 0)
		lib_lowprintf("COMMS: fmu open failed %d\n", errno);

	stream = hx_stream_init(fmu_fd, comms_handle_frame, NULL);

	report.i2f_magic = I2F_MAGIC;
}

void
comms_check(void)
{
	static hrt_abstime last_report_time;
	hrt_abstime now, delta;
	uint8_t c;

	/* should we send a report to the FMU? */
	now = hrt_absolute_time();
	delta = now - last_report_time;
	if ((delta > FMU_MIN_REPORT_INTERVAL) && 
		(system_state.fmu_report_due || (delta > FMU_MAX_REPORT_INTERVAL))) {

		system_state.fmu_report_due = false;
		last_report_time = now;

		/* populate the report */
		for (unsigned i = 0; i < system_state.rc_channels; i++)
			report.rc_channel[i] = system_state.rc_channel_data[i];
		report.channel_count = system_state.rc_channels;
		report.armed = system_state.armed;

		/* and send it */
		hx_stream_send(stream, &report, sizeof(report));
	}

	/* feed any received bytes to the HDLC receive engine */
	while (read(fmu_fd, &c, 1) == 1)
		hx_stream_rx(stream, c);
}

static void
comms_handle_frame(void *arg, const void *buffer, size_t length)
{
	struct px4io_command *cmd;

	/* make sure it's what we are expecting */
	if (length != sizeof(struct px4io_command))
		return;

	cmd = (struct px4io_command *)buffer;

	/* fetch new PWM output values */
	for (unsigned i = 0; i < PX4IO_OUTPUT_CHANNELS; i++)
		system_state.fmu_channel_data[i] = cmd->servo_command[i];

	system_state.arm_ok = cmd->arm_ok;
	system_state.mixer_use_fmu = true;
	system_state.fmu_data_received = true;

	/* handle changes signalled by FMU */
	if (!system_state.arm_ok && system_state.armed)
		system_state.armed = false;

	/* XXX do relay changes here */	
	for (unsigned i = 0; i < PX4IO_RELAY_CHANNELS; i++)
		system_state.relays[i] = cmd->relay_state[i];
}
