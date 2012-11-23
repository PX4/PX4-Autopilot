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
#include <termios.h>
#include <string.h>
#include <poll.h>

#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <systemlib/hx_stream.h>
#include <systemlib/perf_counter.h>

#include "px4io.h"

#define FMU_MIN_REPORT_INTERVAL	  5000	/*  5ms */
#define FMU_MAX_REPORT_INTERVAL	100000	/* 100ms */

static int			fmu_fd;
static hx_stream_t		stream;
static int			rx_fd;
static struct px4io_report	report;

static void			comms_handle_frame(void *arg, const void *buffer, size_t length);

static struct pollfd pollfds[2];
static int pollcount;

void
comms_init(void)
{
	/* initialise the FMU interface */
	fmu_fd = open("/dev/ttyS1", O_RDWR | O_NONBLOCK);
	if (fmu_fd < 0)
		lib_lowprintf("COMMS: fmu open failed %d\n", errno);
	stream = hx_stream_init(fmu_fd, comms_handle_frame, NULL);
	pollfds[0].fd = fmu_fd;
	pollfds[0].events = POLLIN;
	pollcount = 1;

	/* default state in the report to FMU */
	report.i2f_magic = I2F_MAGIC;

}

static void
serial_rx_init(unsigned serial_mode)
{
	if (serial_mode == system_state.serial_rx_mode)
		return;
	system_state.serial_rx_mode = serial_mode;

	if (serial_mode == RX_MODE_PPM_ONLY) {
		if (rx_fd != -1) {
			pollcount = 1;
			close(rx_fd);
			rx_fd = -1;
		}
		return;
	}

	/* open the serial port used for DSM and S.bus communication */
	rx_fd = open("/dev/ttyS0", O_RDONLY | O_NONBLOCK);
	pollfds[1].fd = rx_fd;
	pollfds[1].events = POLLIN;
	pollcount = 2;

	struct termios t;
	tcgetattr(rx_fd, &t);

	switch (serial_mode) {
	case RX_MODE_DSM_10BIT:
	case RX_MODE_DSM_11BIT:

		/* 115200, no parity, one stop bit */
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);

		dsm_init(serial_mode);
		break;

	case RX_MODE_FUTABA_SBUS:
		/* 100000, even parity, two stop bits */
		cfsetspeed(&t, 100000);
		t.c_cflag |= (CSTOPB | PARENB);

		sbus_init(serial_mode);
		break;

	default:
		break;
	}

	tcsetattr(rx_fd, TCSANOW, &t);
}

void
comms_check(void)
{
	/*
	 * Check for serial data
	 */
	int ret = poll(pollfds, pollcount, 0);

	if (ret > 0) {
		/*
		 * Pull bytes from FMU and feed them to the HX engine.
		 * Limit the number of bytes we actually process on any one iteration.
		 */
		if (pollfds[0].revents & POLLIN) {
			char buf[32];
			ssize_t count = read(fmu_fd, buf, sizeof(buf));
			for (int i = 0; i < count; i++)
				hx_stream_rx(stream, buf[i]);
		}

		/*
		 * Pull bytes from the serial RX port and feed them to the decoder
		 * if we care about serial RX data.
		 */
		if ((pollcount > 1) && (pollfds[1].revents & POLLIN)) {
			switch (system_state.serial_rx_mode) {
			case RX_MODE_DSM_10BIT:
			case RX_MODE_DSM_11BIT:
				dsm_input(rx_fd);
				break;

			case RX_MODE_FUTABA_SBUS:
				sbus_input(rx_fd);
				break;

			default:
				break;
			}
		}
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
		report.channel_count = system_state.rc_channels;
		report.armed = system_state.armed;

		/* and send it */
		hx_stream_send(stream, &report, sizeof(report));
	}
}

int frame_rx;
int frame_bad;

static void
comms_handle_config(const void *buffer, size_t length)
{
	const struct px4io_config *cfg = (struct px4io_config *)buffer;

	if (length != sizeof(*cfg)) {
		frame_bad++;
		return;
	}

	frame_rx++;

	serial_rx_init(cfg->serial_rx_mode);
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
			break;
		}
	}
    	frame_bad++;
}

