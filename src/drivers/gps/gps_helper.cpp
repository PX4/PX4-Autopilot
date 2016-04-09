/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#ifndef __PX4_QURT
#include <termios.h>
#include <poll.h>
#else
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#endif

#include <unistd.h>
#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "gps_helper.h"

/**
 * @file gps_helper.cpp
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#define GPS_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls


float
GPS_Helper::get_position_update_rate()
{
	return _rate_lat_lon;
}

float
GPS_Helper::get_velocity_update_rate()
{
	return _rate_vel;
}

void
GPS_Helper::reset_update_rates()
{
	_rate_count_vel = 0;
	_rate_count_lat_lon = 0;
	_interval_rate_start = hrt_absolute_time();
}

void
GPS_Helper::store_update_rates()
{
	_rate_vel = _rate_count_vel / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);
	_rate_lat_lon = _rate_count_lat_lon / (((float)(hrt_absolute_time() - _interval_rate_start)) / 1000000.0f);
}

int
GPS_Helper::set_baudrate(const int &fd, unsigned baud)
{

#if __PX4_QURT
	// TODO: currently QURT does not support configuration with termios.
	dspal_serial_ioctl_data_rate data_rate;

	switch (baud) {
	case 9600: data_rate.bit_rate = DSPAL_SIO_BITRATE_9600; break;

	case 19200: data_rate.bit_rate = DSPAL_SIO_BITRATE_19200; break;

	case 38400: data_rate.bit_rate = DSPAL_SIO_BITRATE_38400; break;

	case 57600: data_rate.bit_rate = DSPAL_SIO_BITRATE_57600; break;

	case 115200: data_rate.bit_rate = DSPAL_SIO_BITRATE_115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	int ret = ::ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&data_rate);

	if (ret != 0) {

		return ret;
	}

#else
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return -1;
	}

#endif
	return 0;
}

int
GPS_Helper::poll_or_read(int fd, uint8_t *buf, size_t buf_length, uint64_t timeout)
{

#ifndef __PX4_QURT

	/* For non QURT, use the usual polling. */

	pollfd fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	/* Poll for new data,  */
	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

	if (ret > 0) {
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			usleep(GPS_WAIT_BEFORE_READ * 1000);
			return ::read(fd, buf, buf_length);

		} else {
			return -1;
		}

	} else {
		return ret;
	}

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	usleep(10000);
	return ::read(fd, buf, buf_length);
#endif
}
