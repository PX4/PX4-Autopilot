/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ft7_technologies.hpp"

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

Ft7Technologies::Ft7Technologies(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	_sensor_airflow_pub.advertise();
}

Ft7Technologies::~Ft7Technologies()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
Ft7Technologies::init()
{
	_interval =  1e6 / 10; // 10 Hz, The maximum query frequency is 10Hz
	start();

	return PX4_OK;
}

int Ft7Technologies::measure()
{
	// Send the command to begin a measurement.
	const char cmd[] = {'$', '/', '/', ',', 'W', 'V', '?', '*', '/', '/', '\r', '\n', '\0'};
	int ret = ::write(_fd, cmd, 12);

	perf_begin(_sample_perf);

	if (ret != 12) {
		perf_count(_comms_errors);
		return ret;
	}

	_last_measure = hrt_absolute_time();

	return PX4_OK;
}

uint8_t Ft7Technologies::hex2int(char ch)
{
	if (ch >= '0' && ch <= '9') {
		return ch - '0';
	}

	if (ch >= 'A' && ch <= 'F') {
		return ch - 'A' + 10;
	}

	if (ch >= 'a' && ch <= 'f') {
		return ch - 'a' + 10;
	}

	return -1;
}

bool Ft7Technologies::checksum(char *buf, uint32_t checksum)
{

	uint32_t checksum_verify = 0;

	for (int i = 0; i < _byte_counter; i++) {

		if (!(buf[i] == '$' || buf[i] == '*')) {
			checksum_verify ^= buf[i];
		}

		if (buf[i] == '*') {
			i = _byte_counter;
		}
	}

	return checksum_verify == checksum;
}

int Ft7Technologies::collect()
{
	// PX4_INFO("collect");

	/* clear buffer if last read was too long ago */
	// int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	char readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	/* read from the sensor (uart buffer) */
	// const hrt_abstime timestamp_sample = hrt_absolute_time();

	int ret = ::read(_fd, readbuf, readlen);

	// PX4_INFO("collect() ret: %d \n", ret);

	if (ret < 0) {
		// PX4_INFO("read err: %d", ret);
		return -EAGAIN;

	} else if (ret == 0) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	bool valid = false;

	//$<talkerID>,WVP=<speed>,<angle>,<status>*<checksum><cr><lf>
	for (int i = 0; i < ret; i++) {
		// _px4_windsensor.update(timestamp_sample, (double)ret, 13.0f, _status);
		// received a full message
		_readbuf[_byte_counter]  = readbuf[i];
		_byte_counter += 1;


		if (readbuf[i] == '\n') {

			_checksum = (uint32_t)atoi((char *)_raw_checksum);
			_hex_checksum = 0;

			sensor_airflow_s sensor_airflow{};
			sensor_airflow.timestamp  = hrt_absolute_time();
			sensor_airflow.speed  = (float)atoi((char *)_raw_speed) / 10.0f;
			sensor_airflow.direction = ((float)atoi((char *)_raw_angle) - 180.0f) * M_PI_F / 180.0f;
			sensor_airflow.status = (uint8_t)atoi((char *)_raw_status);

			if (_checksum_counter == 2) {
				_hex_checksum = hex2int(_raw_checksum[0]) << 4 | hex2int(_raw_checksum[1]);

			} else {
				_hex_checksum = hex2int(_raw_checksum[0]);
			}

			// checksum is verified
			if (checksum(_readbuf, _hex_checksum)) {

				_sensor_airflow_pub.publish(sensor_airflow);
				valid = true;

			}

			// reset counters
			_msg_part_counter = 0;
			_byte_counter = 0;
			_msg_byte_counter = 0;
			_checksum_counter = 0;
			memset(readbuf, 0, sizeof(_linebuf));
			memset(_readbuf, 0, sizeof(_linebuf));
			memset(_raw_speed, 0, 5);
			memset(_raw_angle, 0, 5);
			memset(_raw_status, 0, 2);
			memset(_raw_checksum, 0, 3);

		}

		else if (readbuf[i] == '$' || readbuf[i] == ',' || readbuf[i] == '=' || readbuf[i] == '*' || readbuf[i] == '\r') {
			_msg_part_counter += 1;
			_msg_byte_counter = 0;

		} else {

			if (readbuf[i] != '.') {

				if (_msg_part_counter == 3) { // speed measurement
					_raw_speed[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 4) { // angle measurement
					_raw_angle[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 5) { // status
					_raw_status[_msg_byte_counter] = readbuf[i];

				} else if (_msg_part_counter == 6) { // checksum
					_checksum_counter += 1;
					_raw_checksum[_msg_byte_counter] = readbuf[i];

				}

				_msg_byte_counter += 1;

			}

		}

	}

	if (!valid) {
		return -EAGAIN;
	}

	perf_end(_sample_perf);
	return PX4_OK;
}

void Ft7Technologies::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void Ft7Technologies::stop()
{
	ScheduleClear();
}

void Ft7Technologies::Run()
{

	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		// uart_config.c_oflag &= ~ONLCR;
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

		/* no parity, one stop bit */
		uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

		// No line processing:
		// echo off, echo newline off, canonical mode off,
		// extended input processing off, signal chars off
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		// Turn off character processing
		// clear current char size mask, no parity checking,
		// no output processing, force 8 bit input
		uart_config.c_cflag &= ~(CSIZE | PARENB);

		uart_config.c_cflag |= CS8;

		unsigned speed = B9600;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}

	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {

			if (hrt_elapsed_time(&_last_measure) > (_interval * 15) / 10) {
				// resend the command again
				// we waited to long to receive a response from the sensor
				// so we are resending the command again
				_collect_phase = false;
			}

			ScheduleNow();
			return;
			// we received a valid response from the sensor. we need to
			// send another command through

		} else if (collect_ret == PX4_OK) {

			/* next phase is measurement */
			_collect_phase = false;
		}

	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_INFO("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void Ft7Technologies::print_info()
{
	PX4_INFO_RAW("%s: port: %s\n", MODULE_NAME, _port);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
