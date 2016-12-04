/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_spektrum_rc.cpp
 *
 * This is a wrapper around the Parrot Bebop bus driver of the DriverFramework. It sends the
 * motor and contol commands to the Bebop and reads its status and informations.
 */

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>

#include <errno.h>
#include <string.h>
#include <DevMgr.hpp>

#include <lib/rc/dsm.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/input_rc.h>

#define SPEKTRUM_UART_DEVICE_PATH	"/dev/serialABC"
#define SBUS_BUFFER_SIZE 128

extern "C" { __EXPORT int df_spektrum_rc_main(int argc, char *argv[]); }

namespace df_spektrum_rc
{

volatile bool _task_should_exit = false; // flag indicating if bebop esc control task should exit
static bool _is_running = false;         // flag indicating if bebop esc  app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

input_rc_s _rc_in;
float		_analog_rc_rssi_volt;
bool		_analog_rc_rssi_stable;

int start();
int stop();
int info();
void usage();
void task_main(int argc, char *argv[]);
void fill_rc_in(uint16_t raw_rc_count,
		uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
		hrt_abstime now, bool frame_drop, bool failsafe,
		unsigned frame_drops, int rssi = -1);

void task_main(int argc, char *argv[])
{
	// publications
	orb_advert_t    rc_pub = nullptr;
	uint8_t _rcs_buf[50];

	// important to keep these buffers out of the stack
	// as they might need to be accumulated over multiple
	// iterations of the inner loop
	uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];
	uint16_t raw_rc_count;
	unsigned frame_drops;
	bool dsm_11_bit;

	int uart_fd = dsm_init(SPEKTRUM_UART_DEVICE_PATH);

	_is_running = true;

	// Set up poll topic
	px4_pollfd_struct_t fds[1];
	fds[0].fd     = uart_fd;
	fds[0].events = POLLIN;

	// Main loop
	while (!_task_should_exit) {

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {

			int newbytes = ::read(uart_fd, &_rcs_buf[0], SBUS_BUFFER_SIZE);

			if (newbytes > 0) {

				hrt_abstime now = hrt_absolute_time();

				// parse new data
				bool rc_updated = dsm_parse(now, &_rcs_buf[0], newbytes, &raw_rc_values[0], &raw_rc_count,
							    &dsm_11_bit, &frame_drops, input_rc_s::RC_INPUT_MAX_CHANNELS);

				if (rc_updated) {

					// we have a new DSM frame. Publish it.
					_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_DSM;
					fill_rc_in(raw_rc_count, raw_rc_values, now,
						   false, false, frame_drops);

					if (rc_pub == nullptr) {
						rc_pub = orb_advertise(ORB_ID(input_rc), &_rc_in);

					} else {
						orb_publish(ORB_ID(input_rc), rc_pub, &_rc_in);
					}
				}
			}
		}
	}

	orb_unadvertise(rc_pub);

	_is_running = false;

}

void
fill_rc_in(uint16_t raw_rc_count,
	   uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
	   hrt_abstime now, bool frame_drop, bool failsafe,
	   unsigned frame_drops, int rssi)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values[i];

		if (raw_rc_values[i] != UINT16_MAX) {
			valid_chans++;
		}
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {

		/* set RSSI if analog RSSI input is present */
		if (_analog_rc_rssi_stable) {
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

int start()
{
	// Start the task to handle RC
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("spektrum_rc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return -1;
	}

	_is_running = true;
	return 0;
}

int stop()
{
	// Stop bebop motor control task
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	PX4_INFO("info");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_spektrum_rc 'start', 'info', 'stop'");
}

} /* df_spektrum_rc */


int
df_spektrum_rc_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_spektrum_rc::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_spektrum_rc::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_spektrum_rc::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_spektrum_rc::info();
	}

	else {
		df_spektrum_rc::usage();
		return 1;
	}

	return ret;
}
