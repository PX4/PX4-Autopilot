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
 * @file spektrum_rc.cpp
 *
 * This is a driver for a Spektrum satellite receiver connected to a Snapdragon
 * on the serial port. By default port J12 (next to J13, power module side) is used.
 */

#include <string.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <lib/rc/dsm.h>
#include <px4_log.h>
#include "drv_rc_input.h"
#include <drivers/drv_hrt.h>

#ifdef __PX4_QURT
#include <drivers/device/qurt/uart.h>
#endif

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>

// Snapdraogon: use J12 (next to J13, power module side)
#ifdef __PX4_QURT
#define SPEKTRUM_UART_DEVICE_PATH "7"
#else
#define SPEKTRUM_UART_DEVICE_PATH "/dev/tty-3"
#endif

#define UNUSED(x) (void)(x)

extern "C" { __EXPORT int spektrum_rc_main(int argc, char *argv[]); }


namespace spektrum_rc
{

volatile bool _task_should_exit = false;
static bool _is_running = false;
static px4_task_t _task_handle = -1;

int start();
int stop();
int info();
void usage();
void task_main(int argc, char *argv[]);

void fill_input_rc(uint16_t raw_rc_count, uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
		   hrt_abstime now, bool frame_drop, bool failsafe, unsigned frame_drops, int rssi,
		   input_rc_s &input_rc);

void task_main(int argc, char *argv[])
{
	const char *device_path = SPEKTRUM_UART_DEVICE_PATH;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;
	bool verbose = false;

	while ((ch = px4_getopt(argc, argv, "vd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		case 'v':
			PX4_INFO("Spektrum RC: Enabling verbose mode");
			verbose = true;
			break;

		default:
			break;
		}
	}

	int uart_fd = dsm_init(device_path);

	if (uart_fd < 0) {
		PX4_ERR("dsm init failed");
		return;

	} else if (verbose) {
		PX4_INFO("Spektrum RC: dsm_init succeeded");
	}

	orb_advert_t rc_pub = nullptr;

	// Use a buffer size of the double of the minimum, just to be safe.
	uint8_t rx_buf[2 * DSM_BUFFER_SIZE];

	_is_running = true;
	uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS];
	uint16_t raw_rc_count = 0;
	uint32_t loop_counter = 0;
	bool     print_msg = false;
	bool     first_correct_frame_received = false;
	int      newbytes = 0;

	// Main loop
	while (!_task_should_exit) {

		if (((loop_counter % 20) == 0) && verbose) { print_msg = true; }

		loop_counter++;

#ifdef __PX4_QURT
#define ASYNC_UART_READ_WAIT_US 2000
		// The UART read on SLPI is via an asynchronous service so specify a timeout
		// for the return. The driver will poll periodically until the read comes in
		// so this may block for a while. However, it will timeout if no read comes in.
		newbytes =  qurt_uart_read(uart_fd, (char *) &rx_buf[0], sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);
#else
		newbytes = read(uart_fd, &rx_buf[0], sizeof(rx_buf));
#endif

		uint8_t protocol_version = rx_buf[1] & 0x0F;

		if (newbytes <= 0) {
			if (print_msg) { PX4_INFO("Spektrum RC: Read no bytes from UART"); }

		} else if (((newbytes != DSM_FRAME_SIZE) ||
			    ((protocol_version != 0x02) && (protocol_version != 0x01))) &&
			   (! first_correct_frame_received)) {
			PX4_ERR("Spektrum RC: Invalid DSM frame. %d bytes. Protocol byte 0x%.2x",
				newbytes, rx_buf[1]);

		} else {
			if (print_msg) { PX4_INFO("Spektrum RC: Read %d bytes from UART", newbytes); }

			first_correct_frame_received = true;

			const hrt_abstime now = hrt_absolute_time();

			bool dsm_11_bit;
			unsigned frame_drops;
			int8_t dsm_rssi;

			// parse new data
			bool rc_updated = dsm_parse(now, rx_buf, newbytes, &raw_rc_values[0], &raw_rc_count,
						    &dsm_11_bit, &frame_drops, &dsm_rssi, input_rc_s::RC_INPUT_MAX_CHANNELS);
			UNUSED(dsm_11_bit);

			if (rc_updated) {
				if (print_msg) { PX4_INFO("Spektrum RC: DSM message parsed successfully"); }

				input_rc_s input_rc = {};

				fill_input_rc(raw_rc_count, raw_rc_values, now, false, false, frame_drops, dsm_rssi,
					      input_rc);

				if (rc_pub == nullptr) {
					rc_pub = orb_advertise(ORB_ID(input_rc), &input_rc);

				} else {
					if (print_msg) { PX4_INFO("Spektrum RC: Publishing input_rc"); }

					orb_publish(ORB_ID(input_rc), rc_pub, &input_rc);
				}
			}

			if (print_msg) {
				PX4_INFO("0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x",
					 rx_buf[0],
					 rx_buf[1],
					 rx_buf[2],
					 rx_buf[3],
					 rx_buf[4],
					 rx_buf[5],
					 rx_buf[6],
					 rx_buf[7],
					 rx_buf[8],
					 rx_buf[9],
					 rx_buf[10],
					 rx_buf[11],
					 rx_buf[12],
					 rx_buf[13],
					 rx_buf[14],
					 rx_buf[15]);
			}
		}

		print_msg = false;

		// sleep since no poll for qurt
		usleep(10000);
	}

	orb_unadvertise(rc_pub);
	dsm_deinit();

	_is_running = false;
}

void fill_input_rc(uint16_t raw_rc_count, uint16_t raw_rc_values[input_rc_s::RC_INPUT_MAX_CHANNELS],
		   hrt_abstime now, bool frame_drop, bool failsafe, unsigned frame_drops, int rssi,
		   input_rc_s &input_rc)
{
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_QURT;

	input_rc.channel_count = raw_rc_count;

	if (input_rc.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		input_rc.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < input_rc.channel_count; ++i) {
		input_rc.values[i] = raw_rc_values[i];

		if (raw_rc_values[i] != UINT16_MAX) {
			valid_chans++;
		}
	}

	input_rc.timestamp = now;
	input_rc.timestamp_last_signal = input_rc.timestamp;
	input_rc.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {

		input_rc.rssi = 255;

	} else {
		input_rc.rssi = rssi;
	}

	if (valid_chans == 0) {
		input_rc.rssi = 0;
	}

	input_rc.rc_failsafe = failsafe;
	input_rc.rc_lost = (valid_chans == 0);
	input_rc.rc_lost_frame_count = frame_drops;
	input_rc.rc_total_frame_count = 0;
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("spektrum_rc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

int info()
{
	PX4_INFO("running: %s", _is_running ? "yes" : "no");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: spektrum_rc {start|info|stop}");
}

} // namespace spektrum_rc


int spektrum_rc_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		spektrum_rc::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return spektrum_rc::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return spektrum_rc::stop();
	}

	else if (!strcmp(verb, "info")) {
		return spektrum_rc::info();
	}

	else {
		spektrum_rc::usage();
		return 1;
	}
}
