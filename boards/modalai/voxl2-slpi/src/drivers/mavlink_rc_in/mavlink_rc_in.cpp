/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <string>
#include <stdint.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/qurt/uart.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/radio_status.h>
#include "uORB/uORBManager.hpp"
#include <mavlink.h>
#include <px4_log.h>
#include <lib/rc/crsf.h>
#include <lib/perf/perf_counter.h>


#define ASYNC_UART_READ_WAIT_US 500
#define RC_INPUT_RSSI_MAX	100

#define TX_BUFFER_LEN 20

extern "C" { __EXPORT int mavlink_rc_in_main(int argc, char *argv[]); }

namespace mavlink_rc_in
{

static bool _is_running = false;
volatile bool _task_should_exit = false;

static px4_task_t _task_handle = -1;
static px4_task_t _mav_task_handle = -1;

static int _uart_fd = -1;

static bool debug = false;
static bool mav_en = true;
static bool crsf_en = false;
static bool fake_heartbeat_enable = true;
static bool filter_local_rc_messages = true;
static bool dump_received_messages = false;
static bool dump_transmitted_messages = false;

std::string port = "7";
uint32_t baudrate = 115200;

uORB::PublicationMulti<input_rc_s> _rc_pub{ORB_ID(input_rc)};
uORB::PublicationMulti<radio_status_s> _radio_status_pub{ORB_ID(radio_status)};

perf_counter_t	_perf_rx_rate = nullptr;

static uint8_t _rc_lq;
static uint8_t _rc_rssi_dbm;

int open_port(const char *dev, speed_t speed);
int close_port();

int write_response(void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
int status();

void usage();
void task_main(int argc, char *argv[]);

void handle_message_dsp(mavlink_message_t *msg);
void handle_message_rc_channels_override_dsp(mavlink_message_t *msg);
void handle_message_radio_status_dsp(mavlink_message_t *msg);

void handle_message_dsp(mavlink_message_t *msg)
{
	if (debug) {
		if (filter_local_rc_messages) {
			if ((msg->msgid != MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) &&
			    (msg->msgid != MAVLINK_MSG_ID_RADIO_STATUS)) {
				PX4_INFO("**^^ MSG ID: %d ^^*****************************************************", msg->msgid);
			}

		} else {
			PX4_INFO("msg ID: %d", msg->msgid);
		}
	}

	switch (msg->msgid) {
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override_dsp(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status_dsp(msg);
		break;

	default:
		break;
	}
}

void mavlink_out_task(int argc, char *argv[])
{
	uint64_t last_heartbeat_timestamp = hrt_absolute_time();

	while (! _task_should_exit) {
		uint64_t timestamp = hrt_absolute_time();
		int nwrite = 0;

		// We need to provide heartbeat messages to the receiver. Otherwise it
		// won't send us anything!
		if ((timestamp - last_heartbeat_timestamp) > 1000000) {
			mavlink_heartbeat_t hb = {};
			mavlink_message_t hb_message = {};
			hb.type = MAV_TYPE_QUADROTOR;
			hb.autopilot = MAV_AUTOPILOT_PX4;
			mavlink_msg_heartbeat_encode(1, 1, &hb_message, &hb);

			uint8_t  hb_newBuf[MAVLINK_MAX_PACKET_LEN];
			uint16_t hb_newBufLen = 0;
			hb_newBufLen = mavlink_msg_to_send_buffer(hb_newBuf, &hb_message);

			nwrite = qurt_uart_write(_uart_fd, (char *) &hb_newBuf[0], hb_newBufLen);

			if (nwrite != hb_newBufLen) {
				PX4_ERR("Heartbeat write failed. Expected %d, got %d", hb_newBufLen, nwrite);

			} else if (debug) {
				PX4_INFO("Sending local heartbeat to TBS");

				if (dump_transmitted_messages) {
					for (int i = 0; i <= hb_newBufLen; i += 16) {
						PX4_INFO("  %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x",
							 hb_newBuf[i + 0],  hb_newBuf[i + 1],  hb_newBuf[i + 2],  hb_newBuf[i + 3],
							 hb_newBuf[i + 4],  hb_newBuf[i + 5],  hb_newBuf[i + 6],  hb_newBuf[i + 7],
							 hb_newBuf[i + 8],  hb_newBuf[i + 9],  hb_newBuf[i + 10], hb_newBuf[i + 11],
							 hb_newBuf[i + 12], hb_newBuf[i + 13], hb_newBuf[i + 14], hb_newBuf[i + 15]);
					}
				}
			}

			last_heartbeat_timestamp = timestamp;
		}

		px4_usleep(100000);
	}
}

void task_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "rtdmclfp:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			debug = true;
			PX4_INFO("Setting debug flag on");
			break;

		case 'm':
			mav_en = true;
			crsf_en = false;

			if (debug) { PX4_INFO("Using MAVLink mode"); }

			break;

		case 'c':
			mav_en = false;
			crsf_en = true;

			if (debug) { PX4_INFO("Using CRSF mode"); }

			break;

		case 'p':
			port = myoptarg;

			if (debug) { PX4_INFO("Setting port to %s", port.c_str()); }

			break;

		case 'b':
			baudrate = atoi(myoptarg);

			if (debug) { PX4_INFO("Setting baudrate to %u", baudrate); }

			break;

		case 'f':
			fake_heartbeat_enable = false;

			if (debug) { PX4_INFO("Disabling fake heartbeats"); }

			break;

		case 'l':
			filter_local_rc_messages = true;

			if (debug) { PX4_INFO("Filtering debug messages from the RC receiver"); }

			break;

		case 'r':
			dump_received_messages = true;

			if (debug) { PX4_INFO("Enabling hex dump of received messages"); }

			break;

		case 't':
			dump_transmitted_messages = true;

			if (debug) { PX4_INFO("Enabling hex dump of transmitted messages"); }

			break;

		default:
			break;
		}
	}

	if (open_port(port.c_str(), (speed_t) baudrate) == -1) {
		PX4_ERR("Failed to open UART");
		return;
	}

	_mav_task_handle = px4_task_spawn_cmd("mavlink_rc_in_mav",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT,
					      2000,
					      (px4_main_t)&mavlink_out_task,
					      (char *const *)argv);

	if (_mav_task_handle < 0) {
		PX4_ERR("_mav_task_handle task start failed");
		return;
	}

	_is_running = true;
	_perf_rx_rate = perf_alloc(PC_INTERVAL, "mavlink_rc_in: rx interval");

	uint8_t rx_buf[1024];
	mavlink_message_t msg;
	mavlink_status_t status{};

	while (! _task_should_exit) {
		// Check for incoming messages from the TBS Crossfire receiver
		int nread = qurt_uart_read(_uart_fd, (char *) rx_buf, sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);

		if (nread) {
			if (debug) {
				PX4_INFO("TBS Crossfire (MAVLink mode): read %d bytes", nread);

				if (dump_received_messages) {
					for (int i = 0; i <= nread; i += 16) {
						PX4_INFO("  %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x",
							 rx_buf[i + 0],  rx_buf[i + 1],  rx_buf[i + 2],  rx_buf[i + 3],
							 rx_buf[i + 4],  rx_buf[i + 5],  rx_buf[i + 6],  rx_buf[i + 7],
							 rx_buf[i + 8],  rx_buf[i + 9],  rx_buf[i + 10], rx_buf[i + 11],
							 rx_buf[i + 12], rx_buf[i + 13], rx_buf[i + 14], rx_buf[i + 15]);
					}
				}
			}

			//Take buffer and convert it into mavlink msg
			for (int i = 0; i < nread; i++) {
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
					handle_message_dsp(&msg);
				}
			}
		}

		px4_usleep(5000);
	}
}

void handle_message_radio_status_dsp(mavlink_message_t *msg)
{
	if (debug) { PX4_INFO("Radio status msg received"); }

	mavlink_radio_status_t rstatus;
	mavlink_msg_radio_status_decode(msg, &rstatus);

	radio_status_s status{};

	status.timestamp = hrt_absolute_time();
	status.rssi = rstatus.rssi;
	status.remote_rssi = rstatus.remrssi;
	status.txbuf = rstatus.txbuf;
	status.noise = rstatus.noise;
	status.remote_noise = rstatus.remnoise;
	status.rxerrors = rstatus.rxerrors;
	status.fix = rstatus.fixed;

	_radio_status_pub.publish(status);
}

void handle_message_rc_channels_override_dsp(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	if (debug) { PX4_INFO("RC channels override msg received"); }

	// Check target
	if (man.target_system != 0) {
		PX4_ERR("Message has incorrect target system %u", man.target_system);
		return;
	}

	// fill uORB message
	input_rc_s rc{};

	// metadata
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;
	rc.rssi = RC_INPUT_RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;

	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

	// check how many channels are valid
	for (int i = 17; i >= 0; i--) {
		const bool ignore_max = rc.values[i] == UINT16_MAX; // ignore any channel with value UINT16_MAX
		const bool ignore_zero = (i > 7) && (rc.values[i] == 0); // ignore channel 8-18 if value is 0

		if (ignore_max || ignore_zero) {
			// set all ignored values to zero
			rc.values[i] = 0;

		} else {
			// first channel to not ignore -> set count considering zero-based index
			rc.channel_count = i + 1;
			break;
		}
	}

	// publish uORB message
	_rc_pub.publish(rc);
}

int open_port(const char *dev, speed_t speed)
{
	if (_uart_fd >= 0) {
		PX4_ERR("Port already in use: %s", dev);
		return -1;
	}

	_uart_fd = qurt_uart_open(dev, speed);

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port: %s (%i)", dev, errno);
		return -1;

	} else if (debug) { PX4_INFO("qurt uart opened successfully"); }

	return 0;
}

int close_port()
{
	_uart_fd = -1;

	return 0;
}

int write_response(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for writing or buffer");
		return -1;
	}

	return qurt_uart_write(_uart_fd, (const char *) buf, len);
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("mavlink_rc_in_main",
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

int status()
{
	PX4_INFO("running: %s", _is_running ? "yes" : "no");
	PX4_INFO("");
	PX4_INFO(" RSSI: %i", _rc_rssi_dbm);
	PX4_INFO(" LQ:   %i", _rc_lq);
	PX4_INFO("");
	perf_print_counter(_perf_rx_rate);
	return 0;
}

void
usage()
{
	PX4_INFO("Usage: mavlink_rc_in {start|info|stop} [options]");
	PX4_INFO("Options: -d             enable debug messages");
	PX4_INFO("         -p <number>    uart port number");
	PX4_INFO("         -b <number>    uart baudrate");
	PX4_INFO("         -c             use CRSF protocol over the wire");
	PX4_INFO("         -m             use MAVLink protocol over the wire");
	PX4_INFO("         -f             disable fake hearbeats");
	PX4_INFO("         -l             filter out local rc received debug messages");
	PX4_INFO("         -r             enable received messages hex dump");
	PX4_INFO("         -t             enable transmitted messages hex dump");
}

}

int mavlink_rc_in_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		mavlink_rc_in::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return mavlink_rc_in::start(argc - 1, argv + 1);

	} else if (!strcmp(verb, "stop")) {
		return mavlink_rc_in::stop();

	} else if (!strcmp(verb, "status")) {
		return mavlink_rc_in::status();

	} else {
		mavlink_rc_in::usage();
		return -1;
	}
}
