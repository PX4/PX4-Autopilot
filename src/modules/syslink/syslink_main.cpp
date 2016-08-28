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
 * @file syslink_main.cpp
 * Entry point for syslink module used to communicate with the NRF module on a Crazyflie
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <termios.h>

#include <drivers/drv_rc_input.h>

#include <systemlib/err.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>

#include "crtp.h"
#include "syslink_main.h"

extern "C" { __EXPORT int syslink_main(int argc, char *argv[]); }


Syslink *g_syslink;

Syslink::Syslink() :
	_syslink_task(-1),
	_task_running(false),
	_fd(0),
	_battery_pub(nullptr),
	_rc_pub(nullptr),
	_cmd_pub(nullptr),
	_rssi(RC_INPUT_RSSI_MAX)
{

}


int
Syslink::start()
{
	_task_running = true;
	_syslink_task = px4_task_spawn_cmd(
				"syslink",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				3600,
				Syslink::task_main_trampoline,
				NULL
			);

	return 0;
}


int
Syslink::set_datarate(uint8_t datarate)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_DATARATE;
	msg.length = 1;
	msg.data[0] = datarate;
	return send_message(&msg);
}

int
Syslink::set_channel(uint8_t channel)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_CHANNEL;
	msg.length = 1;
	msg.data[0] = channel;
	return send_message(&msg);
}

int
Syslink::set_address(uint64_t addr)
{
	syslink_message_t msg;
	msg.type = SYSLINK_RADIO_ADDRESS;
	msg.length = 5;
	memcpy(&msg.data, &addr, 5);
	return send_message(&msg);
}


// 1M 8N1 serial connection to NRF51
int
Syslink::open_serial(const char *dev)
{
#ifndef B1000000
#define B1000000 1000000
#endif

	int rate = B1000000;

	// open uart
	int fd = px4_open(dev, O_RDWR | O_NOCTTY);
	int termios_state = -1;

	if (fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	struct termios config;
	tcgetattr(fd, &config);

	// clear ONLCR flag (which appends a CR for every LF)
	config.c_oflag &= ~ONLCR;

	// Disable hardware flow control
	// TODO: CF2 uses a TX EN as output flow control
	config.c_cflag &= ~CRTSCTS;


	/* Set baud rate */
	if (cfsetispeed(&config, rate) < 0 || cfsetospeed(&config, rate) < 0) {
		warnx("ERR SET BAUD %s: %d\n", dev, termios_state);
		px4_close(fd);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", dev);
		px4_close(fd);
		return -1;
	}

	return fd;
}



int
Syslink::task_main_trampoline(int argc, char *argv[])
{
	g_syslink->task_main();
	return 0;
}

void
Syslink::task_main()
{
	param_t _param_radio_channel = param_find("SLNK_RADIO_CHAN");
	param_t _param_radio_rate = param_find("SLNK_RADIO_RATE");
	param_t _param_radio_addr1 = param_find("SLNK_RADIO_ADDR1");
	param_t _param_radio_addr2 = param_find("SLNK_RADIO_ADDR2");

	uint32_t channel, rate;
	uint64_t addr = 0;

	param_get(_param_radio_channel, &channel);
	param_get(_param_radio_rate, &rate);
	param_get(_param_radio_addr1, &addr + 4);
	param_get(_param_radio_addr2, &addr);



	_battery.reset(&_battery_status);


	//	int ret;

	/* Open serial port */
	const char *dev = "/dev/ttyS2";
	_fd = open_serial(dev);

	if (_fd < 0) {
		err(1, "can't open %s", dev);
		return;
	}


	/* Set non-blocking */
	int flags = fcntl(_fd, F_GETFL, 0);
	fcntl(_fd, F_SETFL, flags | O_NONBLOCK);


	set_datarate(rate);
	usleep(1000);
	set_channel(channel);
	usleep(1000);
	set_address(addr);


	px4_pollfd_struct_t fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int error_counter = 0;

	char buf[64];
	int nread;

	syslink_parse_state state;
	syslink_message_t msg;

	syslink_parse_init(&state);


	while (_task_running) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("[syslink] ERROR return value from poll(): %d"
					, poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				if ((nread = read(_fd, buf, sizeof(buf))) < 0) {
					continue;
				}

				for (int i = 0; i < nread; i++) {
					if (syslink_parse_char(&state, buf[i], &msg)) {
						handle_message(&msg);
					}
				}
			}
		}
	}

	close(_fd);

}



static int count = 0;
static int null_count = 0;
static hrt_abstime lasttime = 0;


void
Syslink::handle_message(syslink_message_t *msg)
{
	hrt_abstime t = hrt_absolute_time();

	if (t - lasttime > 1000000) {
		PX4_INFO("%d packets per second (%d null)", count, null_count);
		lasttime = t;
		count = 0;
		null_count = 0;
	}

	count++;

	if (msg->type == SYSLINK_PM_ONOFF_SWITCHOFF) {
		// When the power button is hit
	} else if (msg->type == SYSLINK_PM_BATTERY_STATE) {

		if (msg->length != 9) {
			return;
		}

		//uint8_t flags = msg->data[0];
		//float iset = *((float *)&msg->data[5]);
		float vbat;
		memcpy(&vbat, &msg->data[1], sizeof(float));

		_battery.updateBatteryStatus(t, vbat, 0, 0, false, &_battery_status);

		// announce the battery status if needed, just publish else
		if (_battery_pub != nullptr) {
			orb_publish(ORB_ID(battery_status), _battery_pub, &_battery_status);

		} else {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &_battery_status);
		}

	} else if (msg->type == SYSLINK_RADIO_RSSI) {
		uint8_t rssi = msg->data[0]; // Between 40 and 100 meaning -40 dBm to -100 dBm
		_rssi = 140 - rssi * 100 / (100 - 40);

	} else if (msg->type == SYSLINK_RADIO_CHANNEL) {
		PX4_INFO("Channel ACK %d", msg->data[0]);

	} else if (msg->type == SYSLINK_RADIO_DATARATE) {
		PX4_INFO("Datarate ACK %d", msg->data[0]);

	} else if (msg->type == SYSLINK_RADIO_RAW) {
		handle_raw(msg);
	}


}

void
Syslink::handle_raw(syslink_message_t *sys)
{
	// TODO: Now allow one raw message to be sent from the queue

	crtp_message_t *c = (crtp_message_t *) &sys->data;

	if (CRTP_NULL(*c)) {
		null_count++;
		return;
	}


	if (c->port == CRTP_PORT_COMMANDER) {

		crtp_commander *cmd = (crtp_commander *) &c->data[0];

		struct rc_input_values rc = {};

		rc.timestamp_publication = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp_publication;
		rc.channel_count = 5;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = _rssi;


		double pitch = cmd->pitch, roll = cmd->roll, yaw = cmd->yaw;

		/* channels (scaled to rc limits) */
		rc.values[0] = pitch * 500 / 20 + 1500;
		rc.values[1] = roll * 500 / 20 + 1500;
		rc.values[2] = yaw * 500 / 150 + 1500;
		rc.values[3] = cmd->thrust * 1000 / USHRT_MAX + 1000;
		rc.values[4] = 1000; // Dummy channel as px4 needs at least 5

		if (_rc_pub == nullptr) {
			_rc_pub = orb_advertise(ORB_ID(input_rc), &rc);

		} else {
			orb_publish(ORB_ID(input_rc), _rc_pub, &rc);
		}

	} else if (c->port == CRTP_PORT_MAVLINK) {
		/* Pipe to Mavlink bridge */
	}

}

int
Syslink::send_message(syslink_message_t *msg)
{
	syslink_compute_cksum(msg);
	write(_fd, syslink_magic, 2);
	write(_fd, &msg->type, sizeof(msg->type));
	write(_fd, &msg->length, sizeof(msg->length));
	write(_fd, &msg->data, msg->length);
	write(_fd, &msg->cksum, sizeof(msg->cksum));
	return 0;
}

int syslink_main(int argc, char *argv[])
{
	PX4_INFO("Starting daemon...");
	g_syslink = new Syslink();
	g_syslink->start();

	return 0;
}
