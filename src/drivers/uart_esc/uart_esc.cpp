/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/input_rc.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_multirotor.generated.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <dev_fs_lib_serial.h>
#include <v1.0/checksum.h>
#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>


namespace uart_esc
{

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

volatile bool _task_should_exit = false; // flag indicating if uart_esc task should exit
static char _device[32] = {};
static bool _is_running = false;         // flag indicating if uart_esc app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread

// subscriptions
int	_controls_sub;
int	_armed_sub;
int 	_fd;

// filenames
// /dev/fs/ is mapped to /usr/share/data/adsp/
static const char *MIXER_FILENAME = "/dev/fs/quad_x.main.mix";


// publications
orb_advert_t        	_outputs_pub = nullptr;
orb_advert_t 		_rc_pub = nullptr;

// topic structures
actuator_controls_s     _controls;
actuator_outputs_s      _outputs;
actuator_armed_s	_armed;
input_rc_s 		_rc;

pwm_limit_t	_pwm_limit;


MultirotorMixer *_mixer = nullptr;

void usage();

void start(const char *device);

void stop();

void send_outputs_mavlink(const uint16_t *pwm, const unsigned num_pwm);

void serial_callback(void *context, char *buffer, size_t num_bytes);

void handle_message(mavlink_message_t *rc_message);

void task_main_trampoline(int argc, char *argv[]);

void task_main(int argc, char *argv[]);

/* mixer initialization */
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);


int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];



	return 0;
}


int initialize_mixer(const char *mixer_filename)
{
	char buf[2048];
	size_t buflen = sizeof(buf);

	PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

	int fd_load = ::open(mixer_filename, O_RDONLY);

	if (fd_load != -1) {
		int nRead = ::read(fd_load, buf, buflen);
		close(fd_load);

		if (nRead > 0) {
			_mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

			if (_mixer != nullptr) {
				PX4_INFO("Successfully initialized mixer from config file");
				return 0;

			} else {
				PX4_ERR("Unable to parse from mixer config file");
				return -1;
			}

		} else {
			PX4_WARN("Unable to read from mixer config file");
			return -2;
		}

	} else {
		PX4_WARN("Unable to open mixer config file, try default mixer");

		/* Mixer file loading failed, fall back to default mixer configuration for
		* QUAD_X airframe. */
		float roll_scale = 1;
		float pitch_scale = 1;
		float yaw_scale = 1;
		float deadband = 0;

		_mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
					     MultirotorGeometry::QUAD_X,
					     roll_scale, pitch_scale, yaw_scale, deadband);

		if (_mixer == nullptr) {
			PX4_ERR("Mixer initialization failed");
			return -1;
		}

		PX4_INFO("Successfully initialized default quad x mixer.");
		return 0;
	}
}


int uart_initialize(const char *device)
{
	_fd = ::open(device, O_RDWR | O_NONBLOCK);

	if (_fd == -1) {
		PX4_ERR("Failed to open UART.");
		return -1;
	}

	struct dspal_serial_ioctl_data_rate rate;

	rate.bit_rate = DSPAL_SIO_BITRATE_921600;

	int ret = ioctl(_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&rate);

	if (ret != 0) {
		PX4_ERR("Failed to set UART bitrate.");
		return -2;
	}

	struct dspal_serial_ioctl_receive_data_callback callback;

	callback.rx_data_callback_func_ptr = serial_callback;

	ret = ioctl(_fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&callback);

	if (ret != 0) {
		PX4_ERR("Failed to setup UART flow control options.");
		return -3;
	}

	return 0;
}

int uart_deinitialize()
{
	return close(_fd);
}

// send actuator controls message to Pixhawk
void send_outputs_mavlink(const uint16_t *pwm, const unsigned num_pwm)
{
	// Fill up to number of outputs.
	mavlink_actuator_control_target_t controls_message;

	for (unsigned i = 0; i < num_pwm && i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; ++i) {
		controls_message.controls[i] = pwm[i];
	}

	// And the rest with NAN.
	for (unsigned i = _outputs.noutputs; (i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS)
	     && (i < actuator_controls_s::NUM_ACTUATOR_CONTROLS); ++i) {
		controls_message.controls[i] = NAN;
	}

	controls_message.time_usec = _controls.timestamp;

	const uint8_t msgid = MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	const uint8_t component_ID = 0;
	const uint8_t payload_len = mavlink_message_lengths[msgid];
	const unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	// TODO FIXME: no idea which numbers should be here.
	buf[2] = 100;
	buf[3] = 0;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES], (const void *)&controls_message, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	int ret = ::write(_fd, &buf[0], packet_len);

	//static unsigned counter = 0;
	//if (counter++ % 250 == 0) {
	//	PX4_INFO("send motor controls %d bytes %.2f %.2f %.2f %.2f",
	//		 ret,
	//		 controls_message.controls[0],
	//		 controls_message.controls[1],
	//		 controls_message.controls[2],
	//		 controls_message.controls[3]);
	//}

	if (ret < 1) {
		PX4_WARN("Failed sending rc mavlink message, ret: %d, errno: %d", ret, errno);
	}
}


void serial_callback(void *context, char *buffer, size_t num_bytes)
{
	mavlink_status_t serial_status = {};

	if (num_bytes > 0) {
		mavlink_message_t msg;

		for (int i = 0; i < num_bytes; ++i) {
			// TODO FIXME: we don't know if MAVLINK_COMM_1 is already taken.
			if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &serial_status)) {
				// have a message, handle it
				if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
					// we should publish but would be great if this works
					handle_message(&msg);
				}
			}
		}

	} else {
		PX4_ERR("error: read callback with no data in the buffer");
	}
}


void handle_message(mavlink_message_t *rc_message)
{
	mavlink_rc_channels_t rc;
	mavlink_msg_rc_channels_decode(rc_message, &rc);
	_rc.timestamp_publication = hrt_absolute_time();
	_rc.timestamp_last_signal = hrt_absolute_time();
	_rc.channel_count = rc.chancount;
	_rc.rc_lost = false;
	_rc.rssi = rc.rssi;

	_rc.values[ 0] = rc.chancount > 0 ?  rc.chan1_raw : UINT16_MAX;
	_rc.values[ 1] = rc.chancount > 0 ?  rc.chan2_raw : UINT16_MAX;
	_rc.values[ 2] = rc.chancount > 0 ?  rc.chan3_raw : UINT16_MAX;
	_rc.values[ 3] = rc.chancount > 0 ?  rc.chan4_raw : UINT16_MAX;
	_rc.values[ 4] = rc.chancount > 0 ?  rc.chan5_raw : UINT16_MAX;
	_rc.values[ 5] = rc.chancount > 0 ?  rc.chan6_raw : UINT16_MAX;
	_rc.values[ 6] = rc.chancount > 0 ?  rc.chan7_raw : UINT16_MAX;
	_rc.values[ 7] = rc.chancount > 0 ?  rc.chan8_raw : UINT16_MAX;
	_rc.values[ 8] = rc.chancount > 0 ?  rc.chan9_raw : UINT16_MAX;
	_rc.values[ 9] = rc.chancount > 0 ? rc.chan10_raw : UINT16_MAX;
	_rc.values[10] = rc.chancount > 0 ? rc.chan11_raw : UINT16_MAX;
	_rc.values[11] = rc.chancount > 0 ? rc.chan12_raw : UINT16_MAX;
	_rc.values[12] = rc.chancount > 0 ? rc.chan13_raw : UINT16_MAX;
	_rc.values[13] = rc.chancount > 0 ? rc.chan14_raw : UINT16_MAX;
	_rc.values[14] = rc.chancount > 0 ? rc.chan15_raw : UINT16_MAX;
	_rc.values[15] = rc.chancount > 0 ? rc.chan16_raw : UINT16_MAX;
	_rc.values[16] = rc.chancount > 0 ? rc.chan17_raw : UINT16_MAX;
	_rc.values[17] = rc.chancount > 0 ? rc.chan18_raw : UINT16_MAX;

	if (_rc_pub != nullptr) {
		orb_publish(ORB_ID(input_rc), _rc_pub, &_rc);

	} else {
		_rc_pub = orb_advertise(ORB_ID(input_rc), &_rc);
	}
}

void task_main(int argc, char *argv[])
{
	_is_running = true;

	if (uart_initialize(_device) < 0) {
		PX4_ERR("Failed to initialize UART.");
		return;
	}

	// Subscribe for orb topics
	_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	// Start disarmed
	_armed.armed = false;
	_armed.prearmed = false;

	// Set up poll topic
	px4_pollfd_struct_t fds[1];
	fds[0].fd     = _controls_sub;
	fds[0].events = POLLIN;
	/* Don't limit poll intervall for now, 250 Hz should be fine. */
	//orb_set_interval(_controls_sub, 10);

	// Set up mixer
	if (initialize_mixer(MIXER_FILENAME) < 0) {
		PX4_ERR("Mixer initialization failed.");
		return;
	}

	pwm_limit_init(&_pwm_limit);

	// TODO XXX: this is needed otherwise we crash in the callback context.
	_rc_pub = orb_advertise(ORB_ID(input_rc), &_rc);

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
			orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);

			_outputs.timestamp = _controls.timestamp;

			/* do mixing */
			_outputs.noutputs = _mixer->mix(_outputs.output, 0 /* not used */, NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

			const uint16_t reverse_mask = 0;
			// TODO FIXME: these should probably be params
			const uint16_t disarmed_pwm[4] = {900, 900, 900, 900};
			const uint16_t min_pwm[4] = {1230, 1230, 1230, 1230};
			const uint16_t max_pwm[4] = {1900, 1900, 1900, 1900};
			uint16_t pwm[4];

			// TODO FIXME: pre-armed seems broken
			pwm_limit_calc(_armed.armed, false/*_armed.prearmed*/, _outputs.noutputs, reverse_mask,
				       disarmed_pwm, min_pwm, max_pwm, _outputs.output, pwm, &_pwm_limit);


			send_outputs_mavlink(pwm, 4);

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
			}
		}

		bool updated;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		}
	}

	uart_deinitialize();
	orb_unsubscribe(_controls_sub);
	orb_unsubscribe(_armed_sub);

	_is_running = false;

}

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("uart_esc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

}

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: uart_esc start -d /dev/tty-3");
	PX4_INFO("       uart_esc stop");
	PX4_INFO("       uart_esc status");
}

} // namespace uart_esc

/* driver 'main' command */
extern "C" __EXPORT int uart_esc_main(int argc, char *argv[]);

int uart_esc_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(uart_esc::_device, device, strlen(device));
			break;
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (uart_esc::_is_running) {
			PX4_WARN("uart_esc already running");
			return 1;
		}

		// Check on required arguments
		if (device == nullptr || strlen(device) == 0) {
			uart_esc::usage();
			return 1;
		}

		uart_esc::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!uart_esc::_is_running) {
			PX4_WARN("uart_esc is not running");
			return 1;
		}

		uart_esc::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("uart_esc is %s", uart_esc::_is_running ? "running" : "not running");
		return 0;

	} else {
		uart_esc::usage();
		return 1;
	}

	return 0;
}
