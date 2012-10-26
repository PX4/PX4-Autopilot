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
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via serial (or possibly some other interface at a later
 * point).

 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/mixer/mixer.h>
#include <drivers/drv_mixer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/hx_stream.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/rc_channels.h>

#include "px4io/protocol.h"
#include "uploader.h"


class PX4IO : public device::CDev
{
public:
	PX4IO();
	~PX4IO();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

private:
	static const unsigned	_max_actuators = PX4IO_OUTPUT_CHANNELS;

	int			_serial_fd;	///< serial interface to PX4IO
	hx_stream_t		_io_stream;	///< HX protocol stream

	int			_task;		///< worker task
	volatile bool		_task_should_exit;

	int			_t_actuators;	///< actuator output topic
	actuator_controls_s	_controls;	///< actuator outputs

	int			_t_armed;	///< system armed control topic
	actuator_armed_s	_armed;		///< system armed state

	orb_advert_t		_t_outputs;	///< mixed outputs topic
	actuator_outputs_s	_outputs;	///< mixed outputs

	MixerGroup		*_mixers;	///< loaded mixers

	bool			_primary_pwm_device;	///< true if we are the default PWM output

	volatile bool		_switch_armed;	///< PX4IO switch armed state
	// XXX how should this work?

	bool			_send_needed;	///< If true, we need to send a packet to IO

	/**
	 * Trampoline to the worker task
	 */
	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Handle receiving bytes from PX4IO
	 */
	void			io_recv();

	/**
	 * HX protocol callback trampoline.
	 */
	static void		rx_callback_trampoline(void *arg, const void *buffer, size_t bytes_received);

	/**
	 * Callback invoked when we receive a whole packet from PX4IO
	 */
	void			rx_callback(const uint8_t *buffer, size_t bytes_received);

	/**
	 * Send an update packet to PX4IO
	 */
	void			io_send();

	/**
	 * Mixer control callback; invoked to fetch a control from a specific
	 * group/index during mixing.
	 */
	static int		control_callback(uintptr_t handle,
			uint8_t control_group,
			uint8_t control_index,
			float &input);
};


namespace
{

PX4IO	*g_dev;

}

PX4IO::PX4IO() :
	CDev("px4io", "/dev/px4io"),
	_serial_fd(-1),
	_io_stream(nullptr),
	_task(-1),
	_task_should_exit(false),
	_t_actuators(-1),
	_t_armed(-1),
	_t_outputs(-1),
	_mixers(nullptr),
	_primary_pwm_device(false),
	_switch_armed(false),
	_send_needed(false)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = true;
}

PX4IO::~PX4IO()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		/* spin waiting for the thread to stop */
		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	/* kill the HX stream */
	if (_io_stream != nullptr)
		hx_stream_free(_io_stream);

	g_dev = nullptr;
}

int
PX4IO::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK)
		return ret;

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		log("default PWM output device");
		_primary_pwm_device = true;
	}

	/* start the IO interface task */
	_task = task_create("px4io", SCHED_PRIORITY_DEFAULT, 4096, (main_t)&PX4IO::task_main_trampoline, nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PX4IO::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
PX4IO::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open("/dev/ttyS2", O_RDWR);

	if (_serial_fd < 0) {
		debug("failed to open serial port for IO: %d", errno);
		_task = -1;
		_exit(errno);
	}

	/* protocol stream */
	_io_stream = hx_stream_init(_serial_fd, &PX4IO::rx_callback_trampoline, this);

	perf_counter_t pc_tx_frames = perf_alloc(PC_COUNT, "PX4IO frames transmitted");
	perf_counter_t pc_rx_frames = perf_alloc(PC_COUNT, "PX4IO frames received");
	perf_counter_t pc_rx_errors = perf_alloc(PC_COUNT, "PX4IO receive errors");
	hx_stream_set_counters(_io_stream, pc_tx_frames, pc_rx_frames, pc_rx_errors);

	/* XXX send a who-are-you request */

	/* XXX verify firmware/protocol version */

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));
	/* convert the update rate in hz to milliseconds, rounding down if necessary */
	//int update_rate_in_ms = int(1000 / _update_rate);
	orb_set_interval(_t_actuators, 20);	/* XXX 50Hz hardcoded for now */

	_t_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_armed, 200);		/* 5Hz update rate */

	/* advertise the mixed control outputs */
	_t_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &_outputs);

	/* poll descriptor */
	pollfd fds[3];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;
	fds[1].fd = _t_actuators;
	fds[1].events = POLLIN;
	fds[2].fd = _t_armed;
	fds[2].events = POLLIN;

	log("ready");

	/* loop handling received serial bytes */
	while (!_task_should_exit) {

		/* sleep waiting for data, but no more than 100ms */
		int ret = ::poll(&fds[0], sizeof(fds) / sizeof(fds[0]), 1000);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* if we timed out waiting, we should send an update */
		if (ret == 0)
			_send_needed = true;

		if (ret > 0) {
			/* if we have new data from IO, go handle it */
			if (fds[0].revents & POLLIN)
				io_recv();

			/* if we have new data from the ORB, go handle it */
			if (fds[1].revents & POLLIN) {

				/* get controls */
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _t_actuators, &_controls);

				/* mix */
				if (_mixers != nullptr) {
					/* XXX is this the right count? */
					_mixers->mix(&_outputs.output[0], _max_actuators);

					/* convert to PWM values */
					for (unsigned i = 0; i < _max_actuators; i++)
						_outputs.output[i] = 1500 + (600 * _outputs.output[i]);

					/* and flag for update */
					_send_needed = true;
				}
			}

			if (fds[2].revents & POLLIN) {

				orb_copy(ORB_ID(actuator_armed), _t_armed, &_controls);
				_send_needed = true;
			}
		}

		/* send an update to IO if required */
		if (_send_needed) {
			_send_needed = false;
			io_send();
		}
	}

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
PX4IO::control_callback(uintptr_t handle,
			uint8_t control_group,
			uint8_t control_index,
			float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

void
PX4IO::io_recv()
{
	uint8_t buf[32];
	int count;

	/*
	 * We are here because poll says there is some data, so this
	 * won't block even on a blocking device.  If more bytes are
	 * available, we'll go back to poll() again...
	 */
	count = ::read(_serial_fd, buf, sizeof(buf));

	/* pass received bytes to the packet decoder */
	for (int i = 0; i < count; i++)
		hx_stream_rx(_io_stream, buf[i]);
}

void
PX4IO::rx_callback_trampoline(void *arg, const void *buffer, size_t bytes_received)
{
	g_dev->rx_callback((const uint8_t *)buffer, bytes_received);
}

void
PX4IO::rx_callback(const uint8_t *buffer, size_t bytes_received)
{
	const px4io_report *rep = (const px4io_report *)buffer;

	/* sanity-check the received frame size */
	if (bytes_received != sizeof(px4io_report))
		return;

	/* XXX handle R/C inputs here ... needs code sharing/library */

	/* remember the latched arming switch state */
	_switch_armed = rep->armed;

	unlock();
}

void
PX4IO::io_send()
{
	px4io_command	cmd;

	cmd.f2i_magic = F2I_MAGIC;

	/* set outputs */
	for (unsigned i = 0; i < _max_actuators; i++)
		cmd.servo_command[i] = _outputs.output[i];

	/* publish as we send */
	orb_publish(ORB_ID_VEHICLE_CONTROLS, _t_outputs, &_outputs);

	// XXX relays

	cmd.arm_ok = _armed.armed;

	hx_stream_send(_io_stream, &cmd, sizeof(cmd));
}

int
PX4IO::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* fake an armed transition */
		_armed.armed = true;
		_send_needed = true;
		break;

	case PWM_SERVO_DISARM:
		/* fake a disarmed transition */
		_armed.armed = true;
		_send_needed = true;
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(_max_actuators - 1):

		/* fake an update to the selected servo channel */
		if ((arg >= 900) && (arg <= 2100)) {
			_outputs.output[cmd - PWM_SERVO_SET(0)] = arg;
			_send_needed = true;

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(_max_actuators - 1):
		/* copy the current output value from the channel */
		*(servo_position_t *)arg = _outputs.output[cmd - PWM_SERVO_GET(0)];
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			/* build the new mixer from the supplied argument */
			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)&_controls, mixinfo);

			/* validate the new mixer */
			if (mixer->check()) {
				delete mixer;
				ret = -EINVAL;

			} else {
				/* if we don't have a group yet, allocate one */
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)&_controls);

				/* add the new mixer to the group */
				_mixers->add_mixer(mixer);
			}

		}
		break;

	case MIXERIOCADDMULTIROTOR:
		/* XXX not yet supported */
		ret = -ENOTTY;
		break;

	case MIXERIOCLOADFILE: {
			MixerGroup *newmixers;
			const char *path = (const char *)arg;

			/* allocate a new mixer group and load it from the file */
			newmixers = new MixerGroup(control_callback, (uintptr_t)&_controls);

			if (newmixers->load_from_file(path) != 0) {
				delete newmixers;
				ret = -EINVAL;
			}

			/* swap the new mixers in for the old */
			if (_mixers != nullptr) {
				delete _mixers;
			}

			_mixers = newmixers;

		}
		break;

	default:
		/* not a recognised value */
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

int
px4io_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr)
			errx(1, "already loaded");

		/* create the driver - it will set g_dev */
		(void)new PX4IO;

		if (g_dev == nullptr)
			errx(1, "driver alloc failed");

		if (OK != g_dev->init()) {
			delete g_dev;
			errx(1, "driver init failed");
		}

		exit(0);
	}

	/* note, stop not currently implemented */

	if (!strcmp(argv[1], "update")) {
		PX4IO_Uploader *up;
		const char *fn[3];

		/* work out what we're uploading... */
		if (argc > 2) {
			fn[0] = argv[2];
			fn[1] = nullptr;

		} else {
			fn[0] = "/fs/microsd/px4io.bin";
			fn[1] =	"/etc/px4io.bin";
			fn[2] =	nullptr;
		}

		up = new PX4IO_Uploader;
		int ret = up->upload(&fn[0]);
		delete up;

		switch (ret) {
		case OK:
			break;

		case -ENOENT:
			errx(1, "PX4IO firmware file not found");

		case -EEXIST:
		case -EIO:
			errx(1, "error updating PX4IO - check that bootloader mode is enabled");

		case -EINVAL:
			errx(1, "verify failed - retry the update");

		case -ETIMEDOUT:
			errx(1, "timed out waiting for bootloader - power-cycle and try again");

		default:
			errx(1, "unexpected error %d", ret);
		}

		return ret;
	}

	errx(1, "need a verb, only support 'start' and 'update'");
}
