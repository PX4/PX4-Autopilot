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
#include <termios.h>
#include <fcntl.h>
#include <math.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/perf_counter.h>
#include <systemlib/hx_stream.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rc_channels.h>

#include <px4io/protocol.h>
#include "uploader.h"


class PX4IO : public device::CDev
{
public:
	PX4IO();
	~PX4IO();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * Set the PWM via serial update rate
	 * @warning this directly affects CPU load
	 */
	int 			set_pwm_rate(int hz);

	bool			dump_one;

private:
	static const unsigned	_max_actuators = PX4IO_OUTPUT_CHANNELS;
	int 			_update_rate;	///< serial send rate in Hz

	int			_serial_fd;	///< serial interface to PX4IO
	hx_stream_t		_io_stream;	///< HX protocol stream

	volatile int		_task;		///< worker task
	volatile bool		_task_should_exit;
	volatile bool		_connected;	///< true once we have received a valid frame

	int			_t_actuators;	///< actuator output topic
	actuator_controls_s	_controls;	///< actuator outputs

	orb_advert_t 		_t_actuators_effective;	///< effective actuator controls topic
	actuator_controls_effective_s _controls_effective; ///< effective controls

	int			_t_armed;	///< system armed control topic
	actuator_armed_s	_armed;		///< system armed state
	int 			_t_vstatus;	///< system / vehicle status
	vehicle_status_s	_vstatus;	///< overall system state

	orb_advert_t 		_to_input_rc;	///< rc inputs from io
	rc_input_values		_input_rc;	///< rc input values

	orb_advert_t		_t_outputs;	///< mixed outputs topic
	actuator_outputs_s	_outputs;	///< mixed outputs

	MixerGroup		*_mixers;	///< loaded mixers

	bool			_primary_pwm_device;	///< true if we are the default PWM output

	volatile bool		_switch_armed;	///< PX4IO switch armed state
	// XXX how should this work?

	bool			_send_needed;	///< If true, we need to send a packet to IO
	bool			_config_needed;	///< if true, we need to set a config update to IO

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
	 * Send a config packet to PX4IO
	 */
	void			config_send();

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
	dump_one(false),
	_update_rate(50),
	_serial_fd(-1),
	_io_stream(nullptr),
	_task(-1),
	_task_should_exit(false),
	_connected(false),
	_t_actuators(-1),
	_t_actuators_effective(-1),
	_t_armed(-1),
	_t_vstatus(-1),
	_t_outputs(-1),
	_mixers(nullptr),
	_primary_pwm_device(false),
	_switch_armed(false),
	_send_needed(false),
	_config_needed(false)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;

	_debug_enabled = true;
}

PX4IO::~PX4IO()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}
	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

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

	/* wait a second for it to detect IO */
	for (unsigned i = 0; i < 10; i++) {
		if (_connected) {
			debug("PX4IO connected");
			break;
		}
		usleep(100000);
	}
	if (!_connected) {
		/* error here will result in everything being torn down */
		log("PX4IO not responding");
		return -EIO;
	}
	return OK;
}

int
PX4IO::set_pwm_rate(int hz)
{
	if (hz > 0 && hz <= 400) {
		_update_rate = hz;
		return OK;
	} else {
		return -EINVAL;
	}
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
	int update_rate_in_ms;

	/* open the serial port */
	_serial_fd = ::open("/dev/ttyS2", O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %d", errno);
		goto out;
	}

	/* 115200bps, no parity, one stop bit */
	{
		struct termios t;

		tcgetattr(_serial_fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(_serial_fd, TCSANOW, &t);
	}

	/* protocol stream */
	_io_stream = hx_stream_init(_serial_fd, &PX4IO::rx_callback_trampoline, this);
	if (_io_stream == nullptr) {
		log("failed to allocate HX protocol stream");
		goto out;
	}
	hx_stream_set_counters(_io_stream,
			       perf_alloc(PC_COUNT, "PX4IO frames transmitted"),
			       perf_alloc(PC_COUNT, "PX4IO frames received"),
			       perf_alloc(PC_COUNT, "PX4IO receive errors"));

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));
	/* convert the update rate in hz to milliseconds, rounding down if necessary */
	update_rate_in_ms = int(1000 / _update_rate);
	orb_set_interval(_t_actuators, update_rate_in_ms);

	_t_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_armed, 200);		/* 5Hz update rate */

	_t_vstatus = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(_t_vstatus, 200);		/* 5Hz update rate max. */

	/* advertise the limited control inputs */
	memset(&_controls_effective, 0, sizeof(_controls_effective));
	_t_actuators_effective = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_1),
				   &_controls_effective);

	/* advertise the mixed control outputs */
	memset(&_outputs, 0, sizeof(_outputs));
	_t_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &_outputs);

	/* advertise the rc inputs */
	memset(&_input_rc, 0, sizeof(_input_rc));
	_to_input_rc = orb_advertise(ORB_ID(input_rc), &_input_rc);

	/* poll descriptor */
	pollfd fds[4];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;
	fds[1].fd = _t_actuators;
	fds[1].events = POLLIN;
	fds[2].fd = _t_armed;
	fds[2].events = POLLIN;
	fds[3].fd = _t_vstatus;
	fds[3].events = POLLIN;

	log("ready");

	/* loop handling received serial bytes */
	while (!_task_should_exit) {

		/* sleep waiting for data, but no more than 100ms */
		int ret = ::poll(&fds[0], sizeof(fds) / sizeof(fds[0]), 100);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
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
				orb_copy(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1), _t_actuators, &_controls);

				/* mix */
				if (_mixers != nullptr) {
					/* XXX is this the right count? */
					_mixers->mix(&_outputs.output[0], _max_actuators);

					// XXX output actual limited values
					memcpy(&_controls_effective, &_controls, sizeof(_controls_effective));

					orb_publish(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_effective_1), _t_actuators_effective, &_controls_effective);


					/* convert to PWM values */
					for (unsigned i = 0; i < _max_actuators; i++) {
						/* last resort: catch NaN, INF and out-of-band errors */
						if (isfinite(_outputs.output[i]) && _outputs.output[i] >= -1.0f && _outputs.output[i] <= 1.0f) {
							_outputs.output[i] = 1500 + (600 * _outputs.output[i]);
						} else {
							/*
							 * Value is NaN, INF or out of band - set to the minimum value.
							 * This will be clearly visible on the servo status and will limit the risk of accidentally
							 * spinning motors. It would be deadly in flight.
							 */
							_outputs.output[i] = 900;
						}
					}

					/* and flag for update */
					_send_needed = true;
				}
			}

			if (fds[2].revents & POLLIN) {

				orb_copy(ORB_ID(actuator_armed), _t_armed, &_armed);
				_send_needed = true;
			}

			if (fds[3].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_status), _t_vstatus, &_vstatus);
				_send_needed = true;
			}
		}

		/* send an update to IO if required */
		if (_send_needed) {
			_send_needed = false;
			io_send();
		}

		/* send a config packet to IO if required */
		if (_config_needed) {
			_config_needed = false;
			config_send();
		}
	}

out:
	debug("exiting");

	/* kill the HX stream */
	if (_io_stream != nullptr)
		hx_stream_free(_io_stream);
	::close(_serial_fd);

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

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

	lock();

	/* sanity-check the received frame size */
	if (bytes_received != sizeof(px4io_report)) {
		debug("got %u expected %u", bytes_received, sizeof(px4io_report));
		goto out;
	}
	if (rep->i2f_magic != I2F_MAGIC) {
		debug("bad magic");
		goto out;
	}
	_connected = true;

	/* publish raw rc channel values from IO if valid channels are present */
	if (rep->channel_count > 0) {
		_input_rc.timestamp = hrt_absolute_time();
		_input_rc.channel_count = rep->channel_count;
		for (int i = 0; i < rep->channel_count; i++)
		{
			_input_rc.values[i] = rep->rc_channel[i];
		}

		orb_publish(ORB_ID(input_rc), _to_input_rc, &_input_rc);
	}

	/* remember the latched arming switch state */
	_switch_armed = rep->armed;

	_send_needed = true;

	/* if monitoring, dump the received info */
	if (dump_one) {
		dump_one = false;

		printf("IO: %s armed ", rep->armed ? "" : "not");
		for (unsigned i = 0; i < rep->channel_count; i++)
			printf("%d: %d ", i, rep->rc_channel[i]);
		printf("\n");
	}

out:
	unlock();
}

void
PX4IO::io_send()
{
	px4io_command	cmd;
	int		ret;

	cmd.f2i_magic = F2I_MAGIC;

	/* set outputs */
	for (unsigned i = 0; i < _max_actuators; i++)
		cmd.servo_command[i] = _outputs.output[i];

	/* publish as we send */
	orb_publish(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1), _t_outputs, &_outputs);

	// XXX relays

	/* armed and not locked down -> arming ok */
	cmd.arm_ok = (_armed.armed && !_armed.lockdown);
	/* indicate that full autonomous position control / vector flight mode is available */
	cmd.vector_flight_mode_ok = _vstatus.flag_vector_flight_mode_ok;
	/* allow manual override on IO (not allowed for multirotors or other systems with SAS) */
	cmd.manual_override_ok = _vstatus.flag_external_manual_override_ok;
	/* set desired PWM output rate */
	cmd.servo_rate = _update_rate;

	ret = hx_stream_send(_io_stream, &cmd, sizeof(cmd));
	if (ret)
		debug("send error %d", ret);
}

void
PX4IO::config_send()
{
	px4io_config	cfg;
	int		ret;

	cfg.f2i_config_magic = F2I_CONFIG_MAGIC;

	ret = hx_stream_send(_io_stream, &cfg, sizeof(cfg));
	if (ret)
		debug("config error %d", ret);
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
		_armed.armed = false;
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

namespace
{

void
test(void)
{
	int	fd;

	fd = open(PWM_OUTPUT_DEVICE_PATH, 0);

	if (fd < 0) {
		puts("open fail");
		exit(1);
	}

	ioctl(fd, PWM_SERVO_ARM, 0);
	ioctl(fd, PWM_SERVO_SET(0), 1000);
	ioctl(fd, PWM_SERVO_SET(1), 1100);
	ioctl(fd, PWM_SERVO_SET(2), 1200);
	ioctl(fd, PWM_SERVO_SET(3), 1300);
	ioctl(fd, PWM_SERVO_SET(4), 1400);
	ioctl(fd, PWM_SERVO_SET(5), 1500);
	ioctl(fd, PWM_SERVO_SET(6), 1600);
	ioctl(fd, PWM_SERVO_SET(7), 1700);

	close(fd);

	exit(0);
}

void
monitor(void)
{
	unsigned cancels = 3;
	printf("Hit <enter> three times to exit monitor mode\n");

	for (;;) {
		pollfd fds[1];

		fds[0].fd = 0;
		fds[0].events = POLLIN;
		poll(fds, 1, 500);

		if (fds[0].revents == POLLIN) {
			int c;
			read(0, &c, 1);
			if (cancels-- == 0)
				exit(0);
		}

		if (g_dev != nullptr)
			g_dev->dump_one = true;
	}
}

}

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

		/* look for the optional pwm update rate for the supported modes */
		if (strcmp(argv[2], "-u") == 0 || strcmp(argv[2], "--update-rate") == 0) {
			if (argc > 2 + 1) {
				g_dev->set_pwm_rate(atoi(argv[2 + 1]));
			} else {
				fprintf(stderr, "missing argument for pwm update rate (-u)\n");
				return 1;
			}
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

	if (!strcmp(argv[1], "rx_dsm") ||
	    !strcmp(argv[1], "rx_dsm_10bit") ||
	    !strcmp(argv[1], "rx_dsm_11bit") ||
	    !strcmp(argv[1], "rx_sbus") ||
	    !strcmp(argv[1], "rx_ppm"))
		errx(0, "receiver type is automatically detected, option '%s' is deprecated", argv[1]);

	if (!strcmp(argv[1], "test"))
		test();
	if (!strcmp(argv[1], "monitor"))
		monitor();

	errx(1, "need a command, try 'start', 'test', 'monitor' or 'update'");
}
