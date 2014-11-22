/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file hil.cpp
 *
 * Driver/configurator for the virtual HIL port.
 *
 * This virtual driver emulates PWM / servo outputs for setups where
 * the connected hardware does not provide enough or no PWM outputs.
 *
 * Its only function is to take actuator_control uORB messages,
 * mix them with any loaded mixer and output the result to the
 * actuator_output uORB topic. HIL can also be performed with normal
 * PWM outputs, a special flag prevents the outputs to be operated
 * during HIL mode. If HIL is not performed with a standalone FMU,
 * but in a real system, it is NOT recommended to use this virtual
 * driver. Use instead the normal FMU or IO driver.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <systemlib/systemlib.h>
#include <systemlib/mixer/mixer.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>

#include <systemlib/err.h>

class HIL : public device::CDev
{
public:
	enum Mode {
		MODE_2PWM,
		MODE_4PWM,
        MODE_8PWM,
        MODE_12PWM,
        MODE_16PWM,
		MODE_NONE
	};
	HIL();
	virtual ~HIL();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	int		set_pwm_rate(unsigned rate);

private:
	static const unsigned _max_actuators = 4;

	Mode		_mode;
	int 		_update_rate;
	int 		_current_update_rate;
	int		_task;
	int		_t_actuators;
	int		_t_armed;
	orb_advert_t	_t_outputs;
	unsigned	_num_outputs;
	bool		_primary_pwm_device;

	volatile bool	_task_should_exit;
	bool		_armed;

	MixerGroup	*_mixers;

	actuator_controls_s _controls;

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

	static int	control_callback(uintptr_t handle,
			uint8_t control_group,
			uint8_t control_index,
			float &input);

	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);

};

namespace
{

HIL	*g_hil;

} // namespace

HIL::HIL() :
	CDev("hilservo", PWM_OUTPUT_DEVICE_PATH/*"/dev/hil" XXXL*/),
	_mode(MODE_NONE),
	_update_rate(50),
	_current_update_rate(0),
	_task(-1),
	_t_actuators(-1),
	_t_armed(-1),
	_t_outputs(0),
	_num_outputs(0),
	_primary_pwm_device(false),
	_task_should_exit(false),
	_armed(false),
	_mixers(nullptr)
{
	_debug_enabled = true;
}

HIL::~HIL()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

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

	// XXX already claimed with CDEV
	// /* clean up the alternate device node */
	// if (_primary_pwm_device)
	// 	unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	g_hil = nullptr;
}

int
HIL::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK)
		return ret;

	// XXX already claimed with CDEV
	///* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	//ret = register_driver(PWM_OUTPUT_DEVICE_PATH, &fops, 0666, (void *)this);
	if (ret == OK) {
		log("default PWM output device");
		_primary_pwm_device = true;
	}

	/* reset GPIOs */
	// gpio_reset();

	/* start the HIL interface task */
	_task = task_spawn_cmd("fmuhil",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   1200,
			   (main_t)&HIL::task_main_trampoline,
			   nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
HIL::task_main_trampoline(int argc, char *argv[])
{
	g_hil->task_main();
}

int
HIL::set_mode(Mode mode)
{
	/* 
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:
		debug("MODE_2PWM");
		/* multi-port with flow control lines as PWM */
		_update_rate = 50;	/* default output rate */
		break;

	case MODE_4PWM:
		debug("MODE_4PWM");
		/* multi-port as 4 PWM outs */
		_update_rate = 50;	/* default output rate */
		break;
            
    	case MODE_8PWM:
            debug("MODE_8PWM");
            /* multi-port as 8 PWM outs */
            _update_rate = 50;	/* default output rate */
            break;
            
        case MODE_12PWM:
            debug("MODE_12PWM");
            /* multi-port as 12 PWM outs */
            _update_rate = 50;	/* default output rate */
            break;
            
        case MODE_16PWM:
            debug("MODE_16PWM");
            /* multi-port as 16 PWM outs */
            _update_rate = 50;	/* default output rate */
            break;

	case MODE_NONE:
		debug("MODE_NONE");
		/* disable servo outputs and set a very low update rate */
		_update_rate = 10;
		break;

	default:
		return -EINVAL;
	}
	_mode = mode;
	return OK;
}

int
HIL::set_pwm_rate(unsigned rate)
{
	if ((rate > 500) || (rate < 10))
		return -EINVAL;

	_update_rate = rate;
	return OK;
}

void
HIL::task_main()
{
	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1));
	/* force a reset of the update rate */
	_current_update_rate = 0;

	_t_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_armed, 200);		/* 5Hz update rate */

	/* advertise the mixed control outputs */
	actuator_outputs_s outputs;
	memset(&outputs, 0, sizeof(outputs));
	/* advertise the mixed control outputs */
	_t_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &outputs);

	pollfd fds[2];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_armed;
	fds[1].events = POLLIN;

	unsigned num_outputs;

	/* select the number of virtual outputs */
	switch (_mode) {
	case MODE_2PWM:
		num_outputs = 2;
		break;

	case MODE_4PWM:
		num_outputs = 4;
		break;

	case MODE_8PWM:
	case MODE_12PWM:
	case MODE_16PWM:
		// XXX only support the lower 8 - trivial to extend
		num_outputs = 8;
		break;

	case MODE_NONE:
	default:
		num_outputs = 0;
		break;
	}

	log("starting");

	/* loop until killed */
	while (!_task_should_exit) {

		/* handle update rate changes */
		if (_current_update_rate != _update_rate) {
			int update_rate_in_ms = int(1000 / _update_rate);
			if (update_rate_in_ms < 2)
				update_rate_in_ms = 2;
			orb_set_interval(_t_actuators, update_rate_in_ms);
			// up_pwm_servo_set_rate(_update_rate);
			_current_update_rate = _update_rate;
		}

		/* sleep waiting for data, but no more than a second */
		int ret = ::poll(&fds[0], 2, 1000);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			continue;
		}

		/* do we have a control update? */
		if (fds[0].revents & POLLIN) {

			/* get controls - must always do this to avoid spinning */
			orb_copy(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS :
				     ORB_ID(actuator_controls_1), _t_actuators, &_controls);

			/* can we mix? */
			if (_mixers != nullptr) {

				/* do mixing */
				outputs.noutputs = _mixers->mix(&outputs.output[0], num_outputs);
				outputs.timestamp = hrt_absolute_time();

				/* iterate actuators */
				for (unsigned i = 0; i < num_outputs; i++) {

					/* last resort: catch NaN, INF and out-of-band errors */
					if (i < outputs.noutputs &&
						isfinite(outputs.output[i]) &&
						outputs.output[i] >= -1.0f &&
						outputs.output[i] <= 1.0f) {
						/* scale for PWM output 900 - 2100us */
						outputs.output[i] = 1500 + (600 * outputs.output[i]);
					} else {
						/*
						 * Value is NaN, INF or out of band - set to the minimum value.
						 * This will be clearly visible on the servo status and will limit the risk of accidentally
						 * spinning motors. It would be deadly in flight.
						 */
						outputs.output[i] = 900;
					}
				}

				/* and publish for anyone that cares to see */
				orb_publish(ORB_ID_VEHICLE_CONTROLS, _t_outputs, &outputs);
			}
		}

		/* how about an arming update? */
		if (fds[1].revents & POLLIN) {
			actuator_armed_s aa;

			/* get new value */
			orb_copy(ORB_ID(actuator_armed), _t_armed, &aa);
		}
	}

	::close(_t_actuators);
	::close(_t_armed);

	/* make sure servos are off */
	// up_pwm_servo_deinit();

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
HIL::control_callback(uintptr_t handle,
				      uint8_t control_group,
				      uint8_t control_index,
				      float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

int
HIL::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	debug("ioctl 0x%04x 0x%08x", cmd, arg);

	// /* try it as a GPIO ioctl first */
	// ret = HIL::gpio_ioctl(filp, cmd, arg);
	// if (ret != -ENOTTY)
	// 	return ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch(_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
        case MODE_8PWM:
        case MODE_12PWM:
        case MODE_16PWM:
		ret = HIL::pwm_ioctl(filp, cmd, arg);
		break;
	default:
		ret = -ENOTTY;
		debug("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY)
		ret = CDev::ioctl(filp, cmd, arg);

	return ret;
}

int
HIL::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;
	// int channel;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		// up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_DISARM:
		// up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		// HIL always outputs at the alternate (usually faster) rate 
		g_hil->set_pwm_rate(arg);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		// HIL always outputs at the alternate (usually faster) rate 
		break;

	case PWM_SERVO_SET(2):
	case PWM_SERVO_SET(3):
		if (_mode != MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

		/* FALLTHROUGH */
	case PWM_SERVO_SET(0):
	case PWM_SERVO_SET(1):
		if (arg < 2100) {
			// channel = cmd - PWM_SERVO_SET(0);
//			up_pwm_servo_set(channel, arg); XXX

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(2):
	case PWM_SERVO_GET(3):
		if (_mode != MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

		/* FALLTHROUGH */
	case PWM_SERVO_GET(0):
	case PWM_SERVO_GET(1): {
			// channel = cmd - PWM_SERVO_SET(0);
			// *(servo_position_t *)arg = up_pwm_servo_get(channel);
			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
		// no restrictions on output grouping
		unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

		*(uint32_t *)arg = (1 << channel);
		break;
	}

	case MIXERIOCGETOUTPUTCOUNT:
		if (_mode == MODE_4PWM) {
			*(unsigned *)arg = 4;

		} else {
			*(unsigned *)arg = 2;
		}

		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)&_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)&_controls);

				_mixers->add_mixer(mixer);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr)
				_mixers = new MixerGroup(control_callback, (uintptr_t)&_controls);

			if (_mixers == nullptr) {
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					debug("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					ret = -EINVAL;
				}
			}
			break;
		}


	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNDEFINED = 0,
	PORT1_MODE_UNSET,
	PORT1_FULL_PWM,
	PORT1_PWM_AND_SERIAL,
	PORT1_PWM_AND_GPIO,
	PORT2_MODE_UNSET,
	PORT2_8PWM,
	PORT2_12PWM,
	PORT2_16PWM,
};

PortMode g_port_mode;

int
hil_new_mode(PortMode new_mode)
{
	// uint32_t gpio_bits;
	

//	/* reset to all-inputs */
//	g_hil->ioctl(0, GPIO_RESET, 0);

	// gpio_bits = 0;

	HIL::Mode servo_mode = HIL::MODE_NONE;

	switch (new_mode) {
	case PORT_MODE_UNDEFINED:
	case PORT1_MODE_UNSET:
    	case PORT2_MODE_UNSET:
            /* nothing more to do here */
            break;

	case PORT1_FULL_PWM:
		/* select 4-pin PWM mode */
		servo_mode = HIL::MODE_4PWM;
		break;

	case PORT1_PWM_AND_SERIAL:
		/* select 2-pin PWM mode */
		servo_mode = HIL::MODE_2PWM;
//		/* set RX/TX multi-GPIOs to serial mode */
//		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		break;

	case PORT1_PWM_AND_GPIO:
		/* select 2-pin PWM mode */
		servo_mode = HIL::MODE_2PWM;
		break;
            
        case PORT2_8PWM:
            /* select 8-pin PWM mode */
            servo_mode = HIL::MODE_8PWM;
            break;
            
        case PORT2_12PWM:
            /* select 12-pin PWM mode */
            servo_mode = HIL::MODE_12PWM;
            break;
            
        case PORT2_16PWM:
            /* select 16-pin PWM mode */
            servo_mode = HIL::MODE_16PWM;
            break;
	}

//	/* adjust GPIO config for serial mode(s) */
//	if (gpio_bits != 0)
//		g_hil->ioctl(0, GPIO_SET_ALT_1, gpio_bits);

	/* (re)set the PWM output mode */
	g_hil->set_mode(servo_mode);

	return OK;
}

int
hil_start(void)
{
	int ret = OK;

	if (g_hil == nullptr) {

		g_hil = new HIL;

		if (g_hil == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_hil->init();

			if (ret != OK) {
				delete g_hil;
				g_hil = nullptr;
			}
		}
	}

	return ret;
}

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

	close(fd);

	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		puts("hil fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
		exit(1);
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle < 0) {
		puts("advertise failed");
		exit(1);
	}

	exit(0);
}

} // namespace

extern "C" __EXPORT int hil_main(int argc, char *argv[]);

int
hil_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNDEFINED;
	const char *verb = argv[1];

	if (hil_start() != OK)
		errx(1, "failed to start the HIL driver");

	/*
	 * Mode switches.
	 */

 	// this was all cut-and-pasted from the FMU driver; it's junk
	if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT1_FULL_PWM;

	} else if (!strcmp(verb, "mode_pwm_serial")) {
		new_mode = PORT1_PWM_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_gpio")) {
		new_mode = PORT1_PWM_AND_GPIO;

	} else if (!strcmp(verb, "mode_port2_pwm8")) {
		new_mode = PORT2_8PWM;

	} else if (!strcmp(verb, "mode_port2_pwm12")) {
		new_mode = PORT2_8PWM;

	} else if (!strcmp(verb, "mode_port2_pwm16")) {
		new_mode = PORT2_8PWM;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNDEFINED) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode)
			return OK;

		/* switch modes */
		return hil_new_mode(new_mode);
	}

	if (!strcmp(verb, "test"))
		test();

	if (!strcmp(verb, "fake"))
		fake(argc - 1, argv + 1);


	fprintf(stderr, "HIL: unrecognized command, try:\n");
	fprintf(stderr, "  mode_pwm, mode_gpio_serial, mode_pwm_serial, mode_pwm_gpio, mode_port2_pwm8, mode_port2_pwm12, mode_port2_pwm16\n");
	return -EINVAL;
}
