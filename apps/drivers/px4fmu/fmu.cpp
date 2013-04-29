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
 * @file fmu.cpp
 *
 * Driver/configurator for the PX4 FMU multi-purpose port.
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
#include <drivers/boards/px4fmu/px4fmu_internal.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_rc_input.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_outputs.h>

#include <systemlib/err.h>
#include <systemlib/ppm_decode.h>

class PX4FMU : public device::CDev
{
public:
	enum Mode {
		MODE_2PWM,
		MODE_4PWM,
		MODE_NONE
	};
	PX4FMU();
	virtual ~PX4FMU();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

private:
	static const unsigned _max_actuators = 4;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	int		_task;
	int		_t_actuators;
	int		_t_armed;
	orb_advert_t	_t_outputs;
	orb_advert_t	_t_actuators_effective;
	unsigned	_num_outputs;
	bool		_primary_pwm_device;

	volatile bool	_task_should_exit;
	bool		_armed;

	MixerGroup	*_mixers;

	actuator_controls_s _controls;

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main() __attribute__((noreturn));

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
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

const PX4FMU::GPIOConfig PX4FMU::_gpio_tab[] = {
	{GPIO_GPIO0_INPUT, GPIO_GPIO0_OUTPUT, 0},
	{GPIO_GPIO1_INPUT, GPIO_GPIO1_OUTPUT, 0},
	{GPIO_GPIO2_INPUT, GPIO_GPIO2_OUTPUT, GPIO_USART2_CTS_1},
	{GPIO_GPIO3_INPUT, GPIO_GPIO3_OUTPUT, GPIO_USART2_RTS_1},
	{GPIO_GPIO4_INPUT, GPIO_GPIO4_OUTPUT, GPIO_USART2_TX_1},
	{GPIO_GPIO5_INPUT, GPIO_GPIO5_OUTPUT, GPIO_USART2_RX_1},
	{GPIO_GPIO6_INPUT, GPIO_GPIO6_OUTPUT, GPIO_CAN2_TX_2},
	{GPIO_GPIO7_INPUT, GPIO_GPIO7_OUTPUT, GPIO_CAN2_RX_2},
};

const unsigned PX4FMU::_ngpio = sizeof(PX4FMU::_gpio_tab) / sizeof(PX4FMU::_gpio_tab[0]);

namespace
{

PX4FMU	*g_fmu;

} // namespace

PX4FMU::PX4FMU() :
	CDev("fmuservo", "/dev/px4fmu"),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_task(-1),
	_t_actuators(-1),
	_t_armed(-1),
	_t_outputs(0),
	_t_actuators_effective(0),
	_num_outputs(0),
	_primary_pwm_device(false),
	_task_should_exit(false),
	_armed(false),
	_mixers(nullptr)
{
	_debug_enabled = true;
}

PX4FMU::~PX4FMU()
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

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	g_fmu = nullptr;
}

int
PX4FMU::init()
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

	/* reset GPIOs */
	gpio_reset();

	/* start the IO interface task */
	_task = task_spawn("fmuservo",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2048,
			   (main_t)&PX4FMU::task_main_trampoline,
			   nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PX4FMU::task_main_trampoline(int argc, char *argv[])
{
	g_fmu->task_main();
}

int
PX4FMU::set_mode(Mode mode)
{
	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:	// multi-port with flow control lines as PWM
	case MODE_4PWM: // multi-port as 4 PWM outs
		debug("MODE_%dPWM", (mode == MODE_2PWM) ? 2 : 4);
		
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;

		/* XXX magic numbers */
		up_pwm_servo_init((mode == MODE_2PWM) ? 0x3 : 0xf);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);

		break;

	case MODE_NONE:
		debug("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;

		/* disable servo outputs - no need to set rates */
		up_pwm_servo_deinit();

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
PX4FMU::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	debug("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {
		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_pwm_servo_get_rate_group(group);
			if (mask == 0)
				continue;

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					warn("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}
			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_alt_rate) != OK) {
						warn("rate group set alt failed");
						return -EINVAL;
					}
				} else {
					if (up_pwm_servo_set_rate_group_update(group, _pwm_default_rate) != OK) {
						warn("rate group set default failed");
						return -EINVAL;
					}
				}
			}
		}
	}
	_pwm_alt_rate_channels = rate_map;
	_pwm_default_rate = default_rate;
	_pwm_alt_rate = alt_rate;

	return OK;
}

int
PX4FMU::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
PX4FMU::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

void
PX4FMU::task_main()
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

	/* advertise the effective control inputs */
	actuator_controls_effective_s controls_effective;
	memset(&controls_effective, 0, sizeof(controls_effective));
	/* advertise the effective control inputs */
	_t_actuators_effective = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_effective_1),
				   &controls_effective);

	pollfd fds[2];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_armed;
	fds[1].events = POLLIN;

	unsigned num_outputs = (_mode == MODE_2PWM) ? 2 : 4;

	// rc input, published to ORB
	struct rc_input_values rc_in;
	orb_advert_t to_input_rc = 0;

	memset(&rc_in, 0, sizeof(rc_in));
	rc_in.input_source = RC_INPUT_SOURCE_PX4FMU_PPM;

	log("starting");

	/* loop until killed */
	while (!_task_should_exit) {

		/*
		 * Adjust actuator topic update rate to keep up with
		 * the highest servo update rate configured.
		 *
		 * We always mix at max rate; some channels may update slower.
		 */
		unsigned max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;
		if (_current_update_rate != max_rate) {
			_current_update_rate = max_rate;
			int update_rate_in_ms = int(1000 / _current_update_rate);

			/* reject faster than 500 Hz updates */
			if (update_rate_in_ms < 2) {
				update_rate_in_ms = 2;
			}
			/* reject slower than 10 Hz updates */
			if (update_rate_in_ms > 100) {
				update_rate_in_ms = 100;
			}

			if (_mode != MODE_NONE) {
				debug("adjusted actuator update interval to %ums max_rate=%u _pwm_default_rate=%u _pwm_alt_rate=%u", update_rate_in_ms, max_rate, _pwm_default_rate, _pwm_alt_rate);
			}
			orb_set_interval(_t_actuators, update_rate_in_ms);

			// set to current max rate, even if we are actually checking slower/faster
			_current_update_rate = max_rate;
		}

		/* sleep waiting for data, stopping to check for PPM
		 * input at 100Hz */
		int ret = ::poll(&fds[0], 2, 10);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* do we have a control update? */
		if (fds[0].revents & POLLIN) {

			/* get controls - must always do this to avoid spinning */
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _t_actuators, &_controls);

			/* can we mix? */
			if (_mixers != nullptr) {

				/* do mixing */
				outputs.noutputs = _mixers->mix(&outputs.output[0], num_outputs);
				outputs.timestamp = hrt_absolute_time();

				// XXX output actual limited values
				memcpy(&controls_effective, &_controls, sizeof(controls_effective));

				orb_publish(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_effective_1), _t_actuators_effective, &controls_effective);

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

					/* output to the servo */
					up_pwm_servo_set(i, outputs.output[i]);
				}

				/* and publish for anyone that cares to see */
				orb_publish(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1), _t_outputs, &outputs);
			}
		}

		/* how about an arming update? */
		if (fds[1].revents & POLLIN) {
			actuator_armed_s aa;

			/* get new value */
			orb_copy(ORB_ID(actuator_armed), _t_armed, &aa);

			/* update PWM servo armed status if armed and not locked down */
			up_pwm_servo_arm(aa.armed && !aa.lockdown);
		}

		// see if we have new PPM input data
		if (ppm_last_valid_decode != rc_in.timestamp) {
			// we have a new PPM frame. Publish it.
			rc_in.channel_count = ppm_decoded_channels;
			if (rc_in.channel_count > RC_INPUT_MAX_CHANNELS) {
				rc_in.channel_count = RC_INPUT_MAX_CHANNELS;
			}
			for (uint8_t i=0; i<rc_in.channel_count; i++) {
				rc_in.values[i] = ppm_buffer[i];
			}
			rc_in.timestamp = ppm_last_valid_decode;

			/* lazily advertise on first publication */
			if (to_input_rc == 0) {
				to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_in);
			} else { 
				orb_publish(ORB_ID(input_rc), to_input_rc, &rc_in);
			}
		}
	}

	::close(_t_actuators);
	::close(_t_actuators_effective);
	::close(_t_armed);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	log("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
PX4FMU::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

int
PX4FMU::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	// XXX disabled, confusing users
	//debug("ioctl 0x%04x 0x%08x", cmd, arg);

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY)
		return ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		debug("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY)
		ret = CDev::ioctl(filp, cmd, arg);

	return ret;
}

int
PX4FMU::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_DISARM:
		up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
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
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);
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
	case PWM_SERVO_GET(1):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:	
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

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
PX4FMU::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[4];

	if (count > 4) {
		// we only have 4 PWM outputs on the FMU
		count = 4;
	}

	// allow for misaligned values
	memcpy(values, buffer, count*2);

	for (uint8_t i=0; i<count; i++) {
		up_pwm_servo_set(i, values[i]);
	}
	return count * 2;
}

void
PX4FMU::gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, GPIO driver chip
	 * to input mode.
	 */
	for (unsigned i = 0; i < _ngpio; i++)
		stm32_configgpio(_gpio_tab[i].input);

	stm32_gpiowrite(GPIO_GPIO_DIR, 0);
	stm32_configgpio(GPIO_GPIO_DIR);
}

void
PX4FMU::gpio_set_function(uint32_t gpios, int function)
{
	/*
	 * GPIOs 0 and 1 must have the same direction as they are buffered
	 * by a shared 2-port driver.  Any attempt to set either sets both.
	 */
	if (gpios & 3) {
		gpios |= 3;

		/* flip the buffer to output mode if required */
		if (GPIO_SET_OUTPUT == function)
			stm32_gpiowrite(GPIO_GPIO_DIR, 1);
	}

	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (gpios & (1 << i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				stm32_configgpio(_gpio_tab[i].input);
				break;

			case GPIO_SET_OUTPUT:
				stm32_configgpio(_gpio_tab[i].output);
				break;

			case GPIO_SET_ALT_1:
				if (_gpio_tab[i].alt != 0)
					stm32_configgpio(_gpio_tab[i].alt);

				break;
			}
		}
	}

	/* flip buffer to input mode if required */
	if ((GPIO_SET_INPUT == function) && (gpios & 3))
		stm32_gpiowrite(GPIO_GPIO_DIR, 0);
}

void
PX4FMU::gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (gpios & (1 << i))
			stm32_gpiowrite(_gpio_tab[i].output, value);
}

uint32_t
PX4FMU::gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (stm32_gpioread(_gpio_tab[i].input))
			bits |= (1 << i);

	return bits;
}

int
PX4FMU::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		gpio_set_function(arg, cmd);
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = gpio_read();
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
};

PortMode g_port_mode;

int
fmu_new_mode(PortMode new_mode)
{
	uint32_t gpio_bits;
	PX4FMU::Mode servo_mode;

	/* reset to all-inputs */
	g_fmu->ioctl(0, GPIO_RESET, 0);

	gpio_bits = 0;
	servo_mode = PX4FMU::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		/* nothing more to do here */
		break;

	case PORT_FULL_SERIAL:
		/* set all multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_1 | GPIO_MULTI_2 | GPIO_MULTI_3 | GPIO_MULTI_4;
		break;

	case PORT_FULL_PWM:
		/* select 4-pin PWM mode */
		servo_mode = PX4FMU::MODE_4PWM;
		break;

	case PORT_GPIO_AND_SERIAL:
		/* set RX/TX multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		break;

	case PORT_PWM_AND_SERIAL:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		/* set RX/TX multi-GPIOs to serial mode */
		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		break;

	case PORT_PWM_AND_GPIO:
		/* select 2-pin PWM mode */
		servo_mode = PX4FMU::MODE_2PWM;
		break;
	}

	/* adjust GPIO config for serial mode(s) */
	if (gpio_bits != 0)
		g_fmu->ioctl(0, GPIO_SET_ALT_1, gpio_bits);

	/* (re)set the PWM output mode */
	g_fmu->set_mode(servo_mode);

	return OK;
}

int
fmu_start(void)
{
	int ret = OK;

	if (g_fmu == nullptr) {

		g_fmu = new PX4FMU;

		if (g_fmu == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_fmu->init();

			if (ret != OK) {
				delete g_fmu;
				g_fmu = nullptr;
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

	if (fd < 0)
		errx(1, "open fail");

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0)       err(1, "servo arm failed");

	if (ioctl(fd, PWM_SERVO_SET(0), 1000) < 0) err(1, "servo 1 set failed");

	if (ioctl(fd, PWM_SERVO_SET(1), 1200) < 0) err(1, "servo 2 set failed");

	if (ioctl(fd, PWM_SERVO_SET(2), 1400) < 0) err(1, "servo 3 set failed");

	if (ioctl(fd, PWM_SERVO_SET(3), 1600) < 0) err(1, "servo 4 set failed");

	close(fd);

	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5)
		errx(1, "fmu fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle < 0)
		errx(1, "advertise failed");

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle < 0)
		errx(1, "advertise failed 2");

	exit(0);
}

} // namespace

extern "C" __EXPORT int fmu_main(int argc, char *argv[]);

int
fmu_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	if (fmu_start() != OK)
		errx(1, "failed to start the FMU driver");

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_serial")) {
		new_mode = PORT_FULL_SERIAL;

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;

	} else if (!strcmp(verb, "mode_gpio_serial")) {
		new_mode = PORT_GPIO_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_serial")) {
		new_mode = PORT_PWM_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_gpio")) {
		new_mode = PORT_PWM_AND_GPIO;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode)
			return OK;

		/* switch modes */
		int ret = fmu_new_mode(new_mode);
		exit(ret == OK ? 0 : 1);
	}

	if (!strcmp(verb, "test"))
		test();

	if (!strcmp(verb, "fake"))
		fake(argc - 1, argv + 1);

	fprintf(stderr, "FMU: unrecognised command, try:\n");
	fprintf(stderr, "  mode_gpio, mode_serial, mode_pwm, mode_gpio_serial, mode_pwm_serial, mode_pwm_gpio\n");
	exit(1);
}
