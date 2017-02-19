/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * Driver/configurator for the virtual PWMSim port.
 *
 * This virtual driver emulates PWM / servo outputs for setups where
 * the connected hardware does not provide enough or no PWM outputs.
 *
 * Its only function is to take actuator_control uORB messages,
 * mix them with any loaded mixer and output the result to the
 * actuator_output uORB topic. PWMSim can also be performed with normal
 * PWM outputs, a special flag prevents the outputs to be operated
 * during PWMSim mode. If PWMSim is not performed with a standalone FMU,
 * but in a real system, it is NOT recommended to use this virtual
 * driver. Use instead the normal FMU or IO driver.
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_common.h>

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
#include <cmath>
#include <string.h>
#include <unistd.h>

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

#ifdef __PX4_NUTTX
class PWMSim : public device::CDev
#else
class PWMSim : public device::VDev
#endif
{
	const uint32_t PWM_SIM_DISARMED_MAGIC = 900;
	const uint32_t PWM_SIM_FAILSAFE_MAGIC = 600;
public:
	enum Mode {
		MODE_2PWM,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_12PWM,
		MODE_16PWM,
		MODE_NONE
	};
	PWMSim();
	virtual ~PWMSim();

	virtual int     ioctl(device::file_t *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	int		set_pwm_rate(unsigned rate);
	int		_task;

private:
	static const unsigned _max_actuators = 8;

	Mode		_mode;
	int 		_update_rate;
	int 		_current_update_rate;
	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;
	int		_armed_sub;
	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	bool		_primary_pwm_device;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;

	volatile bool	_task_should_exit;
	static bool	_armed;
	static bool	_lockdown;
	static bool	_failsafe;

	MixerGroup	*_mixers;

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	int		pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg);
	void 	subscribe();

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset();
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read();
	int		gpio_ioctl(device::file_t *filp, int cmd, unsigned long arg);

};

namespace
{

PWMSim	*g_pwm_sim = nullptr;

} // namespace

bool PWMSim::_armed = false;
bool PWMSim::_lockdown = false;
bool PWMSim::_failsafe = false;

PWMSim::PWMSim() :
#ifdef __PX4_NUTTX
	CDev
#else
	VDev
#endif
	("pwm_out_sim", PWM_OUTPUT0_DEVICE_PATH),
	_task(-1),
	_mode(MODE_NONE),
	_update_rate(50),
	_current_update_rate(0),
	_poll_fds{},
	_poll_fds_num(0),
	_armed_sub(-1),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_primary_pwm_device(false),
	_groups_required(0),
	_groups_subscribed(0),
	_task_should_exit(false),
	_mixers(nullptr)
{
	_debug_enabled = true;
	memset(_controls, 0, sizeof(_controls));

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		_control_subs[i] = -1;
	}
}

PWMSim::~PWMSim()
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
				px4_task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	g_pwm_sim = nullptr;
}

int
PWMSim::init()
{
	int ret;

	ASSERT(_task == -1);

	/* do regular cdev init */
#ifdef __PX4_NUTTX
	ret = CDev::init();
#else
	ret = VDev::init();
#endif

	if (ret != OK) {
		return ret;

	} else {
		_primary_pwm_device = true;
	}

	/* start the PWMSim interface task */
	_task = px4_task_spawn_cmd("pwm_out_sim",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT,
				   1200,
				   (px4_main_t)&PWMSim::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_INFO("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PWMSim::task_main_trampoline(int argc, char *argv[])
{
	g_pwm_sim->task_main();
}

int
PWMSim::set_mode(Mode mode)
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
		PX4_INFO("MODE_2PWM");
		/* multi-port with flow control lines as PWM */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 2;
		break;

	case MODE_4PWM:
		PX4_INFO("MODE_4PWM");
		/* multi-port as 4 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 4;
		break;

	case MODE_8PWM:
		PX4_INFO("MODE_8PWM");
		/* multi-port as 8 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 8;
		break;

	case MODE_12PWM:
		PX4_INFO("MODE_12PWM");
		/* multi-port as 12 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 12;
		break;

	case MODE_16PWM:
		PX4_INFO("MODE_16PWM");
		/* multi-port as 16 PWM outs */
		_update_rate = 50;	/* default output rate */
		_num_outputs = 16;
		break;

	case MODE_NONE:
		PX4_INFO("MODE_NONE");
		/* disable servo outputs and set a very low update rate */
		_update_rate = 10;
		_num_outputs = 0;
		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
PWMSim::set_pwm_rate(unsigned rate)
{
	if ((rate > 500) || (rate < 10)) {
		return -EINVAL;
	}

	_update_rate = rate;
	return OK;
}

void
PWMSim::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			PX4_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
PWMSim::task_main()
{
	/* force a reset of the update rate */
	_current_update_rate = 0;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* advertise the mixed control outputs */
	actuator_outputs_s outputs = {};

	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &outputs);


	/* loop until killed */
	while (!_task_should_exit) {

		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
		}

		/* handle update rate changes */
		if (_current_update_rate != _update_rate) {
			int update_rate_in_ms = int(1000 / _update_rate);

			if (update_rate_in_ms < 2) {
				update_rate_in_ms = 2;
			}

			for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] >= 0) {
					orb_set_interval(_control_subs[i], update_rate_in_ms);
				}
			}

			// up_pwm_servo_set_rate(_update_rate);
			_current_update_rate = _update_rate;
		}

		/* this can happen during boot, but after the sleep its likely resolved */
		if (_poll_fds_num == 0) {
			usleep(1000 * 1000);

			PX4_DEBUG("no valid fds");
			continue;
		}

		/* sleep waiting for data, but no more than a second */
		int ret = px4_poll(&_poll_fds[0], _poll_fds_num, 1000);

		/* this would be bad... */
		if (ret < 0) {
			DEVICE_LOG("poll error %d", errno);
			continue;
		}

		if (ret == 0) {
			// timeout
			continue;
		}

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_armed && _mixers != nullptr) {

			size_t num_outputs;

			switch (_mode) {
			case MODE_2PWM:
				num_outputs = 2;
				break;

			case MODE_4PWM:
				num_outputs = 4;
				break;

			case MODE_6PWM:
				num_outputs = 6;
				break;

			case MODE_8PWM:
				num_outputs = 8;
				break;

			case MODE_16PWM:
				num_outputs = 16;
				break;

			default:
				num_outputs = 0;
				break;
			}

			/* do mixing */
			num_outputs = _mixers->mix(&outputs.output[0], num_outputs, nullptr);
			outputs.noutputs = num_outputs;
			outputs.timestamp = hrt_absolute_time();

			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs.output) / sizeof(outputs.output[0]); i++) {
				if (i >= num_outputs) {
					outputs.output[i] = NAN;
				}
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < outputs.noutputs &&
				    PX4_ISFINITE(outputs.output[i]) &&
				    outputs.output[i] >= -1.0f &&
				    outputs.output[i] <= 1.0f) {
					/* scale for PWM output 1000 - 2000us */
					outputs.output[i] = 1500 + (500 * outputs.output[i]);

				} else {
					/*
					 * Value is NaN, INF or out of band - set to the minimum value.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					outputs.output[i] = PWM_SIM_DISARMED_MAGIC;
				}
			}

			/* overwrite outputs in case of force_failsafe */
			if (_failsafe) {
				for (size_t i = 0; i < num_outputs; i++) {
					outputs.output[i] = PWM_SIM_FAILSAFE_MAGIC;
				}
			}

			/* overwrite outputs in case of lockdown */
			if (_lockdown) {
				for (size_t i = 0; i < num_outputs; i++) {
					outputs.output[i] = 0.0;
				}
			}

			/* and publish for anyone that cares to see */
			orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
		}

		/* how about an arming update? */
		bool updated;
		actuator_armed_s aa;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &aa);
			/* do not obey the lockdown value, as lockdown is for PWMSim. Only obey manual lockdown */
			_armed = aa.armed;
			_failsafe = aa.force_failsafe;
			_lockdown = aa.manual_lockdown;
		}
	}

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);

	/* make sure servos are off */
	// up_pwm_servo_deinit();

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
}

int
PWMSim::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	if (_armed) {
		input = controls[control_group].control[control_index];

	} else {
		/* clamp actuator to zero if not armed */
		input = 0.0f;
	}

	return 0;
}

int
PWMSim::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
	case MODE_8PWM:
	case MODE_12PWM:
	case MODE_16PWM:
		ret = PWMSim::pwm_ioctl(filp, cmd, arg);
		break;

	default:
		ret = -ENOTTY;
		PX4_INFO("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
#ifdef __PX4_NUTTX
		ret = CDev::ioctl(filp, cmd, arg);
#else
		ret = VDev::ioctl(filp, cmd, arg);
#endif
	}

	return ret;
}

int
PWMSim::pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg)
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
		// PWMSim always outputs at the alternate (usually faster) rate
		g_pwm_sim->set_pwm_rate(arg);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		// PWMSim always outputs at the alternate (usually faster) rate
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = 400;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = 400;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = 0;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 850;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 900;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 1000;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 1500;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = 2000;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

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

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_num_outputs < 8) {
			ret = -EINVAL;
			break;
		}

	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
		if (_num_outputs < 4) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0): {
			*(servo_position_t *)arg = 1500;
			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
			// no restrictions on output grouping
			unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = (1 << channel);
			break;
		}

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		if (_mode == MODE_16PWM) {
			*(unsigned *)arg = 16;

		} else if (_mode == MODE_8PWM) {

			*(unsigned *)arg = 8;

		} else if (_mode == MODE_4PWM) {

			*(unsigned *)arg = 4;

		} else {

			*(unsigned *)arg = 2;
		}

		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)&_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr) {
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)&_controls);
				}

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)&_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					PX4_ERR("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {
					_mixers->groups_required(_groups_required);
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

PortMode g_port_mode = PORT_MODE_UNDEFINED;

int
hil_new_mode(PortMode new_mode)
{
	// uint32_t gpio_bits;


//	/* reset to all-inputs */
//	g_pwm_sim->ioctl(0, GPIO_RESET, 0);

	// gpio_bits = 0;

	PWMSim::Mode servo_mode = PWMSim::MODE_NONE;

	switch (new_mode) {
	case PORT_MODE_UNDEFINED:
	case PORT1_MODE_UNSET:
	case PORT2_MODE_UNSET:
		/* nothing more to do here */
		break;

	case PORT1_FULL_PWM:
		/* select 4-pin PWM mode */
		servo_mode = PWMSim::MODE_8PWM;
		break;

	case PORT1_PWM_AND_SERIAL:
		/* select 2-pin PWM mode */
		servo_mode = PWMSim::MODE_2PWM;
//		/* set RX/TX multi-GPIOs to serial mode */
//		gpio_bits = GPIO_MULTI_3 | GPIO_MULTI_4;
		break;

	case PORT1_PWM_AND_GPIO:
		/* select 2-pin PWM mode */
		servo_mode = PWMSim::MODE_2PWM;
		break;

	case PORT2_8PWM:
		/* select 8-pin PWM mode */
		servo_mode = PWMSim::MODE_8PWM;
		break;

	case PORT2_12PWM:
		/* select 12-pin PWM mode */
		servo_mode = PWMSim::MODE_12PWM;
		break;

	case PORT2_16PWM:
		/* select 16-pin PWM mode */
		servo_mode = PWMSim::MODE_16PWM;
		break;
	}

//	/* adjust GPIO config for serial mode(s) */
//	if (gpio_bits != 0)
//		g_pwm_sim->ioctl(0, GPIO_SET_ALT_1, gpio_bits);

	/* (re)set the PWM output mode */
	g_pwm_sim->set_mode(servo_mode);

	return OK;
}

int
test()
{
	int	fd;

	fd = px4_open(PWM_OUTPUT0_DEVICE_PATH, 0);

	if (fd < 0) {
		puts("open fail");
		return -ENODEV;
	}

	px4_ioctl(fd, PWM_SERVO_ARM, 0);
	px4_ioctl(fd, PWM_SERVO_SET(0), 1000);

	px4_close(fd);

	return OK;
}

int
fake(int argc, char *argv[])
{
	if (argc < 5) {
		puts("pwm_out_sim fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
		return -EINVAL;
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], nullptr, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], nullptr, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], nullptr, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], nullptr, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		puts("advertise failed");
		return 1;
	}

	return 0;
}

} // namespace

extern "C" __EXPORT int pwm_out_sim_main(int argc, char *argv[]);

static void
usage()
{
	PX4_WARN("unrecognized command, try:");
	PX4_WARN("  mode_pwm, mode_gpio_serial, mode_pwm_serial, mode_pwm_gpio, mode_port2_pwm8, mode_port2_pwm12, mode_port2_pwm16");
}

int
pwm_out_sim_main(int argc, char *argv[])
{
	PortMode new_mode = PORT_MODE_UNDEFINED;
	const char *verb;
	int ret = OK;

	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	verb = argv[1];

	if (g_pwm_sim == nullptr) {
		g_pwm_sim = new PWMSim;

		if (g_pwm_sim == nullptr) {
			return -ENOMEM;
		}
	}

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

	} else if (!strcmp(verb, "mode_pwm16")) {
		new_mode = PORT2_16PWM;
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNDEFINED) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		ret = hil_new_mode(new_mode);

	} else if (!strcmp(verb, "test")) {
		ret = test();
	}

	else if (!strcmp(verb, "fake")) {
		ret = fake(argc - 1, argv + 1);
	}

	else {
		usage();
		ret = -EINVAL;
	}

	if (ret == OK && g_pwm_sim->_task == -1) {
		ret = g_pwm_sim->init();

		if (ret != OK) {
			warnx("failed to start the pwm_out_sim driver");
			delete g_pwm_sim;
			g_pwm_sim = nullptr;
		}
	}

	return ret;
}
