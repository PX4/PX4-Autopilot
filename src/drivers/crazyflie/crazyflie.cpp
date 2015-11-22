/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file crazyflie.cpp
 *
 * Driver/configurator for the PX4 FMU multi-purpose port on v1 and v2 boards.
 */

#include <px4_config.h>

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
#include <drivers/device/i2c.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/board_serial.h>
#include <systemlib/param/param.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_rc_input.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>


#ifdef HRT_PPM_CHANNEL
# include <systemlib/ppm_decode.h>
#endif

/*
 * This is the analog to FMU_INPUT_DROP_LIMIT_US on the IO side
 */

#define CONTROL_INPUT_DROP_LIMIT_MS		20
#define NAN_VALUE	(0.0f/0.0f)

#define CRAZYFLIE_DEVICE_PATH PWM_OUTPUT0_DEVICE_PATH

class Crazyflie : public device::CDev
{
public:
	Crazyflie();
	virtual ~Crazyflie();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

private:
	static const unsigned _max_actuators = 4;

	unsigned _current_update_rate;
	struct work_s	_work;
	int		_armed_sub;
	int		_param_sub;
	orb_advert_t	_outputs_pub;
	int		_class_instance;

	volatile bool	_initialized;
	bool		_servo_armed;
	bool		_pwm_on;

	MixerGroup	*_mixers;

	uint32_t	_groups_required;
	uint32_t	_groups_subscribed;
	int		_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t	_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	pollfd	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned	_poll_fds_num;

	static pwm_limit_t	_pwm_limit;
	static actuator_armed_s	_armed;
	uint16_t	_failsafe_pwm[_max_actuators];
	uint16_t	_disarmed_pwm[_max_actuators];
	uint16_t	_min_pwm[_max_actuators];
	uint16_t	_max_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;

	static bool	arm_nothrottle() { return (_armed.prearmed && !_armed.armed); }

	static void	cycle_trampoline(void *arg);

	void		cycle();
	void		work_start();
	void		work_stop();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);
	void		subscribe();
	void		publish_pwm_outputs(uint16_t *values, size_t numvalues);

	void	set_bounded_pwm(uint16_t &out, struct pwm_output_values *pwm, size_t i);

	/* do not allow to copy due to ptr data members */
	Crazyflie(const Crazyflie &);
	Crazyflie operator=(const Crazyflie &);
};

pwm_limit_t		Crazyflie::_pwm_limit;
actuator_armed_s	Crazyflie::_armed = {};

namespace
{

Crazyflie	*g_crazyflie;

} // namespace

#define CRAZYFLIE_PWM_MIN 0
#define CRAZYFLIE_PWM_MAX 255

Crazyflie::Crazyflie() :
	CDev("crazyflie", CRAZYFLIE_DEVICE_PATH),
	_current_update_rate(0),
	_work{},
	_armed_sub(-1),
	_param_sub(-1),
	_outputs_pub(nullptr),
	_class_instance(0),
	_initialized(false),
	_servo_armed(false),
	_pwm_on(false),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_control_subs{ -1},
	_poll_fds_num(0),
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0)
{
	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = CRAZYFLIE_PWM_MIN;
		_max_pwm[i] = CRAZYFLIE_PWM_MAX;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	/* only enable this during development */
	_debug_enabled = false;
}

Crazyflie::~Crazyflie()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		work_stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	g_crazyflie = nullptr;
}

int
Crazyflie::init()
{
	int ret;

	ASSERT(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		warnx("FAILED registering class device");
	}

	work_start();

	return OK;
}

void
Crazyflie::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] > 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void
Crazyflie::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs;
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == nullptr) {
		int instance = -1;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}

void
Crazyflie::work_start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&Crazyflie::cycle_trampoline, this, 0);
}

void
Crazyflie::cycle_trampoline(void *arg)
{
	Crazyflie *dev = reinterpret_cast<Crazyflie *>(arg);

	dev->cycle();
}

void
Crazyflie::cycle()
{
	if (!_initialized) {
		/* force a reset of the update rate */
		_current_update_rate = 0;

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
		_param_sub = orb_subscribe(ORB_ID(parameter_update));

		/* initialize PWM limit lib */
		pwm_limit_init(&_pwm_limit);

		up_pwm_servo_init(0x0f);

		_initialized = true;
	}


	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;
		/* force setting update rate */
		_current_update_rate = 0;
	}

	/*
	 * Adjust actuator topic update rate to keep up with
	 * the highest servo update rate configured.
	 *
	 * We always mix at max rate; some channels may update slower.
	 */
	unsigned max_rate = 500;

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

		DEVICE_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				orb_set_interval(_control_subs[i], update_rate_in_ms);
			}
		}

		// set to current max rate, even if we are actually checking slower/faster
		_current_update_rate = max_rate;
	}

	/* check if anything updated */
	int ret = ::poll(_poll_fds, _poll_fds_num, 0);

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else if (ret == 0) {
		/* timeout: no control data, switch to failsafe values */
//			warnx("no PWM: failsafe");

	} else {

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] > 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		/* can we mix? */
		if (_mixers != nullptr) {

			size_t num_outputs = 4;

			/* do mixing */
			float outputs[_max_actuators];
			num_outputs = _mixers->mix(outputs, num_outputs, NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
				if (i >= num_outputs) {
					outputs[i] = NAN_VALUE;
				}
			}

			uint16_t pwm_limited[_max_actuators];

			/* the PWM limit call takes care of out of band errors, NaN and constrains */
			pwm_limit_calc(_servo_armed, arm_nothrottle(), num_outputs, _reverse_pwm_mask, _disarmed_pwm, _min_pwm, _max_pwm,
					   outputs, pwm_limited, &_pwm_limit);

			/* output to the servos */
			for (size_t i = 0; i < num_outputs; i++) {
				up_pwm_servo_set(i, pwm_limited[i]);
			}

			publish_pwm_outputs(pwm_limited, num_outputs);
		}
	}

	/* check arming state */
	bool updated = false;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		/* update the armed status and check that we're not locked down */
		bool set_armed = (_armed.armed || _armed.prearmed) && !_armed.lockdown;

		if (_servo_armed != set_armed) {
			_servo_armed = set_armed;
		}

		/* update PWM status if armed or if disarmed PWM values are set */
		bool pwm_on = (set_armed || _num_disarmed_set > 0);

		if (_pwm_on != pwm_on) {
			_pwm_on = pwm_on;
			up_pwm_servo_arm(pwm_on);
		}
	}

	orb_check(_param_sub, &updated);

	if (updated) {
		parameter_update_s pupdate;
		orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);
	}

	work_queue(HPWORK, &_work, (worker_t)&Crazyflie::cycle_trampoline, this, USEC2TICK(CONTROL_INPUT_DROP_LIMIT_MS * 1000));
}

void Crazyflie::work_stop()
{
	work_cancel(HPWORK, &_work);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			::close(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	::close(_armed_sub);
	::close(_param_sub);

	/* make sure servos are off */
	up_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

int
Crazyflie::control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* motor spinup phase - lock throttle to zero */
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle()) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

void
Crazyflie::set_bounded_pwm(uint16_t &out, struct pwm_output_values *pwm, size_t i)
{
	if (pwm->values[i] > 255) {
		out = 255;

	} else {
		out = pwm->values[i];
	}
}

int
Crazyflie::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = 500;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		// these are no-ops, as we have a fixed update rate
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = (1 << 4) - 1;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				set_bounded_pwm(_failsafe_pwm[i], pwm, i);
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_failsafe_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_failsafe_pwm[i] > 0) {
					_num_failsafe_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _failsafe_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				set_bounded_pwm(_disarmed_pwm[i], pwm, i);
			}

			/*
			 * update the counter
			 * this is needed to decide if disarmed PWM output should be turned on or not
			 */
			_num_disarmed_set = 0;

			for (unsigned i = 0; i < _max_actuators; i++) {
				if (_disarmed_pwm[i] > 0) {
					_num_disarmed_set++;
				}
			}

			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _disarmed_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				set_bounded_pwm(_min_pwm[i], pwm, i);
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _min_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				set_bounded_pwm(_max_pwm[i], pwm, i);
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _max_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET(3):
	case PWM_SERVO_SET(2):
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 255) {
			up_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
		*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = 4;
		break;

	case PWM_SERVO_SET_COUNT:
		ret = -EINVAL;

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
								 (uintptr_t)_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)_controls);

				_mixers->add_mixer(mixer);
				_mixers->groups_required(_groups_required);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback, (uintptr_t)_controls);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
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

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
Crazyflie::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[4];

	if (count > 4) {
		// we have at most 4 outputs
		count = 4;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
}

namespace
{
int crazyflie_start(void);
int crazyflie_stop(void);
void test(void);
void fake(int argc, char *argv[]);

int
crazyflie_start(void)
{
	int ret = OK;

	if (g_crazyflie == nullptr) {

		g_crazyflie = new Crazyflie;

		if (g_crazyflie == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_crazyflie->init();

			if (ret != OK) {
				delete g_crazyflie;
				g_crazyflie = nullptr;
			}
		}
	}

	return ret;
}

int
crazyflie_stop(void)
{
	int ret = OK;

	if (g_crazyflie != nullptr) {

		delete g_crazyflie;
		g_crazyflie = nullptr;
	}

	return ret;
}

void
test(void)
{
	int	 fd;
	unsigned servo_count = 0;
	unsigned pwm_value = 1;
	int	 direction = 1;
	int	 ret;

	fd = open(CRAZYFLIE_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		errx(1, "open fail");
	}

	if (ioctl(fd, PWM_SERVO_ARM, 0) < 0) { err(1, "servo arm failed"); }

	if (ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) != 0) {
		err(1, "Unable to get servo count\n");
	}

	warnx("Testing %u motors", (unsigned)servo_count);

	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	for (;;) {
		/* sweep all servos between 0..20 */
		servo_position_t servos[servo_count];

		for (unsigned i = 0; i < servo_count; i++) {
			servos[i] = pwm_value;
		}

		if (direction == 1) {
			// use ioctl interface for one direction
			for (unsigned i = 0; i < servo_count;	i++) {
				if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
					err(1, "servo %u set failed", i);
				}
			}

		} else {
			// and use write interface for the other direction
			ret = write(fd, servos, sizeof(servos));

			if (ret != (int)sizeof(servos)) {
				err(1, "error writing PWM servo data, wrote %u got %d", sizeof(servos), ret);
			}
		}

		if (direction > 0) {
			if (pwm_value < 20) {
				pwm_value++;

			} else {
				direction = -1;
			}

		} else {
			if (pwm_value > 1) {
				pwm_value--;

			} else {
				direction = 1;
			}
		}

		/* readback servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t value;

			if (ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&value)) {
				err(1, "error reading PWM servo %d", i);
			}

			if (value != servos[i]) {
				errx(1, "servo %d readback error, got %u expected %u", i, value, servos[i]);
			}
		}

		/* Check if user wants to quit */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {

			read(0, &c, 1);

			if (c == 0x03 || c == 0x63 || c == 'q') {
				warnx("User abort\n");
				break;
			}
		}

		usleep(50000);
	}

	/* zero motors */
	servo_position_t servos[servo_count];

	for (unsigned i = 0; i < servo_count; i++) {
		servos[i] = 0;
	}

	for (unsigned i = 0; i < servo_count;	i++) {
		if (ioctl(fd, PWM_SERVO_SET(i), servos[i]) < 0) {
			err(1, "servo %u set failed", i);
		}
	}

	close(fd);

	exit(0);
}

void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "crazyflie fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
	}

	actuator_controls_s ac;

	ac.control[0] = strtol(argv[1], 0, 0) / 100.0f;

	ac.control[1] = strtol(argv[2], 0, 0) / 100.0f;

	ac.control[2] = strtol(argv[3], 0, 0) / 100.0f;

	ac.control[3] = strtol(argv[4], 0, 0) / 100.0f;

	orb_advert_t handle = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &ac);

	if (handle == nullptr) {
		errx(1, "advertise failed");
	}

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	exit(0);
}

} // namespace

extern "C" __EXPORT int crazyflie_main(int argc, char *argv[]);

int
crazyflie_main(int argc, char *argv[])
{
	const char *verb = argv[1];

	if (!strcmp(verb, "stop")) {
		crazyflie_stop();
		errx(0, "Crazyflie driver stopped");
	}

	if (crazyflie_start() != OK) {
		errx(1, "failed to start the Crazyflie driver");
	}

	if (!strcmp(verb, "test")) {
		test();
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
	}

	fprintf(stderr, "Crazyflie: unrecognised command %s, try:\n", verb);
	fprintf(stderr, "  test, fake\n");
	exit(1);
}
