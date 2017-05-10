/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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
 * Driver/configurator for the PX4 FMU multi-purpose port on v1 and v2 boards.
 */

#include <px4_config.h>
#include <px4_log.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <px4_workqueue.h>

#include <drivers/device/device.h>
// #include <drivers/device/i2c.h>
#include <drivers/drv_ext_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>

#include <board_config.h>

#include <systemlib/px4_macros.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <drivers/drv_mixer.h>


#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>


#include <systemlib/circuit_breaker.h>

#define SCHEDULE_INTERVAL	2000	/**< The schedule interval in usec (500 Hz) */
#define NAN_VALUE	(0.0f/0.0f)		/**< NaN value for throttle lock mode */
// #define BUTTON_SAFETY	px4_arch_gpioread(GPIO_BTN_SAFETY)
#define CYCLE_COUNT 10			/* safety switch must be held for 1 second to activate */

/*
 * Define the various LED flash sequences for each system state.
 */
// #define LED_PATTERN_FMU_OK_TO_ARM 		0x0003		/**< slow blinking			*/
// #define LED_PATTERN_FMU_REFUSE_TO_ARM 	0x5555		/**< fast blinking			*/
// #define LED_PATTERN_IO_ARMED 			0x5050		*< long off, then double blink
// #define LED_PATTERN_FMU_ARMED 			0x5500		/**< long off, then quad blink 		*/
// #define LED_PATTERN_IO_FMU_ARMED 		0xffff		/**< constantly on			*/

#ifndef BOARD_HAS_EXT_PWM
#  error "board_config.h needs to define BOARD_HAS_EXT_PWM"
#endif
class PX4PWM : public device::CDev
{
public:
	enum Mode {
		MODE_NONE,
		MODE_1PWM,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_6PWM,
		MODE_8PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	PX4PWM(bool run_as_task);
	virtual ~PX4PWM();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	int		set_pwm_alt_rate(unsigned rate);
	int		set_pwm_alt_channels(uint32_t channels);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	void update_pwm_trims();

private:


	hrt_abstime _cycle_timestamp = 0;
	hrt_abstime _time_last_mix = 0;

	static const unsigned _max_actuators = DIRECT_EXT_PWM_OUTPUT_CHANNELS;

	Mode		_mode;
	unsigned	_pwm_default_rate;
	unsigned	_pwm_alt_rate;
	uint32_t	_pwm_alt_rate_channels;
	unsigned	_current_update_rate;
	int 		_task;
	bool 		_run_as_task;
	struct work_s	_work;
	int		_vehicle_cmd_sub;
	int		_armed_sub;
	int		_param_sub;

	orb_advert_t	_outputs_pub;
	unsigned	_num_outputs;
	int		_class_instance;
	volatile bool	_should_exit;

	static void	task_main_trampoline(int argc, char *argv[]);

	volatile bool	_initialized;
	bool		_throttle_armed;
	bool		_pwm_on;
	uint32_t	_pwm_mask;
	bool		_pwm_initialized;

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
	uint16_t	_trim_pwm[_max_actuators];
	uint16_t	_reverse_pwm_mask;
	unsigned	_num_failsafe_set;
	unsigned	_num_disarmed_set;

	orb_advert_t      _to_mixer_status; 	///< mixer status flags

	float _mot_t_max;	// maximum rise time for motor (slew rate limiting)
	float _thr_mdl_fac;	// thrust to pwm modelling factor

	perf_counter_t	_ctl_latency;

	static bool	arm_nothrottle()
	{
		return ((_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode);
	}

	static void	cycle_trampoline(void *arg);
	void		cycle();
	int 		start();
	void		stop();

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	void		subscribe();
	int		set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void		publish_pwm_outputs(uint16_t *values, size_t numvalues);
	void		update_pwm_out_state(bool on);
	void		pwm_output_set(unsigned i, unsigned value);
	void		update_params();

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	/* do not allow to copy due to ptr data members */
	PX4PWM(const PX4PWM &);
	PX4PWM operator=(const PX4PWM &);

};

pwm_limit_t		PX4PWM::_pwm_limit;
actuator_armed_s	PX4PWM::_armed = {};

namespace
{

PX4PWM	*g_fmu;

} // namespace

/*/dev/px4pwm*/
PX4PWM::PX4PWM(bool run_as_task) :
	CDev("fmu", "/dev/px4pwm"),
	_mode(MODE_NONE),
	_pwm_default_rate(50),
	_pwm_alt_rate(50),
	_pwm_alt_rate_channels(0),
	_current_update_rate(0),
	_task(-1),
	_run_as_task(run_as_task),
	_work{},
	_vehicle_cmd_sub(-1),
	_armed_sub(-1),
	_param_sub(-1),
	_outputs_pub(nullptr),
	_num_outputs(0),
	_class_instance(0),
	_should_exit(false),
	_initialized(false),
	_throttle_armed(false),
	_pwm_on(false),
	_pwm_mask(0),
	_pwm_initialized(false),
	_mixers(nullptr),
	_groups_required(0),
	_groups_subscribed(0),
	_control_subs{ -1},
	_poll_fds_num(0),
	_failsafe_pwm{0},
	_disarmed_pwm{0},
	_reverse_pwm_mask(0),
	_num_failsafe_set(0),
	_num_disarmed_set(0),
	_to_mixer_status(nullptr),
	_mot_t_max(0.0f),
	_thr_mdl_fac(0.0f),
	_ctl_latency(perf_alloc(PC_ELAPSED, "ctl_lat"))
{
	for (unsigned i = 0; i < _max_actuators; i++) {
		_min_pwm[i] = PWM_DEFAULT_MIN;
		_max_pwm[i] = PWM_DEFAULT_MAX;
		_trim_pwm[i] = PWM_DEFAULT_TRIM;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	// Safely initialize armed flags.
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	/* only enable this during development */
	_debug_enabled = false;
}

PX4PWM::~PX4PWM()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		stop();

		int i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);
			i--;

		} while (_initialized && i > 0);
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_ctl_latency);

	g_fmu = nullptr;
}

int
PX4PWM::init()
{
	int ret;

	ASSERT(!_initialized);

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	// XXX best would be to register / de-register the device depending on modes

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}


	return start();
}


int
PX4PWM::set_mode(Mode mode)
{
	unsigned old_mask = _pwm_mask;

	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_1PWM:
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x1;
		_pwm_initialized = false;
		_num_outputs = 1;
		break;

	case MODE_2PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_2PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3;
		_pwm_initialized = false;
		_num_outputs = 2;

		break;

	// no break
	case MODE_3PWM:	// v1 multi-port with flow control lines as PWM
		DEVICE_DEBUG("MODE_3PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x7;
		_pwm_initialized = false;
		_num_outputs = 3;

		break;

	case MODE_4PWM: // v1 or v2 multi-port as 4 PWM outs
		DEVICE_DEBUG("MODE_4PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0xf;
		_pwm_initialized = false;
		_num_outputs = 4;

		break;

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6

	case MODE_6PWM:
		DEVICE_DEBUG("MODE_6PWM");

		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x3f;
		_pwm_initialized = false;
		_num_outputs = 6;

		break;
#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8

	case MODE_8PWM: // AeroCore PWMs as 8 PWM outs
		DEVICE_DEBUG("MODE_8PWM");
		/* default output rates */
		_pwm_default_rate = 50;
		_pwm_alt_rate = 50;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0xff;
		_pwm_initialized = false;
		_num_outputs = 8;

		break;
#endif

	case MODE_NONE:
		DEVICE_DEBUG("MODE_NONE");

		_pwm_default_rate = 10;	/* artificially reduced output rate */
		_pwm_alt_rate = 10;
		_pwm_alt_rate_channels = 0;
		_pwm_mask = 0x0;
		_pwm_initialized = false;
		_num_outputs = 0;

		if (old_mask != _pwm_mask) {
			/* disable servo outputs - no need to set rates */
			up_ext_pwm_servo_deinit();
		}

		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

/* When set_pwm_rate is called from either of the 2 IOCTLs:
 *
 * PWM_SERVO_SET_UPDATE_RATE        - Sets the "alternate" channel's rate to the callers's rate specified
 *                                    and the non "alternate" channels to the _pwm_default_rate.
 *
 *                                    rate_map     = _pwm_alt_rate_channels
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = arg of IOCTL (see rates)
 *
 * PWM_SERVO_SET_SELECT_UPDATE_RATE - The caller's specified rate map selects the "alternate" channels
 *                                    to be set to the alt rate. (_pwm_alt_rate)
 *                                    All other channels are set to the default rate. (_pwm_default_rate)
 *
 *                                    rate_map     = arg of IOCTL
 *                                    default_rate = _pwm_default_rate
 *                                    alt_rate     = _pwm_alt_rate

 *  rate_map                        - A mask of 1's for the channels to be set to the
 *                                    alternate rate.
 *                                    N.B. All channels is a given group must be set
 *                                    to the same rate/mode. (default or alt)
 * rates:
 *   alt_rate, default_rate           For PWM is 25 or 400Hz
 *                                    For Oneshot there is no rate, 0 is therefore used
 *                                    to  select Oneshot mode
 */
int
PX4PWM::set_pwm_rate(uint32_t rate_map, unsigned default_rate, unsigned alt_rate)
{
	PX4_DEBUG("set_pwm_rate %x %u %u", rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {

		/* We should note that group is iterated over from 0 to _max_actuators.
		 * This allows for the ideal worlds situation: 1 channel per group
		 * configuration.
		 *
		 * This is typically not what HW supports. A group represents a timer
		 * and channels belongs to a timer.
		 * Therefore all channels in a group are dependent on the timer's
		 * common settings and can not be independent in terms of count frequency
		 * (granularity of pulse width) and rate (period of repetition).
		 *
		 * To say it another way, all channels in a group moust have the same
		 * rate and mode. (See rates above.)
		 */

		for (unsigned group = 0; group < _max_actuators; group++) {

			// get the channel mask for this rate group
			uint32_t mask = up_ext_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					PX4_WARN("rate group %u mask %x bad overlap %x", group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_ext_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
						PX4_WARN("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_ext_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
						PX4_WARN("rate group set default failed");
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
PX4PWM::set_pwm_alt_rate(unsigned rate)
{
	return set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, rate);
}

int
PX4PWM::set_pwm_alt_channels(uint32_t channels)
{
	return set_pwm_rate(channels, _pwm_default_rate, _pwm_alt_rate);
}

void
PX4PWM::subscribe()
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
			orb_unsubscribe(_control_subs[i]);
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
PX4PWM::update_pwm_rev_mask()
{
	_reverse_pwm_mask = 0;

	for (unsigned i = 0; i < _max_actuators; i++) {
		char pname[16];
		int32_t ival;

		/* fill the channel reverse mask from parameters */
		sprintf(pname, "PWM_MAIN_REV%d", i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			param_get(param_h, &ival);
			_reverse_pwm_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}

void
PX4PWM::update_pwm_trims()
{
	PX4_DEBUG("update_pwm_trims");

	if (_mixers != nullptr) {

		int16_t values[_max_actuators] = {};

		for (unsigned i = 0; i < _max_actuators; i++) {
			char pname[16];
			float pval;

			/* fill the struct from parameters */
			sprintf(pname, "PWM_MAIN_TRIM%d", i + 1);
			param_t param_h = param_find(pname);

			if (param_h != PARAM_INVALID) {
				param_get(param_h, &pval);
				values[i] = (int16_t)(10000 * pval);
				PX4_DEBUG("%s: %d", pname, values[i]);
			}
		}

		/* copy the trim values to the mixer offsets */
		unsigned n_out = _mixers->set_trims(values, _max_actuators);
		PX4_DEBUG("set %d trims", n_out);
	}
}

void
PX4PWM::publish_pwm_outputs(uint16_t *values, size_t numvalues)
{
	actuator_outputs_s outputs = {};
	outputs.noutputs = numvalues;
	outputs.timestamp = hrt_absolute_time();

	for (size_t i = 0; i < _max_actuators; ++i) {
		outputs.output[i] = i < numvalues ? (float)values[i] : 0;
	}

	if (_outputs_pub == nullptr) {
		int instance = _class_instance;
		_outputs_pub = orb_advertise_multi(ORB_ID(actuator_outputs), &outputs, &instance, ORB_PRIO_DEFAULT);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &outputs);
	}
}


int
PX4PWM::start()
{

	if (!_run_as_task) {

		/* schedule a cycle to start things */

		work_queue(HPWORK, &_work, (worker_t)&PX4PWM::cycle_trampoline, this, 0);

	} else {

		/* start the IO interface task */

		_task = px4_task_spawn_cmd("fmuservo",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_FAST_DRIVER - 1,
					   1280,
					   (main_t)&PX4PWM::task_main_trampoline,
					   nullptr);

		/* wait until the task is up and running or has failed */
		while (_task > 0 && _should_exit) {
			usleep(100);
		}

		if (_task < 0) {
			return -PX4_ERROR;
		}
	}

	return PX4_OK;
}

void
PX4PWM::cycle_trampoline(void *arg)
{
	PX4PWM *dev = reinterpret_cast<PX4PWM *>(arg);

	dev->cycle();
}

void
PX4PWM::task_main_trampoline(int argc, char *argv[])
{
	cycle_trampoline(g_fmu);
}


void
PX4PWM::pwm_output_set(unsigned i, unsigned value)
{
	if (_pwm_initialized) {
		up_ext_pwm_servo_set(i, value);
	}
}

void
PX4PWM::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {
		up_ext_pwm_servo_init(_pwm_mask);
		set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, _pwm_alt_rate);
		_pwm_initialized = true;
	}

	up_ext_pwm_servo_arm(on);
}

void
PX4PWM::cycle()
{
	while (!_should_exit) {

		if (!_initialized) {
			/* force a reset of the update rate */
			_current_update_rate = 0;

			_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
			_param_sub = orb_subscribe(ORB_ID(parameter_update));

			/* initialize PWM limit lib */
			pwm_limit_init(&_pwm_limit);

			update_pwm_rev_mask();

			// Getting initial parameter values
			this->update_params();

			for (unsigned i = 0; i < _max_actuators; i++) {
				char pname[16];
				sprintf(pname, "PWM_MAIN_TRIM%d", i + 1);
				param_find(pname);
			}

			_initialized = true;
		}

		if (_groups_subscribed != _groups_required) {
			subscribe();
			_groups_subscribed = _groups_required;
			/* force setting update rate */
			_current_update_rate = 0;
		}

		int poll_timeout = 20;

		if (!_run_as_task) {
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

				PX4_DEBUG("adjusted actuator update interval to %ums", update_rate_in_ms);

				for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
					if (_control_subs[i] > 0) {
						orb_set_interval(_control_subs[i], update_rate_in_ms);
					}
				}

				// set to current max rate, even if we are actually checking slower/faster
				_current_update_rate = max_rate;
			}

			/* check if anything updated */
			poll_timeout = 0;
		}

		/* wait for an update */
		unsigned n_updates = 0;
		int ret = ::poll(_poll_fds, _poll_fds_num, poll_timeout);

		/* this would be bad... */
		if (ret < 0) {
			DEVICE_LOG("poll error %d", errno);

		} else if (ret == 0) {
			/* timeout: no control data, switch to failsafe values */
			//			PX4_WARN("no PWM: failsafe");

		} else {
			perf_begin(_ctl_latency);

			/* get controls for required topics */
			unsigned poll_id = 0;

			for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				if (_control_subs[i] > 0) {
					if (_poll_fds[poll_id].revents & POLLIN) {
						if (i == 0) {
							n_updates++;
						}

						orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

#if defined(DEBUG_BUILD)

						static int main_out_latency = 0;
						static int sum_latency = 0;
						static uint64_t last_cycle_time = 0;

						if (i == 0) {
							uint64_t now = hrt_absolute_time();
							uint64_t latency = now - _controls[i].timestamp;

							if (latency > main_out_latency) { main_out_latency = latency; }

							sum_latency += latency;

							if ((now - last_cycle_time) >= 1000000) {
								last_cycle_time = now;
								PX4_DEBUG("pwm max latency: %d, avg: %5.3f", main_out_latency, (double)(sum_latency / 100.0));
								main_out_latency = latency;
								sum_latency = 0;
							}
						}

#endif
					}

					poll_id++;
				}

				/* During ESC calibration, we overwrite the throttle value. */
				if (i == 0 && _armed.in_esc_calibration_mode) {

					/* Set all controls to 0 */
					memset(&_controls[i], 0, sizeof(_controls[i]));

					/* except thrust to maximum. */
					_controls[i].control[3] = 1.0f;

					/* Switch off the PWM limit ramp for the calibration. */
					_pwm_limit.state = PWM_LIMIT_STATE_ON;
				}
			}
		} // poll_fds

		/* run the mixers on every cycle */
		{
			/* can we mix? */
			if (_mixers != nullptr) {

				hrt_abstime now = hrt_absolute_time();
				float dt = (now - _time_last_mix) / 1e6f;
				_time_last_mix = now;

				if (dt < 0.0001f) {
					dt = 0.0001f;

				} else if (dt > 0.02f) {
					dt = 0.02f;
				}

				if (_mot_t_max > FLT_EPSILON) {
					// maximum value the ouputs of the multirotor mixer are allowed to change in this cycle
					// factor 2 is needed because actuator ouputs are in the range [-1,1]
					float delta_out_max = 2.0f * 1000.0f * dt / (_max_pwm[0] - _min_pwm[0]) / _mot_t_max;
					_mixers->set_max_delta_out_once(delta_out_max);
				}

				if (_thr_mdl_fac > FLT_EPSILON) {
					_mixers->set_thrust_factor(_thr_mdl_fac);
				}

				/* do mixing */
				float outputs[_max_actuators];
				size_t mixed_num_outputs = _mixers->mix(outputs, _num_outputs, NULL);

				/* publish mixer status */
				multirotor_motor_limits_s multirotor_motor_limits = {};
				multirotor_motor_limits.saturation_status = _mixers->get_saturation_status();

				if (_to_mixer_status == nullptr) {
					/* publish limits */
					int instance = _class_instance;
					_to_mixer_status = orb_advertise_multi(ORB_ID(multirotor_motor_limits), &multirotor_motor_limits, &instance,
									       ORB_PRIO_DEFAULT);

				} else {
					orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &multirotor_motor_limits);

				}

				/* disable unused ports by setting their output to NaN */
				for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); i++) {
					if (i >= mixed_num_outputs) {
						outputs[i] = NAN_VALUE;
					}
				}

				uint16_t pwm_limited[_max_actuators];

				/* the PWM limit call takes care of out of band errors, NaN and constrains */
				pwm_limit_calc(_throttle_armed, arm_nothrottle(), mixed_num_outputs, _reverse_pwm_mask,
					       _disarmed_pwm, _min_pwm, _max_pwm, outputs, pwm_limited, &_pwm_limit);


				/* overwrite outputs in case of force_failsafe with _failsafe_pwm PWM values */
				if (_armed.force_failsafe) {
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						pwm_limited[i] = _failsafe_pwm[i];
					}
				}

				/* overwrite outputs in case of lockdown with disarmed PWM values */
				if (_armed.lockdown || _armed.manual_lockdown) {
					for (size_t i = 0; i < mixed_num_outputs; i++) {
						pwm_limited[i] = _disarmed_pwm[i];
					}
				}

				/* output to the servos */
				for (size_t i = 0; i < mixed_num_outputs; i++) {
					pwm_output_set(i, pwm_limited[i]);
				}

				/* Trigger all timer's channels in Oneshot mode to fire
				 * the oneshots with updated values.
				 */

				if (n_updates > 0) {
					up_ext_pwm_update();
				}

				publish_pwm_outputs(pwm_limited, mixed_num_outputs);
				perf_end(_ctl_latency);
			}
		}

		_cycle_timestamp = hrt_absolute_time();

		/* check arming state */
		bool updated = false;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

			/* Update the armed status and check that we're not locked down.
			 * We also need to arm throttle for the ESC calibration. */
			_throttle_armed = (_armed.armed && !_armed.lockdown) ||  _armed.in_esc_calibration_mode;


			/* update PWM status if armed or if disarmed PWM values are set */
			bool pwm_on = _armed.armed || _num_disarmed_set > 0 || _armed.in_esc_calibration_mode;

			if (_pwm_on != pwm_on) {
				_pwm_on = pwm_on;

				update_pwm_out_state(pwm_on);
			}
		}


		orb_check(_param_sub, &updated);

		if (updated) {
			this->update_params();
		}

		if (!_run_as_task && !_should_exit) {

			/*
			 * schedule next cycle
			 */

			work_queue(HPWORK, &_work, (worker_t)&PX4PWM::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
			//																			  USEC2TICK(SCHEDULE_INTERVAL - main_out_latency));
			/* Running a worker. So exit the loop */
			break;
		}
	}
}

void PX4PWM::update_params()
{
	parameter_update_s pupdate;
	orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);

	update_pwm_rev_mask();
	update_pwm_trims();

	param_t param_handle;

	/* see if bind parameter has been set, and reset it to -1 */

	// maximum motor slew rate parameter
	param_handle = param_find("MOT_SLEW_MAX");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_mot_t_max);
	}

	// thrust to pwm modelling factor
	param_handle = param_find("THR_MDL_FAC");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_thr_mdl_fac);
	}
}


void PX4PWM::stop()
{
	/* Signal that we want to stop the task or work.
	 *
	 * In the case of work we do not want to reschedule
	 * to avoid race on cancel
	 */

	_should_exit = true;

	if (!_run_as_task) {
		work_cancel(HPWORK, &_work);
	}

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] > 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_param_sub);

	orb_unadvertise(_outputs_pub);
	orb_unadvertise(_to_mixer_status);

	/* make sure servos are off */
	up_ext_pwm_servo_deinit();

	DEVICE_LOG("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_initialized = false;
}

int
PX4PWM::control_callback(uintptr_t handle,
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
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (arm_nothrottle() && !_armed.in_esc_calibration_mode) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN_VALUE;
		}
	}

	return 0;
}

int
PX4PWM::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = -1;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	switch (_mode) {
	case MODE_1PWM:
	case MODE_2PWM:
	case MODE_3PWM:
	case MODE_4PWM:
	case MODE_2PWM2CAP:
	case MODE_3PWM1CAP:
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6
	case MODE_6PWM:
#endif
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8
	case MODE_8PWM:
#endif
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		PX4_DEBUG("not in a PWM mode");
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
PX4PWM::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("fmu ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		update_pwm_out_state(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		break;

	case PWM_SERVO_DISARM:

		/* Ignore disarm if disarmed PWM is set already. */
		if (_num_disarmed_set == 0) {
			update_pwm_out_state(false);
		}

		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_default_rate;
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = set_pwm_rate(_pwm_alt_rate_channels, _pwm_default_rate, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = set_pwm_rate(arg, _pwm_default_rate, _pwm_alt_rate);
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = _pwm_alt_rate_channels;
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_failsafe_pwm[i] = PWM_HIGHEST_MAX;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_failsafe_pwm[i] = PWM_LOWEST_MIN;

				}

#endif

				else {
					_failsafe_pwm[i] = pwm->values[i];
				}
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
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_disarmed_pwm[i] = PWM_HIGHEST_MAX;
				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_disarmed_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_disarmed_pwm[i] = pwm->values[i];
				}
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
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_min_pwm[i] = PWM_HIGHEST_MIN;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_min_pwm[i] = PWM_LOWEST_MIN;
				}

#endif

				else {
					_min_pwm[i] = pwm->values[i];
				}
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
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_max_pwm[i] = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_max_pwm[i] = PWM_HIGHEST_MAX;

				} else {
					_max_pwm[i] = pwm->values[i];
				}
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

	case PWM_SERVO_SET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > _max_actuators) {
				PX4_DEBUG("error: too many trim values: %d", pwm->channel_count);
				ret = -EINVAL;
				break;
			}

			/* copy the trim values to the mixer offsets */
			_mixers->set_trims((int16_t *)pwm->values, pwm->channel_count);
			PX4_DEBUG("set_trims: %d, %d, %d, %d", pwm->values[0], pwm->values[1], pwm->values[2], pwm->values[3]);

			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _max_actuators; i++) {
				pwm->values[i] = _trim_pwm[i];
			}

			pwm->channel_count = _max_actuators;
			arg = (unsigned long)&pwm;
			break;
		}

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8

	case PWM_SERVO_SET(7):
	case PWM_SERVO_SET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6

	case PWM_SERVO_SET(5):
	case PWM_SERVO_SET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_SET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_SET(1):
	case PWM_SERVO_SET(0):
		if (arg <= 2100) {
			up_ext_pwm_servo_set(cmd - PWM_SERVO_SET(0), arg);

		} else {
			ret = -EINVAL;
		}

		break;

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8

	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
		if (_mode < MODE_8PWM) {
			ret = -EINVAL;
			break;
		}

#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6

	case PWM_SERVO_GET(5):
	case PWM_SERVO_GET(4):
		if (_mode < MODE_6PWM) {
			ret = -EINVAL;
			break;
		}

#endif

	/* FALLTHROUGH */
	case PWM_SERVO_GET(3):
		if (_mode < MODE_4PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(2):
		if (_mode < MODE_3PWM) {
			ret = -EINVAL;
			break;
		}

	/* FALLTHROUGH */
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		*(servo_position_t *)arg = up_ext_pwm_servo_get(cmd - PWM_SERVO_GET(0));
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6
	case PWM_SERVO_GET_RATEGROUP(4):
	case PWM_SERVO_GET_RATEGROUP(5):
#endif
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
#endif
		*(uint32_t *)arg = up_ext_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		switch (_mode) {

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 8

		case MODE_8PWM:
			*(unsigned *)arg = 8;
			break;
#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6

		case MODE_6PWM:
			*(unsigned *)arg = 6;
			break;
#endif

		case MODE_4PWM:
			*(unsigned *)arg = 4;
			break;

		case MODE_3PWM:
		case MODE_3PWM1CAP:
			*(unsigned *)arg = 3;
			break;

		case MODE_2PWM:
		case MODE_2PWM2CAP:
			*(unsigned *)arg = 2;
			break;

		case MODE_1PWM:
			*(unsigned *)arg = 1;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters. Note that this does not allow for
			 * changing a set of pins to be used for serial on
			 * FMUv1
			 */
			switch (arg) {
			case 0:
				set_mode(MODE_NONE);
				break;

			case 1:
				set_mode(MODE_1PWM);
				break;

			case 2:
				set_mode(MODE_2PWM);
				break;

			case 3:
				set_mode(MODE_3PWM);
				break;

			case 4:
				set_mode(MODE_4PWM);
				break;

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >=6

			case 6:
				set_mode(MODE_6PWM);
				break;
#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >=8

			case 8:
				set_mode(MODE_8PWM);
				break;
#endif

			default:
				ret = -EINVAL;
				break;
			}

			break;
		}

	case PWM_SERVO_SET_MODE: {
			switch (arg) {
			case PWM_SERVO_MODE_NONE:
				ret = set_mode(MODE_NONE);
				break;

			case PWM_SERVO_MODE_1PWM:
				ret = set_mode(MODE_1PWM);
				break;

			case PWM_SERVO_MODE_2PWM:
				ret = set_mode(MODE_2PWM);
				break;

			case PWM_SERVO_MODE_2PWM2CAP:
				ret = set_mode(MODE_2PWM2CAP);
				break;

			case PWM_SERVO_MODE_3PWM:
				ret = set_mode(MODE_3PWM);
				break;

			case PWM_SERVO_MODE_3PWM1CAP:
				ret = set_mode(MODE_3PWM1CAP);
				break;

			case PWM_SERVO_MODE_4PWM:
				ret = set_mode(MODE_4PWM);
				break;

			case PWM_SERVO_MODE_6PWM:
				ret = set_mode(MODE_6PWM);
				break;

			case PWM_SERVO_MODE_8PWM:
				ret = set_mode(MODE_8PWM);
				break;

			case PWM_SERVO_MODE_4CAP:
				ret = set_mode(MODE_4CAP);
				break;

			case PWM_SERVO_MODE_5CAP:
				ret = set_mode(MODE_5CAP);
				break;

			case PWM_SERVO_MODE_6CAP:
				ret = set_mode(MODE_6CAP);
				break;

			default:
				ret = -EINVAL;
			}

			break;
		}

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
					PX4_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
					PX4_DEBUG("loaded mixers \n%s\n", buf);
					update_pwm_trims();
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
PX4PWM::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[8];

#if BOARD_HAS_EXT_PWM == 0
	return 0;
#endif

	if (count > BOARD_HAS_EXT_PWM) {
		// we have at most BOARD_HAS_EXT_PWM outputs
		count = BOARD_HAS_EXT_PWM;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		if (values[i] != PWM_IGNORE_THIS_CHANNEL) {
			up_ext_pwm_servo_set(i, values[i]);
		}
	}

	return count * 2;
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
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM1,
	PORT_PWM3CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

PortMode g_port_mode;

int
fmu_new_mode(PortMode new_mode)
{
	uint32_t gpio_bits;
	PX4PWM::Mode servo_mode;
	bool mode_with_input = false;

	gpio_bits = 0;
	servo_mode = PX4PWM::MODE_NONE;

	switch (new_mode) {
	case PORT_FULL_GPIO:
	case PORT_MODE_UNSET:
		break;

	case PORT_FULL_PWM:

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM == 4
		/* select 4-pin PWM mode */
		servo_mode = PX4PWM::MODE_4PWM;
#endif
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM == 6
		servo_mode = PX4PWM::MODE_6PWM;
#endif
#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM == 8
		servo_mode = PX4PWM::MODE_8PWM;
#endif
		break;

	case PORT_PWM1:
		/* select 2-pin PWM mode */
		servo_mode = PX4PWM::MODE_1PWM;
		break;

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6

	case PORT_PWM4:
		/* select 4-pin PWM mode */
		servo_mode = PX4PWM::MODE_4PWM;
		break;

	case PORT_PWM3:
		/* select 3-pin PWM mode */
		servo_mode = PX4PWM::MODE_3PWM;
		break;

	case PORT_PWM3CAP1:
		/* select 3-pin PWM mode 1 capture */
		servo_mode = PX4PWM::MODE_3PWM1CAP;
		mode_with_input = true;
		break;

	case PORT_PWM2:
		/* select 2-pin PWM mode */
		servo_mode = PX4PWM::MODE_2PWM;
		break;

	case PORT_PWM2CAP2:
		/* select 2-pin PWM mode 2 capture */
		servo_mode = PX4PWM::MODE_2PWM2CAP;
		mode_with_input = true;
		break;
#endif

	default:
		return -1;
	}

	if (servo_mode != g_fmu->get_mode()) {

		/* reset to all-inputs */
		if (mode_with_input) {
			g_fmu->ioctl(0, GPIO_RESET, 0);

			/* adjust GPIO config for serial mode(s) */
			if (gpio_bits != 0) {
				g_fmu->ioctl(0, GPIO_SET_ALT_1, gpio_bits);
			}
		}

		/* (re)set the PWM output mode */
		g_fmu->set_mode(servo_mode);
	}

	return OK;
}


int
fmu_start(bool run_as_task)
{
	int ret = OK;

	if (g_fmu == nullptr) {

		g_fmu = new PX4PWM(run_as_task);

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

int
fmu_stop(void)
{
	int ret = OK;

	if (g_fmu != nullptr) {

		delete g_fmu;
		g_fmu = nullptr;
	}

	return ret;
}




void
fake(int argc, char *argv[])
{
	if (argc < 5) {
		errx(1, "fmu fake <roll> <pitch> <yaw> <thrust> (values -100 .. 100)");
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

	orb_unadvertise(handle);

	actuator_armed_s aa;

	aa.armed = true;
	aa.lockdown = false;

	handle = orb_advertise(ORB_ID(actuator_armed), &aa);

	if (handle == nullptr) {
		errx(1, "advertise failed 2");
	}

	orb_unadvertise(handle);

	exit(0);
}

} // namespace

extern "C" __EXPORT int px4pwm_main(int argc, char *argv[]);

int
px4pwm_main(int argc, char *argv[])
{
	bool run_as_task = false;
	PortMode new_mode = PORT_MODE_UNSET;
	const char *verb = argv[1];

	if (!strcmp(verb, "stop")) {
		fmu_stop();
		errx(0, "FMU driver stopped");
	}

	run_as_task = !strcmp(verb, "task");

	if (run_as_task && argc > 2) {
		verb = argv[2];
	}

	if (fmu_start(run_as_task) != OK) {
		errx(1, "failed to start the FMU driver");
	}

	/*
	 * Mode switches.
	 */
	if (!strcmp(verb, "mode_gpio")) {
		new_mode = PORT_FULL_GPIO;

	} else if (!strcmp(verb, "mode_rcin")) {
		exit(0);

	} else if (!strcmp(verb, "mode_pwm")) {
		new_mode = PORT_FULL_PWM;

#if defined(BOARD_HAS_EXT_PWM)

	} else if (!strcmp(verb, "mode_pwm1")) {
		new_mode = PORT_PWM1;
#endif

#if defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6


	} else if (!strcmp(verb, "mode_pwm4")) {
		new_mode = PORT_PWM4;

	} else if (!strcmp(verb, "mode_pwm2")) {
		new_mode = PORT_PWM2;

	} else if (!strcmp(verb, "mode_pwm3")) {
		new_mode = PORT_PWM3;

	} else if (!strcmp(verb, "mode_pwm3cap1")) {
		new_mode = PORT_PWM3CAP1;

	} else if (!strcmp(verb, "mode_pwm2cap2")) {
		new_mode = PORT_PWM2CAP2;
#endif
#if defined(BOARD_HAS_MULTI_PURPOSE_GPIO)

	} else if (!strcmp(verb, "mode_serial")) {
		new_mode = PORT_FULL_SERIAL;

	} else if (!strcmp(verb, "mode_gpio_serial")) {
		new_mode = PORT_GPIO_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_serial")) {
		new_mode = PORT_PWM_AND_SERIAL;

	} else if (!strcmp(verb, "mode_pwm_gpio")) {
		new_mode = PORT_PWM_AND_GPIO;
#endif
	}

	/* was a new mode set? */
	if (new_mode != PORT_MODE_UNSET) {

		/* yes but it's the same mode */
		if (new_mode == g_port_mode) {
			return OK;
		}

		/* switch modes */
		int ret = fmu_new_mode(new_mode);
		exit(ret == OK ? 0 : 1);
	}


	if (!strcmp(verb, "info")) {
		return 0;
	}

	if (!strcmp(verb, "fake")) {
		fake(argc - 1, argv + 1);
		return 0;
	}


	fprintf(stderr, "FMU: unrecognized command %s, try:\n", verb);
#if defined(RC_SERIAL_PORT)
	fprintf(stderr, " mode_rcin");
#endif
#if defined(BOARD_HAS_MULTI_PURPOSE_GPIO)
	fprintf(stderr,
		" , mode_gpio, mode_serial, mode_pwm, mode_gpio_serial, mode_pwm_serial, mode_pwm_gpio, test, fake, sensor_reset, id\n");
#elif defined(BOARD_HAS_EXT_PWM) && BOARD_HAS_EXT_PWM >= 6
	fprintf(stderr, "  mode_gpio, mode_pwm, mode_pwm4, test, sensor_reset [milliseconds], i2c <bus> <hz>, bind\n");
#endif
	fprintf(stderr, "\n");
	exit(1);
}
