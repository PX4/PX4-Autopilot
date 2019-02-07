/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include "PWMSim.hpp"

#include <px4_time.h>
#include <mathlib/mathlib.h>

#include <uORB/topics/multirotor_motor_limits.h>

PWMSim::PWMSim() :
	CDev(PWM_OUTPUT0_DEVICE_PATH),
	_perf_control_latency(perf_alloc(PC_ELAPSED, "pwm_out_sim control latency"))
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_pwm_min[i] = PWM_SIM_PWM_MIN_MAGIC;
		_pwm_max[i] = PWM_SIM_PWM_MAX_MAGIC;
	}

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		_control_subs[i] = -1;
	}

	CDev::init();

	// default to MODE_16PWM
	set_mode(MODE_16PWM);
}

PWMSim::~PWMSim()
{
	perf_free(_perf_control_latency);
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
	case MODE_8PWM:
		/* multi-port as 8 PWM outs */
		_update_rate = 400;	/* default output rate */
		_num_outputs = 8;
		break;

	case MODE_16PWM:
		/* multi-port as 16 PWM outs */
		_update_rate = 400;	/* default output rate */
		_num_outputs = 16;
		break;

	case MODE_NONE:
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

void PWMSim::update_params()
{
	// multicopter air-mode
	param_t param_handle = param_find("MC_AIRMODE");

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &_airmode);
	}
}

void
PWMSim::run()
{
	/* force a reset of the update rate */
	_current_update_rate = 0;

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuator_outputs);

	update_params();
	int params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* loop until killed */
	while (!should_exit()) {

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

		if (_mixers) {
			_mixers->set_airmode(_airmode);
		}

		/* this can happen during boot, but after the sleep its likely resolved */
		if (_poll_fds_num == 0) {
			px4_sleep(1);

			PX4_DEBUG("no valid fds");
			continue;
		}

		/* sleep waiting for data, but no more than a second */
		int ret = px4_poll(&_poll_fds[0], _poll_fds_num, 1000);

		/* this would be bad... */
		if (ret < 0) {
			PX4_ERR("poll error %d", errno);
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
		/* We also publish if not armed, this way we make sure SITL gets feedback. */
		if (_mixers != nullptr) {

			/* do mixing */
			_actuator_outputs.noutputs = _mixers->mix(&_actuator_outputs.output[0], _num_outputs);

			/* disable unused ports by setting their output to NaN */
			const size_t actuator_outputs_size = sizeof(_actuator_outputs.output) / sizeof(_actuator_outputs.output[0]);

			for (size_t i = _actuator_outputs.noutputs; i < actuator_outputs_size; i++) {
				_actuator_outputs.output[i] = NAN;
			}

			/* iterate actuators */
			for (unsigned i = 0; i < _actuator_outputs.noutputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				const bool sane_mixer_output = PX4_ISFINITE(_actuator_outputs.output[i]) &&
							       _actuator_outputs.output[i] >= -1.0f &&
							       _actuator_outputs.output[i] <= 1.0f;

				if (_armed && sane_mixer_output) {
					/* scale for PWM output 1000 - 2000us */
					_actuator_outputs.output[i] = 1500 + (500 * _actuator_outputs.output[i]);
					_actuator_outputs.output[i] = math::constrain(_actuator_outputs.output[i], (float)_pwm_min[i], (float)_pwm_max[i]);

				} else {
					/* Disarmed or insane value - set disarmed pwm value
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight. */
					_actuator_outputs.output[i] = PWM_SIM_DISARMED_MAGIC;
				}
			}

			/* overwrite outputs in case of force_failsafe */
			if (_failsafe) {
				for (size_t i = 0; i < _actuator_outputs.noutputs; i++) {
					_actuator_outputs.output[i] = PWM_SIM_FAILSAFE_MAGIC;
				}
			}

			/* overwrite outputs in case of lockdown */
			if (_lockdown) {
				for (size_t i = 0; i < _actuator_outputs.noutputs; i++) {
					_actuator_outputs.output[i] = 0.0;
				}
			}

			/* publish mixer status */
			MultirotorMixer::saturation_status saturation_status;
			saturation_status.value = _mixers->get_saturation_status();

			if (saturation_status.flags.valid) {
				multirotor_motor_limits_s motor_limits;
				motor_limits.timestamp = hrt_absolute_time();
				motor_limits.saturation_status = saturation_status.value;

				int instance;
				orb_publish_auto(ORB_ID(multirotor_motor_limits), &_mixer_status, &motor_limits, &instance, ORB_PRIO_DEFAULT);
			}

			/* and publish for anyone that cares to see */
			_actuator_outputs.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_actuator_outputs);
			PX4_INFO("Published ORB_ID(actuator_outputs)");

			// use first valid timestamp_sample for latency tracking
			for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
				const bool required = _groups_required & (1 << i);
				const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

				if (required && (timestamp_sample > 0)) {
					perf_set_elapsed(_perf_control_latency, _actuator_outputs.timestamp - timestamp_sample);
					break;
				}
			}
		}

		/* how about an arming update? */
		bool updated;

		orb_check(_armed_sub, &updated);

		if (updated) {
			actuator_armed_s aa = {};

			if (orb_copy(ORB_ID(actuator_armed), _armed_sub, &aa) == PX4_OK) {
				/* do not obey the lockdown value, as lockdown is for PWMSim. Only obey manual lockdown */
				_armed = aa.armed;
				_failsafe = aa.force_failsafe;
				_lockdown = aa.manual_lockdown;
			}
		}

		/* check for parameter updates */
		bool param_updated = false;
		orb_check(params_sub, &param_updated);

		if (param_updated) {
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), params_sub, &update);
			update_params();
		}
	}

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(params_sub);
}

int
PWMSim::control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	return 0;
}

int
PWMSim::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		// up_pwm_servo_arm(true);
		break;

	case PWM_SERVO_DISARM:
		// up_pwm_servo_arm(false);
		break;

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {

				if (i < MAX_ACTUATORS) {
					_pwm_min[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < MAX_ACTUATORS) {
					_pwm_max[i] = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_SET_UPDATE_RATE:
		// PWMSim always outputs at the alternate (usually faster) rate
		set_pwm_rate(arg);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		// PWMSim always outputs at the alternate (usually faster) rate
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		*(uint32_t *)arg = 400;
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		*(uint32_t *)arg = _current_update_rate;
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		*(uint32_t *)arg = 0;
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = PWM_SIM_FAILSAFE_MAGIC;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = PWM_SIM_DISARMED_MAGIC;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = PWM_SIM_PWM_MIN_MAGIC;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_TRIM_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = (PWM_SIM_PWM_MAX_MAGIC + PWM_SIM_PWM_MIN_MAGIC) / 2;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < _num_outputs; i++) {
				pwm->values[i] = PWM_SIM_PWM_MAX_MAGIC;
			}

			pwm->channel_count = _num_outputs;
			break;
		}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= _num_outputs) {
				ret = -EINVAL;

			} else {
				/* fetch a current PWM value */
				*(servo_position_t *)arg = _actuator_outputs.output[channel];
			}

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

			SimpleMixer *mixer = new SimpleMixer(control_callback, (uintptr_t)&_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				_groups_required = 0;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr) {
					_mixers = new MixerGroup(control_callback, (uintptr_t)&_controls);
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

int
PWMSim::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("pwm_out_sim",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				      1100,
				      (px4_main_t)&run_trampoline,
				      nullptr);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

PWMSim *PWMSim::instantiate(int argc, char *argv[])
{
	return new PWMSim();
}

int PWMSim::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	/* start the task if not running */
	if (!is_running()) {
		int ret = PWMSim::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	Mode servo_mode = MODE_NONE;

	if (!strcmp(verb, "mode_pwm")) {
		servo_mode = PWMSim::MODE_8PWM;

	} else if (!strcmp(verb, "mode_pwm16")) {
		servo_mode = PWMSim::MODE_16PWM;
	}

	/* was a new mode set? */
	if (servo_mode != MODE_NONE) {
		/* switch modes */
		PWMSim *object = get_instance();

		if (servo_mode != object->get_mode()) {
			/* (re)set the PWM output mode */
			return object->set_mode(servo_mode);
		}

		return 0;
	}

	return print_usage("unknown command");
}

int PWMSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages,
mix them with any loaded mixer and output the result to the
`actuator_output` uORB topic.

It is used in SITL and HITL.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out_sim", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task in mode_pwm16");

	PRINT_MODULE_USAGE_PARAM_COMMENT("All of the mode_* commands will start the pwm sim if not running already");

	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm", "use 8 PWM outputs");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode_pwm16", "use 16 PWM outputs");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int PWMSim::print_status()
{
	PX4_INFO("Running max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Polling %i actuator controls", _poll_fds_num);

	const char *mode_str = nullptr;

	switch (_mode) {
	case MODE_8PWM: mode_str = "pwm8"; break;
	case MODE_16PWM: mode_str = "pwm16"; break;

	case MODE_NONE: mode_str = "no pwm"; break;

	default:
		break;
	}

	if (mode_str) {
		PX4_INFO("PWM Mode: %s", mode_str);
	}

	return 0;
}

extern "C" __EXPORT int pwm_out_sim_main(int argc, char *argv[]);

int
pwm_out_sim_main(int argc, char *argv[])
{
	return PWMSim::main(argc, argv);
}
