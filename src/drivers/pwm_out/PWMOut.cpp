/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include "PWMOut.hpp"

#include <px4_platform_common/sem.hpp>

pthread_mutex_t pwm_out_module_mutex = PTHREAD_MUTEX_INITIALIZER;
static px4::atomic<PWMOut *> _objects[PWM_OUT_MAX_INSTANCES] {};
static px4::atomic_bool _require_arming[PWM_OUT_MAX_INSTANCES] {};

static bool is_running()
{
	for (auto &obj : _objects) {
		if (obj.load() != nullptr) {
			return true;
		}
	}

	return false;
}

PWMOut::PWMOut(int instance, uint8_t output_base) :
	CDev((instance == 0) ? PX4FMU_DEVICE_PATH : PX4FMU_DEVICE_PATH"1"),
	OutputModuleInterface((instance == 0) ? MODULE_NAME"0" : MODULE_NAME"1", px4::wq_configurations::hp_default),
	_instance(instance),
	_output_base(output_base),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
	if (!_mixing_output.useDynamicMixing()) {
		_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
		_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
	}
}

PWMOut::~PWMOut()
{
	/* make sure servos are off */
	up_pwm_servo_deinit(_pwm_mask);

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int PWMOut::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	if (_mixing_output.useDynamicMixing()) {
		_num_outputs = FMU_MAX_ACTUATORS;

	} else {
		_num_outputs = math::min(FMU_MAX_ACTUATORS - (int)_output_base, MAX_PER_INSTANCE);
	}

	_pwm_mask = ((1u << _num_outputs) - 1) << _output_base;
	_mixing_output.setMaxNumOutputs(_num_outputs);

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
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
int PWMOut::set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate)
{
	if (_mixing_output.useDynamicMixing()) {
		return -EINVAL;
	}

	PX4_DEBUG("pwm_out%u set_pwm_rate %x %u %u", _instance, rate_map, default_rate, alt_rate);

	for (unsigned pass = 0; pass < 2; pass++) {

		/* We should note that group is iterated over from 0 to FMU_MAX_ACTUATORS.
		 * This allows for the ideal worlds situation: 1 channel per group
		 * configuration.
		 *
		 * This is typically not what HW supports. A group represents a timer
		 * and channels belongs to a timer.
		 * Therefore all channels in a group are dependent on the timer's
		 * common settings and can not be independent in terms of count frequency
		 * (granularity of pulse width) and rate (period of repetition).
		 *
		 * To say it another way, all channels in a group must have the same
		 * rate and mode. (See rates above.)
		 */

		for (unsigned group = 0; group < FMU_MAX_ACTUATORS; group++) {

			// get the channel mask for this rate group
			uint32_t mask = _pwm_mask & up_pwm_servo_get_rate_group(group);

			if (mask == 0) {
				continue;
			}

			// all channels in the group must be either default or alt-rate
			uint32_t alt = rate_map & mask;

			if (pass == 0) {
				// preflight
				if ((alt != 0) && (alt != mask)) {
					PX4_WARN("rate group %u mask %" PRIx32 " bad overlap %" PRIx32, group, mask, alt);
					// not a legal map, bail
					return -EINVAL;
				}

			} else {
				// set it - errors here are unexpected
				if (alt != 0) {
					if (up_pwm_servo_set_rate_group_update(group, alt_rate) != OK) {
						PX4_WARN("rate group set alt failed");
						return -EINVAL;
					}

				} else {
					if (up_pwm_servo_set_rate_group_update(group, default_rate) != OK) {
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

	// minimum rate for backup schedule
	unsigned backup_schedule_rate_hz = math::min(_pwm_default_rate, _pwm_alt_rate);

	if (backup_schedule_rate_hz == 0) {
		// OneShot rate is 0
		backup_schedule_rate_hz = 50;
	}

	// constrain reasonably (1 to 50 Hz)
	backup_schedule_rate_hz = math::constrain(backup_schedule_rate_hz, 1u, 50u);

	_backup_schedule_interval_us = roundf(1e6f / backup_schedule_rate_hz);

	_current_update_rate = 0; // force update

	return OK;
}

void PWMOut::update_current_rate()
{
	/*
	* Adjust actuator topic update rate to keep up with
	* the highest servo update rate configured.
	*
	* We always mix at max rate; some channels may update slower.
	*/
	int max_rate = (_pwm_default_rate > _pwm_alt_rate) ? _pwm_default_rate : _pwm_alt_rate;

	// oneshot
	if ((_pwm_default_rate == 0) || (_pwm_alt_rate == 0)) {
		max_rate = 2000;

	} else {
		// run up to twice PWM rate to reduce end-to-end latency
		//  actual pulse width only updated for next period regardless of output module
		max_rate *= 2;
	}

	// max interval 0.5 - 100 ms (10 - 2000Hz)
	const int update_interval_in_us = math::constrain(1000000 / max_rate, 500, 100000);

	if (_current_update_rate != max_rate) {
		PX4_INFO("instance: %d, max rate: %d, default: %d, alt: %d", _instance, max_rate, _pwm_default_rate, _pwm_alt_rate);
	}

	_current_update_rate = max_rate;
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);
}

int PWMOut::task_spawn(int argc, char *argv[])
{
	for (unsigned instance = 0; instance < (sizeof(_objects) / sizeof(_objects[0])); instance++) {

		if (instance < PWM_OUT_MAX_INSTANCES) {
			uint8_t base = instance * MAX_PER_INSTANCE;  // TODO: configurable
			PWMOut *dev = new PWMOut(instance, base);

			if (dev) {
				_objects[instance].store(dev);

				if (dev->init() != PX4_OK) {
					PX4_ERR("%d - init failed", instance);
					delete dev;
					_objects[instance].store(nullptr);
					return PX4_ERROR;
				}

				// only start one instance with dynamic mixing
				if (dev->_mixing_output.useDynamicMixing()) {
					break;
				}

			} else {
				PX4_ERR("alloc failed");
			}

		} else {
			// This hardware platform does not support
			// this many devices, set the storage to
			// a sane default
			_objects[instance].store(nullptr);
		}
	}

	return PX4_OK;
}

bool PWMOut::update_pwm_out_state(bool on)
{
	if (on && !_pwm_initialized && _pwm_mask != 0) {

		if (_mixing_output.useDynamicMixing()) {

			for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
				_timer_rates[timer] = -1;

				uint32_t channels = io_timer_get_group(timer);

				if (channels == 0) {
					continue;
				}

				char param_name[17];
				snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

				int32_t tim_config = 0;
				param_t handle = param_find(param_name);
				param_get(handle, &tim_config);

				if (tim_config > 0) {
					_timer_rates[timer] = tim_config;

				} else if (tim_config == -1) { // OneShot
					_timer_rates[timer] = 0;

				} else {
					_pwm_mask &= ~channels; // don't use for pwm
				}
			}

			int ret = up_pwm_servo_init(_pwm_mask);

			if (ret < 0) {
				PX4_ERR("up_pwm_servo_init failed (%i)", ret);
				return false;
			}

			_pwm_mask = ret;

			// set the timer rates
			for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
				uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

				if (channels == 0) {
					continue;
				}

				ret = up_pwm_servo_set_rate_group_update(timer, _timer_rates[timer]);

				if (ret != 0) {
					PX4_ERR("up_pwm_servo_set_rate_group_update failed for timer %i, rate %i (%i)", timer, _timer_rates[timer], ret);
					_timer_rates[timer] = -1;
					_pwm_mask &= ~channels;
				}
			}

			_pwm_initialized = true;

			// disable unused functions
			for (unsigned i = 0; i < _num_outputs; ++i) {
				if (((1 << i) & _pwm_mask) == 0) {
					_mixing_output.disableFunction(i);
				}
			}

		} else {
			// Collect all PWM masks from all instances
			uint32_t pwm_mask_new = 0;
			// Collect the PWM alt rate channels across all instances
			uint32_t pwm_alt_rate_channels_new = 0;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {

					pwm_mask_new |= _objects[i].load()->get_pwm_mask();
					pwm_alt_rate_channels_new |= _objects[i].load()->get_alt_rate_channels();
				}
			}

			// Initialize the PWM output state for all instances
			// this is re-done once per instance, but harmless
			int ret = up_pwm_servo_init(pwm_mask_new);

			if (ret >= 0) {
				for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
					if (_objects[i].load()) {
						_objects[i].load()->set_pwm_mask(_objects[i].load()->get_pwm_mask() & ret);
					}
				}

				// Set rate is not affecting non-masked channels, so can be called
				// individually
				set_pwm_rate(get_alt_rate_channels(), get_default_rate(), get_alt_rate());

				_pwm_initialized = true;

				// Other instances need to call up_pwm_servo_arm again after we initialized
				for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
					if (i != _instance) {
						_require_arming[i].store(true);
					}
				}

			} else {
				PX4_ERR("up_pwm_servo_init failed (%i)", ret);
			}
		}
	}

	_require_arming[_instance].store(false);
	up_pwm_servo_arm(on, _pwm_mask);
	return true;
}

bool PWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	/* output to the servos */
	if (_pwm_initialized) {
		for (size_t i = 0; i < num_outputs; i++) {
			if (!_mixing_output.isFunctionSet(i)) {
				// do not run any signal on disabled channels
				outputs[i] = 0;
			}

			if (_pwm_mask & (1 << (i + _output_base))) {
				up_pwm_servo_set(_output_base + i, outputs[i]);
			}
		}
	}

	/* Trigger all timer's channels in Oneshot mode to fire
	 * the oneshots with updated values.
	 */
	if (num_control_groups_updated > 0) {
		up_pwm_update(_pwm_mask);
	}

	return true;
}

void PWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		//exit_and_cleanup();
		return;
	}

	SmartLock lock_guard(_lock);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	if (!_mixing_output.useDynamicMixing()) {
		// push backup schedule
		ScheduleDelayed(_backup_schedule_interval_us);
	}

	_mixing_output.update();

	/* update PWM status if armed or if disarmed PWM values are set */
	bool pwm_on = _mixing_output.armed().armed || (_num_disarmed_set > 0) || _mixing_output.useDynamicMixing()
		      || _mixing_output.armed().in_esc_calibration_mode;

	if (_pwm_on != pwm_on || _require_arming[_instance].load()) {

		if (update_pwm_out_state(pwm_on)) {
			_pwm_on = pwm_on;
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		if (_mixing_output.useDynamicMixing()) { // do not update PWM params for now (was interfering with VTOL PWM settings)
			update_params();
		}
	}

	if (_pwm_initialized && _current_update_rate == 0 && !_mixing_output.useDynamicMixing()) {
		update_current_rate();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

void PWMOut::update_params()
{
	updateParams();

	if (_mixing_output.useDynamicMixing()) {
		return;
	}

	int32_t pwm_min_default = PWM_DEFAULT_MIN;
	int32_t pwm_max_default = PWM_DEFAULT_MAX;
	int32_t pwm_disarmed_default = 0;
	int32_t pwm_rate_default = 50;
	int32_t pwm_default_channels = 0;

	const char *prefix;

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		prefix = "PWM_MAIN";

		param_get(param_find("PWM_MAIN_MIN"), &pwm_min_default);
		param_get(param_find("PWM_MAIN_MAX"), &pwm_max_default);
		param_get(param_find("PWM_MAIN_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_MAIN_RATE"), &pwm_rate_default);
		param_get(param_find("PWM_MAIN_OUT"), &pwm_default_channels);

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		prefix = "PWM_AUX";

		param_get(param_find("PWM_AUX_MIN"), &pwm_min_default);
		param_get(param_find("PWM_AUX_MAX"), &pwm_max_default);
		param_get(param_find("PWM_AUX_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_AUX_RATE"), &pwm_rate_default);
		param_get(param_find("PWM_AUX_OUT"), &pwm_default_channels);

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		prefix = "PWM_EXTRA";

		param_get(param_find("PWM_EXTRA_MIN"), &pwm_min_default);
		param_get(param_find("PWM_EXTRA_MAX"), &pwm_max_default);
		param_get(param_find("PWM_EXTRA_DISARM"), &pwm_disarmed_default);
		param_get(param_find("PWM_EXTRA_RATE"), &pwm_rate_default);

	} else {
		PX4_ERR("invalid class instance %d", _class_instance);
		return;
	}

	uint32_t single_ch = 0;
	uint32_t pwm_default_channel_mask = 0;

	while ((single_ch = pwm_default_channels % 10)) {
		pwm_default_channel_mask |= 1 << (single_ch - 1);
		pwm_default_channels /= 10;
	}

	// update the counter
	// this is needed to decide if disarmed PWM output should be turned on or not
	int num_disarmed_set = 0;

	char str[17];

	for (unsigned i = 0; i < _num_outputs; i++) {
		// PWM_MAIN_MINx
		{
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			int32_t pwm_min = -1;

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				if (pwm_min >= 0 && pwm_min != 1000) {
					_mixing_output.minValue(i) = math::constrain(pwm_min, (int32_t) PWM_LOWEST_MIN, (int32_t) PWM_HIGHEST_MIN);

					if (pwm_min != _mixing_output.minValue(i)) {
						int32_t pwm_min_new = _mixing_output.minValue(i);
						param_set(param_find(str), &pwm_min_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.minValue(i) = pwm_min_default;
				}

			} else {
				PX4_ERR("param %s not found", str);
			}
		}

		// PWM_MAIN_MAXx
		{
			sprintf(str, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_max = -1;

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				if (pwm_max >= 0 && pwm_max != 2000) {
					_mixing_output.maxValue(i) = math::constrain(pwm_max, (int32_t) PWM_LOWEST_MAX, (int32_t) PWM_HIGHEST_MAX);

					if (pwm_max != _mixing_output.maxValue(i)) {
						int32_t pwm_max_new = _mixing_output.maxValue(i);
						param_set(param_find(str), &pwm_max_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.maxValue(i) = pwm_max_default;
				}

			} else {
				PX4_ERR("param %s not found", str);
			}
		}

		// PWM_MAIN_DISx
		{
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis >= 0 && pwm_dis != 900) {
					_mixing_output.disarmedValue(i) = math::constrain(pwm_dis, (int32_t) 0, (int32_t) PWM_HIGHEST_MAX);

					if (pwm_dis != _mixing_output.disarmedValue(i)) {
						int32_t pwm_dis_new = _mixing_output.disarmedValue(i);
						param_set(param_find(str), &pwm_dis_new);
					}

				} else if (pwm_default_channel_mask & 1 << i) {
					_mixing_output.disarmedValue(i) = pwm_disarmed_default;
				}

			} else {
				PX4_ERR("param %s not found", str);
			}

			if (_mixing_output.disarmedValue(i) > 0) {
				num_disarmed_set++;
			}
		}

		// PWM_MAIN_FAILx
		{
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_failsafe = -1;

			if (param_get(param_find(str), &pwm_failsafe) == PX4_OK) {
				if (pwm_failsafe >= 0) {
					_mixing_output.failsafeValue(i) = math::constrain(pwm_failsafe, (int32_t) 0, (int32_t) PWM_HIGHEST_MAX);

					if (pwm_failsafe != _mixing_output.failsafeValue(i)) {
						int32_t pwm_fail_new = _mixing_output.failsafeValue(i);
						param_set(param_find(str), &pwm_fail_new);
					}

				} else {
					// if no channel specific failsafe value is configured, use the disarmed value
					_mixing_output.failsafeValue(i) = _mixing_output.disarmedValue(i);
				}

			} else {
				PX4_ERR("param %s not found", str);
			}
		}

		// PWM_MAIN_REVx
		{
			sprintf(str, "%s_REV%u", prefix, i + 1);
			int32_t pwm_rev = 0;

			if (param_get(param_find(str), &pwm_rev) == PX4_OK) {
				uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();

				if (pwm_rev >= 1) {
					reverse_pwm_mask = reverse_pwm_mask | (1 << i);

				} else {
					reverse_pwm_mask = reverse_pwm_mask & ~(1 << i);
				}

			} else {
				PX4_ERR("param %s not found", str);
			}
		}
	}

	if (_mixing_output.mixers()) {
		int16_t values[FMU_MAX_ACTUATORS] {};

		for (unsigned i = 0; i < _num_outputs; i++) {
			sprintf(str, "%s_TRIM%u", prefix, i + 1);

			float pval = 0.0f;

			if (param_get(param_find(str), &pval) != PX4_OK) {
				PX4_ERR("param %s not found", str);
			}

			values[i] = roundf(10000 * pval);
		}

		// copy the trim values to the mixer offsets
		_mixing_output.mixers()->set_trims(values, _num_outputs);
	}

	_num_disarmed_set = num_disarmed_set;
}

int PWMOut::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	SmartLock lock_guard(_lock);

	int ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int PWMOut::pwm_ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("pwm_out%u: ioctl cmd: %d, arg: %ld", _instance, cmd, arg);

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

	case PWM_SERVO_GET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.failsafeValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.disarmedValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			break;
		}

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS || _mixing_output.useDynamicMixing()) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] > PWM_HIGHEST_MIN) {
					_mixing_output.minValue(i) = PWM_HIGHEST_MIN;

				}

#if PWM_LOWEST_MIN > 0

				else if (pwm->values[i] < PWM_LOWEST_MIN) {
					_mixing_output.minValue(i) = PWM_LOWEST_MIN;
				}

#endif

				else {
					_mixing_output.minValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.minValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
			break;
		}

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			/* discard if too many values are sent */
			if (pwm->channel_count > FMU_MAX_ACTUATORS || _mixing_output.useDynamicMixing()) {
				ret = -EINVAL;
				break;
			}

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (pwm->values[i] == 0) {
					/* ignore 0 */
				} else if (pwm->values[i] < PWM_LOWEST_MAX) {
					_mixing_output.maxValue(i) = PWM_LOWEST_MAX;

				} else if (pwm->values[i] > PWM_HIGHEST_MAX) {
					_mixing_output.maxValue(i) = PWM_HIGHEST_MAX;

				} else {
					_mixing_output.maxValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < FMU_MAX_ACTUATORS; i++) {
				pwm->values[i] = _mixing_output.maxValue(i);
			}

			pwm->channel_count = FMU_MAX_ACTUATORS;
			arg = (unsigned long)&pwm;
		}
		break;

#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 14

	case PWM_SERVO_GET(13):
	case PWM_SERVO_GET(12):
	case PWM_SERVO_GET(11):
	case PWM_SERVO_GET(10):
	case PWM_SERVO_GET(9):
	case PWM_SERVO_GET(8):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 8
	case PWM_SERVO_GET(7):
	case PWM_SERVO_GET(6):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 6
	case PWM_SERVO_GET(5):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 5
	case PWM_SERVO_GET(4):
#endif
	case PWM_SERVO_GET(3):
	case PWM_SERVO_GET(2):
	case PWM_SERVO_GET(1):
	case PWM_SERVO_GET(0):
		if (cmd - PWM_SERVO_GET(0) >= (int)_num_outputs) {
			ret = -EINVAL;
			break;
		}

		*(servo_position_t *)arg = up_pwm_servo_get(cmd - PWM_SERVO_GET(0) +  _output_base);
		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 5
	case PWM_SERVO_GET_RATEGROUP(4):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 6
	case PWM_SERVO_GET_RATEGROUP(5):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 8
	case PWM_SERVO_GET_RATEGROUP(6):
	case PWM_SERVO_GET_RATEGROUP(7):
#endif
#if defined(DIRECT_PWM_OUTPUT_CHANNELS) && DIRECT_PWM_OUTPUT_CHANNELS >= 14
	case PWM_SERVO_GET_RATEGROUP(8):
	case PWM_SERVO_GET_RATEGROUP(9):
	case PWM_SERVO_GET_RATEGROUP(10):
	case PWM_SERVO_GET_RATEGROUP(11):
	case PWM_SERVO_GET_RATEGROUP(12):
	case PWM_SERVO_GET_RATEGROUP(13):
#endif
		*(uint32_t *)arg = _pwm_mask & up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _num_outputs;
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixer();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixer(buf, buflen);
			update_params();

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

int PWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMOut::print_status()
{
	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		PX4_INFO("%d - PWM_MAIN 0x%04" PRIx32, _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_SECONDARY) {
		PX4_INFO("%d - PWM_AUX 0x%04" PRIx32, _instance, _pwm_mask);

	} else if (_class_instance == CLASS_DEVICE_TERTIARY) {
		PX4_INFO("%d - PWM_EXTRA 0x%04" PRIx32, _instance, _pwm_mask);
	}

	PX4_INFO("%d - Max update rate: %i Hz", _instance, _current_update_rate);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	if (_mixing_output.useDynamicMixing() && _pwm_initialized) {
		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			if (_timer_rates[timer] >= 0) {
				PX4_INFO_RAW("Timer %i: rate: %3i", timer, _timer_rates[timer]);
				uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

				if (channels > 0) {
					PX4_INFO_RAW(" channels: ");

					for (uint32_t channel = 0; channel < _num_outputs; ++channel) {
						if ((1 << channel) & channels) {
							PX4_INFO_RAW("%" PRIu32 " ", channel);
						}
					}
				}

				PX4_INFO_RAW("\n");
			}
		}
	}

	return 0;
}

int PWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving the output pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

On startup, the module tries to occupy all available pins for PWM/Oneshot output.
It skips all pins already in use (e.g. by a camera trigger module).

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_main(int argc, char *argv[])
{
	if (argc <= 1 || strcmp(argv[1], "-h") == 0) {
		return PWMOut::print_usage();
	}

	if (strcmp(argv[1], "start") == 0) {

		if (is_running()) {
			return 0;
		}

		int ret = 0;

		PWMOut::lock_module();

		ret = PWMOut::task_spawn(argc - 1, argv + 1);

		if (ret < 0) {
			PX4_ERR("start failed (%i)", ret);
		}

		PWMOut::unlock_module();
		return ret;

	} else if (strcmp(argv[1], "status") == 0) {
		if (PWMOut::trylock_module()) {

			unsigned count = 0;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				if (_objects[i].load()) {
					PX4_INFO_RAW("\n");
					_objects[i].load()->print_status();
					count++;
				}
			}

			PWMOut::unlock_module();

			if (count == 0) {
				PX4_INFO("not running");
				return 1;
			}

		} else {
			PX4_WARN("module locked, try again later");
		}

		return 0;

	} else if (strcmp(argv[1], "stop") == 0) {
		PWMOut::lock_module();

		if (argc > 2) {
			int instance = atoi(argv[2]);

			if (instance >= 0 && instance < PWM_OUT_MAX_INSTANCES) {
				PX4_INFO("stopping instance %d", instance);
				PWMOut *inst = _objects[instance].load();

				if (inst) {
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[instance].store(nullptr);
				}
			} else {
				PX4_ERR("invalid instance %d", instance);
			}

		} else {
			// otherwise stop everything
			bool was_running = false;

			for (int i = 0; i < PWM_OUT_MAX_INSTANCES; i++) {
				PWMOut *inst = _objects[i].load();

				if (inst) {
					PX4_INFO("stopping pwm_out instance %d", i);
					was_running = true;
					inst->request_stop();
					px4_usleep(20000); // 20 ms
					delete inst;
					_objects[i].store(nullptr);
				}
			}

			if (!was_running) {
				PX4_WARN("not running");
			}
		}

		PWMOut::unlock_module();
		return PX4_OK;
	}

	PWMOut::lock_module(); // Lock here, as the method could access _object.
	int ret = PWMOut::custom_command(argc - 1, argv + 1);
	PWMOut::unlock_module();

	return ret;
}
