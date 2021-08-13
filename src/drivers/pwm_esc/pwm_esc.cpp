/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * @file pwm_esc.cpp
 * Driver for the NuttX PWM driver controleed escs
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.hpp>

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
#include <math.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>

#include <lib/mixer_module/mixer_module.hpp>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <nuttx/timers/pwm.h>

#include <debug.h>

#ifndef PWMESC_OUT_PATH
#define PWMESC_OUT_PATH "/dev/pwmX";
#endif

#ifndef PWMESC_MAX_DEVICES
#define PWMESC_MAX_DEVICES 2
#endif

#ifndef PWMESC_MAX_CHANNELS
#define PWMESC_MAX_CHANNELS 8
#endif

#ifndef PWM_DEFAULT_RATE
#define PWM_DEFAULT_RATE 400
#endif

//using namespace time_literals;

/**
 * The PWMESC class.
 *
 */
class PWMESC : public cdev::CDev, public OutputModuleInterface
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PWMESC();

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~PWMESC();

	/**
	 * Initialize the PWMESC class.
	 *
	 * Retrieve relevant initial system parameters. Connect to PWM device
	 *
	 * @param hitl_mode set to suppress publication of actuator_outputs
	 */
	int			init(bool hitl_mode);


	/**
	 * Get the singleton instance
	 */
	static inline PWMESC *getInstance() {return _instance;}

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PWMESC file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * updateOutputs
	 *
	 * Sets the actual PWM outputs. See OutputModuleInterface
	 *
	 */

	bool		updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				      unsigned num_outputs, unsigned num_control_groups_updated) override;

	/**
	 * Don't allow more channels than MAX_ACTUATORS
	*/
	static_assert(PWMESC_MAX_CHANNELS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

private:

	bool _initialized{false};

	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag

	px4_sem_t _update_sem;

	perf_counter_t		_perf_update;		///< local performance counter for PWM updates

	/* subscribed topics */

	int		_actuator_armed_sub{-1};	///< system armed control topic

	MixingOutput _mixing_output{PWMESC_MAX_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	/* advertised topics */
	uORB::PublicationMulti<actuator_outputs_s>		_actuator_outputs_pub{ORB_ID(actuator_outputs)};

	actuator_armed_s		_actuator_armed;

	bool                    _hitl_mode;     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	int		_pwm_fd[PWMESC_MAX_DEVICES];

	int32_t		_pwm_min_default;
	int32_t		_pwm_max_default;
	int32_t		_pwm_disarmed_default;
	uint16_t		_pwm_min[MAX_ACTUATORS];
	uint16_t		_pwm_max[MAX_ACTUATORS];
	uint16_t		_pwm_disarmed[MAX_ACTUATORS];
	uint16_t		_pwm_fail[MAX_ACTUATORS];
	int32_t		_pwm_rate{PWM_DEFAULT_RATE};

	int		init_pwm_outputs();

	/* Singleton pointer */
	static PWMESC	*_instance;

	/**
	 * Trampoline to the worker task
	 */
	static int		task_main_trampoline(int argc, char *argv[]);

	/**
	 * worker task
	 */
	void			task_main();

	/**
	 * Callback for mixer subscriptions
	 */
	void Run() override;

	void update_params();

	/* No copy constructor */
	PWMESC(const PWMESC &);
	PWMESC operator=(const PWMESC &);
};

PWMESC *PWMESC::_instance = nullptr;

PWMESC::PWMESC() :
	CDev(PWM_OUTPUT0_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_task(-1),
	_task_should_exit(false),
	_perf_update(perf_alloc(PC_ELAPSED, "pwm update")),
	_actuator_armed_sub(-1),
	_hitl_mode(false)
{

	/* initialize tick semaphores */
	px4_sem_init(&_update_sem, 0, 0);
	px4_sem_setprotocol(&_update_sem, SEM_PRIO_NONE);

	/* we need this potentially before it could be set in task_main */
	_instance = this;
}

PWMESC::~PWMESC()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		px4_usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		task_delete(_task);
	}

	/* deallocate perfs */
	perf_free(_perf_update);

	px4_sem_destroy(&_update_sem);

	_instance = nullptr;
}

int
PWMESC::init(bool hitl_mode)
{
	int ret;

	_hitl_mode = hitl_mode;

	/* do regular cdev init */
	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* start the main task */
	_task = px4_task_spawn_cmd("pwm_esc",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   3048,
				   (px4_main_t)&PWMESC::task_main_trampoline,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("task start failed: %d", errno);
		return -errno;
	}

	/* schedule workqueue */
	ScheduleNow();

	return OK;
}

int
PWMESC::task_main_trampoline(int argc, char *argv[])
{
	_instance->task_main();
	return 0;
}

bool
PWMESC::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		      unsigned num_control_groups_updated)
{
	bool ret = true;
	struct pwm_info_s pwm;

	memset(&pwm, 0, sizeof(struct pwm_info_s));
	pwm.frequency = _pwm_rate;

	for (unsigned i = 0; i < num_outputs; i++) {
		// TODO: channel to proper pwm device map.
		// this is now just quick hack for one pwm device, direct map of channels

		uint16_t pwm_val = outputs[i];
		pwm.channels[i].duty = ((((uint32_t)pwm_val) << 16) / (1000000 / _pwm_rate));
		pwm.channels[i].channel = i + 1;
	}

	/* Only publish if not in hitl */

	if (!_hitl_mode &&
	    ::ioctl(_pwm_fd[0], PWMIOC_SETCHARACTERISTICS,
		    (unsigned long)((uintptr_t)&pwm)) < 0) {
		PX4_ERR("PWMIOC_SETCHARACTERISTICS) failed: %d\n",
			errno);
		ret = false;
	}

	return ret;
}


int
PWMESC::init_pwm_outputs()
{
	int ret;

	/* Update parameters initially to get pwm min/max etc */

	update_params();

	_mixing_output.setIgnoreLockdown(_hitl_mode);
	_mixing_output.setMaxNumOutputs(PWMESC_MAX_CHANNELS);
	const int update_interval_in_us = math::constrain(1000000 / (_pwm_rate * 2), 500, 100000);
	_mixing_output.setMaxTopicUpdateRate(update_interval_in_us);

	/* Open the PWM devnode */

	// TODO: loop through the devices, open all fd:s
	char pwm_device_name[] = PWMESC_OUT_PATH;
	int n_pwm_devices = 1;

	for (int i = 0; i < 1 && i < n_pwm_devices; i++) {
		pwm_device_name[sizeof(pwm_device_name) - 2] = '0' + i;

		_pwm_fd[i] = ::open(pwm_device_name, O_RDONLY);

		if (_pwm_fd[i] < 0) {
			PX4_ERR("pwm_main: open %s failed: %d\n", pwm_device_name, errno);
			return -ENODEV;
		}

		/* Configure PWM to default rate, disarmed pulse and start */

		struct pwm_info_s pwm;
		memset(&pwm, 0, sizeof(struct pwm_info_s));
		pwm.frequency = _pwm_rate;

		for (int j = 0; j < PWMESC_MAX_CHANNELS; j++) {
			pwm.channels[j].duty = ((((uint32_t)_pwm_disarmed[j]) << 16) / 2500);
			pwm.channels[j].channel = j + 1;
		}

		ret = ::ioctl(_pwm_fd[i], PWMIOC_SETCHARACTERISTICS,
			      (unsigned long)((uintptr_t)&pwm));


		if (ret < 0) {
			PX4_ERR("PWMIOC_SETCHARACTERISTICS) failed: %d\n",
				errno);
		}

		ret = ::ioctl(_pwm_fd[i], PWMIOC_START, 0);

		if (ret < 0) {
			PX4_ERR("PWMIOC_START failed: %d\n", errno);
			return -ENODEV;
		}
	}

	return 0;
}

void
PWMESC::Run()
{
	/* Just trigger the main task */
	px4_sem_post(&_update_sem);
}

void
PWMESC::task_main()
{
	bool first = true;

	/* Wait for system to be initialized fully */

	while (!_initialized) {
		usleep(10000);
	}

	_actuator_armed_sub =  orb_subscribe(ORB_ID(actuator_armed));

	if (_actuator_armed_sub < 0) {
		PX4_ERR("arming status subscription failed");
		_task_should_exit = true;
	}

	while (!_task_should_exit) {

		/* Get the armed status */

		orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &_actuator_armed);


		/* Initialize PWM outputs on first execution */
		if (first) {
			first = false;

			if (init_pwm_outputs() != 0) {
				_task_should_exit = true;
			}
		}

		/* sleep waiting for mixer update */

		px4_sem_wait(&_update_sem);

		perf_begin(_perf_update);

		_mixing_output.update();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);
		}

		_mixing_output.updateSubscriptions(true);

		perf_end(_perf_update);
	}

	PX4_DEBUG("exiting");

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

void PWMESC::update_params()
{
	// skip update when armed
	if (_actuator_armed.armed) {
		return;
	}

	/* Call MixingOutput::updateParams */

	updateParams();

	_pwm_min_default = PWM_DEFAULT_MIN;
	_pwm_max_default = PWM_DEFAULT_MAX;
	_pwm_disarmed_default = PWM_MOTOR_OFF;

	const char *prefix = "PWM_MAIN";

	param_get(param_find("PWM_MAIN_MIN"), &_pwm_min_default);
	param_get(param_find("PWM_MAIN_MAX"), &_pwm_max_default);
	param_get(param_find("PWM_MAIN_DISARM"), &_pwm_disarmed_default);
	param_get(param_find("PWM_MAIN_RATE"), &_pwm_rate);

	char str[17];

	// PWM_MAIN_MINx
	{
		for (unsigned i = 0; i < PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_MIN%u", prefix, i + 1);
			int32_t pwm_min = -1;

			if (param_get(param_find(str), &pwm_min) == PX4_OK) {
				if (pwm_min >= 0) {
					_pwm_min[i] = math::constrain(pwm_min, static_cast<int32_t>(PWM_LOWEST_MIN), static_cast<int32_t>(PWM_HIGHEST_MIN));

					if (pwm_min != _pwm_min[i]) {
						int32_t pwm_min_new = _pwm_min[i];
						param_set(param_find(str), &pwm_min_new);
					}

				} else {
					_pwm_min[i] = _pwm_min_default;
				}

			} else {
				_pwm_min[i] = 0;
			}

			_mixing_output.minValue(i) = _pwm_min[i];
		}

	}

	// PWM_MAIN_MAXx
	{
		for (unsigned i = 0; i <  PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_MAX%u", prefix, i + 1);
			int32_t pwm_max = -1;

			if (param_get(param_find(str), &pwm_max) == PX4_OK) {
				if (pwm_max >= 0) {
					_pwm_max[i] = math::constrain(pwm_max, static_cast<int32_t>(PWM_LOWEST_MAX), static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_max != _pwm_max[i]) {
						int32_t pwm_max_new = _pwm_max[i];
						param_set(param_find(str), &pwm_max_new);
					}

				} else {
					_pwm_max[i] = _pwm_max_default;
				}

			} else {
				_pwm_max[i] = 0;
			}

			_mixing_output.maxValue(i) = _pwm_max[i];
		}
	}

	// PWM_MAIN_FAILx
	{
		for (unsigned i = 0; i <  PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_FAIL%u", prefix, i + 1);
			int32_t pwm_fail = -1;

			if (param_get(param_find(str), &pwm_fail) == PX4_OK) {
				if (pwm_fail >= 0) {
					_pwm_fail[i] = math::constrain(pwm_fail, static_cast<int32_t>(0), static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_fail != _pwm_fail[i]) {
						int32_t pwm_fail_new = _pwm_fail[i];
						param_set(param_find(str), &pwm_fail_new);
					}

				} else {
					_pwm_fail[i] = -1;
				}

			} else {
				_pwm_fail[i] = 0;
			}

			_mixing_output.failsafeValue(i) = _pwm_fail[i];
		}
	}

	// PWM_MAIN_DISx
	{
		for (unsigned i = 0; i <  PWMESC_MAX_CHANNELS; i++) {
			sprintf(str, "%s_DIS%u", prefix, i + 1);
			int32_t pwm_dis = -1;

			if (param_get(param_find(str), &pwm_dis) == PX4_OK) {
				if (pwm_dis >= 0) {
					_pwm_disarmed[i] = math::constrain(pwm_dis, static_cast<int32_t>(0), static_cast<int32_t>(PWM_HIGHEST_MAX));

					if (pwm_dis != _pwm_disarmed[i]) {
						int32_t pwm_dis_new = _pwm_disarmed[i];
						param_set(param_find(str), &pwm_dis_new);
					}

				} else {
					_pwm_disarmed[i] = _pwm_disarmed_default;
				}

			} else {
				_pwm_disarmed[i] = 0;
			}

			_mixing_output.disarmedValue(i) = _pwm_disarmed[i];
		}
	}
}


int
PWMESC::ioctl(file *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		_alert("PWM_SERVO_ARM\n");
		break;

	case PWM_SERVO_SET_ARM_OK:
		/* set the 'OK to arm' bit */
		_alert("PWM_SERVO_SET_ARM_OK\n");
		break;

	case PWM_SERVO_CLEAR_ARM_OK:
		/* clear the 'OK to arm' bit */
		_alert("PWM_SERVO_CLEAR_ARM_OK\n");
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		_alert("PWM_SERVO_DISARM\n");
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		/* get the default update rate */
		_alert("PWM_SERVO_GET_DEFAULT_UPDATE_RATE\n");
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the requested alternate rate */
		_alert("PWM_SERVO_SET_UPDATE_RATE\n");
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		/* get the alternative update rate */
		_alert("PWM_SERVO_GET_UPDATE_RATE\n");
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		_alert("PWM_SERVO_SET_SELECT_UPDATE_RATE\n");
		break;

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:
		_alert("PWM_SERVO_GET_SELECT_UPDATE_RATE\n");
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM:
		_alert("PWM_SERVO_SET_FAILSAFE_PWM\n");
		break;

	case PWM_SERVO_GET_FAILSAFE_PWM:
		_alert("PWM_SERVO_GET_FAILSAFE_PWM\n");
		break;

	case PWM_SERVO_SET_DISARMED_PWM:
		_alert("PWM_SERVO_SET_DISARMED_PWM\n");
		break;

	case PWM_SERVO_GET_DISARMED_PWM:
		_alert("PWM_SERVO_GET_DISARMED_PWM\n");
		break;

	case PWM_SERVO_SET_MIN_PWM: {
			_alert("PWM_SERVO_SET_MIN_PWM\n");
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < MAX_ACTUATORS) {
					_mixing_output.minValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MIN_PWM:
		_alert("PWM_SERVO_GET_MIN_PWM\n");
		break;

	case PWM_SERVO_SET_MAX_PWM: {
			_alert("PWM_SERVO_SET_MAX_PWM\n");
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			for (unsigned i = 0; i < pwm->channel_count; i++) {
				if (i < MAX_ACTUATORS) {
					_mixing_output.maxValue(i) = pwm->values[i];
				}
			}

			break;
		}

	case PWM_SERVO_GET_MAX_PWM:
		_alert("PWM_SERVO_GET_MAX_PWM\n");
		break;

	case PWM_SERVO_GET_TRIM_PWM:
		_alert("PWM_SERVO_GET_TRIM_PWM\n");
		break;

	case PWM_SERVO_GET_COUNT:
		_alert("PWM_SERVO_GET_COUNT\n");
		break;

	case PWM_SERVO_SET_DISABLE_LOCKDOWN:
		_alert("PWM_SERVO_SET_DISABLE_LOCKDOWN\n");
		break;

	case PWM_SERVO_GET_DISABLE_LOCKDOWN:
		_alert("PWM_SERVO_GET_DISABLE_LOCKDOWN\n");
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		_alert("PWM_SERVO_SET_FORCE_SAFETY_OFF\n");
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		_alert("PWM_SERVO_SET_FORCE_SAFETY_ON\n");
		break;

	case PWM_SERVO_SET_FORCE_FAILSAFE:
		_alert("PWM_SERVO_SET_FORCE_FAILSAFE\n");
		break;

	case PWM_SERVO_SET_TERMINATION_FAILSAFE:
		_alert("PWM_SERVO_SET_TERMINATION_FAILSAFE\n");
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS - 1):
		_alert("PWM_SERVO_SET\n");
		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1):
		_alert("PWM_SERVO_GET\n");
		break;

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1):
		_alert("PWM_SERVO_GET_RATEGROUP\n");
		break;

	case PWM_SERVO_SET_MODE:
		_alert("PWM_SERVO_SET_MODE\n");
		break;

	case MIXERIOCRESET:
		_alert("MIXERIOCRESET\n");
		/* Can start the main thread now */
		_initialized = true;
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			_alert("MIXERIOCLOADBUF\n");
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			break;
		}

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}

extern "C" __EXPORT int pwm_esc_main(int argc, char *argv[]);

namespace
{

void
start(int argc, char *argv[])
{
	if (PWMESC::getInstance() != nullptr) {
		errx(0, "already loaded");
	}

	/* create the driver */
	(void)new PWMESC();

	if (PWMESC::getInstance == nullptr) {
		errx(1, "driver allocation failed");
	}

	bool hitl_mode = false;

	/* Check if started in hil mode */
	for (int extra_args = 1; extra_args < argc; extra_args++) {
		if (!strcmp(argv[extra_args], "hil")) {
			hitl_mode = true;

		} else if (argv[extra_args][0] != '\0') {
			PX4_WARN("unknown argument: %s", argv[extra_args]);
		}
	}

	if (OK != PWMESC::getInstance()->init(hitl_mode)) {
		delete PWMESC::getInstance();
		errx(1, "driver init failed");
	}

	exit(0);
}

} /* namespace */

int
pwm_esc_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2) {
		goto out;
	}

	if (!strcmp(argv[1], "start")) {
		start(argc - 1, argv + 1);
	}

out:
	errx(1, "need a command, try 'start'");
}
