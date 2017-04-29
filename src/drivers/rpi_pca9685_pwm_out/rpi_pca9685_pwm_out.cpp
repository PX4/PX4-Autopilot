/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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
#include "rpi_pca9685_pwm_out.h"

using namespace rpi_pca9685_pwm_out;
//--------------------------------------------------------------------------//
int rpi_pca9685_pwm_out::mixer_control_callback(uintptr_t handle,
		uint8_t control_group,
		uint8_t control_index,
		float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;
	input = controls[control_group].control[control_index];

	return 0;
}

//----------------------------------------------------------------------------//
int rpi_pca9685_pwm_out::initialize_mixer(const char *mixer_filename)
{
	char buf[4096];
	unsigned buflen = sizeof(buf);
	memset(buf, '\0', buflen);

	_mixer_group = new MixerGroup(mixer_control_callback, (uintptr_t) &_controls);

	// PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

	if (load_mixer_file(mixer_filename, buf, buflen) == 0) {
		if (_mixer_group->load_from_buf(buf, buflen) == 0) {
			PX4_INFO("Loaded mixer from file %s", mixer_filename);
			return 0;

		} else {
			PX4_ERR("Unable to parse from mixer config file %s", mixer_filename);
		}

	} else {
		PX4_ERR("Unable to load config file %s", mixer_filename);
	}

	if (_mixer_group->count() <= 0) {
		PX4_ERR("Mixer initialization failed");
		return -1;
	}

	return 0;
}
//----------------------------------------------------------------------------//
int rpi_pca9685_pwm_out::pwm_initialize()
{
	/**************初始化PCA9685开始*************/
	/**************PCA965 initializing********/
	pwm.init(1, 0x40);
	usleep(1000 * 100);
	/****12BIT 精度输出下，好赢电调可以到200HZ刷新***/
	/****200HZ for 12bit Resolution, support most of the esc***/
	pwm.setPWMFreq(200);
	usleep(1000 * 1000);
	/**************初始化PCA9685结束************/
	/**************PCA965 initialized********/
	return 0;
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::pwm_deinitialize()
{
	rpi_pca9685_pwm_out::pwm.reset();
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::send_outputs_pwm(const uint16_t *pwm)
{
	/*************向PCA9685发送数据*************/
	/*************send pwm signal to pca9685 initializing*************/
	int i;

	for (i = 0; i < NUM_PWM; ++i) {
		PX4_WARN("PWM%d:%d\n", i, *(pwm + i));
		rpi_pca9685_pwm_out::pwm.setPWM(i, *(pwm + i));
	}
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::subscribe()
{
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	/* set up ORB topic names */
	_controls_topics[0] = ORB_ID(actuator_controls_0);
	_controls_topics[1] = ORB_ID(actuator_controls_1);
	_controls_topics[2] = ORB_ID(actuator_controls_2);
	_controls_topics[3] = ORB_ID(actuator_controls_3);

	// Subscribe for orb topics
	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_required & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_controls_subs[i] = orb_subscribe(_controls_topics[i]);

		} else {
			_controls_subs[i] = -1;
		}

		if (_controls_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _controls_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	}
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::task_main(int argc, char *argv[])
{
	_is_running = true;

	/***************初始化PCA9685************/
	/***************rpc_pca9685_pwm_out*************/
	rpi_pca9685_pwm_out::pwm_initialize();

	// Set up mixer
	if (initialize_mixer(_mixer_filename) < 0) {
		PX4_ERR("无法初始化通道混合配置文件 Can't loading mixer file");
		return;
	}

	_mixer_group->groups_required(_groups_required);
	// subscribe and set up polling
	subscribe();

	// Start disarmed
	_armed.armed = false;
	_armed.prearmed = false;

	pwm_limit_init(&_pwm_limit);

	// Main loop
	while (!_task_should_exit) {
		int pret = px4_poll(_poll_fds, _poll_fds_num, 10);

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(10000);
			continue;
		}

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_controls_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_controls_topics[i], _controls_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		if (_mixer_group != nullptr) {
			_outputs.timestamp = hrt_absolute_time();
			/* do mixing */
			_outputs.noutputs = _mixer_group->mix(_outputs.output,
							      _outputs.NUM_ACTUATOR_OUTPUTS,
							      NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs;
			     i < _outputs.NUM_ACTUATOR_OUTPUTS;
			     i++) {
				_outputs.output[i] = NAN;
			}

			const uint16_t reverse_mask = 0;
			uint16_t disarmed_pwm[4];
			uint16_t min_pwm[4];
			uint16_t max_pwm[4];

			for (unsigned int i = 0; i < NUM_PWM; i++) {
				disarmed_pwm[i] = _pwm_disarmed;
				min_pwm[i] = _pwm_min;
				max_pwm[i] = _pwm_max;
			}

			uint16_t pwm[4];

			// TODO FIXME: pre-armed seems broken
			pwm_limit_calc(_armed.armed,
				       false/*_armed.prearmed*/,
				       _outputs.noutputs,
				       reverse_mask,
				       disarmed_pwm,
				       min_pwm,
				       max_pwm,
				       _outputs.output,
				       pwm,
				       &_pwm_limit);

			if (_armed.lockdown) {
				send_outputs_pwm(disarmed_pwm);

			} else {
				send_outputs_pwm(pwm);
			}

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
			}

		} else {
			PX4_ERR("Could not mix output! Exiting...");
			_task_should_exit = true;
		}

		bool updated;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		}
	}

	pwm_deinitialize();

	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_controls_subs[i] >= 0) {
			orb_unsubscribe(_controls_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);

	_is_running = false;

}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::start()
{
	ASSERT(_task_handle == -1);
	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pwm_out_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
}
//----------------------------------------------------------------------------//
void rpi_pca9685_pwm_out::usage()
{
	PX4_INFO("usage: pwm_out start [-d pwmdevice] -m mixerfile");
	PX4_INFO("       -d pwmdevice : sysfs device for pwm generation");
	PX4_INFO("                       (default /sys/class/pwm/pwmchip0)");
	PX4_INFO("       -m mixerfile : path to mixerfile");
	PX4_INFO("                       (default ROMFS/px4fmu_common/mixers/quad_x.main.mix)");
	PX4_INFO("       pwm_out stop");
	PX4_INFO("       pwm_out status");
}
//----------------------------------------------------------------------------//
int rpi_pca9685_pwm_out_main(int argc, char **argv)
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];

	} else {
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "d:m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			strncpy(rpi_pca9685_pwm_out::_device, myoptarg, sizeof(rpi_pca9685_pwm_out::_device));
			break;

		case 'm':
			strncpy(rpi_pca9685_pwm_out::_mixer_filename, myoptarg, sizeof(rpi_pca9685_pwm_out::_mixer_filename));
			break;
		}
	}

	// gets the parameters for the esc's pwm
	param_get(param_find("PWM_DISARMED"), &rpi_pca9685_pwm_out::_pwm_disarmed);
	param_get(param_find("PWM_MIN"), &rpi_pca9685_pwm_out::_pwm_min);
	param_get(param_find("PWM_MAX"), &rpi_pca9685_pwm_out::_pwm_max);

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (rpi_pca9685_pwm_out::_is_running) {
			PX4_WARN("pwm_out already running");
			return 1;
		}

		rpi_pca9685_pwm_out::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!rpi_pca9685_pwm_out::_is_running) {
			PX4_WARN("pwm_out is not running");
			return 1;
		}

		rpi_pca9685_pwm_out::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("pwm_out is %s", rpi_pca9685_pwm_out::_is_running ? "running" : "not running");
		return 0;

	} else {
		rpi_pca9685_pwm_out::usage();
		return 1;
	}

	return 0;
}
