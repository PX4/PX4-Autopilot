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
#include "linux_pca9685.h"

using namespace linux_pca9685;
//--------------------------------------------------------------------------//
int linux_pca9685::mixer_control_callback(uintptr_t handle,
		uint8_t control_group, uint8_t control_index, float &input) {
	const actuator_controls_s *controls = (actuator_controls_s *) handle;
	input = controls[control_group].control[control_index];

	return 0;
}

//----------------------------------------------------------------------------//
int linux_pca9685::initialize_mixer(const char *mixer_filename) {
	char buf[4096];
	unsigned buflen = sizeof(buf);
	memset(buf, '\0', buflen);

	_mixer_group = new MixerGroup(mixer_control_callback,
			(uintptr_t) &_controls);

	// PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

	if (load_mixer_file(mixer_filename, buf, buflen) == 0) {
		if (_mixer_group->load_from_buf(buf, buflen) == 0) {
			PX4_INFO("Loaded mixer from file %s", mixer_filename);
			return 0;

		} else {
			PX4_ERR("Unable to parse from mixer config file %s",
					mixer_filename);
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
//-----------------------------------------------------------------------------//
int linux_pca9685::open_device() {
	/*configure pca9685*/
	if (0 > (device_fd = open(device_path, O_RDWR))) {
		PX4_ERR("Couldn't open pca9685 on %s,errno: %d .", device_path, errno);
		close(device_fd);
		return -1;
	}
	if (0 > ioctl(device_fd, I2C_SLAVE, PCA_9685_ADDR)) {
		PX4_ERR("Couldnn't set pca9685 as slave,errno: %d .", errno);
		close(device_fd);
		return -1;
	}
	return 0;
}

//----------------------------------------------------------------------------//
int linux_pca9685::pwm_initialize() {
	/**************初始化PCA9685开始*************/
	/**************PCA965 initializing********/
	/*configure pca9685*/
	if(0>linux_pca9685::open_device())
		return -1;
	//Normal mode
	if (-1 == write_byte(device_fd, MODE1, 0x00)) {
		close(device_fd);
		return -1;
	}
	if (-1 == write_byte(device_fd, MODE2, 0x04)) {
		close(device_fd);
		return -1;
	}

	//set frequency,200HZ
	uint8_t prescale = (CLOCK_FREQ / MAX_PWM_RES / 200) - 1;
	uint8_t oldmode = 0;
	uint8_t newmode = (0 & 0x7F) | 0x10;    //sleep
	write_byte(device_fd, MODE1, newmode);        // go to sleep
	write_byte(device_fd, PRE_SCALE, prescale);
	write_byte(device_fd, MODE1, oldmode);
	usleep(10 * 1000);
	write_byte(device_fd, MODE1, oldmode | 0x80);
	/**************初始化PCA9685结束************/
	/**************PCA965 initialized********/
	close(device_fd);
	return 0;
}
//----------------------------------------------------------------------------//
void linux_pca9685::pwm_deinitialize() {
	//Set output pwm as minmux pwm and reset
	if(0>linux_pca9685::open_device())
		return;

	uint16_t pwm[NUM_PWM];
	for (int start = 0; start < NUM_PWM; ++start) {
		pwm[start] = (uint16_t) _pwm_disarmed;
	}
	linux_pca9685::send_outputs_pwm(pwm);
	write_byte(device_fd, MODE1, 0x00);
	write_byte(device_fd, MODE2, 0x04);
	close(device_fd);
}
//----------------------------------------------------------------------------//
void linux_pca9685::send_outputs_pwm(const uint16_t *pwm) {
	/*************向PCA9685发送数据*************/
	/*************send pwm signal to pca9685*************/
	if(0>linux_pca9685::open_device())
		return;
	int i;

	for (i = 0; i < NUM_PWM; ++i) {
		write_byte(device_fd, LED0_ON_L + LED_MULTIPLYER * i, 0 & 0xFF);
		write_byte(device_fd, LED0_ON_H + LED_MULTIPLYER * i, 0 >> 8);
		write_byte(device_fd, LED0_OFF_L + LED_MULTIPLYER * i,
				*(pwm + i) & 0xFF);
		write_byte(device_fd, LED0_OFF_H + LED_MULTIPLYER * i, *(pwm + i) >> 8);
	}
	close(device_fd);
}
//----------------------------------------------------------------------------//
void linux_pca9685::subscribe() {
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	/* set up ORB topic names */
	_controls_topics[0] = ORB_ID(actuator_controls_0);
	_controls_topics[1] = ORB_ID(actuator_controls_1);
	_controls_topics[2] = ORB_ID(actuator_controls_2);
	_controls_topics[3] = ORB_ID(actuator_controls_3);

// Subscribe for orb topics
	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS;
			i++) {
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
void linux_pca9685::task_main(int argc, char *argv[]) {
	_is_running = true;

	/***************初始化PCA9685************/
	/***************Inialize pca9685*************/
	linux_pca9685::pwm_initialize();

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

		for (uint8_t i = 0;
				i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_controls_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_controls_topics[i], _controls_subs[i],
							&_controls[i]);
				}

				poll_id++;
			}
		}

		if (nullptr != _mixer_group) {
			_outputs.timestamp = hrt_absolute_time();
			/* do mixing */
			_outputs.noutputs = _mixer_group->mix(_outputs.output,
					_outputs.NUM_ACTUATOR_OUTPUTS,
					NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs;
					i < _outputs.NUM_ACTUATOR_OUTPUTS; i++) {
				_outputs.output[i] = NAN;
			}

			const uint16_t reverse_mask = 0;
			uint16_t disarmed_pwm[NUM_PWM];
			uint16_t min_pwm[NUM_PWM];
			uint16_t max_pwm[NUM_PWM];

			for (unsigned int i = 0; i < NUM_PWM; i++) {
				disarmed_pwm[i] = _pwm_disarmed;
				min_pwm[i] = _pwm_min;
				max_pwm[i] = _pwm_max;
			}

			uint16_t pwm[4];

			// TODO FIXME: pre-armed seems broken
			pwm_limit_calc(_armed.armed,
			false /*_armed.prearmed*/, _outputs.noutputs, reverse_mask,
					disarmed_pwm, min_pwm, max_pwm, _outputs.output, pwm,
					&_pwm_limit);

			if (_armed.lockdown) {
				send_outputs_pwm(disarmed_pwm);
			} else {
				send_outputs_pwm(pwm);
			}

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs),
						&_outputs);
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

	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS;
			i++) {
		if (_controls_subs[i] >= 0) {
			orb_unsubscribe(_controls_subs[i]);
		}
	}

	orb_unsubscribe(_armed_sub);

	_is_running = false;

}
//----------------------------------------------------------------------------//
void linux_pca9685::task_main_trampoline(int argc, char *argv[]) {
	task_main(argc, argv);
}
//----------------------------------------------------------------------------//
void linux_pca9685::start() {
	ASSERT(_task_handle == -1);
	_task_should_exit = false;
	/* init pca9685 device*/
	if (-1 == linux_pca9685::pwm_initialize()) {
		return;
	}
	/* start the task */
	_task_handle = px4_task_spawn_cmd("pwm_out_main", SCHED_DEFAULT,
	SCHED_PRIORITY_MAX, 1500, (px4_main_t) &task_main_trampoline, nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return;
	}

}
//---------------------------------------------------------------------------------------------------------------------//
int linux_pca9685::write_byte(int fd, uint8_t address, uint8_t data) {
	uint8_t buff[2];
	buff[0] = address;
	buff[1] = data;
	if (2 != write(fd, buff, sizeof(buff))) {
		PX4_ERR("Faild to write data into PCA9685, errno: %d", errno);
		usleep(5000);
		return -1;
	}
	return 0;
}
//----------------------------------------------------------------------------//
void linux_pca9685::stop() {
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}
	_task_handle = -1;
}
//----------------------------------------------------------------------------//
void linux_pca9685::usage() {
	PX4_INFO(
			"usage: linux_pca9685 start [-d pca9685 device] -m mixerfile --motors 4");
	PX4_INFO("       -d pwmdevice : pca9685 device for pwm generation");
	PX4_INFO("                       (default /sys/class/pwm/pwmchip0)");
	PX4_INFO("       -m mixerfile : path to mixerfile");
	PX4_INFO("       --motors quantity of motors : 4 is default");
	PX4_INFO(
			"                       (default ROMFS/px4fmu_common/mixers/quad_x.main.mix)");
	PX4_INFO("       linux_pca9685 stop");
	PX4_INFO("       linux_pca9685 status");
}
//----------------------------------------------------------------------------//
int linux_pca9685_main(int argc, char **argv) {

	int command = -1;  //指令 0:start,1:stop 2:status
	PX4_WARN("Load command from console");
	//读取控制台参数
	int start;
	for (start = 0; start < argc; ++start) {
		if (0 == strcmp("start", argv[start])) {
			command = 0;
			continue;
		}

		if (0 == strcmp("stop", argv[start])) {
			command = 1;
			continue;
		}

		if (0 == strcmp("status", argv[start])) {
			command = 2;
			continue;
		}

		if (0 == strcmp("-d", argv[start])) {
			if(argc<=start+1)
				continue;
			strncpy(device_path, argv[start + 1], sizeof(device_path));
			continue;
		}

		if (0 == strcmp("-m", argv[start])) {
			if(argc<=start+1)
				continue;
			strncpy(_mixer_filename,argv[start+1],
					sizeof(_mixer_filename));
			continue;
		}

		if (0 == strcmp("--motors", argv[start])) {
			if(argc<=start+1)
				continue;
			NUM_PWM = atoi(argv[start+1]);
			if (NUM_PWM > 8 || NUM_PWM < 4)
				NUM_PWM = 4;
			continue;
		}
	}


	//如果没有接收到指令，则退出
	if (-1 == command) {
		linux_pca9685::usage();
		return 1;
	}
	PX4_WARN("Load param from rootfs\n");
	// gets the parameters for the esc's pwm
	param_get(param_find("PWM_DISARMED"), &linux_pca9685::_pwm_disarmed);
	param_get(param_find("PWM_MIN"), &linux_pca9685::_pwm_min);
	param_get(param_find("PWM_MAX"), &linux_pca9685::_pwm_max);

	// 处理命令
	switch (command) {
	case 0: //start
		if (linux_pca9685::_is_running) {
			PX4_WARN("linux_pca9685 already running");
			return 1;
		}
		linux_pca9685::start();
		break;
	case 1: //stop;
		if (!linux_pca9685::_is_running) {
			PX4_WARN("linux_pca9685 is not running");
			return 1;
		}
		linux_pca9685::stop();
		break;
	case 2: //status
		PX4_WARN("linux_pca9685 is %s",
				linux_pca9685::_is_running ? "running" : "not running");
		break;

	}
	return 0;
}
