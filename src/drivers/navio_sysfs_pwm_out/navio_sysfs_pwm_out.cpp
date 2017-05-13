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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <cmath>	// NAN

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_load.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

namespace navio_sysfs_pwm_out
{
static px4_task_t _task_handle = -1;
volatile bool _task_should_exit = false;
static bool _is_running = false;

static const int NUM_PWM = 4;
static char _device[64] = "/sys/class/pwm/pwmchip0";
static int  _pwm_fd[NUM_PWM];

static const int FREQUENCY_PWM = 400;
static char _mixer_filename[64] = "ROMFS/px4fmu_common/mixers/quad_x.main.mix";

// subscriptions
int     _controls_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
int     _armed_sub;

// publications
orb_advert_t    _outputs_pub = nullptr;
orb_advert_t    _rc_pub = nullptr;

// topic structures
actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
orb_id_t 			_controls_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
actuator_outputs_s  _outputs;
actuator_armed_s    _armed;

// polling
uint8_t _poll_fds_num = 0;
px4_pollfd_struct_t _poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

// control groups related
uint32_t	_groups_required = 0;
uint32_t	_groups_subscribed = 0;

pwm_limit_t     _pwm_limit;

// esc parameters
int32_t _pwm_disarmed;
int32_t _pwm_min;
int32_t _pwm_max;

MixerGroup *_mixer_group = nullptr;

void usage();

void start();

void stop();

int pwm_write_sysfs(char *path, int value);

int pwm_initialize(const char *device);

void pwm_deinitialize();

void send_outputs_pwm(const uint16_t *pwm);

void task_main_trampoline(int argc, char *argv[]);

void subscribe();

void task_main(int argc, char *argv[]);

/* mixer initialization */
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);


int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;
	input = controls[control_group].control[control_index];

	return 0;
}


int initialize_mixer(const char *mixer_filename)
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

int pwm_write_sysfs(char *path, int value)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);
	int n;
	char data[16];

	if (fd == -1) {
		return -errno;
	}

	n = ::snprintf(data, sizeof(data), "%u", value);

	if (n > 0) {
		n = ::write(fd, data, n);	// This n is not used, but to avoid a compiler error.
	}

	::close(fd);

	return 0;
}

int pwm_initialize(const char *device)
{
	int i;
	char path[128];

	for (i = 0; i < NUM_PWM; ++i) {
		::snprintf(path, sizeof(path), "%s/export", device);

		if (pwm_write_sysfs(path, i) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/enable", device, i);

		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/period", device, i);

		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}

	for (i = 0; i < NUM_PWM; ++i) {
		::snprintf(path, sizeof(path), "%s/pwm%u/duty_cycle", device, i);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

void pwm_deinitialize()
{
	for (int i = 0; i < NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

void send_outputs_pwm(const uint16_t *pwm)
{
	int n;
	char data[16];

	//convert this to duty_cycle in ns
	for (unsigned i = 0; i < NUM_PWM; ++i) {
		n = ::snprintf(data, sizeof(data), "%u", pwm[i] * 1000);
		n = ::write(_pwm_fd[i], data, n);	// This n is not used, but to avoid a compiler error.
	}
}

void subscribe()
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

void task_main(int argc, char *argv[])
{
	_is_running = true;

	if (pwm_initialize(_device) < 0) {
		PX4_ERR("Failed to initialize PWM.");
		return;
	}

	// Set up mixer
	if (initialize_mixer(_mixer_filename) < 0) {
		PX4_ERR("Mixer initialization failed.");
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

			for (unsigned int i = 0; i < 4; i++) {
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

			if (_armed.lockdown || _armed.manual_lockdown) {
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

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
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

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: pwm_out start [-d pwmdevice] -m mixerfile");
	PX4_INFO("       -d pwmdevice : sysfs device for pwm generation");
	PX4_INFO("                       (default /sys/class/pwm/pwmchip0)");
	PX4_INFO("       -m mixerfile : path to mixerfile");
	PX4_INFO("                       (default ROMFS/px4fmu_common/mixers/quad_x.main.mix)");
	PX4_INFO("       pwm_out stop");
	PX4_INFO("       pwm_out status");
}

} // namespace navio_sysfs_pwm_out

/* driver 'main' command */
extern "C" __EXPORT int navio_sysfs_pwm_out_main(int argc, char *argv[]);

int navio_sysfs_pwm_out_main(int argc, char *argv[])
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
			strncpy(navio_sysfs_pwm_out::_device, myoptarg, sizeof(navio_sysfs_pwm_out::_device));
			break;

		case 'm':
			strncpy(navio_sysfs_pwm_out::_mixer_filename, myoptarg, sizeof(navio_sysfs_pwm_out::_mixer_filename));
			break;
		}
	}

	// gets the parameters for the esc's pwm
	param_get(param_find("PWM_DISARMED"), &navio_sysfs_pwm_out::_pwm_disarmed);
	param_get(param_find("PWM_MIN"), &navio_sysfs_pwm_out::_pwm_min);
	param_get(param_find("PWM_MAX"), &navio_sysfs_pwm_out::_pwm_max);

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (navio_sysfs_pwm_out::_is_running) {
			PX4_WARN("pwm_out already running");
			return 1;
		}

		navio_sysfs_pwm_out::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!navio_sysfs_pwm_out::_is_running) {
			PX4_WARN("pwm_out is not running");
			return 1;
		}

		navio_sysfs_pwm_out::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("pwm_out is %s", navio_sysfs_pwm_out::_is_running ? "running" : "not running");
		return 0;

	} else {
		navio_sysfs_pwm_out::usage();
		return 1;
	}

	return 0;
}
