/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file gimbal.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Matosov <anton.matosov@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 *
 * Driver to control a gimbal - relies on input via telemetry or RC
 * and output via the standardized control group #2 and a mixer.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>

#include <board_config.h>
#include <mathlib/math/test/test.hpp>
#include <mathlib/math/Quaternion.hpp>

/* Configuration Constants */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define GIMBAL_DEVICE_PATH	"/dev/gimbal"

#define GIMBAL_UPDATE_INTERVAL (5 * 1000)

#define GIMBALIOCATTCOMPENSATE		1

class Gimbal : public device::CDev
{
public:
	Gimbal();
	virtual ~Gimbal();

	virtual int 			init();

	virtual ssize_t			read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	int				_vehicle_command_sub;
	int				_att_sub;
	int				_vehicle_control_mode_sub;
	int				_manual_control_sub;
	int				_params_sub;

	bool				_attitude_compensation_roll;
	bool				_attitude_compensation_pitch;
	bool				_attitude_compensation_yaw;
	bool				_initialized;
	bool				_control_cmd_set;
	bool				_config_cmd_set;
	int					_mount_mode; // see MAV_MOUNT_MODE

	orb_advert_t			_actuator_controls_2_topic;

	struct vehicle_command_s _control_cmd;
	struct vehicle_command_s _config_cmd;
	struct manual_control_setpoint_s _manual_control;
	struct vehicle_control_mode_s _control_mode;

	struct {
		int aux_mnt_chn;
		int use_mnt;
	} _parameters;

	struct {
		param_t aux_mnt_chn;
		param_t use_mnt;
	} _params_handles;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Update parameters
	*/
	void				update_params();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gimbal_main(int argc, char *argv[]);

Gimbal::Gimbal() :
	CDev("Gimbal", GIMBAL_DEVICE_PATH),
	_vehicle_command_sub(-1),
	_att_sub(-1),
	_vehicle_control_mode_sub(-1),
	_manual_control_sub(-1),
	_params_sub(-1),
	_attitude_compensation_roll(true),
	_attitude_compensation_pitch(true),
	_attitude_compensation_yaw(true),
	_initialized(false),
	_actuator_controls_2_topic(nullptr)
{
	// disable debug() calls
	_debug_enabled = false;

	_params_handles.aux_mnt_chn = param_find("GMB_AUX_MNT_CHN");
	_params_handles.use_mnt = param_find("GMB_USE_MNT");
	update_params();

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

Gimbal::~Gimbal()
{
	/* make sure we are truly inactive */
	stop();

	::close(_vehicle_command_sub);
}

int
Gimbal::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK) {
		goto out;
	}

	start();
	ret = OK;

out:
	return ret;
}

int
Gimbal::probe()
{
	return OK;
}

int
Gimbal::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case GIMBALIOCATTCOMPENSATE:
		_attitude_compensation_roll = (arg != 0);
		_attitude_compensation_pitch = (arg != 0);
		_attitude_compensation_yaw = (arg != 0);
		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
Gimbal::read(struct file *filp, char *buffer, size_t buflen)
{
	return 0;
}

void
Gimbal::start()
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&Gimbal::cycle_trampoline, this, 1);
}

void
Gimbal::stop()
{
	work_cancel(LPWORK, &_work);
}

void
Gimbal::cycle_trampoline(void *arg)
{
	Gimbal *dev = static_cast<Gimbal *>(arg);

	dev->cycle();
}

void
Gimbal::update_params()
{
	param_get(_params_handles.aux_mnt_chn, &_parameters.aux_mnt_chn);
	param_get(_params_handles.use_mnt, &_parameters.use_mnt);
}

void
Gimbal::cycle()
{
	if (!_initialized) {
		/* get subscription handles */
		_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
		_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
		_vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
		_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
		_params_sub = orb_subscribe(ORB_ID(parameter_update));

		/* get a publication handle on actuator output topic */
		struct actuator_controls_s zero_report;
		memset(&zero_report, 0, sizeof(zero_report));
		zero_report.timestamp = hrt_absolute_time();
		_actuator_controls_2_topic = orb_advertise(ORB_ID(actuator_controls_2), &zero_report);

		if (_actuator_controls_2_topic == nullptr) {
			warnx("advert err");
		}

		_mount_mode = vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT;

		_initialized = true;
	}

	bool	updated = false;

	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	float out_mount_mode = 0.0f;

	/* Parameter update */
	bool params_updated;

	orb_check(_params_sub, &params_updated);

	if (params_updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update); // XXX: is this actually necessary?

		update_params();
	}

	/* Control mode update */
	bool vehicle_control_mode_updated;

	orb_check(_vehicle_control_mode_sub, &vehicle_control_mode_updated);

	if (vehicle_control_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_control_mode);
	}

	/* Check attitude */
	struct vehicle_attitude_s att;

	orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);

	if (_attitude_compensation_roll) {
		roll = 1.0f / M_PI_F * -att.roll;
		updated = true;
	}

	if (_attitude_compensation_pitch) {
		pitch = 1.0f / M_PI_F * -att.pitch;
		updated = true;
	}

	if (_attitude_compensation_yaw) {
		yaw = 1.0f / M_PI_F * att.yaw;
		updated = true;
	}

	/* Check manual input */
	bool manual_updated;

	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual_control);

		/* only check manual input for mount mode when not in offboard and aux chan is configured */
		if (!_control_mode.flag_control_offboard_enabled && _parameters.aux_mnt_chn > 0) {
			float aux = 0.0f;

			switch (_parameters.aux_mnt_chn) {
			case 1:
				aux = _manual_control.aux1;
				break;

			case 2:
				aux = _manual_control.aux2;
				break;

			case 3:
				aux = _manual_control.aux3;
				break;
			}

			if (aux < 0.0f && _mount_mode != vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT) {
				_mount_mode = vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT;
				updated = true;
			}

			if (aux > 0.0f && _mount_mode != vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL) {
				_mount_mode = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;
				updated = true;
			}
		}
	}

	/* Check command input */
	struct vehicle_command_s cmd;

	bool cmd_updated;

	orb_check(_vehicle_command_sub, &cmd_updated);

	if (cmd_updated) {

		orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL
		    || cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT) {

			_control_cmd = cmd;
			_control_cmd_set = true;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {

			_config_cmd = cmd;
			_config_cmd_set = true;

		}

	}

	if (_config_cmd_set) {

		_config_cmd_set = false;

		_attitude_compensation_roll = (int)_config_cmd.param2 == 1;
		_attitude_compensation_pitch = (int)_config_cmd.param3 == 1;
		_attitude_compensation_yaw = (int)_config_cmd.param4 == 1;

		/* only check commanded mount mode when in offboard */
		if (_control_mode.flag_control_offboard_enabled &&
		    fabsf(_config_cmd.param1 - _mount_mode) > FLT_EPSILON) {
			_mount_mode = int(_config_cmd.param1 + 0.5f);
			updated = true;
		}

	}

	if (_control_cmd_set) {

		unsigned mountMode = _control_cmd.param7;
		DEVICE_DEBUG("control_cmd: %d, mountMode %d | param1: %8.4f param2: %8.4f", _control_cmd.command, mountMode,
			     (double)_control_cmd.param1, (double)_control_cmd.param2);

		if (_control_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL &&
		    mountMode == vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING) {
			/* Convert to range -1 ... 1, which corresponds to -180deg ... 180deg */
			roll += 1.0f / M_PI_F * M_DEG_TO_RAD_F * _control_cmd.param1;
			pitch += 1.0f / M_PI_F * M_DEG_TO_RAD_F * _control_cmd.param2;
			yaw += 1.0f / M_PI_F * M_DEG_TO_RAD_F * _control_cmd.param3;

			updated = true;
		}

		if (_control_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT &&
		    mountMode == vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING) {
			float gimbalDirectionQuat[] = {_control_cmd.param1, _control_cmd.param2, _control_cmd.param3, _control_cmd.param4};
			math::Vector<3> gimablDirectionEuler = math::Quaternion(gimbalDirectionQuat).to_dcm().to_euler();

			roll += gimablDirectionEuler(0);
			pitch += gimablDirectionEuler(1);
			yaw += gimablDirectionEuler(2);

			updated = true;
		}

	}

	/* consider mount mode if parameter is set */
	if (_parameters.use_mnt > 0) {
		switch (_mount_mode) {
		case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
			out_mount_mode = -1.0f;
			roll = 0.0f;
			pitch = 0.0f;
			yaw = 0.0f;
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
		case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
		case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
		case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
			out_mount_mode = 1.0f;
			break;

		default:
			out_mount_mode = -1.0f;
		}
	}

	if (updated) {

		struct actuator_controls_s controls;

		// DEVICE_DEBUG("publishing | roll: %8.4f pitch: %8.4f yaw: %8.4f", (double)roll, (double)pitch, (double)yaw);

		/* fill in the final control values */
		controls.timestamp = hrt_absolute_time();
		controls.control[0] = roll;
		controls.control[1] = pitch;
		controls.control[2] = yaw;
		//controls.control[3] = ; // camera shutter
		controls.control[4] = out_mount_mode;

		/* publish it */
		orb_publish(ORB_ID(actuator_controls_2), _actuator_controls_2_topic, &controls);

	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(LPWORK,
		   &_work,
		   (worker_t)&Gimbal::cycle_trampoline,
		   this,
		   USEC2TICK(GIMBAL_UPDATE_INTERVAL));
}

void
Gimbal::print_info()
{
}

/**
 * Local functions in support of the shell command.
 */
namespace gimbal
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

Gimbal	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new Gimbal();

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd = open(GIMBAL_DEVICE_PATH, O_RDONLY);

	if (ioctl(fd, GIMBALIOCATTCOMPENSATE, 1) < 0) {
		err(1, "failed enabling compensation");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GIMBAL_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	// if (ioctl(fd, GIMBALIOCATTCOMPENSATE, 1) < 0) {
	// 	err(1, "failed enabling compensation");
	// }

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
gimbal_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		gimbal::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		gimbal::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		gimbal::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		gimbal::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		gimbal::info();
	}

	errx(1, "unrecognized command, try 'start', 'stop', 'reset', 'test' or 'info'");
}
