/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file bottle_drop.cpp
 *
 * Bottle drop module for Outback Challenge 2014, Team Swiss Fang
 *
 * @author Dominik Juchli <juchlid@ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>


/**
 * bottle_drop app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int bottle_drop_main(int argc, char *argv[]);

class BottleDrop  
{
public:
	/**
	 * Constructor
	 */
	BottleDrop();

	/**
	 * Destructor, also kills task.
	 */
	~BottleDrop();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */
	void		status();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */
	int		_mavlink_fd;
	
	int		_command_sub;
	struct vehicle_command_s	_command;

	orb_advert_t	_actuator_pub;
	struct actuator_controls_s _actuators;

	bool		_open_door;
	bool		_drop;
	hrt_abstime	_doors_opened;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
};

namespace bottle_drop
{
BottleDrop	*g_bottle_drop;
}

BottleDrop::BottleDrop() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1),
	_command_sub(-1),
	_command({}),
	_actuator_pub(-1),
	_actuators({}),
	_open_door(false),
	_drop(false),
	_doors_opened(0)
{
}

BottleDrop::~BottleDrop()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	bottle_drop::g_bottle_drop = nullptr;
}

int
BottleDrop::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = task_spawn_cmd("bottle_drop",
			       SCHED_DEFAULT,
			       SCHED_PRIORITY_MAX - 5,
			       2048,
			       (main_t)&BottleDrop::task_main_trampoline,
			       nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


void
BottleDrop::status()
{
	warnx("Doors: %s", _open_door ? "OPEN" : "CLOSED");
	warnx("Dropping: %s", _drop ? "YES" : "NO");
}

void
BottleDrop::task_main()
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[bottle_drop] started");

	_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			handle_command(&_command);
		}

		/* update door actuators */
		if (_open_door) {
			_actuators.control[0] = -1.0f;
			_actuators.control[1] = 1.0f;

			if (_doors_opened == 0) {
				_doors_opened = hrt_absolute_time();
			}
		} else {
			_actuators.control[0] = 0.5f;
			_actuators.control[1] = -0.5f;

			_doors_opened = 0;
		}

		/* update drop actuator, wait 0.5s until the doors are open before dropping */
		if (_drop && _doors_opened != 0 && hrt_elapsed_time(&_doors_opened) > 500000) {
			_actuators.control[2] = 0.5f;
		} else {
			_actuators.control[2] = -0.5f;
		}

		/* 2s after drop, reset and close everything again */
		if (_drop && _doors_opened != 0 && hrt_elapsed_time(&_doors_opened) > 2000000) {
			_open_door = false;
			_drop = false;
		}

		/* lazily publish _actuators only once available */
		if (_actuator_pub > 0) {
			orb_publish(ORB_ID(actuator_controls_1), _actuator_pub, &_actuators);

		} else {
			_actuator_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators);
		}
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void
BottleDrop::handle_command(struct vehicle_command_s *cmd)
{
	switch (cmd->command) {
	case VEHICLE_CMD_CUSTOM_0:

		/*
		 * param1 and param2 set to 1: open and drop
		 * param1 set to 1: open
		 * else: close (and don't drop)
		 */
		if (cmd->param1 > 0.5f && cmd->param2 > 0.5f) {
			_open_door = true;
			_drop = true;
			mavlink_log_info(_mavlink_fd, "#audio: drop bottle");
		}
		else if (cmd->param1 > 0.5f) {
			_open_door = true;
			_drop = false;
			mavlink_log_info(_mavlink_fd, "#audio: open doors");
		} else {
			_open_door = false;
			_drop = false;
			mavlink_log_info(_mavlink_fd, "#audio: close doors");
		}

		answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
		break;
	default:
		break;
	}
}

void
BottleDrop::answer_command(struct vehicle_command_s *cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(_mavlink_fd, "#audio: command denied: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(_mavlink_fd, "#audio: command failed: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command temporarily rejected: %u", cmd->command);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(_mavlink_fd, "#audio: command unsupported: %u", cmd->command);
		break;

	default:
		break;
	}
}

void
BottleDrop::task_main_trampoline(int argc, char *argv[])
{
	bottle_drop::g_bottle_drop->task_main();
}

static void usage()
{
	errx(1, "usage: bottle_drop {start|stop|status}");
}

int bottle_drop_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (bottle_drop::g_bottle_drop != nullptr) {
			errx(1, "already running");
		}

		bottle_drop::g_bottle_drop = new BottleDrop;

		if (bottle_drop::g_bottle_drop == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != bottle_drop::g_bottle_drop->start()) {
			delete bottle_drop::g_bottle_drop;
			bottle_drop::g_bottle_drop = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (bottle_drop::g_bottle_drop == nullptr)
		errx(1, "not running");

	if (!strcmp(argv[1], "stop")) {
		delete bottle_drop::g_bottle_drop;
		bottle_drop::g_bottle_drop = nullptr;

	} else if (!strcmp(argv[1], "status")) {
		bottle_drop::g_bottle_drop->status();
	} else {
		usage();
	}

	return 0;
}
