/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

//#include <px4_config.h>
//#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <mavlink_log.h>

#include "McPosControl.hpp"
//#include <errno.h>
//#include <math.h>
//#include <poll.h>
//#include <drivers/drv_hrt.h>
//#include <arch/board/board.h>


/*#include <float.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>
*/

static volatile bool thread_should_exit = true;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */
static int control_task = -1;             /**< Handle of deamon task / thread */


/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */

/**
 * Daemon management function
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

/**
 * Main thread of daemon
 */
int mc_pos_control_main_thread(int argc, char *argv[]);


namespace pos_control
{
McPosControl *g_control;
}


/**
 * Function implementations
 */


int mc_pos_control_main(int argc, char *argv[])
{



	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}


	if (!strcmp(argv[1], "start")) {


		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		pos_control::g_control = new McPosControl;

		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}


		ASSERT(control_task == -1);

		thread_should_exit = false;

		control_task = px4_task_spawn_cmd("mc_pos_control",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_MAX - 5,
						  1900,
						  mc_pos_control_main_thread,
						  nullptr);


		if (control_task < 0) {
			warn("task start failed");
			return -errno;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		thread_should_exit = true;

		if (control_task != -1) {
			/* task wakes up every 100ms or so at the longest */
			/* wait for a second for the task to quit at our request */
			unsigned i = 0;

			do {
				/* wait 20ms */
				usleep(20000);

				/* if we have given up, kill it */
				if (++i > 50) {
					px4_task_delete(control_task);
					break;
				}
			} while (control_task != -1);
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr; \
		control_task = -1;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}


	warnx("unrecognized command");
	return 1;
}



int mc_pos_control_main_thread(int argc, char *argv[])
{

	while (!thread_should_exit) {
		pos_control::g_control->run();
	}

	orb_advert_t	mavlink_log_pub;
	mavlink_log_info(&mavlink_log_pub, "[mpc] stopped");

	return 0;
}
