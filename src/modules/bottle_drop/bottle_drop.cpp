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
#include <uORB/topics/vehicle_global_position.h>
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
	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_main_task;			/**< task handle for sensor task */

	int		_mavlink_fd;
};

namespace bottle_drop
{
BottleDrop	*g_bottle_drop;
}

BottleDrop::BottleDrop() :

	_task_should_exit(false),
	_main_task(-1),
	_mavlink_fd(-1)
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

int BottleDrop::start()
{
}


void BottleDrop::status()
{
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
