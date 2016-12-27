/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file brkp_detector_main.cpp
 *
 * discrible
 *
 * @author tang liang  <tangliang@qbao.com>
 */

#include <px4_config.h>
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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/brkpoint.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>

#define POSITION_TIMEOUT 1*1000*1000

extern "C" __EXPORT int brkp_detector_main(int argc, char *argv[]);

class BrkpDetector
{
public:
	BrkpDetector();
	~BrkpDetector();

	int start();

private:
	bool _task_should_exit;
	int _main_task;

	int _battery_sub;
	int _land_detector_sub;
	int _global_position_sub;

	void task_main();

	static void task_main_trampoline(int argc, char *argv[]);
};

namespace brkp_detector
{
	BrkpDetector *g_brkp_detector;
}

BrkpDetector::BrkpDetector() :
	_task_should_exit(false),
	_main_task(-1),
	_battery_sub(-1),
	_land_detector_sub(-1),
	_global_position_sub(-1)
{
}

BrkpDetector::~BrkpDetector()
{
	if (_main_task != -1) {
		_task_should_exit = true;
		unsigned int i = 0;

		do {
			usleep(20000);

			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);

	}

	brkp_detector::g_brkp_detector = nullptr;
}

int BrkpDetector::start()
{
	ASSERT(_main_task == -1);

	_main_task = px4_task_spawn_cmd("brkp_detector",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
					(px4_main_t)&BrkpDetector::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("brkp_detector task start failed");
		return -errno;
	}

	return OK;
}

void BrkpDetector::task_main()
{
	bool updated = false;
	bool global_position_valid = false;
	struct battery_status_s battery = {};
	struct vehicle_global_position_s global_position = {};
	struct vehicle_land_detected_s land_detector = {};
	land_detector.landed = true;

	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_land_detector_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	while (!_task_should_exit) {

		orb_check(_land_detector_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_land_detected), _land_detector_sub, &land_detector);
		}

		orb_check(_global_position_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_global_position), _global_position_sub, &global_position);
		}

		if (hrt_absolute_time() - global_position.timestamp > POSITION_TIMEOUT) {
			global_position_valid = false;

		} else if (global_position.timestamp != 0) {
			if (!global_position_valid) {
				global_position_valid = true;
			}
		}

		orb_check(_battery_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

			if (global_position_valid && !land_detector.landed &&battery.warning == battery_status_s::BATTERY_WARNING_LOW) {
				//TODO
			//	struct brkpoint_s bp = {};
			//	bp.lon = 
			//	bp.lat = 
			//	bp.alt = 
			}
		}


		usleep(100000);
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}

void BrkpDetector::task_main_trampoline(int argc, char *argv[])
{
	brkp_detector::g_brkp_detector->task_main();
}

static void usage()
{
	errx(1, "usage: brkp_detector {start|stop}");
}

int brkp_detector_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {
		if (brkp_detector::g_brkp_detector != nullptr) {
			errx(1, "already running");
		}

		brkp_detector::g_brkp_detector = new BrkpDetector;

		if (brkp_detector::g_brkp_detector == nullptr) {
			errx(1, "alloc failed");
		}

		if (brkp_detector::g_brkp_detector->start() != OK) {
			delete brkp_detector::g_brkp_detector;
			brkp_detector::g_brkp_detector = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (brkp_detector::g_brkp_detector == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete brkp_detector::g_brkp_detector;
		brkp_detector::g_brkp_detector = nullptr;

	} else {
		usage();
	}

	return 0;
}
