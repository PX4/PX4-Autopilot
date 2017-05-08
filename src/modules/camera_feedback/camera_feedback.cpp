/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file camera_feedback.cpp
 *
 * Online and offline geotagging from camera feedback
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

typedef enum : uint8_t {
	CAMERA_FEEDBACK_MODE_NONE = 0,
	CAMERA_FEEDBACK_MODE_TRIGGER,
	CAMERA_FEEDBACK_MODE_PWM
} camera_feedback_mode_t;

class CameraFeedback
{
public:
	/**
	 * Constructor
	 */
	CameraFeedback();

	/**
	 * Destructor, also kills task.
	 */
	~CameraFeedback();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int			start();

	/**
	 * Stop the task.
	 */
	void		stop();

private:

	bool		_task_should_exit;		/**< if true, task should exit */
	int			_main_task;				/**< handle for task */

	int			_trigger_sub;
	int			_lpos_sub;
	int			_gpos_sub;
	int			_att_sub;

	orb_advert_t	_capture_pub;

	param_t			_p_feedback;

	camera_feedback_mode_t _camera_feedback_mode;

	void		task_main();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

};

namespace camera_feedback
{
CameraFeedback	*g_camera_feedback;
}

CameraFeedback::CameraFeedback() :
	_task_should_exit(false),
	_main_task(-1),
	_trigger_sub(-1),
	_lpos_sub(-1),
	_gpos_sub(-1),
	_att_sub(-1),
	_capture_pub(nullptr),
	_camera_feedback_mode(CAMERA_FEEDBACK_MODE_NONE)
{

	// Parameters
	_p_feedback = param_find("CAM_FBACK_MODE");

	param_get(_p_feedback, &_camera_feedback_mode);

}

CameraFeedback::~CameraFeedback()
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
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	camera_feedback::g_camera_feedback = nullptr;
}

int
CameraFeedback::start()
{

	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("camera_feedback",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1600,
					(px4_main_t)&CameraFeedback::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;

}

void
CameraFeedback::stop()
{
	if (camera_feedback::g_camera_feedback != nullptr) {
		delete (camera_feedback::g_camera_feedback);
	}
}


void
CameraFeedback::task_main()
{

	// We only support trigger feedback for now
	// This will later be extended to support hardware feedback from the camera.
	if (_camera_feedback_mode != CAMERA_FEEDBACK_MODE_TRIGGER) {
		return;
	}

	// Polling sources
	_trigger_sub = orb_subscribe(ORB_ID(camera_trigger));
	struct camera_trigger_s trig = {};

	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _trigger_sub;
	fds[0].events = POLLIN;

	// Geotagging subscriptions
	_lpos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_gpos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_local_position_s lpos = {};
	struct vehicle_global_position_s gpos = {};
	struct vehicle_attitude_s att = {};

	bool updated = false;

	while (!_task_should_exit) {

		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			continue;
		}

		/* trigger subscription updated */
		if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);

			/* update geotagging subscriptions */
			orb_check(_gpos_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_global_position), _gpos_sub, &gpos);
			}

			orb_check(_lpos_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_local_position), _lpos_sub, &lpos);
			}

			orb_check(_att_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);
			}

			if (gpos.timestamp == 0 ||
			    lpos.timestamp == 0 ||
			    att.timestamp == 0 ||
			    !lpos.xy_valid) {
				// reject until we have valid data
				continue;
			}

			struct camera_capture_s capture = {};

			// Fill timestamps
			capture.timestamp = trig.timestamp;

			capture.timestamp_utc = trig.timestamp_utc;

			// Fill image sequence
			capture.seq = trig.seq;

			// Fill position data
			capture.lat = gpos.lat;

			capture.lon = gpos.lon;

			capture.alt = gpos.alt;

			capture.ground_distance = lpos.dist_bottom_valid ? lpos.dist_bottom : -1.0f;

			// Fill attitude data
			// TODO : this needs to be rotated by camera orientation or set to gimbal orientation when available
			capture.q[0] = att.q[0];

			capture.q[1] = att.q[1];

			capture.q[2] = att.q[2];

			capture.q[3] = att.q[3];

			// Indicate that no capture feedback from camera is available
			capture.result = -1;

			int instance_id;

			orb_publish_auto(ORB_ID(camera_capture), &_capture_pub, &capture, &instance_id, ORB_PRIO_DEFAULT);

		}

	}

	PX4_INFO("Exiting.");
	_main_task = -1;

}

void
CameraFeedback::task_main_trampoline(int argc, char *argv[])
{
	camera_feedback::g_camera_feedback->task_main();
}

static int usage()
{
	PX4_INFO("usage: camera_feedback {start|stop}\n");
	return 1;
}

extern "C" __EXPORT int camera_feedback_main(int argc, char *argv[]);

int camera_feedback_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_feedback::g_camera_feedback != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_feedback::g_camera_feedback = new CameraFeedback();

		if (camera_feedback::g_camera_feedback == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		camera_feedback::g_camera_feedback->start();
		return 0;
	}

	if (camera_feedback::g_camera_feedback == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_feedback::g_camera_feedback->stop();

	} else {
		return usage();
	}

	return 0;
}