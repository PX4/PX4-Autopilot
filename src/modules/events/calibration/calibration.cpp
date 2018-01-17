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
 * @file calibration.cpp
 * Implements a framework to spawn calibration routines as px4 command
 *
 * @author Sugnan Prabhu S <sugnan.prabhu.s@intel.com>
 */

#include <string.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <uORB/topics/calibration_status.h>

#include "calibration.h"

class Calibration;

namespace calibration
{
Calibration 	*instance = nullptr;
}

class Calibration
{
public:
	Calibration(const char *name, calibration_routine calib_routine);
	~Calibration() = default;
	int start();
	static void do_calibration(int argc, char *argv[]);
	const char *get_task_name() { return task_name;}

private:
	void publish_led_control(int result);
	int	_control_task = -1;		// task handle for task
	char 	task_name[TASK_NAME_LEN];
	static calibration_routine *routine;
};

calibration_routine *Calibration::routine = nullptr;

Calibration::Calibration(const char *taskname, calibration_routine calib_routine)
{
	strncpy(this->task_name, taskname, sizeof(this->task_name));
	routine = calib_routine;
}

void Calibration::do_calibration(int argc, char *argv[])
{
	struct calibration_status_s calib_status = {};
	orb_advert_t _mavlink_log_pub = nullptr;
	orb_advert_t _calib_status_pub = nullptr;

	if (routine(&_mavlink_log_pub) == PX4_OK) {
		calib_status.result = calibration_status_s::CALIBRATION_OK;

	} else {
		calib_status.result = calibration_status_s::CALIBRATION_FAILED;
	}

	/* publish result */
	_calib_status_pub = orb_advertise(ORB_ID(calibration_status), &calib_status);

	orb_unadvertise(_calib_status_pub);
	orb_unadvertise(_mavlink_log_pub);
	delete calibration::instance;
	calibration::instance = nullptr;
}

int Calibration::start()
{
	ASSERT(_control_task == -1);
	_control_task = px4_task_spawn_cmd(this->task_name,
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2500,
					   (px4_main_t)&Calibration::do_calibration,
					   nullptr);

	if (_control_task < 0) {
		PX4_ERR("%s start failed", this->task_name);
		delete calibration::instance;
		calibration::instance = nullptr;
		publish_led_control(PX4_ERROR);
		return calibration_status_s::CALIBRATION_FAILED;
	}

	publish_led_control(PX4_OK);
	return calibration_status_s::CALIBRATION_OK;
}

void Calibration::publish_led_control(int result)
{
	static led_control_s led_control = {};
	orb_advert_t _led_control_pub = nullptr;

	if (result == PX4_OK) {
		led_control.color = led_control_s::COLOR_GREEN;

	} else {
		led_control.color = led_control_s::COLOR_RED;
	}

	led_control.timestamp = hrt_absolute_time();
	led_control.mode = led_control_s::MODE_BLINK_FAST;
	led_control.led_mask = 0xff;
	led_control.priority = 0;
	led_control.num_blinks = 0;

	_led_control_pub = orb_advertise_queue(ORB_ID(led_control), &led_control, LED_UORB_QUEUE_LENGTH);
	orb_unadvertise(_led_control_pub);
}

int run_calibration(const char *task_name, calibration_routine routine)
{
	if (calibration::instance) {
		PX4_ERR("%s is currently active", calibration::instance->get_task_name());
		return calibration_status_s::CALIBRATION_ACTIVE;

	} else {
		calibration::instance = new Calibration(task_name, routine);
	}

	if (calibration::instance == nullptr) {
		PX4_ERR("calibration instance allocation failed");
		return calibration_status_s::CALIBRATION_FAILED;
	}

	return calibration::instance->start();
}
