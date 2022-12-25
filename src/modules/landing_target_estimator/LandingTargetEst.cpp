/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file LandingTargetEst.cpp
 * Landing target estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include "LandingTargetEst.h"


namespace landing_target_estimator
{

static constexpr uint32_t ltest_pos_UPDATE_RATE_HZ = 50;
static constexpr uint32_t ltest_yaw_UPDATE_RATE_HZ = 50;

LandingTargetEst::LandingTargetEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

}

LandingTargetEst::~LandingTargetEst()
{
	perf_free(_cycle_perf_pos);
	perf_free(_cycle_perf_yaw);
}

int LandingTargetEst::task_spawn(int argc, char *argv[])
{

	LandingTargetEst *instance = new LandingTargetEst();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool LandingTargetEst::init()
{
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("vehicle_acceleration callback registration failed!");
		return false;
	}

	updateParams();

	delete _ltest_pos;
	delete _ltest_yaw;

	_ltest_pos_valid = false;
	_ltest_yaw_valid = false;

	if (_param_ltest_pos_en.get()) {
		PX4_INFO("LTEst position estimator enabled.");
		_ltest_pos = new LTEstPos;
		_ltest_pos_valid = (_ltest_pos != nullptr && _ltest_pos->init());
	}

	if (_param_ltest_yaw_en.get()) {
		PX4_INFO("LTEst yaw estimator enabled.");
		_ltest_yaw = new LTEstYaw;
		_ltest_yaw_valid = (_ltest_yaw != nullptr && _ltest_yaw->init());
	}

	return _ltest_pos_valid || _ltest_yaw_valid;
}

void LandingTargetEst::updateParams()
{
	ModuleParams::updateParams();
}

void LandingTargetEst::Run()
{
	if (should_exit()) {
		_vehicle_acceleration_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	//TODO: Average acceleration (and attitude?s). Correct for gravity (rotate gravity into body frame and subtract)

	// Update pos filter at ltest_pos_UPDATE_RATE_HZ
	if (_ltest_pos_valid && ((hrt_absolute_time() - _last_update_pos) > (1000000 / ltest_pos_UPDATE_RATE_HZ))) {
		perf_begin(_cycle_perf_pos);
		_ltest_pos->update();
		_last_update_pos = hrt_absolute_time();
		perf_end(_cycle_perf_pos);
	}

	// Update Yaw filter at ltest_yaw_UPDATE_RATE_HZ
	if (_ltest_yaw_valid && ((hrt_absolute_time() - _last_update_yaw) > (1000000 / ltest_yaw_UPDATE_RATE_HZ))) {
		perf_begin(_cycle_perf_yaw);
		_ltest_yaw->update();
		_last_update_yaw = hrt_absolute_time();
		perf_end(_cycle_perf_yaw);
	}
}


int LandingTargetEst::print_status()
{
	PX4_INFO("LTEst running");
	return 0;
}
int LandingTargetEst::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to estimate the position and orientation of a landing target.

The module runs periodically on the HP work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("landing_target_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int landing_target_estimator_main(int argc, char *argv[])
{
	return LandingTargetEst::main(argc, argv);
}

} // namespace landing_target_est