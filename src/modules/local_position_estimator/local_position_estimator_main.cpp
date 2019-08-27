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
 * @file local_position_estimator.cpp
 * @author James Goppert <james.goppert@gmail.com>
 * @author Mohammed Kabir
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * Local position estimator
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>

#include "BlockLocalPositionEstimator.hpp"

extern "C" __EXPORT int local_position_estimator_main(int argc, char *argv[]);

class LocalPositionEstimatorModule : public ModuleBase<LocalPositionEstimatorModule>
{
public:
	virtual ~LocalPositionEstimatorModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static LocalPositionEstimatorModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:
	BlockLocalPositionEstimator _estimator;
};

int LocalPositionEstimatorModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("local_position_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int LocalPositionEstimatorModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int LocalPositionEstimatorModule::task_spawn(int argc, char *argv[])
{
		_task_id = px4_task_spawn_cmd("lp_estimator",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_ESTIMATOR,
						 7900,
						 (px4_main_t)&run_trampoline,
						 (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

LocalPositionEstimatorModule *LocalPositionEstimatorModule::instantiate(int argc, char *argv[])
{
	LocalPositionEstimatorModule *instance = new LocalPositionEstimatorModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void LocalPositionEstimatorModule::run()
{
	while (!should_exit()) {
		_estimator.update();
	}
}


int local_position_estimator_main(int argc, char *argv[])
{
	return LocalPositionEstimatorModule::main(argc, argv);
}
