/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "AuxiliaryActuatorsController.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace auxiliary_actuators_controller
{

AuxiliaryActuatorsController::AuxiliaryActuatorsController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{

}

AuxiliaryActuatorsController::~AuxiliaryActuatorsController()
{
	perf_free(_cycle_perf);
}

int AuxiliaryActuatorsController::task_spawn(int argc, char *argv[])
{
	AuxiliaryActuatorsController *obj = new AuxiliaryActuatorsController();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void AuxiliaryActuatorsController::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void AuxiliaryActuatorsController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);


	perf_end(_cycle_perf);
}

int AuxiliaryActuatorsController::print_status()
{
	return 0;
}

int AuxiliaryActuatorsController::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Auxiliary Flight Control Actuators Controller
Controls landing gear, flaps, spoilers, landing gear wheel. Not intended for payload control.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("auxiliary_actuators_controller", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int auxiliary_actuators_controller_main(int argc, char *argv[])
{
	return AuxiliaryActuatorsController::main(argc, argv);
}

} // namespace auxiliary_actuators_controller
