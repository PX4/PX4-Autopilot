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

#include "VehicleOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleOpticalFlow::VehicleOpticalFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleOpticalFlow::Start()
{
	ScheduleNow();
	return true;
}

void VehicleOpticalFlow::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleOpticalFlow::ParametersUpdate()
{
	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();


	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void VehicleOpticalFlow::PrintStatus()
{

}

}; // namespace sensors
