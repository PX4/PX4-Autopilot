/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include "argus.h"

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

class AFBRS50 : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AFBRS50(const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~AFBRS50() override;

	enum class STATE : uint8_t {
		CONFIGURE,
		TRIGGER,
		COLLECT,
	};

	int init();
	void printInfo();

private:
	void Run() override;

	void recordCallbackError();
	void schedule(STATE state);
	bool processMeasurement();
	void updateMeasurementRateFromRange();

	static status_t measurementReadyCallback(status_t status, argus_hnd_t *hnd);

	status_t setRateAndDfm(uint32_t rate_hz, argus_dfm_mode_t dfm_mode);
	argus_mode_t argusModeFromParameter();

private:
	argus_hnd_t *_hnd {nullptr};

	STATE _state{STATE::CONFIGURE};

	PX4Rangefinder _px4_rangefinder;

	hrt_abstime _last_rate_switch{0};

	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": sample count")};
	perf_counter_t _callback_error{perf_alloc(PC_COUNT, MODULE_NAME": callback error")};
	perf_counter_t _process_measurement_error{perf_alloc(PC_COUNT, MODULE_NAME": process measure error")};
	perf_counter_t _status_not_ready_perf{perf_alloc(PC_COUNT, MODULE_NAME": not ready")};
	perf_counter_t _trigger_fail_perf{perf_alloc(PC_COUNT, MODULE_NAME": trigger fail")};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop interval")};


	float _current_distance{0};
	int8_t _current_quality{0};
	float _max_distance{30.f};
	int _current_rate{0};

	hrt_abstime _trigger_time{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uint32_t _measurement_inverval {1000000 / 50}; // 50Hz

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_AFBR_MODE>)   _p_sens_afbr_mode,
		(ParamInt<px4::params::SENS_AFBR_S_RATE>) _p_sens_afbr_s_rate,
		(ParamInt<px4::params::SENS_AFBR_L_RATE>) _p_sens_afbr_l_rate,
		(ParamInt<px4::params::SENS_AFBR_THRESH>) _p_sens_afbr_thresh,
		(ParamInt<px4::params::SENS_AFBR_HYSTER>) _p_sens_afbr_hyster
	);
};
