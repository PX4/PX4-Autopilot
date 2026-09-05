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
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/tasks.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

class AFBRS50 : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AFBRS50();
	~AFBRS50() override;

	enum class STATE : uint8_t {
		CONFIGURE,
		TRIGGER,
		COLLECT,
		CALIBRATE,
	};

	int init();
	void printInfo();

	// NSH-triggered absolute range offset calibration (AFBRS50_Calibration.cpp).
	void requestCalibration(float target_range_m);
	void cancelCalibration();
	void printCalInfo();
	bool calibrationInProgress() const { return _cal_state == CalState::RUNNING; }

private:
	void Run() override;

	void run_state_configure();
	void run_state_trigger();
	void run_state_collect();

	// The CALIBRATE state hands the blocking vendor sequence to a dedicated
	// low-priority task (AFBRS50_Calibration.cpp).
	void run_state_calibrate();
	void runCalibration();
	void resumeFromCalibration();
	static int calibrationTaskTrampoline(int argc, char *argv[]);

	void recordCallbackError();
	void schedule(STATE state);
	void recoverFromTriggerStall(const char *reason);

	static status_t measurementReadyCallback(status_t status, argus_hnd_t *hnd);

	status_t awaitIdle();
	status_t setDfm(argus_dfm_mode_t dfm_mode);
	status_t setRate(uint32_t rate_hz);
	uint32_t defaultRate() const;
	int defaultDfm() const;
	argus_mode_t argusModeFromParameter();
	void updateMaxDistanceFromApi();
	void printConfiguration();

	void recordMeasurementError(status_t status);
	static const char *argusStatusName(status_t status);
	static const char *argusModeName(argus_mode_t mode);

	argus_hnd_t *_hnd {nullptr};

	STATE _state{STATE::CONFIGURE};

	argus_module_version_t _module{MODULE_NONE};

	// Absolute range offset calibration state (AFBRS50_Calibration.cpp).
	enum class CalState : uint8_t { IDLE, REQUESTED, RUNNING, DONE, FAILED };
	px4::atomic_bool _calibration_requested{false};
	q9_22_t _calibration_target_range{0};      // Q9.22 meter
	CalState _cal_state{CalState::IDLE};
	status_t _cal_result_status{STATUS_OK};
	q0_15_t _cal_offset_low{0};                 // last applied offsets, Q0.15 meter
	q0_15_t _cal_offset_high{0};
	px4_task_t _cal_task{-1};

	PX4Rangefinder _px4_rangefinder;

	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, MODULE_NAME": sample count")};
	perf_counter_t _callback_error{perf_alloc(PC_COUNT, MODULE_NAME": callback error")};
	perf_counter_t _process_measurement_error{perf_alloc(PC_COUNT, MODULE_NAME": process measure error")};
	perf_counter_t _status_not_ready_perf{perf_alloc(PC_COUNT, MODULE_NAME": not ready")};
	perf_counter_t _trigger_fail_perf{perf_alloc(PC_COUNT, MODULE_NAME": trigger fail")};
	perf_counter_t _recovery_perf{perf_alloc(PC_COUNT, MODULE_NAME": pipeline recovery")};
	perf_counter_t _quality_gated_perf{perf_alloc(PC_COUNT, MODULE_NAME": quality gated")};
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": loop interval")};

	// Distribution of non-OK measurement results (the Argus_EvaluateData
	// return or res.Status), dumped by 'afbrs50 status': the raw codes are the
	// only clue to why frames go invalid in the field.
	static constexpr unsigned kErrorStatusSlots = 8;
	struct ErrorStatusCount {
		status_t status{STATUS_OK};
		uint32_t count{0};
	};
	ErrorStatusCount _error_status_counts[kErrorStatusSlots] {};
	uint32_t _error_status_other{0};
	status_t _last_error_status{STATUS_OK};
	hrt_abstime _last_error_time{0};

	float _current_distance{0};
	float _max_distance{30.f};

	hrt_abstime _trigger_time{0};

	// Consecutive failed trigger cycles (device not idle or trigger rejected),
	// polled at a quarter frame time: 20 retries = 5 frame times without
	// progress before recoverFromTriggerStall() aborts and reconfigures.
	static constexpr uint32_t kMaxTriggerRetries = 20;
	uint32_t _trigger_retry_count{0};

	// The API rejects frame times above 200 ms (Argus_Dev_CheckCfg) with
	// ERROR_ARGUS_INVALID_CFG, so 5 Hz is a hard rate floor.
	static constexpr uint32_t kMinRateHz = 5;

	// SENS_AFBR_PROF on the two characterized modules. Frame rate is the
	// dominant range knob on both: reliable range is flat across a band of
	// rates and falls off above it, so Range sits at the top of that band and
	// Fast at the module's native rate. DFM 4X beat the mode-default 8X on
	// validity, spread and wrong-window returns at every rate flown.
	struct RateDfmProfile {
		uint8_t rate_hz;
		uint8_t dfm;
	};

	static constexpr unsigned kProfileCount = 2;

	// LV85D: 12-20 Hz reach the same holds, 25 Hz loses ~4 m.
	static constexpr RateDfmProfile kProfilesLv[kProfileCount] {
		{20, DFM_MODE_4X},  // 0: Range
		{50, DFM_MODE_4X},  // 1: Fast
	};

	// LX85D: 5-15 Hz reach the same holds, 25 Hz loses 3-6 m.
	static constexpr RateDfmProfile kProfilesLx[kProfileCount] {
		{15, DFM_MODE_4X},  // 0: Range
		{25, DFM_MODE_4X},  // 1: Fast
	};

	// Selected profile for the detected module, nullptr when uncharacterized.
	const RateDfmProfile *profile() const;

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uint32_t _measurement_inverval {1000000 / 50}; // 50Hz

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_AFBR_MODE>)     _p_sens_afbr_mode,
		(ParamInt<px4::params::SENS_AFBR_RATE>)     _p_sens_afbr_rate,
		(ParamInt<px4::params::SENS_AFBR_DFM>)      _p_sens_afbr_dfm,
		(ParamInt<px4::params::SENS_AFBR_SNM>)      _p_sens_afbr_snm,
		(ParamInt<px4::params::SENS_AFBR_ROT>)      _p_sens_afbr_rot,
		(ParamInt<px4::params::SENS_AFBR_PROF>)     _p_sens_afbr_prof,
		(ParamInt<px4::params::SENS_AFBR_QMIN>)     _p_sens_afbr_qmin,
		(ParamFloat<px4::params::SENS_AFBR_OFS_LO>) _p_sens_afbr_ofs_lo,
		(ParamFloat<px4::params::SENS_AFBR_OFS_HI>) _p_sens_afbr_ofs_hi
	);
};
