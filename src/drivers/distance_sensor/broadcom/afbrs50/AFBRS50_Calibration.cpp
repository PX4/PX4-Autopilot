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

// AFBRS50 absolute range offset calibration (vendor sequence
// Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence), driven from NSH:
//
//   afbrs50 cal start <distance_m>    flat target, perpendicular to the optical
//   afbrs50 cal status                axis, at a measured distance; offsets
//                                     persist to SENS_AFBR_OFS_LO/HI and are
//                                     re-applied at every boot (CONFIGURE).
//
// One run at a single known distance derives the global range offsets for BOTH
// laser power stages (low/high) plus the per-pixel relative offsets; the API has
// no multi-distance procedure. Motivating case: LX85D units bias low at short
// range; calibrate against a short-range target (0.3-1 m works well).
//
// This file holds every calibration-related AFBRS50 member so the main driver
// file stays measurement-oriented.

#include "AFBRS50.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <sched.h>
#include <stdlib.h>
#include <string.h>

using namespace time_literals;

void AFBRS50::run_state_calibrate()
{
	_calibration_requested.store(false);
	_cal_state = CalState::RUNNING;

	// The sequence blocks this task for several seconds and its ADS_AwaitIdle
	// loops do not yield. SCHED_PRIORITY_SLOW_DRIVER already sits below the
	// work queues, so drop to SCHED_PRIORITY_DEFAULT to get out of the way of
	// commander, navigator and the other application tasks too. Nothing is
	// in flight: TRIGGER intercepts the request before starting a measurement.
	struct sched_param param {};
	sched_getparam(0, &param);
	const int task_priority = param.sched_priority;
	param.sched_priority = SCHED_PRIORITY_DEFAULT;
	sched_setparam(0, &param);

	runCalibration();

	param.sched_priority = task_priority;
	sched_setparam(0, &param);

	_state = STATE::CONFIGURE;
	_wake_delay = 0;
}

void AFBRS50::runCalibration()
{
	// Runs on the driver task in the CALIBRATE state, so nothing else touches
	// the device for the duration of the sequence.

	// Make sure the device is idle and no measurement is pending. Bounded so
	// a wedged device fails the calibration instead of hanging the task
	// forever (a RUNNING calibration also blocks 'afbrs50 stop').
	const hrt_abstime wait_start = hrt_absolute_time();

	while (Argus_GetStatus(_hnd) != STATUS_IDLE) {
		if (hrt_elapsed_time(&wait_start) > 1_s) {
			_cal_result_status = ERROR_TIMEOUT;
			_cal_state = CalState::FAILED;
			PX4_ERR("calibration failed: device not idle");
			return;
		}

		px4_usleep(1_ms);
	}

	// The absolute sequence rewrites the per-pixel relative offset tables as
	// well as the two global offsets, but only the globals can be persisted:
	// these boards have no NVM for the API and 64 values do not belong in
	// params. Restore the factory pixel tables afterwards so the sensor runs
	// in the same state CONFIGURE re-creates from the params at the next boot.
	argus_cal_offset_table_t factory_pixel_offsets{};
	const bool have_pixel_offsets = (Argus_GetCalibrationPixelRangeOffsets(_hnd, &factory_pixel_offsets) == STATUS_OK);

	auto restore_pixel_offsets = [&]() {
		if (!have_pixel_offsets) {
			PX4_WARN("factory pixel offsets were not read, the calibrated tables stay until reboot");

		} else if (Argus_SetCalibrationPixelRangeOffsets(_hnd, &factory_pixel_offsets) != STATUS_OK) {
			PX4_WARN("could not restore the factory pixel offsets, the calibrated tables stay until reboot");
		}
	};

	const float target_m = (float)_calibration_target_range / Q9_22_ONE;
	PX4_INFO("running absolute range offset calibration at %.3f m (takes a few seconds)...", (double)target_m);

	status_t status = Argus_ExecuteAbsoluteRangeOffsetCalibrationSequence(_hnd, _calibration_target_range);

	if (status < STATUS_OK) {
		_cal_result_status = status;
		_cal_state = CalState::FAILED;
		PX4_ERR("calibration start failed: %i", (int)status);
		return;
	}

	// The sequence runs asynchronously (driven by background measurement
	// frames); poll until it returns to idle, matching the vendor reference
	// flow. Bounded by a timeout so a stalled sequence can't hold the device
	// forever.
	const hrt_abstime cal_start = hrt_absolute_time();

	do {
		status = Argus_GetStatus(_hnd);
		px4_usleep(1_ms);

		if (should_exit()) {
			Argus_Abort(_hnd);
			restore_pixel_offsets();
			_cal_result_status = ERROR_ABORTED;
			_cal_state = CalState::FAILED;
			PX4_WARN("calibration aborted by stop");
			return;
		}

		if (hrt_elapsed_time(&cal_start) > 30_s) {
			restore_pixel_offsets();
			_cal_result_status = ERROR_TIMEOUT;
			_cal_state = CalState::FAILED;
			PX4_ERR("calibration timed out");
			return;
		}
	} while (status > STATUS_IDLE);

	restore_pixel_offsets();
	_cal_result_status = status;

	if (status < STATUS_OK) {
		_cal_state = CalState::FAILED;
		PX4_ERR("calibration failed: %i", (int)status);
		return;
	}

	// Read back the offsets the API applied, then persist them (as meters) to
	// params so the CONFIGURE state re-applies them on the next boot.
	Argus_GetCalibrationGlobalRangeOffsets(_hnd, &_cal_offset_low, &_cal_offset_high);

	_p_sens_afbr_ofs_lo.commit_no_notification((float)_cal_offset_low / 32768.f);
	_p_sens_afbr_ofs_hi.commit_no_notification((float)_cal_offset_high / 32768.f);

	_cal_state = CalState::DONE;
	PX4_INFO("calibration done, offsets persisted: low=%.4f m high=%.4f m",
		 (double)((float)_cal_offset_low / 32768.f),
		 (double)((float)_cal_offset_high / 32768.f));
}

void AFBRS50::requestCalibration(float target_range_m)
{
	_calibration_target_range = (q9_22_t)(target_range_m * Q9_22_ONE);
	_cal_result_status = STATUS_OK;
	_cal_state = CalState::REQUESTED;
	_calibration_requested.store(true);
}

void AFBRS50::cancelCalibration()
{
	_calibration_requested.store(false);

	// Only meaningful if it has not started yet; a running sequence is
	// blocking and cannot be interrupted from here.
	if (_cal_state == CalState::REQUESTED) {
		_cal_state = CalState::IDLE;
	}
}

void AFBRS50::printCalInfo()
{
	const char *state_str = "IDLE";

	switch (_cal_state) {
	case CalState::IDLE:      state_str = "IDLE";      break;

	case CalState::REQUESTED: state_str = "REQUESTED"; break;

	case CalState::RUNNING:   state_str = "RUNNING";   break;

	case CalState::DONE:      state_str = "DONE";      break;

	case CalState::FAILED:    state_str = "FAILED";    break;
	}

	PX4_INFO_RAW("calibration: %s\n", state_str);
	PX4_INFO_RAW("  target:      %.3f m\n", (double)((float)_calibration_target_range / Q9_22_ONE));
	PX4_INFO_RAW("  last status: %i\n", (int)_cal_result_status);
	PX4_INFO_RAW("  offset_low:  %.4f m\n", (double)((float)_cal_offset_low / 32768.f));
	PX4_INFO_RAW("  offset_high: %.4f m\n", (double)((float)_cal_offset_high / 32768.f));
	PX4_INFO_RAW("  stored (params, applied at boot): low=%.4f m  high=%.4f m\n",
		     (double)_p_sens_afbr_ofs_lo.get(), (double)_p_sens_afbr_ofs_hi.get());
}

int AFBRS50::calibrationCommand(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_ERR("usage: afbrs50 cal start <distance_m> | status | stop");
		return PX4_ERROR;
	}

	if (!strcmp(argv[0], "start")) {
		if (argc < 2) {
			PX4_ERR("usage: afbrs50 cal start <distance_m>");
			return PX4_ERROR;
		}

		if (calibrationInProgress()) {
			PX4_ERR("calibration already in progress");
			return PX4_ERROR;
		}

		float range_m = strtof(argv[1], nullptr);

		if (!(range_m > 0.f) || (range_m > 50.f)) {
			PX4_ERR("invalid distance: %s", argv[1]);
			return PX4_ERROR;
		}

		requestCalibration(range_m);
		PX4_INFO("calibration requested at %.3f m; poll with 'afbrs50 cal status'", (double)range_m);
		return PX4_OK;

	} else if (!strcmp(argv[0], "status")) {
		printCalInfo();
		return PX4_OK;

	} else if (!strcmp(argv[0], "stop")) {
		if (calibrationInProgress()) {
			PX4_ERR("sequence already running; it cannot be interrupted");
			return PX4_ERROR;
		}

		cancelCalibration();
		PX4_INFO("calibration request cancelled");
		return PX4_OK;
	}

	PX4_ERR("unknown cal subcommand: %s", argv[0]);
	return PX4_ERROR;
}
