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

#include <stdlib.h>
#include <string.h>

using namespace time_literals;

extern AFBRS50 *g_dev;

void AFBRS50::run_state_calibrate()
{
	// Hand the calibration sequence to a dedicated task. It must NOT run on
	// the driver's work queue: the AFBR calibration busy-waits for several
	// seconds (ADS_AwaitIdle does not yield), hp_default's priority is above
	// wq:uavcan, and wq:uavcan pets the IWDG - a busy-wait here starves the
	// feeder and resets the board. The task runs at SCHED_PRIORITY_DEFAULT
	// (100 on NuttX), below wq:uavcan, so uavcan preempts it and keeps
	// petting the dog; the calibration's SPI frames still advance via the
	// SPI work item and the DRDY IRQ.
	_calibration_requested.store(false);
	_cal_state = CalState::RUNNING;

	_cal_task = px4_task_spawn_cmd("afbr_cal",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT,
				       8192,
				       &AFBRS50::calibrationTaskTrampoline,
				       nullptr);

	if (_cal_task < 0) {
		PX4_ERR("failed to spawn calibration task: %d", _cal_task);
		_cal_result_status = ERROR_FAIL;
		_cal_state = CalState::FAILED;
		_state = STATE::TRIGGER;
		ScheduleDelayed(_measurement_inverval);
	}

	// On success: do NOT reschedule. The calibration task owns the
	// device while it runs and re-arms the work queue via
	// resumeFromCalibration().
}

int AFBRS50::calibrationTaskTrampoline(int argc, char *argv[])
{
	if (g_dev != nullptr) {
		g_dev->runCalibration();
		g_dev->resumeFromCalibration();
	}

	return 0;
}

void AFBRS50::runCalibration()
{
	// Runs on the dedicated low-priority calibration task. The work queue is
	// idle (CALIBRATE did not reschedule), so this task has exclusive access
	// to the device for the duration of the sequence.

	// Make sure the device is idle and no measurement is pending. Bounded so
	// a wedged device fails the calibration instead of hanging the task
	// forever (a RUNNING calibration also blocks 'afbrs50 stop').
	const hrt_abstime wait_start = hrt_absolute_time();

	while (Argus_GetStatus(_hnd) > STATUS_IDLE) {
		if (hrt_elapsed_time(&wait_start) > 1_s) {
			_cal_result_status = ERROR_TIMEOUT;
			_cal_state = CalState::FAILED;
			PX4_ERR("calibration failed: device not idle");
			return;
		}

		px4_usleep(1_ms);
	}

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

		if (hrt_elapsed_time(&cal_start) > 30_s) {
			_cal_result_status = ERROR_TIMEOUT;
			_cal_state = CalState::FAILED;
			PX4_ERR("calibration timed out");
			return;
		}
	} while (status > STATUS_IDLE);

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

void AFBRS50::resumeFromCalibration()
{
	// Hand control back to the work-queue measurement loop, just before the
	// calibration task exits. Resume via CONFIGURE so rate/DFM/shot-noise
	// settings are re-asserted after the API's internal reconfiguration
	// during the calibration sequence.
	_cal_task = -1;
	_state = STATE::CONFIGURE;
	ScheduleNow();
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

namespace afbrs50
{

// args: [0]=start|status|stop, [1]=distance (m) for start
int calibrate(int argc, char *argv[])
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	if (argc < 1) {
		PX4_ERR("usage: afbrs50 cal start <distance_m> | status | stop");
		return PX4_ERROR;
	}

	if (!strcmp(argv[0], "start")) {
		if (argc < 2) {
			PX4_ERR("usage: afbrs50 cal start <distance_m>");
			return PX4_ERROR;
		}

		if (g_dev->calibrationInProgress()) {
			PX4_ERR("calibration already in progress");
			return PX4_ERROR;
		}

		float range_m = strtof(argv[1], nullptr);

		if (!(range_m > 0.f) || (range_m > 50.f)) {
			PX4_ERR("invalid distance: %s", argv[1]);
			return PX4_ERROR;
		}

		g_dev->requestCalibration(range_m);
		PX4_INFO("calibration requested at %.3f m; poll with 'afbrs50 cal status'", (double)range_m);
		return PX4_OK;

	} else if (!strcmp(argv[0], "status")) {
		g_dev->printCalInfo();
		return PX4_OK;

	} else if (!strcmp(argv[0], "stop")) {
		if (g_dev->calibrationInProgress()) {
			PX4_ERR("sequence already running; it cannot be interrupted");
			return PX4_ERROR;
		}

		g_dev->cancelCalibration();
		PX4_INFO("calibration request cancelled");
		return PX4_OK;
	}

	PX4_ERR("unknown cal subcommand: %s", argv[0]);
	return PX4_ERROR;
}

} // namespace afbrs50
