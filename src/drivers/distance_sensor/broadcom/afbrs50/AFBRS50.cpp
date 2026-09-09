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

#include "AFBRS50.hpp"
#include "s2pi.h"
#include "timer.h"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/time.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace time_literals;

ModuleBase::Descriptor AFBRS50::desc{AFBRS50::task_spawn, AFBRS50::custom_command, AFBRS50::print_usage};

constexpr AFBRS50::RateDfmProfile AFBRS50::kProfilesLv[];
constexpr AFBRS50::RateDfmProfile AFBRS50::kProfilesLx[];

namespace
{
// Range offset in meters to the API's Q0.15 meter format, clamped to [-1, 1).
q0_15_t metersToQ0_15(float meters)
{
	if (!PX4_ISFINITE(meters)) {
		return 0;
	}

	float scaled = roundf(meters * 32768.f);

	if (scaled > 32767.f) { scaled = 32767.f; }

	if (scaled < -32768.f) { scaled = -32768.f; }

	return (q0_15_t)scaled;
}
} // namespace

AFBRS50::AFBRS50():
	ModuleParams(nullptr),
	_px4_rangefinder(0, distance_sensor_s::ROTATION_DOWNWARD_FACING)
{
	px4_sem_init(&_wake_sem, 0, 0);
	px4_sem_setprotocol(&_wake_sem, SEM_PRIO_NONE);

	int32_t rotation = _p_sens_afbr_rot.get();

	// Valid orientations from distance_sensor_s: ROTATION_YAW_* (0-7), ROTATION_UPWARD_FACING (24), ROTATION_DOWNWARD_FACING (25).
	const bool is_yaw = (rotation >= 0 && rotation <= 7);
	const bool is_pitch = (rotation == distance_sensor_s::ROTATION_UPWARD_FACING)
			      || (rotation == distance_sensor_s::ROTATION_DOWNWARD_FACING);

	if (!is_yaw && !is_pitch) {
		PX4_ERR("Invalid SENS_AFBR_ROT %d, using downward facing", (int)rotation);
		rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	}

	_px4_rangefinder.set_orientation((uint8_t)rotation);

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;
	device_id.devid_s.bus = BROADCOM_AFBR_S50_S2PI_SPI_BUS;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_AFBRS50;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

AFBRS50::~AFBRS50()
{
	if (_hnd != nullptr) {
		Argus_StopMeasurementTimer(_hnd);
		// Argus_Deinit aborts any ongoing transfer and drains it synchronously:
		// the library polls the S2PI layer until the (possibly deferred, see
		// s2pi.cpp) abort completion has run.
		Argus_Deinit(_hnd);

		// The library binds the DRDY interrupt callback in ADS_Init and never
		// clears it: detach the EXTI handler and the stored callback so a stray
		// edge after teardown cannot dispatch into the destroyed handle.
		px4_arch_gpiosetevent(BROADCOM_AFBR_S50_S2PI_IRQ, false, false, false, nullptr, nullptr);
		S2PI_SetIrqCallback(BROADCOM_AFBR_S50_S2PI_SPI_BUS, nullptr, nullptr);

		// Same for the API's periodic timer: its hrt callout is the last
		// path that could still reach measurementReadyCallback.
		Timer_SetInterval(0, nullptr);

		Argus_DestroyHandle(_hnd);
	}

	px4_sem_destroy(&_wake_sem);

	perf_free(_sample_perf);
	perf_free(_callback_error);
	perf_free(_process_measurement_error);
	perf_free(_status_not_ready_perf);
	perf_free(_trigger_fail_perf);
	perf_free(_recovery_perf);
	perf_free(_quality_gated_perf);
	perf_free(_loop_perf);
}

status_t AFBRS50::measurementReadyCallback(status_t status, argus_hnd_t *hnd)
{
	// Called from the SPI bus work queue, or from an hrt callout on the
	// API's internal timeout paths. ModuleBase deletes the instance before it
	// clears the pointer, so the destructor's teardown order (drain, detach
	// EXTI and timer) is what keeps a late completion out of freed memory;
	// the null check only covers the window before the task has started.
	AFBRS50 *dev = get_instance<AFBRS50>(desc);

	if (dev == nullptr) {
		return ERROR_FAIL;
	}

	if (status != STATUS_OK) {
		dev->recordCallbackError();
	}

	// Always collect, error or not: Argus_EvaluateData is what releases the
	// API's raw data buffer, and after two unreleased buffers the API refuses
	// to start measurements and rejects configuration writes. COLLECT reports
	// the non-OK result and publishes it as out of range.
	dev->wake(STATE::COLLECT);

	return status;
}

void AFBRS50::wake(STATE state)
{
	// A stale completion (an abort issued by recoverFromTriggerStall, or a
	// frame that was in flight when the calibration finished) must not pull
	// the state machine out of a reconfiguration.
	if (_state == STATE::CONFIGURE) {
		return;
	}

	_state = state;
	px4_sem_post(&_wake_sem);
}

void AFBRS50::waitForWake(hrt_abstime delay)
{
	if (delay == 0) {
		return;
	}

	struct timespec ts;

	px4_clock_gettime(CLOCK_REALTIME, &ts);

	abstime_to_ts(&ts, ts_to_abstime(&ts) + delay);

	// Times out or returns on a post; both are handled the same way.
	px4_sem_timedwait(&_wake_sem, &ts);
}

void AFBRS50::request_stop()
{
	// A calibration that has not started yet must not start now: the vendor
	// sequence cannot be interrupted and outlives ModuleBase's stop deadline.
	_calibration_requested.store(false);
	ModuleBase::request_stop();
	px4_sem_post(&_wake_sem);
}

void AFBRS50::recordCallbackError()
{
	perf_count(_callback_error);
}

int AFBRS50::init()
{
	if (hrt_absolute_time() < 1_ms) {
		PX4_WARN("Power-up time requires at least 1ms!");
	}

	if (_hnd != nullptr) {
		Argus_Deinit(_hnd);
		Argus_DestroyHandle(_hnd);
		_hnd = nullptr;
	}

	_hnd = Argus_CreateHandle();

	if (_hnd == nullptr) {
		PX4_ERR("Handle not initialized");
		return PX4_ERROR;
	}

	// Initialize the S2PI hardware required by the API.
	static constexpr uint32_t SPI_BAUD_RATE = 5000000;
	S2PI_Init(BROADCOM_AFBR_S50_S2PI_SPI_BUS, SPI_BAUD_RATE);

	// A negative mode parameter selects the module's default measurement mode
	// as defined by the API; a mode the API rejects falls back to it as well.
	int32_t mode_param = _p_sens_afbr_mode.get();

	if (mode_param > 4) {
		PX4_ERR("Invalid SENS_AFBR_MODE %ld, using module default mode", (long)mode_param);
		mode_param = -1;
	}

	status_t status;

	if (mode_param < 0) {
		status = Argus_Init(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS);

	} else {
		status = Argus_InitMode(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS, argusModeFromParameter());

		if (status != STATUS_OK) {
			PX4_ERR("Argus_InitMode %ld failed: %ld, falling back to module default mode", (long)mode_param, (long)status);
			Argus_Deinit(_hnd);
			Argus_DestroyHandle(_hnd);
			_hnd = Argus_CreateHandle();

			if (_hnd == nullptr) {
				PX4_ERR("Handle not initialized");
				return PX4_ERROR;
			}

			status = Argus_Init(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS);
		}
	}

	if (status != STATUS_OK) {
		PX4_ERR("Argus_Init failed: %ld", (long)status);
		return PX4_ERROR;
	}

	uint32_t id = Argus_GetChipID(_hnd);
	uint32_t value = Argus_GetAPIVersion();
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	// Packed Argus_GetAPIVersion() is major.minor.bugfix only; 1.6.6.1 is the vendored release.
	static constexpr uint8_t api_version_patch{1};
	const char *build = Argus_GetBuildNumber();
	PX4_INFO("AFBR-S50 Chip ID: %u, API Version: v%d.%d.%d.%u (%s)",
		 (uint)id, a, b, c, api_version_patch, build != nullptr ? build : ARGUS_API_VERSION_BUILD);

	_module = Argus_GetModuleVersion(_hnd);

	float min_distance = 0.f;
	float max_distance = 30.f;
	float fov_degrees = 6.f;

	switch (_module) {
	case AFBR_S50MV85G_V1:
	case AFBR_S50MV85G_V2:
	case AFBR_S50MV85G_V3:
		max_distance = 10.f;
		fov_degrees = 6.f;
		break;

	case AFBR_S50MX85G_V1:
		max_distance = 30.f;
		fov_degrees = 6.f;
		break;

	// Tx beam width 2x2 deg per datasheet.
	case AFBR_S50LV85D_V1:
		max_distance = 30.f;
		fov_degrees = 2.f;
		break;

	case AFBR_S50LX85D_V1:
		min_distance = 0.1f;
		max_distance = 50.f;
		fov_degrees = 2.f;
		break;

	case AFBR_S50MV68B_V1:
		max_distance = 10.f;
		fov_degrees = 1.f;
		break;

	case AFBR_S50MX68B_V1:
		max_distance = 30.f;
		fov_degrees = 1.f;
		break;

	case AFBR_S50MV85I_V1:
		max_distance = 5.f;
		fov_degrees = 6.f;
		break;

	case AFBR_S50MX85I_V1:
		max_distance = 15.f;
		fov_degrees = 6.f;
		break;

	case AFBR_S50SV85K_V1:
		max_distance = 10.f;
		fov_degrees = 4.f;
		break;

	case AFBR_S50SX85K_V1:
		max_distance = 30.f;
		fov_degrees = 4.f;
		break;

	default:
		PX4_WARN("Unknown module version: %d", (int)_module);
		break;
	}

	PX4_INFO("Module: %s", Argus_GetModuleName(_hnd));
	_min_distance = min_distance;
	_max_distance = max_distance;
	_px4_rangefinder.set_min_distance(min_distance);
	_px4_rangefinder.set_max_distance(max_distance);
	_px4_rangefinder.set_fov(math::radians(fov_degrees));

	_state = STATE::CONFIGURE;
	// Initialization Time is 300ms
	_wake_delay = 350_ms;
	return PX4_OK;
}

void AFBRS50::run()
{
	while (!should_exit()) {
		waitForWake(_wake_delay);

		if (should_exit()) {
			break;
		}

		perf_end(_loop_perf);
		perf_begin(_loop_perf);

		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			ModuleParams::updateParams();
		}

		switch (_state) {
		case STATE::CONFIGURE:
			run_state_configure();
			break;

		case STATE::TRIGGER:
			run_state_trigger();
			break;

		case STATE::COLLECT:
			run_state_collect();
			break;

		case STATE::CALIBRATE:
			run_state_calibrate();
			break;

		default:
			break;
		}
	}
}

// Shipped configuration (SENS_AFBR_MODE = -1, SENS_AFBR_RATE = 0,
// SENS_AFBR_DFM = -1):
//   LV85D: Long Range mode @ 20 Hz, DFM 4X (50 m unambiguous range)
//   LX85D: Long Range mode @ 15 Hz, DFM 4X (~100 m unambiguous range)
//   other modules: the API's mode defaults
//
// Frame time is the exposure budget: the dynamic configuration adaption
// raises integration depth until the frame is full, so radiometric reach
// scales inversely with rate, down to the API's 200 ms frame time cap.
void AFBRS50::run_state_configure()
{
	// A raw buffer left behind by an aborted or lost frame blocks every
	// configuration write until it has been evaluated.
	for (unsigned i = 0; (i < 2) && Argus_IsDataEvaluationPending(_hnd); i++) {
		argus_results_t res{};
		Argus_EvaluateData(_hnd, &res);
	}

	if (_configure_failures >= kMaxConfigureFailures) {
		// Argus_GetStatus reports a sticky error (TIMEOUT, NOT_CONNECTED)
		// as its status, and Argus_Abort is not documented to clear it, so
		// awaitIdle() alone cannot make progress: re-initialize the device
		// in its current mode and apply the configuration from scratch.
		_configure_failures = 0;
		PX4_ERR("CONFIGURE failed %u times, reinitializing device", (uint)kMaxConfigureFailures);
		status_t reinit_status = Argus_ReinitMode(_hnd, (argus_mode_t)0);

		if (reinit_status != STATUS_OK) {
			PX4_ERR("Argus_ReinitMode failed: %i", (int)reinit_status);
			_wake_delay = 350_ms;
			return;
		}
	}

	status_t status = STATUS_OK;
	const int32_t dfm_param = _p_sens_afbr_dfm.get();

	if (dfm_param >= 0 && dfm_param <= DFM_MODE_8X) {
		status = setDfm(static_cast<argus_dfm_mode_t>(dfm_param));

	} else if (dfm_param > DFM_MODE_8X) {
		// Parameter metadata is not enforced at set time: degrade to the
		// mode default instead of failing.
		PX4_ERR("Invalid SENS_AFBR_DFM %ld, keeping mode default", (long)dfm_param);

	} else {
		const int dfm_default = defaultDfm();

		if (dfm_default >= 0) {
			status = setDfm(static_cast<argus_dfm_mode_t>(dfm_default));
		}
	}

	if (status == STATUS_OK) {
		const int32_t rate_param = _p_sens_afbr_rate.get();
		uint32_t rate_hz = (rate_param > 0) ? (uint32_t)rate_param : defaultRate();

		if (rate_hz > 0) {
			// Below kMinRateHz the API rejects the configuration outright
			// and CONFIGURE would retry the same invalid value forever.
			if (rate_hz < kMinRateHz) {
				PX4_WARN("SENS_AFBR_RATE %u Hz is below the API's 5 Hz floor, clamping", (uint)rate_hz);
				rate_hz = kMinRateHz;
			}

			status = setRate(rate_hz);

		} else {
			// Uncharacterized module: keep the measurement mode's frame time.
			uint32_t frame_time_us = 0;
			status = Argus_GetConfigurationFrameTime(_hnd, &frame_time_us);

			if ((status == STATUS_OK) && (frame_time_us == 0)) {
				status = ERROR_FAIL;
			}

			if (status == STATUS_OK) {
				_measurement_inverval = frame_time_us;
			}
		}
	}

	if (status != STATUS_OK) {
		PX4_ERR("CONFIGURE status not okay: %i", (int)status);
		configureFailed();
		return;
	}

	status = Argus_SetConfigurationSmartPowerSaveEnabled(_hnd, false);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationSmartPowerSaveEnabled status not okay: %i", (int)status);
		configureFailed();
		return;
	}

	int32_t snm_mode = _p_sens_afbr_snm.get();

	if (snm_mode < 0 || snm_mode > SNM_MODE_DYNAMIC_PLUS) {
		PX4_ERR("Invalid SENS_AFBR_SNM %ld, using dynamic plus", (long)snm_mode);
		snm_mode = SNM_MODE_DYNAMIC_PLUS;
	}

	status = Argus_SetConfigurationShotNoiseMonitorMode(_hnd, static_cast<argus_snm_mode_t>(snm_mode));

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationShotNoiseMonitorMode status not okay: %i", (int)status);
		configureFailed();
		return;
	}

	// Apply the stored range offset calibration on top of the factory
	// calibration loaded at init. Zero means "not calibrated": leave the
	// factory global offsets untouched. The smallest representable offset is
	// 1/32768 m, so the epsilon separates a real calibration from 0.0.
	const float ofs_lo = _p_sens_afbr_ofs_lo.get();
	const float ofs_hi = _p_sens_afbr_ofs_hi.get();

	if ((fabsf(ofs_lo) > 1e-6f) || (fabsf(ofs_hi) > 1e-6f)) {
		// Non-fatal: a rejected offset write must not block measurements.
		if (Argus_SetCalibrationGlobalRangeOffsets(_hnd, metersToQ0_15(ofs_lo),
				metersToQ0_15(ofs_hi)) != STATUS_OK) {
			PX4_ERR("Argus_SetCalibrationGlobalRangeOffsets failed");

		} else {
			PX4_INFO("applied stored range offsets: low=%.4f m high=%.4f m", (double)ofs_lo, (double)ofs_hi);
		}
	}

	// Enable interrupt on falling edge
	px4_arch_configgpio(BROADCOM_AFBR_S50_S2PI_IRQ);

	updateMaxDistanceFromApi();
	printConfiguration();

	_configure_failures = 0;
	_state = STATE::TRIGGER;
	_wake_delay = _measurement_inverval;
}

void AFBRS50::configureFailed()
{
	_configure_failures++;
	_wake_delay = 350_ms;
}

void AFBRS50::run_state_trigger()
{
	// A calibration request takes priority over the next measurement.
	// Intercept here, before triggering, so no measurement/callback is
	// in flight when the (blocking) calibration sequence runs.
	if (_calibration_requested.load()) {
		_state = STATE::CALIBRATE;
		_wake_delay = 0;
		return;
	}

	if (Argus_GetStatus(_hnd) != STATUS_IDLE) {
		perf_count(_status_not_ready_perf);

		if (++_trigger_retry_count >= kMaxTriggerRetries) {
			recoverFromTriggerStall("device not idle");

		} else {
			_wake_delay = _measurement_inverval / 4;
		}

		return;
	}

	_trigger_time = hrt_absolute_time();
	status_t status = Argus_TriggerMeasurement(_hnd, measurementReadyCallback);

	if (status != STATUS_OK) {
		perf_count(_trigger_fail_perf);

		if (++_trigger_retry_count >= kMaxTriggerRetries) {
			recoverFromTriggerStall("trigger rejected");
			return;
		}

		_state = STATE::TRIGGER;
		_wake_delay = _measurement_inverval / 4;

	} else {
		_trigger_retry_count = 0;
		// The measurement callback normally wakes the task into COLLECT
		// well before this expires; if the callback is lost the state
		// machine re-enters TRIGGER, where the status gate keeps it polling.
		_wake_delay = 4 * _measurement_inverval;
	}
}

void AFBRS50::run_state_collect()
{
	perf_count(_sample_perf);

	argus_results_t res{};
	status_t evaluate_status = Argus_EvaluateData(_hnd, &res);

	// Every frame is published so a receiver can tell "online, invalid
	// readings" from "dropped off the bus"; signal_quality 0 marks the
	// invalid ones. The distance carried alongside matters because collision
	// prevention ignores quality: it enters anything at or beyond
	// max_distance as free space and discards anything at or below
	// min_distance. So only the device's own "nothing in range" verdict goes
	// out beyond max_distance (READING_TYPE_TOO_FAR over DroneCAN); errors
	// (STALLED, TIMEOUT, integrity faults) carry min_distance, which every
	// consumer drops and DroneCAN encodes as READING_TYPE_UNDEFINED; a gated
	// frame keeps its measured distance, since a weak return is still a
	// return, with the quality floored to invalid.
	if ((evaluate_status != STATUS_OK) || (res.Status != STATUS_OK)) {
		perf_count(_process_measurement_error);
		recordMeasurementError((evaluate_status != STATUS_OK) ? evaluate_status : res.Status);

		const bool no_object = (evaluate_status == STATUS_OK) && (res.Status == STATUS_ARGUS_NO_OBJECT);
		_px4_rangefinder.update(hrt_absolute_time(), no_object ? (_max_distance + 1.0f) : _min_distance, 0);

	} else {
		uint32_t result_mm = res.Bin.Range / (Q9_22_ONE / 1000);
		float distance = static_cast<float>(result_mm) / 1000.f;
		int8_t quality = res.Bin.SignalQuality;

		if ((_p_sens_afbr_qmin.get() > 0) && (quality < _p_sens_afbr_qmin.get())) {
			perf_count(_quality_gated_perf);
			quality = 0;
		}

		// max_distance is the module's rated reach, which DFM's unambiguous
		// range can exceed: a return this far out is a real object beyond
		// the rated range, not an error, so report it as too far.
		if (distance > _max_distance * 1.5f) {
			distance = _max_distance + 1.0f;
			quality = 0;
		}

		_current_distance = distance;
		_px4_rangefinder.update(((res.TimeStamp.sec * 1000000ULL) + res.TimeStamp.usec), distance, quality);
	}

	_state = STATE::TRIGGER;

	auto elapsed = hrt_elapsed_time(&_trigger_time);

	_wake_delay = (elapsed > _measurement_inverval) ? 0 : (_measurement_inverval - elapsed);
}

void AFBRS50::recoverFromTriggerStall(const char *reason)
{
	// The 4x frame-time watchdog recovers a lost completion callback, but a
	// device or library wedged out of IDLE would keep the TRIGGER state
	// polling forever: abort whatever is stuck and run a full reconfigure.
	perf_count(_recovery_perf);
	PX4_ERR("no trigger progress after %u attempts (%s), aborting and reconfiguring",
		(uint)kMaxTriggerRetries, reason);

	// Enter CONFIGURE before aborting so the abort completion, whether it
	// arrives synchronously or once the in-flight SPI exchange drains, is
	// dropped by wake() instead of restarting the trigger loop.
	_trigger_retry_count = 0;
	_state = STATE::CONFIGURE;

	status_t status = Argus_Abort(_hnd);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_Abort failed: %i", (int)status);
	}

	_wake_delay = 350_ms;
}

void AFBRS50::recordMeasurementError(status_t status)
{
	_last_error_status = status;
	_last_error_time = hrt_absolute_time();

	for (auto &slot : _error_status_counts) {
		if ((slot.count == 0) || (slot.status == status)) {
			slot.status = status;
			slot.count++;
			return;
		}
	}

	_error_status_other++;
}

const char *AFBRS50::argusStatusName(status_t status)
{
	switch (status) {
	case STATUS_ARGUS_NO_OBJECT: return "NO_OBJECT";

	case ERROR_ARGUS_STALLED: return "STALLED";

	case ERROR_ARGUS_BIAS_VOLTAGE_REINIT: return "BIAS_VOLTAGE_REINIT";

	case ERROR_ARGUS_LASER_MONITOR_INACTIVE: return "LASER_MONITOR_INACTIVE";

	case ERROR_ARGUS_BGL_EXCEEDANCE: return "BGL_EXCEEDANCE";

	case ERROR_ARGUS_XTALK_AMPLITUDE_EXCEEDANCE: return "XTALK_AMPLITUDE_EXCEEDANCE";

	case ERROR_ARGUS_LASER_FAILURE: return "LASER_FAILURE";

	case ERROR_ARGUS_DATA_INTEGRITY_LOST: return "DATA_INTEGRITY_LOST";

	case STATUS_ARGUS_PLL_NOT_LOCKED: return "PLL_NOT_LOCKED";

	case ERROR_ARGUS_BUFFER_EMPTY: return "BUFFER_EMPTY";

	case ERROR_TIMEOUT: return "TIMEOUT";

	case ERROR_ABORTED: return "ABORTED";

	case ERROR_FAIL: return "FAIL";

	default: return "?";
	}
}

const char *AFBRS50::argusModeName(argus_mode_t mode)
{
	switch (mode) {
	case ARGUS_MODE_SHORT_RANGE: return "Short Range";

	case ARGUS_MODE_LONG_RANGE: return "Long Range";

	case ARGUS_MODE_HIGH_SPEED_SHORT_RANGE: return "High Speed Short Range";

	case ARGUS_MODE_HIGH_SPEED_LONG_RANGE: return "High Speed Long Range";

	case ARGUS_MODE_HIGH_PRECISION_SHORT_RANGE: return "High Precision Short Range";

	default: return "?";
	}
}

status_t AFBRS50::awaitIdle()
{
	// Bounded wait; only used while configuring, before measurements run.
	for (int i = 0; i < 100; i++) {
		if (Argus_GetStatus(_hnd) == STATUS_IDLE) {
			return STATUS_OK;
		}

		px4_usleep(1_ms);
	}

	return ERROR_TIMEOUT;
}

status_t AFBRS50::setDfm(argus_dfm_mode_t dfm_mode)
{
	status_t status = awaitIdle();

	if (status != STATUS_OK) {
		PX4_ERR("setDfm: device not idle: %i", (int)status);
		return status;
	}

	status = Argus_SetConfigurationDFMMode(_hnd, dfm_mode);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationDFMMode status not okay: %i", (int)status);
		return status;
	}

	argus_dfm_mode_t result_mode;
	status = Argus_GetConfigurationDFMMode(_hnd, &result_mode);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_GetConfigurationDFMMode status not okay: %i", (int)status);
		return status;
	}

	if (result_mode != dfm_mode) {
		PX4_ERR("DFM mode not applied: requested %i, got %i", (int)dfm_mode, (int)result_mode);
		return ERROR_FAIL;
	}

	return STATUS_OK;
}

status_t AFBRS50::setRate(uint32_t rate_hz)
{
	status_t status = awaitIdle();

	if (status != STATUS_OK) {
		PX4_ERR("setRate: device not idle: %i", (int)status);
		return status;
	}

	status = Argus_SetConfigurationFrameTime(_hnd, (1000000 / rate_hz));

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationFrameTime status not okay: %i", (int)status);
		return status;
	}

	uint32_t frame_time_us;
	status = Argus_GetConfigurationFrameTime(_hnd, &frame_time_us);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_GetConfigurationFrameTime status not okay: %i", (int)status);
		return status;
	}

	if (frame_time_us == 0) {
		PX4_ERR("Argus_GetConfigurationFrameTime returned 0");
		return ERROR_FAIL;
	}

	_measurement_inverval = frame_time_us;

	if (frame_time_us != (1000000 / rate_hz)) {
		PX4_WARN("Requested %u Hz, API set %u Hz", (uint)rate_hz, (uint)(1000000 / frame_time_us));
	}

	return STATUS_OK;
}

const AFBRS50::RateDfmProfile *AFBRS50::profile() const
{
	int32_t index = _p_sens_afbr_prof.get();

	if ((index < 0) || (index >= (int32_t)kProfileCount)) {
		index = 0;
	}

	switch (_module) {
	case AFBR_S50LV85D_V1:
		return &kProfilesLv[index];

	case AFBR_S50LX85D_V1:
		return &kProfilesLx[index];

	default:
		return nullptr;
	}
}

int AFBRS50::defaultDfm() const
{
	// -1: the measurement mode's own DFM default stands.
	const RateDfmProfile *p = profile();
	return (p != nullptr) ? (int)p->dfm : -1;
}

uint32_t AFBRS50::defaultRate() const
{
	// 0: the measurement mode's own frame time stands.
	const RateDfmProfile *p = profile();
	return (p != nullptr) ? p->rate_hz : 0;
}

void AFBRS50::updateMaxDistanceFromApi()
{
	uint32_t range_mm = 0;

	if (Argus_GetConfigurationUnambiguousRange(_hnd, &range_mm) != STATUS_OK || range_mm == 0) {
		return;
	}

	// The unambiguous range follows the measurement mode and DFM setting and
	// can exceed the module's radiometric reach; the module table value from
	// init() stays the upper bound.
	const float api_max = static_cast<float>(range_mm) / 1000.f;

	if (api_max < _max_distance) {
		PX4_INFO("max distance limited to %.1f m by the unambiguous range", (double)api_max);
		_max_distance = api_max;
		_px4_rangefinder.set_max_distance(api_max);
	}
}

argus_mode_t AFBRS50::argusModeFromParameter()
{
	int32_t mode_param = _p_sens_afbr_mode.get();
	argus_mode_t mode = ARGUS_MODE_SHORT_RANGE;

	if (mode_param < 0 || mode_param > 4) {
		PX4_ERR("Invalid mode parameter: %li", mode_param);
		return mode;
	}

	switch (mode_param) {
	case 0:
		mode = ARGUS_MODE_SHORT_RANGE;
		break;

	case 1:
		mode = ARGUS_MODE_LONG_RANGE;
		break;

	case 2:
		mode = ARGUS_MODE_HIGH_SPEED_SHORT_RANGE;
		break;

	case 3:
		mode = ARGUS_MODE_HIGH_SPEED_LONG_RANGE;
		break;

	case 4:
		mode = ARGUS_MODE_HIGH_PRECISION_SHORT_RANGE;
		break;

	default:
		break;
	}

	return mode;
}

int AFBRS50::print_status()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_callback_error);
	perf_print_counter(_process_measurement_error);
	perf_print_counter(_status_not_ready_perf);
	perf_print_counter(_trigger_fail_perf);
	perf_print_counter(_recovery_perf);
	perf_print_counter(_quality_gated_perf);
	perf_print_counter(_loop_perf);

	if (_last_error_status != STATUS_OK) {
		PX4_INFO_RAW("last invalid result: %d (%s), %.1fs ago\n",
			     (int)_last_error_status, argusStatusName(_last_error_status),
			     (double)hrt_elapsed_time(&_last_error_time) / 1e6);

		for (const auto &slot : _error_status_counts) {
			if (slot.count > 0) {
				PX4_INFO_RAW("  status %d (%s): %lu\n", (int)slot.status,
					     argusStatusName(slot.status), (unsigned long)slot.count);
			}
		}

		if (_error_status_other > 0) {
			PX4_INFO_RAW("  other statuses: %lu\n", (unsigned long)_error_status_other);
		}
	}

	PX4_INFO_RAW("profile: %ld   quality gate: %ld\n",
		     (long)_p_sens_afbr_prof.get(), (long)_p_sens_afbr_qmin.get());
	PX4_INFO_RAW("distance: %.3fm\n", (double)_current_distance);
	PX4_INFO_RAW("rate: %u Hz\n", (uint)(1000000 / _measurement_inverval));

	return 0;
}

void AFBRS50::printConfiguration()
{
	if (_hnd == nullptr) {
		return;
	}

	PX4_INFO_RAW("===== AFBR-S50 Configuration =====\n");

	argus_mode_t mode;

	if (Argus_GetMeasurementMode(_hnd, &mode) == STATUS_OK) {
		PX4_INFO_RAW("Measurement Mode:       %s (0x%02X)\n", argusModeName(mode), (uint)mode);
	}

	uint32_t frame_time_us = 0;

	if (Argus_GetConfigurationFrameTime(_hnd, &frame_time_us) == STATUS_OK && frame_time_us > 0) {
		PX4_INFO_RAW("Frame Time:             %u us (%u Hz)\n",
			     (uint)frame_time_us, (uint)(1000000 / frame_time_us));
	}

	argus_dfm_mode_t dfm;

	if (Argus_GetConfigurationDFMMode(_hnd, &dfm) == STATUS_OK) {
		const char *s = (dfm == DFM_MODE_OFF) ? "OFF" :
				(dfm == DFM_MODE_4X)  ? "4X"  :
				(dfm == DFM_MODE_8X)  ? "8X"  : "?";
		PX4_INFO_RAW("DFM Mode:               %s\n", s);
	}

	bool sps_enabled = false;

	if (Argus_GetConfigurationSmartPowerSaveEnabled(_hnd, &sps_enabled) == STATUS_OK) {
		PX4_INFO_RAW("Smart Power Save:       %s\n", sps_enabled ? "enabled" : "disabled");
	}

	argus_snm_mode_t snm;

	if (Argus_GetConfigurationShotNoiseMonitorMode(_hnd, &snm) == STATUS_OK) {
		const char *s = (snm == SNM_MODE_STATIC_INDOOR)  ? "Static Indoor"  :
				(snm == SNM_MODE_STATIC_OUTDOOR) ? "Static Outdoor" :
				(snm == SNM_MODE_DYNAMIC)        ? "Dynamic"        :
				(snm == SNM_MODE_DYNAMIC_PLUS)   ? "Dynamic Plus"   : "?";
		PX4_INFO_RAW("Shot Noise Monitor:     %s\n", s);
	}

	argus_cfg_dca_t dca{};

	if (Argus_GetConfigurationDynamicAdaption(_hnd, &dca) == STATUS_OK) {
		const char *en = (dca.Enabled == DCA_ENABLE_DYNAMIC) ? "DYNAMIC" :
				 (dca.Enabled == DCA_ENABLE_STATIC)  ? "STATIC"  :
				 (dca.Enabled == DCA_ENABLE_OFF)     ? "OFF"     : "?";
		const char *pwr = (dca.Power == DCA_POWER_LOW)  ? "LOW"  :
				  (dca.Power == DCA_POWER_HIGH) ? "HIGH" :
				  (dca.Power == DCA_POWER_AUTO) ? "AUTO" : "?";
		PX4_INFO_RAW("DCA:                    %s, Power=%s, PowerSavingRatio=%u/255\n",
			     en, pwr, (uint)dca.PowerSavingRatio);
		PX4_INFO_RAW("DCA Depth (Q10.6):      Nom=%u Min(LP)=%u Min(HP)=%u Max=%u\n",
			     (uint)dca.DepthNom, (uint)dca.DepthMin_LowPower,
			     (uint)dca.DepthMin_HighPower, (uint)dca.DepthMax);
		PX4_INFO_RAW("DCA Gain:               Nom=%u Min=%u Max=%u\n",
			     (uint)dca.GainNom, (uint)dca.GainMin, (uint)dca.GainMax);
		PX4_INFO_RAW("DCA Amplitude (Q12.4):  Target=%u Low=%u High=%u\n",
			     (uint)dca.Atarget, (uint)dca.AthLow, (uint)dca.AthHigh);
	}

	uint32_t uar_mm = 0;

	if (Argus_GetConfigurationUnambiguousRange(_hnd, &uar_mm) == STATUS_OK) {
		PX4_INFO_RAW("Unambiguous Range:      %u mm\n", (uint)uar_mm);
	}

	PX4_INFO_RAW("Published Range:        %.2f m\n", (double)_max_distance);
	PX4_INFO_RAW("==================================\n");
}

int AFBRS50::task_spawn(int argc, char *argv[])
{
	// Argus_EvaluateData and the calibration sequence run here.
	static constexpr int kStackSize = PX4_STACK_ADJUSTED(6000);

	int task_id = px4_task_spawn_cmd("afbrs50", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER, kStackSize,
					 (px4_main_t)&run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	desc.task_id = task_id;

	// instantiate() brings the device up, so a missing sensor fails 'start'.
	if (wait_until_running(desc, 3000) < 0) {
		return -1;
	}

	return 0;
}

int AFBRS50::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return AFBRS50::instantiate(ac, av);
	}, argc, argv);
}

AFBRS50 *AFBRS50::instantiate(int argc, char *argv[])
{
	AFBRS50 *instance = new AFBRS50();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

int AFBRS50::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && !strcmp(argv[0], "cal")) {
		AFBRS50 *instance = get_instance<AFBRS50>(desc);

		if (instance == nullptr) {
			PX4_ERR("driver not running");
			return PX4_ERROR;
		}

		return instance->calibrationCommand(argc - 1, argv + 1);
	}

	return print_usage("unknown command");
}

int AFBRS50::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the Broadcom AFBR-S50 time-of-flight rangefinder.

### Examples

$ afbrs50 start
$ afbrs50 stop

Run an absolute range offset calibration against a flat target at a known
distance (in meters), then check the result. Offsets persist to
SENS_AFBR_OFS_LO/HI and are re-applied at startup.
$ afbrs50 cal start 0.3
$ afbrs50 cal status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("afbrs50", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("cal", "Range offset calibration: cal start <distance_m> | status | stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int afbrs50_main(int argc, char *argv[])
{
	// ModuleBase::stop_command deletes the task after 5 s regardless, and the
	// vendor calibration sequence blocks longer than that while the API holds
	// pointers into this task's stack. Refuse the stop instead.
	if ((argc > 1) && !strcmp(argv[1], "stop") && AFBRS50::calibrationRunning()) {
		PX4_ERR("calibration running, retry once 'afbrs50 cal status' reports it finished");
		return PX4_ERROR;
	}

	return ModuleBase::main(AFBRS50::desc, argc, argv);
}
