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

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

AFBRS50 *g_dev{nullptr};

AFBRS50::AFBRS50(uint8_t device_orientation):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	// ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::SPI6),
	_px4_rangefinder(0, device_orientation)
{
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;
	device_id.devid_s.bus = BROADCOM_AFBR_S50_S2PI_SPI_BUS;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_AFBRS50;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
}

AFBRS50::~AFBRS50()
{
	ScheduleClear();

	Argus_StopMeasurementTimer(_hnd);
	Argus_Deinit(_hnd);
	Argus_DestroyHandle(_hnd);

	perf_free(_sample_perf);
	perf_free(_callback_error);
	perf_free(_process_measurement_error);
	perf_free(_status_not_ready_perf);
	perf_free(_trigger_fail_perf);
}

status_t AFBRS50::measurementReadyCallback(status_t status, argus_hnd_t *hnd)
{
	// Called from the SPI comms thread context
	STATE state = STATE::COLLECT;

	if (up_interrupt_context()) {
		// We cannot be in interrupt context
		g_dev->recordCallbackError();
		status = ERROR_FAIL;
		state = STATE::TRIGGER;
	}

	if ((g_dev == nullptr) || (status != STATUS_OK)) {
		g_dev->recordCallbackError();
		status = ERROR_FAIL;
		state = STATE::TRIGGER;
	}

	g_dev->schedule(state);

	return status;
}

void AFBRS50::schedule(STATE state)
{
	_state = state;
	ScheduleNow();
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

	// Initialize device with initial mode
	status_t status = Argus_InitMode(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS, argusModeFromParameter());

	if (status != STATUS_OK) {
		PX4_ERR("Argus_InitMode failed: %ld", status);
		return PX4_ERROR;
	}

	uint32_t id = Argus_GetChipID(_hnd);
	uint32_t value = Argus_GetAPIVersion();
	uint8_t a = (value >> 24) & 0xFFU;
	uint8_t b = (value >> 16) & 0xFFU;
	uint8_t c = value & 0xFFFFU;
	PX4_INFO("AFBR-S50 Chip ID: %u, API Version: %u v%d.%d.%d", (uint)id, (uint)value, a, b, c);

	char module_string[20] = {};
	argus_module_version_t mv = Argus_GetModuleVersion(_hnd);

	float min_distance = 0.f;
	float max_distance = 30.f;
	float fov_degrees = 6.f;

	switch (mv) {
	case AFBR_S50MV85G_V1:
	case AFBR_S50MV85G_V2:
	case AFBR_S50MV85G_V3:
		max_distance = 10.f;
		fov_degrees = 6.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50MV85G");
		break;

	case AFBR_S50LV85D_V1:
		max_distance = 30.f;
		fov_degrees = 6.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50LV85D");
		break;

	case AFBR_S50LX85D_V1:
		max_distance = 50.f;
		fov_degrees = 6.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50LX85D");
		break;

	case AFBR_S50MV68B_V1:
		max_distance = 10.f;
		fov_degrees = 1.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50MV68B");
		break;

	case AFBR_S50MV85I_V1:
		max_distance = 5.f;
		fov_degrees = 6.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50MV85I");
		break;

	case AFBR_S50SV85K_V1:
		max_distance = 10.f;
		fov_degrees = 4.f;
		snprintf(module_string, sizeof(module_string), "AFBR-S50SV85K");
		break;

	default:
		break;
	}

	PX4_INFO("Module: %s", module_string);
	_max_distance = max_distance;
	_px4_rangefinder.set_min_distance(min_distance);
	_px4_rangefinder.set_max_distance(max_distance);
	_px4_rangefinder.set_fov(math::radians(fov_degrees));

	_state = STATE::CONFIGURE;
	// Initialization Time is 300ms
	ScheduleDelayed(350_ms);
	return PX4_OK;
}

void AFBRS50::Run()
{
	perf_end(_loop_perf);
	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		ModuleParams::updateParams();
	}

	switch (_state) {
	case STATE::CONFIGURE: {
			_current_rate = (uint32_t)_p_sens_afbr_s_rate.get();
			status_t status = setRateAndDfm(_current_rate, DFM_MODE_OFF);

			if (status != STATUS_OK) {
				PX4_ERR("CONFIGURE status not okay: %i", (int)status);
				ScheduleDelayed(350_ms);
				return;
			}

			status = Argus_SetConfigurationSmartPowerSaveEnabled(_hnd, false);

			if (status != STATUS_OK) {
				PX4_ERR("Argus_SetConfigurationSmartPowerSaveEnabled status not okay: %i", (int)status);
				// TODO: delay?
				ScheduleNow();
				return;
			}

			// Enable interrupt on falling edge
			px4_arch_configgpio(BROADCOM_AFBR_S50_S2PI_IRQ);

			_state = STATE::TRIGGER;
			ScheduleDelayed(_measurement_inverval);
		}
		break;

	case STATE::TRIGGER: {
			if (Argus_GetStatus(_hnd) != STATUS_IDLE) {
				perf_count(_status_not_ready_perf);
				ScheduleDelayed(_measurement_inverval / 4);
				return;
			}

			_trigger_time = hrt_absolute_time();
			status_t status = Argus_TriggerMeasurement(_hnd, measurementReadyCallback);

			if (status != STATUS_OK) {
				perf_count(_trigger_fail_perf);
				_state = STATE::TRIGGER;
				ScheduleDelayed(1_ms); // Try again immediately
			}

			// We don't reschedule, the trigger callback will schedule a collect
		}
		break;

	case STATE::COLLECT: {
			if (processMeasurement()) {
				updateMeasurementRateFromRange();
			}

			_state = STATE::TRIGGER;

			auto elapsed = hrt_elapsed_time(&_trigger_time);

			if (elapsed > _measurement_inverval) {
				ScheduleNow();

			} else {
				ScheduleDelayed(_measurement_inverval - elapsed);
			}
		}
		break;

	default:
		break;
	}
}

bool AFBRS50::processMeasurement()
{
	perf_count(_sample_perf);

	argus_results_t res{};
	status_t evaluate_status = Argus_EvaluateData(_hnd, &res);

	if ((evaluate_status != STATUS_OK) || (res.Status != STATUS_OK)) {
		perf_count(_process_measurement_error);
		return 0;
	}

	uint32_t result_mm = res.Bin.Range / (Q9_22_ONE / 1000);
	float distance = static_cast<float>(result_mm) / 1000.f;
	int8_t quality = res.Bin.SignalQuality;

	// Signal quality indicates 100% for good signals, 50% and lower for weak signals.
	// 1% is an errored signal (not reliable). Signal Quality of 0% is unknown.
	if (quality == 1) {
		quality = 0;
	}

	if (distance > _max_distance * 1.5f) {
		distance = 0.0;
		quality = 0;
	}

	_current_distance = distance;
	_current_quality = quality;
	_px4_rangefinder.update(((res.TimeStamp.sec * 1000000ULL) + res.TimeStamp.usec), distance, quality);
	return 1;
}

// TODO: Require 3 consecutive readings on same side of fence to mode switch
void AFBRS50::updateMeasurementRateFromRange()
{
	bool valid_distance = (_current_distance > 0) && (_current_quality > 0);
	bool switch_allowed = hrt_elapsed_time(&_last_rate_switch) > 1_s;

	if (valid_distance && switch_allowed) {

		bool above_threshold = _current_distance >= (_p_sens_afbr_thresh.get() + _p_sens_afbr_hyster.get());
		bool in_long_range_mode = _current_rate == _p_sens_afbr_l_rate.get();

		bool below_threshold = _current_distance <= (_p_sens_afbr_thresh.get() - _p_sens_afbr_hyster.get());
		bool in_short_range_mode = _current_rate == _p_sens_afbr_s_rate.get();

		if (above_threshold && !in_long_range_mode) {

			_current_rate = _p_sens_afbr_l_rate.get();
			auto status = setRateAndDfm(_current_rate, DFM_MODE_8X);

			if (status != STATUS_OK) {
				PX4_ERR("set_rate status not okay: %ld", status);

			} else {
				PX4_INFO("switched to long range rate: %d", _current_rate);
				_last_rate_switch = hrt_absolute_time();
			}

		} else if (below_threshold && !in_short_range_mode) {

			_current_rate = _p_sens_afbr_s_rate.get();
			auto status = setRateAndDfm(_current_rate, DFM_MODE_OFF);

			if (status != STATUS_OK) {
				PX4_ERR("set_rate status not okay: %ld", status);

			} else {
				PX4_INFO("switched to short range rate: %d", _current_rate);
				_last_rate_switch = hrt_absolute_time();
			}
		}
	}
}

status_t AFBRS50::setRateAndDfm(uint32_t rate_hz, argus_dfm_mode_t dfm_mode)
{
	while (Argus_GetStatus(_hnd) != STATUS_IDLE) {
		px4_usleep(1_ms);
	}

	status_t status = Argus_SetConfigurationDFMMode(_hnd, dfm_mode);

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
		PX4_ERR("Argus_SetConfigurationDFMMode failed: %i", (int)status);
		return status;
	}

	status = Argus_SetConfigurationFrameTime(_hnd, (1000000 / rate_hz));

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationFrameTime status not okay: %i", (int)status);
		return status;
	}

	uint32_t current_rate;
	status = Argus_GetConfigurationFrameTime(_hnd, &current_rate);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_GetConfigurationFrameTime status not okay: %i", (int)status);
		return status;

	} else {
		_measurement_inverval = current_rate;
	}

	return status;
}

argus_mode_t AFBRS50::argusModeFromParameter()
{
	int32_t mode_param = _p_sens_afbr_mode.get();
	argus_mode_t mode = ARGUS_MODE_SHORT_RANGE;

	if (mode_param < 0 || mode_param > 3) {
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

	default:
		break;
	}

	return mode;
}

void AFBRS50::printInfo()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_callback_error);
	perf_print_counter(_process_measurement_error);
	perf_print_counter(_status_not_ready_perf);
	perf_print_counter(_trigger_fail_perf);
	perf_print_counter(_loop_perf);
	PX4_INFO_RAW("distance: %.3fm\n", (double)_current_distance);
	PX4_INFO_RAW("rate: %u Hz\n", (uint)(1000000 / _measurement_inverval));
}

namespace afbrs50
{

static int start(const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	g_dev = new AFBRS50(rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->printInfo();

	return PX4_OK;
}

static int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the Broadcom AFBRS50.

### Examples

Attempt to start driver on a specified serial device.
$ afbrs50 start
Stop driver
$ afbrs50 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("afbrs50", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int afbrs50_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option");
			return afbrs50::usage();
		}
	}

	if (myoptind >= argc) {
		return afbrs50::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return afbrs50::start(rotation);

	} else if (!strcmp(argv[myoptind], "status")) {
		return afbrs50::status();

	} else if (!strcmp(argv[myoptind], "stop")) {
		return afbrs50::stop();

	}

	return afbrs50::usage();
}
