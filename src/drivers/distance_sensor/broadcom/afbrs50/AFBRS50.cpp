/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/* Include Files */
#include "AFBRS50.hpp"
#include "argus_hal_test.h"

#include <lib/drivers/device/Device.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000

#include "s2pi.h"
#include "timer.h"
#include "argus_hal_test.h"

AFBRS50 *g_dev{nullptr};

AFBRS50::AFBRS50(uint8_t device_orientation):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
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
	stop();

	perf_free(_sample_perf);
}

status_t AFBRS50::measurement_ready_callback(status_t status, argus_hnd_t *hnd)
{
	if (!up_interrupt_context()) {
		if (status == STATUS_OK) {
			if (g_dev) {
				g_dev->ProcessMeasurement(hnd);
			}

		} else {
			PX4_ERR("Measurement Ready Callback received error!: %i", (int)status);
		}
	}

	return status;
}

void AFBRS50::ProcessMeasurement(argus_hnd_t *hnd)
{
	perf_count(_sample_perf);

	argus_results_t res{};
	status_t evaluate_status = Argus_EvaluateData(hnd, &res);

	if ((evaluate_status == STATUS_OK) && (res.Status == STATUS_OK)) {
		uint32_t result_mm = res.Bin.Range / (Q9_22_ONE / 1000);
		float result_m = static_cast<float>(result_mm) / 1000.f;
		int8_t quality = res.Bin.SignalQuality;

		// Signal quality indicates 100% for good signals, 50% and lower for weak signals.
		// 1% is an errored signal (not reliable). Signal Quality of 0% is unknown.
		if (quality == 1) {
			quality = 0;
		}

		// distance quality check
		if (result_m > _max_distance) {
			result_m = 0.0;
			quality = 0;
		}

		_current_distance = result_m;
		_current_quality = quality;
		_px4_rangefinder.update(((res.TimeStamp.sec * 1000000ULL) + res.TimeStamp.usec), result_m, quality);
	}
}

int AFBRS50::init()
{
	// Retry initialization 3 times
	for (int32_t i = 0; i < 3; i++) {
		if (_hnd != nullptr) {
			// retry
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
		S2PI_Init(BROADCOM_AFBR_S50_S2PI_SPI_BUS, SPI_BAUD_RATE);

		int32_t mode_param = _p_sens_afbr_mode.get();

		if (mode_param < 0 || mode_param > 3) {
			PX4_ERR("Invalid mode parameter: %li", mode_param);
			return PX4_ERROR;
		}

		argus_mode_t mode = ARGUS_MODE_SHORT_RANGE;

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

		status_t status = Argus_InitMode(_hnd, BROADCOM_AFBR_S50_S2PI_SPI_BUS, mode);

		if (status == STATUS_OK) {
			uint32_t id = Argus_GetChipID(_hnd);
			uint32_t value = Argus_GetAPIVersion();
			uint8_t a = (value >> 24) & 0xFFU;
			uint8_t b = (value >> 16) & 0xFFU;
			uint8_t c = value & 0xFFFFU;
			PX4_INFO_RAW("AFBR-S50 Chip ID: %u, API Version: %u v%d.%d.%d\n", (uint)id, (uint)value, a, b, c);

			argus_module_version_t mv = Argus_GetModuleVersion(_hnd);

			switch (mv) {
			case AFBR_S50MV85G_V1:

			// FALLTHROUGH
			case AFBR_S50MV85G_V2:

			// FALLTHROUGH
			case AFBR_S50MV85G_V3:
				_min_distance = 0.0f;
				_max_distance = 10.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(6.f));
				PX4_INFO_RAW("AFBR-S50MV85G\n");
				break;

			case AFBR_S50LV85D_V1:
				_min_distance = 0.0f;
				_max_distance = 30.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(6.f));
				PX4_INFO_RAW("AFBR-S50LV85D\n");
				break;

			case AFBR_S50LX85D_V1:
				_min_distance = 0.0f;
				_max_distance = 50.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(6.f));
				PX4_INFO_RAW("AFBR-S50LX85D\n");
				break;

			case AFBR_S50MV68B_V1:
				_min_distance = 0.0f;
				_max_distance = 10.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(1.f));
				PX4_INFO_RAW("AFBR-S50MV68B (v1)\n");
				break;

			case AFBR_S50MV85I_V1:
				_min_distance = 0.0f;
				_max_distance = 5.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(6.f));
				PX4_INFO_RAW("AFBR-S50MV85I (v1)\n");
				break;

			case AFBR_S50SV85K_V1:
				_min_distance = 0.0f;
				_max_distance = 10.f;
				_px4_rangefinder.set_min_distance(_min_distance);
				_px4_rangefinder.set_max_distance(_max_distance);
				_px4_rangefinder.set_fov(math::radians(4.f));
				PX4_INFO_RAW("AFBR-S50SV85K (v1)\n");
				break;

			default:
				break;
			}

			if (_testing) {
				_state = STATE::TEST;

			} else {
				_state = STATE::CONFIGURE;
			}

			ScheduleDelayed(_measure_interval);
			return PX4_OK;

		} else {
			PX4_ERR("Argus_InitMode failed: %ld", status);
		}
	}

	return PX4_ERROR;
}

void AFBRS50::Run()
{
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}

	switch (_state) {
	case STATE::TEST: {
			if (_testing) {
				Argus_VerifyHALImplementation(Argus_GetSPISlave(_hnd));
				_testing = false;

			} else {
				_state = STATE::CONFIGURE;
			}
		}
		break;

	case STATE::CONFIGURE: {
			_current_rate = (uint32_t)_p_sens_afbr_s_rate.get();
			status_t status = set_rate_and_dfm(_current_rate, DFM_MODE_OFF);

			if (status != STATUS_OK) {
				PX4_ERR("CONFIGURE status not okay: %i", (int)status);
				_state = STATE::STOP;
				ScheduleNow();
			}

			status = Argus_SetConfigurationSmartPowerSaveEnabled(_hnd, false);

			if (status != STATUS_OK) {
				PX4_ERR("Argus_SetConfigurationSmartPowerSaveEnabled status not okay: %i", (int)status);
				ScheduleNow();

			} else {
				_state = STATE::COLLECT;
				ScheduleDelayed(_measure_interval);
			}
		}
		break;

	case STATE::COLLECT: {
			// Only start a new measurement if one is not ongoing
			if (Argus_GetStatus(_hnd) == STATUS_IDLE) {
				status_t status = Argus_TriggerMeasurement(_hnd, measurement_ready_callback);

				if (status != STATUS_OK) {
					PX4_ERR("Argus_TriggerMeasurement status not okay: %i", (int)status);
				}
			}

			Evaluate_rate();
		}
		break;

	case STATE::STOP: {
			Argus_StopMeasurementTimer(_hnd);
			Argus_Deinit(_hnd);
			Argus_DestroyHandle(_hnd);
		}
		break;

	default:
		break;
	}

	ScheduleDelayed(_measure_interval);
}

void AFBRS50::Evaluate_rate()
{
	// only update mode if _current_distance is a valid measurement and if the last rate switch was more than 1 second ago
	if ((_current_distance > 0) && (_current_quality > 0) && ((hrt_absolute_time() - _last_rate_switch) > 1_s)) {

		status_t status = STATUS_OK;

		if ((_current_distance >= (_p_sens_afbr_thresh.get() + _p_sens_afbr_hyster.get()))
		    && (_current_rate != (uint32_t)_p_sens_afbr_l_rate.get())) {

			_current_rate = (uint32_t)_p_sens_afbr_l_rate.get();
			status = set_rate_and_dfm(_current_rate, DFM_MODE_8X);

			if (status != STATUS_OK) {
				PX4_ERR("set_rate status not okay: %i", (int)status);

			} else {
				PX4_INFO("switched to long range rate: %i", (int)_current_rate);
				_last_rate_switch = hrt_absolute_time();
			}

		} else if ((_current_distance <= (_p_sens_afbr_thresh.get() - _p_sens_afbr_hyster.get()))
			   && (_current_rate != (uint32_t)_p_sens_afbr_s_rate.get())) {

			_current_rate = (uint32_t)_p_sens_afbr_s_rate.get();
			status = set_rate_and_dfm(_current_rate, DFM_MODE_OFF);

			if (status != STATUS_OK) {
				PX4_ERR("set_rate status not okay: %i", (int)status);

			} else {
				PX4_INFO("switched to short range rate: %i", (int)_current_rate);
				_last_rate_switch = hrt_absolute_time();
			}
		}
	}
}

void AFBRS50::stop()
{
	_state = STATE::STOP;
	ScheduleNow();
}

int AFBRS50::test()
{
	_testing = true;

	init();

	return PX4_OK;
}

void AFBRS50::print_info()
{
	perf_print_counter(_sample_perf);
	get_info();
}

status_t AFBRS50::set_rate_and_dfm(uint32_t rate_hz, argus_dfm_mode_t dfm_mode)
{
	while (Argus_GetStatus(_hnd) != STATUS_IDLE) {
		px4_usleep(1_ms);
	}

	status_t status = Argus_SetConfigurationDFMMode(_hnd, dfm_mode);

	if (status != STATUS_OK) {
		PX4_ERR("Argus_SetConfigurationDFMMode status not okay: %i", (int)status);
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
		_measure_interval = current_rate;
	}

	return status;
}

void AFBRS50::get_info()
{
	argus_dfm_mode_t dfm_mode;
	Argus_GetConfigurationDFMMode(_hnd, &dfm_mode);

	PX4_INFO_RAW("distance: %.3fm\n", (double)_current_distance);
	PX4_INFO_RAW("dfm mode: %d\n", dfm_mode);
	PX4_INFO_RAW("rate: %u Hz\n", (uint)(1000000 / _measure_interval));
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

	// Initialize the sensor.
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

	g_dev->print_info();

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

static int test(const uint8_t rotation)
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

	if (g_dev->test() != PX4_OK) {
		PX4_ERR("driver test failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test driver");
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

	} else if (!strcmp(argv[myoptind], "test")) {
		return afbrs50::test(rotation);

	}

	return afbrs50::usage();
}
