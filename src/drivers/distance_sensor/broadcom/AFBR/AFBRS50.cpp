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

#include <lib/drivers/device/Device.hpp>

#define AFBRS50_FIELD_OF_VIEW        (0.105f) // 6 deg cone angle.
#define AFBRS50_MAX_DISTANCE         30.0f
#define AFBRS50_MIN_DISTANCE         0.01f
#define AFBRS50_MEASURE_INTERVAL     (1000000 / 100) // 10Hz

/*! Define the SPI slave (to be used in the SPI module). */
#define SPI_SLAVE 2
/*! Define the SPI baud rate (to be used in the SPI module). */
#define SPI_BAUD_RATE 5000000

#include "s2pi.h"
#include "timer.h"

static argus_hnd_t *_hnd{nullptr};
static volatile void *_myData{nullptr};

AFBRS50::AFBRS50(uint8_t device_orientation):
	ScheduledWorkItem(MODULE_NAME, px4::ins_instance_to_wq(0)),
	_px4_rangefinder(0, device_orientation)
{
	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SPI;

	uint8_t bus_num = 0;

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_AFBRS50);

	_px4_rangefinder.set_max_distance(AFBRS50_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(AFBRS50_MIN_DISTANCE);
	_px4_rangefinder.set_fov(AFBRS50_FIELD_OF_VIEW);
}

AFBRS50::~AFBRS50()
{
	stop();

	perf_free(_comms_error);
	perf_free(_sample_perf);
}

static status_t measurement_ready_callback(status_t status, void *data)
{
	if (status != STATUS_OK) {
		/* Error Handling ...*/
		_myData = nullptr;

	} else {
		/* Inform the main task about new data ready.
		 * Note: do not call the evaluate measurement method
		 * from within this callback since it is invoked in
		 * a interrupt service routine and should return as
		 * soon as possible. */
		assert(_myData == 0);

		_myData = data;
	}

	return status;
}

int AFBRS50::init()
{
	_hnd = Argus_CreateHandle();

	if (_hnd == 0) {
		PX4_ERR("Handle not initialized");
		Argus_DestroyHandle(_hnd);
		return PX4_ERROR;
	}

	// Initialize the S2PI hardware required by the API.
	S2PI_Init(SPI_SLAVE, SPI_BAUD_RATE);

	status_t status = Argus_Init(_hnd, SPI_SLAVE);

	if (status != STATUS_OK) {
		PX4_ERR("Init status not okay: %i", status);
		Argus_Deinit(_hnd);
		Argus_DestroyHandle(_hnd);
		return PX4_ERROR;
	}

	// Schedule the driver at regular intervals.
	ScheduleOnInterval(AFBRS50_MEASURE_INTERVAL, AFBRS50_MEASURE_INTERVAL);

	return PX4_OK;
}

void AFBRS50::Run()
{
	switch (_state) {
	case STATE::CONFIGURE: {
			uint32_t value = Argus_GetAPIVersion();
			PX4_INFO_RAW("AFBR API Version %d\n", value);

			uint8_t a = (value >> 24) & 0xFFU;
			uint8_t b = (value >> 16) & 0xFFU;
			uint8_t c = value & 0xFFFFU;
			PX4_INFO_RAW("AFBR API Version: v%d.%d.%d\n", a, b, c);

			uint32_t id = Argus_GetChipID(_hnd);
			PX4_INFO_RAW("Chip ID: %d\n", id);

			argus_module_version_t mv = Argus_GetModuleVersion(_hnd);

			switch (mv) {
			case AFBR_S50MV85G_V1:
				PX4_INFO_RAW("AFBR-S50MV85G (v1)\n");
				break;

			case AFBR_S50MV85G_V2:
				PX4_INFO_RAW("AFBR-S50MV85G (v2)\n");
				break;

			case AFBR_S50MV85G_V3:
				PX4_INFO_RAW("AFBR-S50MV85G (v3)\n");
				break;

			case AFBR_S50LV85D_V1:
				PX4_INFO_RAW("AFBR-S50LV85D (v1)\n");
				break;

			case AFBR_S50MV68B_V1:
				PX4_INFO_RAW("AFBR-S50MV68B (v1)\n");
				break;

			case AFBR_S50MV85I_V1:
				PX4_INFO_RAW("AFBR-S50MV85I (v1)\n");
				break;

			case AFBR_S50SV85K_V1:
				PX4_INFO_RAW("AFBR-S50SV85K (v1)\n");
				break;

			default:
				//"unknown";
				break;
			}

			Argus_SetConfigurationFrameTime(_hnd, AFBRS50_MEASURE_INTERVAL);

			status_t status = Argus_StartMeasurementTimer(_hnd, measurement_ready_callback);

			if (status != STATUS_OK) {
				PX4_ERR("CONFIGURE status not okay: %i", status);
				Argus_Deinit(_hnd);
				Argus_DestroyHandle(_hnd);
				stop();
			}

			_state = STATE::COLLECT;
			ScheduleDelayed(AFBRS50_MEASURE_INTERVAL);
		}
		break;

	case STATE::COLLECT: {

			if (_myData != nullptr) {
				/* Release for next measurement data. */
				void *data = (void *) _myData;
				_myData = nullptr;

				argus_results_t res{};
				status_t status = Argus_EvaluateData(_hnd, &res, (void *)data);

				if (status == STATUS_OK) {
					if (res.Status == 0) {
						float result = res.Bin.Range / (Q9_22_ONE / 1000);
						result = result / 1000.0f;

						//fprintf(stderr, "result = %.3f m\n", (double)result);

						_px4_rangefinder.update(((res.TimeStamp.sec * 1000000ULL) + res.TimeStamp.usec), result);
					}

					_state = STATE::COLLECT;
					ScheduleDelayed(1_ms);
					break;

				} else {
					// check again
					ScheduleDelayed(1_ms);
				}

			} else {
				// retry
				_state = STATE::COLLECT;
				ScheduleDelayed(1_ms);
			}
		}
		break;

	default:
		break;
	}
}

void AFBRS50::stop()
{
	Argus_Deinit(_hnd);
	Argus_DestroyHandle(_hnd);
	// Clear the work queue schedule.
	ScheduleClear();
}

void AFBRS50::print_info()
{
	perf_print_counter(_comms_error);
	perf_print_counter(_sample_perf);
}
