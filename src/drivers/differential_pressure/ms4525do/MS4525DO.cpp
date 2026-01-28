/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "MS4525DO.hpp"
#include <parameters/param.h>

using namespace time_literals;

MS4525DO::MS4525DO(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

MS4525DO::~MS4525DO()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

int MS4525DO::probe()
{
	_retries = 1;

	for (int i = 0; i < 10; i++) {
		// perform 2 x Read_DF3 (Data Fetch 3 Bytes)
		//  1st read: require status = Normal Operation. Good Data Packet
		//  2nd read: require status = Stale Data, data should match first read
		uint8_t data_1[3] {};
		int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

		uint8_t data_2[3] {};
		int ret2 = transfer(nullptr, 0, &data_2[0], sizeof(data_2));

		if (ret1 == PX4_OK && ret2 == PX4_OK) {
			// Status bits
			const uint8_t status_1 = (data_1[0] & 0b1100'0000) >> 6;
			const uint8_t status_2 = (data_2[0] & 0b1100'0000) >> 6;

			if ((status_1 == (uint8_t)STATUS::Normal_Operation)
			    && (status_2 == (uint8_t)STATUS::Stale_Data)
			    && (data_1[2] == data_1[2])) {

				_retries = 1; // enable retries during operation
				return PX4_OK;

			} else {
				PX4_ERR("status: %X status: %X", status_1, status_2);
			}

		} else {
			px4_usleep(1000); // TODO
		}
	}

	return PX4_ERROR;
}

int MS4525DO::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void MS4525DO::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}

void MS4525DO::RunImpl()
{
	switch (_state) {
	case STATE::MEASURE: {
			// Send the command to begin a measurement (Read_MR)
			uint8_t cmd = ADDR_READ_MR;

			if (transfer(&cmd, 1, nullptr, 0) == OK) {
				_timestamp_sample = hrt_absolute_time();
				_state = STATE::READ;
				ScheduleDelayed(2_ms);

			} else {
				perf_count(_comms_errors);
				_state = STATE::MEASURE;
				ScheduleDelayed(10_ms); // try again in 10 ms
			}
		}

		break;

	case STATE::READ:
		// perform 2 x Read_DF4 (Data Fetch 4 Bytes)
		//  1st read: require status = Normal Operation. Good Data Packet
		//  2nd read: require status = Stale Data, data should match first read
		perf_begin(_sample_perf);
		uint8_t data_1[4] {};
		int ret1 = transfer(nullptr, 0, &data_1[0], sizeof(data_1));

		uint8_t data_2[4] {};
		int ret2 = transfer(nullptr, 0, &data_2[0], sizeof(data_2));
		perf_end(_sample_perf);

		if (ret1 != PX4_OK || ret2 != PX4_OK) {
			perf_count(_comms_errors);

		} else {
			// Status bits
			const uint8_t status_1 = (data_1[0] & 0b1100'0000) >> 6;
			const uint8_t status_2 = (data_2[0] & 0b1100'0000) >> 6;

			const uint8_t bridge_data_1_msb = (data_1[0] & 0b0011'1111);
			const uint8_t bridge_data_2_msb = (data_2[0] & 0b0011'1111);

			const uint8_t bridge_data_1_lsb = data_1[1];
			const uint8_t bridge_data_2_lsb = data_2[1];

			// Bridge Data [13:8] + Bridge Data [7:0]
			int16_t bridge_data_1 = (bridge_data_1_msb << 8) + bridge_data_1_lsb;
			int16_t bridge_data_2 = (bridge_data_2_msb << 8) + bridge_data_2_lsb;

			// 11-bit temperature data
			//  Temperature Data [10:3] + Temperature Data [2:0]
			int16_t temperature_1 = ((data_1[2] << 8) + (0b1110'0000 & data_1[3])) / (1 << 5);
			int16_t temperature_2 = ((data_2[2] << 8) + (0b1110'0000 & data_2[3])) / (1 << 5);

			if ((status_1 == (uint8_t)STATUS::Fault_Detected) || (status_2 == (uint8_t)STATUS::Fault_Detected)) {
				// Fault Detected
				perf_count(_fault_perf);

			} else if ((status_1 == (uint8_t)STATUS::Normal_Operation) && (status_2 == (uint8_t)STATUS::Stale_Data)
				   && (bridge_data_1_msb == bridge_data_2_msb) && (temperature_1 == temperature_2)) {

				float temperature_c = ((200.f * temperature_1) / 2047) - 50.f;

				// Output is proportional to the difference between Port 1 and Port 2. Output swings
				// positive when Port 1> Port 2. Output is 50% of supply voltage when Port 1=Port 2.

				// Calculate differential pressure. As its centered around 8000
				// and can go positive or negative
				static constexpr float P_min = -1.f; // -1 PSI
				static constexpr float P_max = 1.f;  // +1 PSI

				// this equation is an inversion of the equation in the
				// pressure transfer function figure on page 4 of the datasheet

				// We negate the result so that positive differential pressures
				// are generated when the bottom port is used as the static
				// port on the pitot and top port is used as the dynamic port
				const float diff_press_PSI = -((bridge_data_1 - 0.1f * 16383.f) * (P_max - P_min) / (0.8f * 16383.f) + P_min);

				static constexpr float PSI_to_Pa = 6894.757f;
				float diff_press_pa = diff_press_PSI * PSI_to_Pa;

				if (hrt_elapsed_time(&_timestamp_sample) < 20_ms) {
					differential_pressure_s differential_pressure{};
					differential_pressure.timestamp_sample = _timestamp_sample;
					differential_pressure.device_id = get_device_id();
					differential_pressure.differential_pressure_pa = diff_press_pa;
					int32_t differential_press_rev = 0;
					param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

					//If differential pressure reverse param set, swap positive and negative
					if (differential_press_rev == 1) {
						differential_pressure.differential_pressure_pa = -1.0f * diff_press_pa;
					}

					differential_pressure.temperature = temperature_c;
					differential_pressure.error_count = perf_event_count(_comms_errors);
					differential_pressure.timestamp = hrt_absolute_time();
					_differential_pressure_pub.publish(differential_pressure);

					_timestamp_sample = 0;
				}

			} else {
				PX4_DEBUG("status:%X|%X, B:%X|%X, T:%X|%X", status_1, status_2, bridge_data_1, bridge_data_2, temperature_1,
					  temperature_2);
			}
		}

		_state = STATE::MEASURE;
		ScheduleDelayed(10_ms);
		break;
	}
}
