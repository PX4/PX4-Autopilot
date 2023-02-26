/****************************************************************************
 *
 *   Copyright (c) 2023-2023 PX4 Development Team. All rights reserved.
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

#include "VirtualIMU.hpp"

#include <drivers/drv_hrt.h>

using namespace matrix;

VirtualIMU::VirtualIMU() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::virtual_imu),
	_px4_accel(_accel_device_id),
	_px4_gyro(_gyro_device_id)
{

}

VirtualIMU::~VirtualIMU()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);

	for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		perf_free(_accel_fifo_generation_gap_perf[i]);
		perf_free(_gyro_fifo_generation_gap_perf[i]);
	}
}

bool VirtualIMU::init()
{
	ScheduleNow();
	return true;
}

void VirtualIMU::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	sensor_gyro_fifo_s sensor_gyro_fifo[MAX_SENSOR_COUNT];
	sensor_accel_fifo_s sensor_accel_fifo[MAX_SENSOR_COUNT];

	switch (_state) {
	case STATE::CONFIGURE:

		if ((_gyro_fifo_subs.advertised_count() != MAX_SENSOR_COUNT)
		    || (_accel_fifo_subs.advertised_count() != MAX_SENSOR_COUNT)) {
			// wait for all imu instances to be advertised
			ScheduleDelayed(10_ms);

		} else {

			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				_gyro_fifo_subs[i].copy(&sensor_gyro_fifo[i]);
				_accel_fifo_subs[i].copy(&sensor_accel_fifo[i]);

				_gyro_device_id ^= sensor_gyro_fifo[i].device_id;
				_accel_device_id ^= sensor_accel_fifo[i].device_id;
			}

			_px4_accel.set_device_id(_accel_device_id);
			_px4_gyro.set_device_id(_gyro_device_id);

			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (_gyro_fifo_generation_gap_perf[i] == nullptr) {
					_gyro_fifo_generation_gap_perf[i] = perf_alloc(PC_COUNT, MODULE_NAME": gyro FIFO data gap");
				}

				if (_accel_fifo_generation_gap_perf[i] == nullptr) {
					_accel_fifo_generation_gap_perf[i] = perf_alloc(PC_COUNT, MODULE_NAME": accel FIFO data gap");
				}
			}

			_state = STATE::RUN;

			check_newest_timestamp_and_register_callback(sensor_gyro_fifo, sensor_accel_fifo);
		}

		break;

	case STATE::RUN:

		bool gyro_updated = false;
		bool accel_updated = false;

		while (_gyro_fifo_subs.updated()) {
			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (_gyro_fifo_subs[i].update(&sensor_gyro_fifo[i])) {

					if (_gyro_fifo_subs[i].get_last_generation() != _gyro_last_generation[i] + 1) {
						perf_count(_gyro_fifo_generation_gap_perf[i]);
					}

					_gyro_last_generation[i] = _gyro_fifo_subs[i].get_last_generation();

					for (size_t j = 0; j < sensor_gyro_fifo[i].samples; j++) {

						gyroFIFOSample gyro_fifo_sample;
						gyro_fifo_sample.time_us = sensor_gyro_fifo[i].timestamp_sample - (((sensor_gyro_fifo[i].samples - 1) - j) *
									   sensor_gyro_fifo[i].dt);
						gyro_fifo_sample.data[0] = sensor_gyro_fifo[i].x[j];
						gyro_fifo_sample.data[1] = sensor_gyro_fifo[i].y[j];
						gyro_fifo_sample.data[2] = sensor_gyro_fifo[i].z[j];
						gyro_fifo_sample.dt = sensor_gyro_fifo[i].dt;
						gyro_fifo_sample.scale = sensor_gyro_fifo[i].scale;

						_gyro_fifo_buffer[i].push(gyro_fifo_sample);

						gyro_updated = true;
					}
				}
			}
		}

		sensor_gyro_s sensor_gyro[MAX_SENSOR_COUNT];
		float average_temperature = 0.f;
		size_t average_count = 0;

		while (_sensor_gyro_subs.updated()) {
			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (_sensor_gyro_subs[i].update(&sensor_gyro[i])) {
					average_temperature += sensor_gyro[i].temperature;
					average_count++;
				}
			}
		}

		_px4_gyro.set_temperature(average_temperature / average_count);

		while (_accel_fifo_subs.updated()) {
			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (_accel_fifo_subs[i].update(&sensor_accel_fifo[i])) {

					if (_accel_fifo_subs[i].get_last_generation() != _accel_last_generation[i] + 1) {
						perf_count(_accel_fifo_generation_gap_perf[i]);
					}

					_accel_last_generation[i] = _accel_fifo_subs[i].get_last_generation();

					for (size_t j = 0; j < sensor_accel_fifo[i].samples; j++) {

						accelFIFOSample accel_fifo_sample;
						accel_fifo_sample.time_us = sensor_accel_fifo[i].timestamp_sample - (((sensor_accel_fifo[i].samples - 1) - j) *
									    sensor_accel_fifo[i].dt);
						accel_fifo_sample.data[0] = sensor_accel_fifo[i].x[j];
						accel_fifo_sample.data[1] = sensor_accel_fifo[i].y[j];
						accel_fifo_sample.data[2] = sensor_accel_fifo[i].z[j];
						accel_fifo_sample.dt = sensor_accel_fifo[i].dt;
						accel_fifo_sample.scale = sensor_accel_fifo[i].scale;

						_accel_fifo_buffer[i].push(accel_fifo_sample);

						accel_updated = true;
					}
				}
			}
		}

		if (gyro_updated && accel_updated) {
			check_newest_timestamp_and_register_callback(sensor_gyro_fifo, sensor_accel_fifo);
		}

		sensor_accel_s sensor_accel[MAX_SENSOR_COUNT];
		average_temperature = 0.f;
		average_count = 0;

		while (_sensor_accel_subs.updated()) {
			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (_sensor_accel_subs[i].update(&sensor_accel[i])) {
					average_temperature += sensor_accel[i].temperature;
					average_count++;
				}
			}
		}

		_px4_accel.set_temperature(average_temperature / average_count);

		process_gyro();

		process_accel();

		break;
	}

	perf_end(_cycle_perf);
}

// void VirtualIMU::process_gyro()
// {

// 	bool gyro_data_available = true;
// 	gyroFIFOSample gyro_fifo_sample[MAX_SENSOR_COUNT];

// 	sensor_gyro_fifo_s sensor_gyro_fifo_median{};
// 	Vector3f gyro_data_median[32] {};

// 	sensor_gyro_fifo_median.samples = 0;
// 	sensor_gyro_fifo_median.scale = 0.0f;
// 	sensor_gyro_fifo_median.dt = 0.0f;

// 	// find the oldest and newest sample
// 	uint64_t oldest_sample_us = UINT64_MAX;
// 	uint64_t newest_sample_us = 0;
// 	size_t oldest_sample_index = 0;

// 	while (gyro_data_available) {

// 		// Check if all FIFOs have data
// 		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 			if (_gyro_fifo_buffer[i].entries() == 0) {
// 				gyro_data_available = false;
// 				break;
// 			}
// 		}

// 		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 			_gyro_fifo_buffer[i].pop_oldest(&gyro_fifo_sample[i]);

// 			if (gyro_fifo_sample[i].time_us < oldest_sample_us) {
// 				oldest_sample_us = gyro_fifo_sample[i].time_us;
// 				oldest_sample_index = i;
// 			}

// 			if (gyro_fifo_sample[i].time_us > newest_sample_us) {
// 				newest_sample_us = gyro_fifo_sample[i].time_us;
// 			}
// 		}

// 		uint64_t timestamp_diff = newest_sample_us - oldest_sample_us;

// 		while (timestamp_diff > MAX_TIMESTAMP_GYRO_DT_US) {
// 			// we have a sample gap, discard the oldest sample
// 			if (_gyro_fifo_buffer[oldest_sample_index].pop_oldest(&gyro_fifo_sample[oldest_sample_index]) == true) {

// 				oldest_sample_us = UINT64_MAX;
// 				newest_sample_us = 0;

// 				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 					if (gyro_fifo_sample[i].time_us < oldest_sample_us) {
// 						oldest_sample_us = gyro_fifo_sample[i].time_us;
// 						oldest_sample_index = i;
// 					}

// 					if (gyro_fifo_sample[i].time_us > newest_sample_us) {
// 						newest_sample_us = gyro_fifo_sample[i].time_us;
// 					}
// 				}

// 				timestamp_diff = newest_sample_us - oldest_sample_us;

// 			} else {
// 				// we have a sample gap, but no data to fill it
// 				gyro_data_available = false;
// 				break;
// 			}
// 		}

// 		if (gyro_data_available)  {
// 			sensor_gyro_fifo_median.timestamp_sample = newest_sample_us;

// 			float x_median = 0.f;
// 			float y_median = 0.f;
// 			float z_median = 0.f;

// 			float median_scale = 0.f;

// 			if (_param_gyro_axis_median.get() == 1) {
// 				// Find the median of each fifo datapoint
// 				x_median = median(gyro_fifo_sample[0].data[0] * gyro_fifo_sample[0].scale,
// 						  gyro_fifo_sample[1].data[0] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[0] * gyro_fifo_sample[2].scale);

// 				y_median = median(gyro_fifo_sample[0].data[1] * gyro_fifo_sample[0].scale,
// 						  gyro_fifo_sample[1].data[1] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[1] * gyro_fifo_sample[2].scale);

// 				z_median = median(gyro_fifo_sample[0].data[2] * gyro_fifo_sample[0].scale,
// 						  gyro_fifo_sample[1].data[2] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[2] * gyro_fifo_sample[2].scale);

// 				// Average the dt and the highest scale value
// 				float dt_sum = 0.0f;

// 				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 					dt_sum = dt_sum + gyro_fifo_sample[i].dt;

// 					if (gyro_fifo_sample[i].scale > median_scale) {
// 						median_scale = gyro_fifo_sample[i].scale;
// 					}
// 				}

// 				dt_sum = dt_sum / MAX_SENSOR_COUNT;
// 				sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt + dt_sum;

// 			} else {
// 				// Calculate the magnitude of each vector and find the index of the median
// 				float magnitude[MAX_SENSOR_COUNT];

// 				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 					float x = gyro_fifo_sample[i].data[0] * gyro_fifo_sample[i].scale;
// 					float y = gyro_fifo_sample[i].data[1] * gyro_fifo_sample[i].scale;
// 					float z = gyro_fifo_sample[i].data[2] * gyro_fifo_sample[i].scale;
// 					magnitude[i] = sqrtf(x * x + y * y + z * z);
// 				}

// 				size_t median_index = find_median_index(magnitude[0], magnitude[1], magnitude[2]);

// 				x_median = gyro_fifo_sample[median_index].data[0] * gyro_fifo_sample[median_index].scale;
// 				y_median = gyro_fifo_sample[median_index].data[1] * gyro_fifo_sample[median_index].scale;
// 				z_median = gyro_fifo_sample[median_index].data[2] * gyro_fifo_sample[median_index].scale;

// 				median_scale = gyro_fifo_sample[median_index].scale;
// 				sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt + gyro_fifo_sample[median_index].dt;
// 			}

// 			// Keep track of the highest scale value
// 			if (median_scale > sensor_gyro_fifo_median.scale) {
// 				sensor_gyro_fifo_median.scale = median_scale;
// 			}

// 			// Fill the temporary buffer will adjust for the largest scale later
// 			gyro_data_median[sensor_gyro_fifo_median.samples](0) = x_median;
// 			gyro_data_median[sensor_gyro_fifo_median.samples](1) = y_median;
// 			gyro_data_median[sensor_gyro_fifo_median.samples](2) = z_median;

// 			sensor_gyro_fifo_median.samples++;

// 			if (sensor_gyro_fifo_median.samples == 32) {
// 				break;
// 			}
// 		}
// 	}

// 	if (sensor_gyro_fifo_median.samples > 0) {

// 		for (size_t i = 0; i < sensor_gyro_fifo_median.samples; i++) {
// 			sensor_gyro_fifo_median.x[i] = (int16_t)(gyro_data_median[i](0) / sensor_gyro_fifo_median.scale);
// 			sensor_gyro_fifo_median.y[i] = (int16_t)(gyro_data_median[i](1) / sensor_gyro_fifo_median.scale);
// 			sensor_gyro_fifo_median.z[i] = (int16_t)(gyro_data_median[i](2) / sensor_gyro_fifo_median.scale);
// 		}

// 		_last_gyro_timestamp = sensor_gyro_fifo_median.timestamp_sample;

// 		_px4_gyro.set_scale(sensor_gyro_fifo_median.scale);

// 		sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt / sensor_gyro_fifo_median.samples;
// 		sensor_gyro_fifo_median.device_id = _gyro_device_id;
// 		sensor_gyro_fifo_median.timestamp = hrt_absolute_time();

// 		_px4_gyro.updateFIFO(sensor_gyro_fifo_median);
// 	}
// }

void VirtualIMU::process_gyro()
{
	gyroFIFOSample gyro_fifo_sample[MAX_SENSOR_COUNT];

	sensor_gyro_fifo_s sensor_gyro_fifo_median{};
	Vector3f gyro_data_median[32] {};

	sensor_gyro_fifo_median.samples = 0;
	sensor_gyro_fifo_median.scale = 0.0f;
	sensor_gyro_fifo_median.dt = 0.0f;

	uint64_t now = hrt_absolute_time();

	for (uint64_t time = _last_gyro_timestamp + _median_gyro_dt_us; time < now; time += _median_gyro_dt_us) {

		bool gyro_data_available = false;

		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			gyro_data_available = _gyro_fifo_buffer[i].pop_oldest(time - _median_gyro_dt_us / 2, time + _median_gyro_dt_us / 2,
					      &gyro_fifo_sample[i]);
		}

		if (gyro_data_available)  {
			sensor_gyro_fifo_median.timestamp_sample = 0;

			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (gyro_fifo_sample[i].time_us > sensor_gyro_fifo_median.timestamp_sample) {
					sensor_gyro_fifo_median.timestamp_sample = gyro_fifo_sample[i].time_us;
				}
			}

			float x_median = 0.f;
			float y_median = 0.f;
			float z_median = 0.f;

			float median_scale = 0.f;

			if (_param_gyro_axis_median.get() == 1) {
				// Find the median of each fifo datapoint
				x_median = median(gyro_fifo_sample[0].data[0] * gyro_fifo_sample[0].scale,
						  gyro_fifo_sample[1].data[0] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[0] * gyro_fifo_sample[2].scale);

				y_median = median(gyro_fifo_sample[0].data[1] * gyro_fifo_sample[0].scale,
						  gyro_fifo_sample[1].data[1] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[1] * gyro_fifo_sample[2].scale);

				z_median = median(gyro_fifo_sample[0].data[2] * gyro_fifo_sample[0].scale,
						  gyro_fifo_sample[1].data[2] * gyro_fifo_sample[1].scale, gyro_fifo_sample[2].data[2] * gyro_fifo_sample[2].scale);

				// Average the dt and the highest scale value
				float dt_sum = 0.0f;

				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
					dt_sum = dt_sum + gyro_fifo_sample[i].dt;

					if (gyro_fifo_sample[i].scale > median_scale) {
						median_scale = gyro_fifo_sample[i].scale;
					}
				}

				dt_sum = dt_sum / MAX_SENSOR_COUNT;
				sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt + dt_sum;

			} else {
				// Calculate the magnitude of each vector and find the index of the median
				float magnitude[MAX_SENSOR_COUNT];

				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
					float x = gyro_fifo_sample[i].data[0] * gyro_fifo_sample[i].scale;
					float y = gyro_fifo_sample[i].data[1] * gyro_fifo_sample[i].scale;
					float z = gyro_fifo_sample[i].data[2] * gyro_fifo_sample[i].scale;
					magnitude[i] = sqrtf(x * x + y * y + z * z);
				}

				size_t median_index = find_median_index(magnitude[0], magnitude[1], magnitude[2]);

				x_median = gyro_fifo_sample[median_index].data[0] * gyro_fifo_sample[median_index].scale;
				y_median = gyro_fifo_sample[median_index].data[1] * gyro_fifo_sample[median_index].scale;
				z_median = gyro_fifo_sample[median_index].data[2] * gyro_fifo_sample[median_index].scale;

				median_scale = gyro_fifo_sample[median_index].scale;
				sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt + gyro_fifo_sample[median_index].dt;
			}

			// Keep track of the highest scale value
			if (median_scale > sensor_gyro_fifo_median.scale) {
				sensor_gyro_fifo_median.scale = median_scale;
			}

			// Fill the temporary buffer will adjust for the largest scale later
			gyro_data_median[sensor_gyro_fifo_median.samples](0) = x_median;
			gyro_data_median[sensor_gyro_fifo_median.samples](1) = y_median;
			gyro_data_median[sensor_gyro_fifo_median.samples](2) = z_median;

			sensor_gyro_fifo_median.samples++;

			if (sensor_gyro_fifo_median.samples == 32) {
				break;
			}
		}
	}

	if (sensor_gyro_fifo_median.samples > 0) {

		for (size_t i = 0; i < sensor_gyro_fifo_median.samples; i++) {
			sensor_gyro_fifo_median.x[i] = (int16_t)(gyro_data_median[i](0) / sensor_gyro_fifo_median.scale);
			sensor_gyro_fifo_median.y[i] = (int16_t)(gyro_data_median[i](1) / sensor_gyro_fifo_median.scale);
			sensor_gyro_fifo_median.z[i] = (int16_t)(gyro_data_median[i](2) / sensor_gyro_fifo_median.scale);
		}

		_px4_gyro.set_scale(sensor_gyro_fifo_median.scale);

		sensor_gyro_fifo_median.dt = sensor_gyro_fifo_median.dt / sensor_gyro_fifo_median.samples;
		sensor_gyro_fifo_median.device_id = _gyro_device_id;
		sensor_gyro_fifo_median.timestamp = hrt_absolute_time();

		_px4_gyro.updateFIFO(sensor_gyro_fifo_median);

		_median_gyro_dt_us = (uint16_t)sensor_gyro_fifo_median.dt;
		_last_gyro_timestamp = sensor_gyro_fifo_median.timestamp_sample;
	}
}

// void VirtualIMU::process_accel()
// {

// 	bool accel_data_available = true;
// 	accelFIFOSample accel_fifo_sample[MAX_SENSOR_COUNT];

// 	sensor_accel_fifo_s sensor_accel_fifo_median{};
// 	Vector3f accel_data_median[32] {};

// 	sensor_accel_fifo_median.samples = 0;
// 	sensor_accel_fifo_median.scale = 0.0f;
// 	sensor_accel_fifo_median.dt = 0.0f;

// 	// find the oldest and newest sample
// 	uint64_t oldest_sample_us = UINT64_MAX;
// 	uint64_t newest_sample_us = 0;
// 	size_t oldest_sample_index = 0;

// 	while (accel_data_available) {

// 		// Check if all FIFOs have data
// 		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 			if (_accel_fifo_buffer[i].entries() == 0) {
// 				accel_data_available = false;
// 				break;
// 			}
// 		}

// 		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 			_accel_fifo_buffer[i].pop_oldest(&accel_fifo_sample[i]);

// 			if (accel_fifo_sample[i].time_us < oldest_sample_us) {
// 				oldest_sample_us = accel_fifo_sample[i].time_us;
// 				oldest_sample_index = i;
// 			}

// 			if (accel_fifo_sample[i].time_us > newest_sample_us) {
// 				newest_sample_us = accel_fifo_sample[i].time_us;
// 			}
// 		}

// 		uint64_t timestamp_diff = newest_sample_us - oldest_sample_us;

// 		while (timestamp_diff > MAX_TIMESTAMP_ACCEL_DT_US) {
// 			// we have a sample gap, discard the oldest sample
// 			if (_accel_fifo_buffer[oldest_sample_index].pop_oldest(&accel_fifo_sample[oldest_sample_index]) == true) {

// 				oldest_sample_us = UINT64_MAX;
// 				newest_sample_us = 0;

// 				for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 					if (accel_fifo_sample[i].time_us < oldest_sample_us) {
// 						oldest_sample_us = accel_fifo_sample[i].time_us;
// 						oldest_sample_index = i;
// 					}

// 					if (accel_fifo_sample[i].time_us > newest_sample_us) {
// 						newest_sample_us = accel_fifo_sample[i].time_us;
// 					}
// 				}

// 				timestamp_diff = newest_sample_us - oldest_sample_us;

// 			} else {
// 				// we have a sample gap, but no data to fill it
// 				accel_data_available = false;
// 				break;
// 			}
// 		}

// 		if (accel_data_available)  {
// 			sensor_accel_fifo_median.timestamp_sample = newest_sample_us;

// 			// Find the median of each fifo datapoint
// 			float x_median = median(accel_fifo_sample[0].data[0] * accel_fifo_sample[0].scale,
// 						accel_fifo_sample[1].data[0] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[0] * accel_fifo_sample[2].scale);

// 			float y_median = median(accel_fifo_sample[0].data[1] * accel_fifo_sample[0].scale,
// 						accel_fifo_sample[1].data[1] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[1] * accel_fifo_sample[2].scale);

// 			float z_median = median(accel_fifo_sample[0].data[2] * accel_fifo_sample[0].scale,
// 						accel_fifo_sample[1].data[2] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[2] * accel_fifo_sample[2].scale);

// 			// Average the dt and the highest scale value
// 			float dt_sum = 0.0f;

// 			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
// 				dt_sum = dt_sum + accel_fifo_sample[i].dt;

// 				if (accel_fifo_sample[i].scale > sensor_accel_fifo_median.scale) {
// 					sensor_accel_fifo_median.scale = accel_fifo_sample[i].scale;
// 				}
// 			}

// 			dt_sum = dt_sum / MAX_SENSOR_COUNT;
// 			sensor_accel_fifo_median.dt = sensor_accel_fifo_median.dt + dt_sum;

// 			// Fill the temporary buffer will adjust for the largest scale later
// 			accel_data_median[sensor_accel_fifo_median.samples](0) = x_median;
// 			accel_data_median[sensor_accel_fifo_median.samples](1) = y_median;
// 			accel_data_median[sensor_accel_fifo_median.samples](2) = z_median;

// 			sensor_accel_fifo_median.samples++;

// 			if (sensor_accel_fifo_median.samples == 32) {
// 				break;
// 			}
// 		}
// 	}

// 	if (sensor_accel_fifo_median.samples > 0) {

// 		_last_accel_timestamp = sensor_accel_fifo_median.timestamp_sample;

// 		for (size_t i = 0; i < sensor_accel_fifo_median.samples; i++) {
// 			sensor_accel_fifo_median.x[i] = (int16_t)(accel_data_median[i](0) / sensor_accel_fifo_median.scale);
// 			sensor_accel_fifo_median.y[i] = (int16_t)(accel_data_median[i](1) / sensor_accel_fifo_median.scale);
// 			sensor_accel_fifo_median.z[i] = (int16_t)(accel_data_median[i](2) / sensor_accel_fifo_median.scale);
// 		}

// 		_px4_accel.set_scale(sensor_accel_fifo_median.scale);

// 		sensor_accel_fifo_median.dt = sensor_accel_fifo_median.dt / sensor_accel_fifo_median.samples;
// 		sensor_accel_fifo_median.device_id = _accel_device_id;
// 		sensor_accel_fifo_median.timestamp = hrt_absolute_time();

// 		_px4_accel.updateFIFO(sensor_accel_fifo_median);
// 	}
// }

void VirtualIMU::process_accel()
{
	accelFIFOSample accel_fifo_sample[MAX_SENSOR_COUNT];

	sensor_accel_fifo_s sensor_accel_fifo_median{};
	Vector3f accel_data_median[32] {};

	sensor_accel_fifo_median.samples = 0;
	sensor_accel_fifo_median.scale = 0.0f;
	sensor_accel_fifo_median.dt = 0.0f;

	uint64_t now = hrt_absolute_time();

	for (uint64_t time = _last_accel_timestamp + _median_accel_dt_us; time < now; time += _median_accel_dt_us) {

		bool accel_data_available = false;

		for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			accel_data_available = _accel_fifo_buffer[i].pop_oldest(time - _median_accel_dt_us / 2, time + _median_accel_dt_us / 2,
					       &accel_fifo_sample[i]);
		}

		if (accel_data_available)  {
			sensor_accel_fifo_median.timestamp_sample = 0;

			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (accel_fifo_sample[i].time_us > sensor_accel_fifo_median.timestamp_sample) {
					sensor_accel_fifo_median.timestamp_sample = accel_fifo_sample[i].time_us;
				}
			}

			// Find the median of each fifo datapoint
			float x_median = median(accel_fifo_sample[0].data[0] * accel_fifo_sample[0].scale,
						accel_fifo_sample[1].data[0] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[0] * accel_fifo_sample[2].scale);

			float y_median = median(accel_fifo_sample[0].data[1] * accel_fifo_sample[0].scale,
						accel_fifo_sample[1].data[1] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[1] * accel_fifo_sample[2].scale);

			float z_median = median(accel_fifo_sample[0].data[2] * accel_fifo_sample[0].scale,
						accel_fifo_sample[1].data[2] * accel_fifo_sample[1].scale, accel_fifo_sample[2].data[2] * accel_fifo_sample[2].scale);

			// Average the dt and the highest scale value
			float dt_sum = 0.0f;

			for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				dt_sum = dt_sum + accel_fifo_sample[i].dt;

				if (accel_fifo_sample[i].scale > sensor_accel_fifo_median.scale) {
					sensor_accel_fifo_median.scale = accel_fifo_sample[i].scale;
				}
			}

			dt_sum = dt_sum / MAX_SENSOR_COUNT;
			sensor_accel_fifo_median.dt = sensor_accel_fifo_median.dt + dt_sum;

			// Fill the temporary buffer will adjust for the largest scale later
			accel_data_median[sensor_accel_fifo_median.samples](0) = x_median;
			accel_data_median[sensor_accel_fifo_median.samples](1) = y_median;
			accel_data_median[sensor_accel_fifo_median.samples](2) = z_median;

			sensor_accel_fifo_median.samples++;

			if (sensor_accel_fifo_median.samples == 32) {
				break;
			}
		}
	}

	if (sensor_accel_fifo_median.samples > 0) {

		for (size_t i = 0; i < sensor_accel_fifo_median.samples; i++) {
			sensor_accel_fifo_median.x[i] = (int16_t)(accel_data_median[i](0) / sensor_accel_fifo_median.scale);
			sensor_accel_fifo_median.y[i] = (int16_t)(accel_data_median[i](1) / sensor_accel_fifo_median.scale);
			sensor_accel_fifo_median.z[i] = (int16_t)(accel_data_median[i](2) / sensor_accel_fifo_median.scale);
		}

		_px4_accel.set_scale(sensor_accel_fifo_median.scale);

		sensor_accel_fifo_median.dt = sensor_accel_fifo_median.dt / sensor_accel_fifo_median.samples;
		sensor_accel_fifo_median.device_id = _accel_device_id;
		sensor_accel_fifo_median.timestamp = hrt_absolute_time();

		_px4_accel.updateFIFO(sensor_accel_fifo_median);

		_last_accel_timestamp = sensor_accel_fifo_median.timestamp_sample;
		_median_accel_dt_us = (uint16_t)sensor_accel_fifo_median.dt;
	}
}

float VirtualIMU::median(float x, float y, float z)
{
	if (x < y) {
		if (y < z) {
			return y;

		} else if (x < z) {
			return z;

		} else {
			return x;
		}

	} else {
		if (x < z) {
			return x;

		} else if (y < z) {
			return z;

		} else {
			return y;
		}
	}
}

size_t VirtualIMU::find_median_index(float x, float y, float z)
{
	if (x < y) {
		if (y < z) {
			return 1;

		} else if (x < z) {
			return 2;

		} else {
			return 0;
		}

	} else {
		if (x < z) {
			return 0;

		} else if (y < z) {
			return 2;

		} else {
			return 1;
		}
	}
}

void VirtualIMU::check_newest_timestamp_and_register_callback(sensor_gyro_fifo_s *sensor_gyro_fifo,
		sensor_accel_fifo_s *sensor_accel_fifo)
{
	// Check for the newest timestamp and register the callback to the newest once
	uint64_t newest_accel_timestamp = 0;
	size_t newest_accel_index = 0;

	for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (sensor_accel_fifo[i].timestamp > newest_accel_timestamp) {
			newest_accel_timestamp = sensor_accel_fifo[i].timestamp;
			newest_accel_index = i;
		}
	}

	uint64_t newest_gyro_timestamp = 0;
	size_t newest_gyro_index = 0;

	for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (sensor_gyro_fifo[i].timestamp > newest_gyro_timestamp) {
			newest_gyro_timestamp = sensor_gyro_fifo[i].timestamp;
			newest_gyro_index = i;
		}
	}

	if (newest_accel_timestamp > newest_gyro_timestamp) {
		_sensor_gyro_fifo_sub_callback.unregisterCallback();
		_sensor_accel_fifo_sub_callback.registerCallback();
		_sensor_accel_fifo_sub_callback.ChangeInstance(newest_accel_index);

	} else {
		_sensor_accel_fifo_sub_callback.unregisterCallback();
		_sensor_gyro_fifo_sub_callback.registerCallback();
		_sensor_gyro_fifo_sub_callback.ChangeInstance(newest_gyro_index);
	}
}

int VirtualIMU::task_spawn(int argc, char *argv[])
{
	VirtualIMU *instance = new VirtualIMU();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VirtualIMU::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);

	for (size_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		perf_print_counter(_gyro_fifo_generation_gap_perf[i]);
		perf_print_counter(_accel_fifo_generation_gap_perf[i]);
	}

	return 0;
}

int VirtualIMU::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VirtualIMU::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("virtual_imu", "sensors");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int virtual_imu_main(int argc, char *argv[])
{
	return VirtualIMU::main(argc, argv);
}
