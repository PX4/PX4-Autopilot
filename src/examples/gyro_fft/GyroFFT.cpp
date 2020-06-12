/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "GyroFFT.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (int axis = 0; axis < 3; axis++) {
		arm_rfft_init_q15(&_rfft_q15[axis], FFT_LENGTH, 0, 1);
	}

	// init Hanning window
	float hanning_window[FFT_LENGTH];

	for (int n = 0; n < FFT_LENGTH; n++) {
		hanning_window[n] = 0.5f - 0.5f * cosf(2.f * M_PI_F * n / (FFT_LENGTH - 1));
	}

	arm_float_to_q15(hanning_window, _hanning_window, FFT_LENGTH);
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_fifo_generation_gap_perf);
}

bool GyroFFT::init()
{
	if (!SensorSelectionUpdate(true)) {
		PX4_ERR("sensor_gyro_fifo callback registration failed!");
		return false;
	}

	return true;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_gyro_fifo_sub.ChangeInstance(i) && _sensor_gyro_fifo_sub.registerCallback()) {
						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

// helper function used for frequency estimation
static constexpr float tau(float x)
{
	float p1 = logf(3.f * powf(x, 2.f) + 6 * x + 1);
	float part1 = x + 1 - sqrtf(2.f / 3.f);
	float part2 = x + 1 + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (1.f / 4.f * p1 - sqrtf(6) / 24 * p2);
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_fifo_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(500_ms);

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	SensorSelectionUpdate();

	// run on sensor gyro fifo updates
	sensor_gyro_fifo_s sensor_gyro_fifo;

	while (_sensor_gyro_fifo_sub.update(&sensor_gyro_fifo)) {

		if (_sensor_gyro_fifo_sub.get_last_generation() != _gyro_last_generation + 1) {
			// force reset if we've missed a sample
			_fft_buffer_index[0] = 0;
			_fft_buffer_index[1] = 0;
			_fft_buffer_index[2] = 0;

			perf_count(_gyro_fifo_generation_gap_perf);
		}

		_gyro_last_generation = _sensor_gyro_fifo_sub.get_last_generation();

		const int N = sensor_gyro_fifo.samples;

		for (int axis = 0; axis < 3; axis++) {
			int16_t *input = nullptr;

			switch (axis) {
			case 0:
				input = sensor_gyro_fifo.x;
				break;

			case 1:
				input = sensor_gyro_fifo.y;
				break;

			case 2:
				input = sensor_gyro_fifo.z;
				break;
			}

			for (int n = 0; n < N; n++) {
				int &buffer_index = _fft_buffer_index[axis];

				_data_buffer[axis][buffer_index] = input[n] / 2;

				buffer_index++;

				// if we have enough samples, begin processing
				if (buffer_index >= FFT_LENGTH) {

					arm_mult_q15(_data_buffer[axis], _hanning_window, _fft_input_buffer, FFT_LENGTH);

					perf_begin(_fft_perf);
					arm_rfft_q15(&_rfft_q15[axis], _fft_input_buffer, _fft_outupt_buffer);
					perf_end(_fft_perf);


					// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
					int16_t max_peak = 0;
					uint16_t max_peak_index = 0;

					// start at 1 to skip DC
					for (uint16_t bucket_index = 1; bucket_index < FFT_LENGTH; bucket_index++) {
						if (abs(_fft_outupt_buffer[bucket_index]) >= max_peak) {
							max_peak_index = bucket_index;
							max_peak = abs(_fft_outupt_buffer[bucket_index]);
						}
					}


					int k = max_peak_index;
					float divider = powf(_fft_outupt_buffer[k], 2.f);
					float ap = (_fft_outupt_buffer[k + 1] * _fft_outupt_buffer[k]) / divider;
					float dp = -ap  / (1.f - ap);
					float am = (_fft_outupt_buffer[k - 1] * _fft_outupt_buffer[k]) / divider;

					float dm = am / (1.f - am);
					float d = (dp + dm) / 2 + tau(dp * dp) - tau(dm * dm);

					float adjustedBinLocation = k + d;
					float peakFreqAdjusted = (_gyro_sample_rate * adjustedBinLocation / (FFT_LENGTH * 2));


					// publish sensor_gyro_fft_axis (one instance per axis)
					sensor_gyro_fft_axis_s sensor_gyro_fft_axis{};
					const int N_publish = math::min((unsigned)FFT_LENGTH,
									sizeof(sensor_gyro_fft_axis_s::fft) / sizeof(sensor_gyro_fft_axis_s::fft[0]));
					memcpy(sensor_gyro_fft_axis.fft, _fft_outupt_buffer, N_publish);

					sensor_gyro_fft_axis.resolution_hz = _gyro_sample_rate / (FFT_LENGTH * 2);

					sensor_gyro_fft_axis.peak_index = max_peak_index;
					sensor_gyro_fft_axis.peak_frequency = max_peak_index * sensor_gyro_fft_axis.resolution_hz;

					sensor_gyro_fft_axis.peak_index_quinns = adjustedBinLocation;
					sensor_gyro_fft_axis.peak_frequency_quinns = peakFreqAdjusted;

					sensor_gyro_fft_axis.samples = FFT_LENGTH;
					sensor_gyro_fft_axis.dt = 1e6f / _gyro_sample_rate;
					sensor_gyro_fft_axis.device_id = sensor_gyro_fifo.device_id;
					sensor_gyro_fft_axis.axis = axis;
					sensor_gyro_fft_axis.timestamp_sample = sensor_gyro_fifo.timestamp_sample;
					sensor_gyro_fft_axis.timestamp = hrt_absolute_time();
					_sensor_gyro_fft_axis_pub[axis].publish(sensor_gyro_fft_axis);

					// reset
					buffer_index = 0;
				}
			}
		}
	}

	perf_end(_cycle_perf);
}

int GyroFFT::task_spawn(int argc, char *argv[])
{
	GyroFFT *instance = new GyroFFT();

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

int GyroFFT::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	perf_print_counter(_gyro_fifo_generation_gap_perf);
	return 0;
}

int GyroFFT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFFT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_fft", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_fft_main(int argc, char *argv[])
{
	return GyroFFT::main(argc, argv);
}
