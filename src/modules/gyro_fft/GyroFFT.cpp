/****************************************************************************
 *
 *   Copyright (c) 2020 - 2021 PX4 Development Team. All rights reserved.
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

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	for (int i = 0; i < MAX_NUM_PEAKS; i++) {
		_sensor_gyro_fft.peak_frequencies_x[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z[i] = NAN;
	}
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_generation_gap_perf);
	perf_free(_gyro_fifo_generation_gap_perf);

	delete[] _gyro_data_buffer_x;
	delete[] _gyro_data_buffer_y;
	delete[] _gyro_data_buffer_z;
	delete[] _hanning_window;
	delete[] _fft_input_buffer;
	delete[] _fft_outupt_buffer;
}

bool GyroFFT::init()
{
	bool buffers_allocated = false;

	// arm_rfft_init_q15(&_rfft_q15, _imu_gyro_fft_len, 0, 1) manually inlined to save flash
	_rfft_q15.pTwiddleAReal = (q15_t *) realCoefAQ15;
	_rfft_q15.pTwiddleBReal = (q15_t *) realCoefBQ15;
	_rfft_q15.ifftFlagR = 0;
	_rfft_q15.bitReverseFlagR = 1;

	switch (_param_imu_gyro_fft_len.get()) {
	// case 128:
	// 	buffers_allocated = AllocateBuffers<128>();
	// 	_rfft_q15.fftLenReal = 128;
	// 	_rfft_q15.twidCoefRModifier = 64U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len64;
	// 	break;

	case 256:
		buffers_allocated = AllocateBuffers<256>();
		_rfft_q15.fftLenReal = 256;
		_rfft_q15.twidCoefRModifier = 32U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len128;
		break;

	case 512:
		buffers_allocated = AllocateBuffers<512>();
		_rfft_q15.fftLenReal = 512;
		_rfft_q15.twidCoefRModifier = 16U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len256;
		break;

	case 1024:
		buffers_allocated = AllocateBuffers<1024>();
		_rfft_q15.fftLenReal = 1024;
		_rfft_q15.twidCoefRModifier = 8U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len512;
		break;

	// case 2048:
	// 	buffers_allocated = AllocateBuffers<2048>();
	// 	_rfft_q15.fftLenReal = 2048;
	// 	_rfft_q15.twidCoefRModifier = 4U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len1024;
	// 	break;

	case 4096:
		buffers_allocated = AllocateBuffers<4096>();
		_rfft_q15.fftLenReal = 4096;
		_rfft_q15.twidCoefRModifier = 2U;
		_rfft_q15.pCfft = &arm_cfft_sR_q15_len2048;
		break;

	// case 8192:
	// 	buffers_allocated = AllocateBuffers<8192>();
	// 	_rfft_q15.fftLenReal = 8192;
	// 	_rfft_q15.twidCoefRModifier = 1U;
	// 	_rfft_q15.pCfft = &arm_cfft_sR_q15_len4096;
	// 	break;

	default:
		// otherwise default to 256
		PX4_ERR("Invalid IMU_GYRO_FFT_LEN=%" PRId32 ", resetting", _param_imu_gyro_fft_len.get());
		buffers_allocated = AllocateBuffers<256>();
		_param_imu_gyro_fft_len.set(256);
		_param_imu_gyro_fft_len.commit();
		break;
	}

	if (buffers_allocated) {
		_imu_gyro_fft_len = _param_imu_gyro_fft_len.get();

		// init Hanning window
		for (int n = 0; n < _imu_gyro_fft_len; n++) {
			const float hanning_value = 0.5f * (1.f - cosf(2.f * M_PI_F * n / (_imu_gyro_fft_len - 1)));
			arm_float_to_q15(&hanning_value, &_hanning_window[n], 1);
		}

		if (!SensorSelectionUpdate(true)) {
			ScheduleDelayed(500_ms);
		}

		return true;
	}

	PX4_ERR("failed to allocate buffers");
	delete[] _gyro_data_buffer_x;
	delete[] _gyro_data_buffer_y;
	delete[] _gyro_data_buffer_z;
	delete[] _hanning_window;
	delete[] _fft_input_buffer;
	delete[] _fft_outupt_buffer;

	return false;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_selected_sensor_device_id != sensor_selection.gyro_device_id)) {
			// prefer sensor_gyro_fifo if available
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_fifo_sub.ChangeInstance(i) && _sensor_gyro_fifo_sub.registerCallback()) {
						_sensor_gyro_sub.unregisterCallback();
						_sensor_gyro_fifo_sub.set_required_updates(sensor_gyro_fifo_s::ORB_QUEUE_LENGTH / 2);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						_gyro_fifo = true;

						if (_gyro_fifo_generation_gap_perf == nullptr) {
							_gyro_fifo_generation_gap_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro FIFO data gap");
						}

						return true;
					}
				}
			}

			// otherwise use sensor_gyro
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						_sensor_gyro_fifo_sub.unregisterCallback();
						_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH / 2);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						_gyro_fifo = false;

						if (_gyro_generation_gap_perf == nullptr) {
							_gyro_generation_gap_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro data gap");
						}

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%" PRIu32 ")", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

void GyroFFT::VehicleIMUStatusUpdate(bool force)
{
	if (_vehicle_imu_status_sub.updated() || force) {
		vehicle_imu_status_s vehicle_imu_status;

		if (_vehicle_imu_status_sub.copy(&vehicle_imu_status)) {
			// find corresponding vehicle_imu_status instance if the device_id doesn't match
			if (vehicle_imu_status.gyro_device_id != _selected_sensor_device_id) {

				for (uint8_t imu_status = 0; imu_status < MAX_SENSOR_COUNT; imu_status++) {
					uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_status};

					if (imu_status_sub.copy(&vehicle_imu_status)) {
						if (vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) {
							_vehicle_imu_status_sub.ChangeInstance(imu_status);
							break;
						}
					}
				}
			}

			// update gyro sample rate
			if ((vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) && (vehicle_imu_status.gyro_rate_hz > 0)) {
				if (_gyro_fifo) {
					_gyro_sample_rate_hz = vehicle_imu_status.gyro_raw_rate_hz;

				} else {
					_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
				}

				return;
			}
		}
	}
}

// helper function used for frequency estimation
static inline float tau(float x)
{
	// tau(x) = 1/4 * log(3x^2 + 6x + 1) – sqrt(6)/24 * log((x + 1 – sqrt(2/3))  /  (x + 1 + sqrt(2/3)))
	float p1 = logf(3.f * powf(x, 2.f) + 6.f * x + 1.f);
	float part1 = x + 1.f - sqrtf(2.f / 3.f);
	float part2 = x + 1.f + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (0.25f * p1 - sqrtf(6.f) / 24.f * p2);
}

float GyroFFT::EstimatePeakFrequencyBin(q15_t fft[], int peak_index)
{
	if (peak_index >= 2) {
		// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
		float real[3] { (float)fft[peak_index - 2], (float)fft[peak_index], (float)fft[peak_index + 2]     };
		float imag[3] { (float)fft[peak_index - 2 + 1], (float)fft[peak_index + 1], (float)fft[peak_index + 2 + 1] };

		static constexpr int k = 1;

		const float divider = (real[k] * real[k] + imag[k] * imag[k]);

		// ap = (X[k + 1].r * X[k].r + X[k+1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float ap = (real[k + 1] * real[k] + imag[k + 1] * imag[k]) / divider;

		// dp = -ap / (1 – ap)
		float dp = -ap  / (1.f - ap);

		// am = (X[k - 1].r * X[k].r + X[k – 1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float am = (real[k - 1] * real[k] + imag[k - 1] * imag[k]) / divider;

		// dm = am / (1 – am)
		float dm = am / (1.f - am);

		// d = (dp + dm) / 2 + tau(dp * dp) – tau(dm * dm)
		float d = (dp + dm) / 2.f + tau(dp * dp) - tau(dm * dm);

		// k’ = k + d
		return peak_index + 2.f * d;
	}

	return NAN;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
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

	const bool selection_updated = SensorSelectionUpdate();
	VehicleIMUStatusUpdate(selection_updated);

	// reset
	_fft_updated = false;

	if (_gyro_fifo) {
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

			if (fabsf(sensor_gyro_fifo.scale - _fifo_last_scale) > FLT_EPSILON) {
				// force reset if scale has changed
				_fft_buffer_index[0] = 0;
				_fft_buffer_index[1] = 0;
				_fft_buffer_index[2] = 0;

				_fifo_last_scale = sensor_gyro_fifo.scale;
			}

			int16_t *input[] {sensor_gyro_fifo.x, sensor_gyro_fifo.y, sensor_gyro_fifo.z};
			Update(sensor_gyro_fifo.timestamp_sample, input, sensor_gyro_fifo.samples);
		}

	} else {
		// run on sensor gyro fifo updates
		sensor_gyro_s sensor_gyro;

		while (_sensor_gyro_sub.update(&sensor_gyro)) {
			if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
				// force reset if we've missed a sample
				_fft_buffer_index[0] = 0;
				_fft_buffer_index[1] = 0;
				_fft_buffer_index[2] = 0;

				perf_count(_gyro_generation_gap_perf);
			}

			_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

			const float gyro_scale = math::radians(1000.f); // arbitrary scaling float32 rad/s -> raw int16
			int16_t gyro_x[1] {(int16_t)roundf(sensor_gyro.x * gyro_scale)};
			int16_t gyro_y[1] {(int16_t)roundf(sensor_gyro.y * gyro_scale)};
			int16_t gyro_z[1] {(int16_t)roundf(sensor_gyro.z * gyro_scale)};

			int16_t *input[] {gyro_x, gyro_y, gyro_z};
			Update(sensor_gyro.timestamp_sample, input, 1);
		}
	}

	if (_publish) {
		Publish();
		_publish = false;
	}

	perf_end(_cycle_perf);
}

void GyroFFT::Update(const hrt_abstime &timestamp_sample, int16_t *input[], uint8_t N)
{
	q15_t *gyro_data_buffer[] {_gyro_data_buffer_x, _gyro_data_buffer_y, _gyro_data_buffer_z};

	for (int axis = 0; axis < 3; axis++) {
		int &buffer_index = _fft_buffer_index[axis];

		for (int n = 0; n < N; n++) {
			if (buffer_index < _imu_gyro_fft_len) {
				// convert int16_t -> q15_t (scaling isn't relevant)
				gyro_data_buffer[axis][buffer_index] = input[axis][n] / 2;
				buffer_index++;
			}

			// if we have enough samples begin processing, but only one FFT per cycle
			if ((buffer_index >= _imu_gyro_fft_len) && !_fft_updated) {
				perf_begin(_fft_perf);

				arm_mult_q15(gyro_data_buffer[axis], _hanning_window, _fft_input_buffer, _imu_gyro_fft_len);
				arm_rfft_q15(&_rfft_q15, _fft_input_buffer, _fft_outupt_buffer);

				_fft_updated = true;

				FindPeaks(timestamp_sample, axis, _fft_outupt_buffer);

				// reset
				// shift buffer (3/4 overlap)
				const int overlap_start = _imu_gyro_fft_len / 4;
				memmove(&gyro_data_buffer[axis][0], &gyro_data_buffer[axis][overlap_start], sizeof(q15_t) * overlap_start * 3);
				buffer_index = overlap_start * 3;

				perf_end(_fft_perf);
			}
		}
	}
}

void GyroFFT::FindPeaks(const hrt_abstime &timestamp_sample, int axis, q15_t *fft_outupt_buffer)
{
	const float resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;

	// sum total energy across all used buckets for SNR
	float bin_mag_sum = 0;

	// find raw peaks
	uint16_t raw_peak_index[MAX_NUM_PEAKS] {};
	float peak_magnitude[MAX_NUM_PEAKS] {};

	// FFT output buffer is ordered [real[0], imag[0], real[1], imag[1], real[2], imag[2] ... real[(N/2)-1], imag[(N/2)-1]
	for (uint16_t bucket_index = 0; bucket_index < (2 * _imu_gyro_fft_len - 1); bucket_index = bucket_index + 2) {

		const float freq_hz = (bucket_index / 2) * resolution_hz;

		if ((bucket_index > 0) && (bucket_index < (_imu_gyro_fft_len - 1))
		    && (freq_hz >= _param_imu_gyro_fft_min.get())
		    && (freq_hz <= _param_imu_gyro_fft_max.get())) {

			const float real = fft_outupt_buffer[bucket_index];
			const float imag = fft_outupt_buffer[bucket_index + 1];

			const float fft_magnitude_squared = real * real + imag * imag;
			bin_mag_sum += fft_magnitude_squared;

			for (int i = 0; i < MAX_NUM_PEAKS; i++) {
				if (fft_magnitude_squared > peak_magnitude[i]) {
					peak_magnitude[i] = fft_magnitude_squared;
					raw_peak_index[i] = bucket_index;
					break;
				}
			}
		}
	}

	// keep if peak has been previously seen and SNR > MIN_SNR
	//   or
	// peak has SNR > MIN_SNR_INITIAL
	static constexpr float MIN_SNR = 1.f; // TODO: configurable?

	int num_peaks_found = 0;
	float peak_frequencies[MAX_NUM_PEAKS] {};
	float peak_snr[MAX_NUM_PEAKS] {};

	float *peak_frequencies_publish[] { _sensor_gyro_fft.peak_frequencies_x, _sensor_gyro_fft.peak_frequencies_y, _sensor_gyro_fft.peak_frequencies_z };

	for (int peak_new = 0; peak_new < MAX_NUM_PEAKS; peak_new++) {
		if (raw_peak_index[peak_new] > 0) {

			const float snr = 10.f * log10f((_imu_gyro_fft_len - 1) * peak_magnitude[peak_new] /
							(bin_mag_sum - peak_magnitude[peak_new]));

			if (snr > MIN_SNR) {
				// estimate adjusted frequency bin, magnitude, and SNR for the largest peaks found
				const float adjusted_bin = EstimatePeakFrequencyBin(fft_outupt_buffer, raw_peak_index[peak_new]);
				const float freq_adjusted = (adjusted_bin / 2.f) * resolution_hz;

				if (PX4_ISFINITE(adjusted_bin) && PX4_ISFINITE(freq_adjusted)
				    && (freq_adjusted > _param_imu_gyro_fft_min.get())
				    && (freq_adjusted < _param_imu_gyro_fft_max.get())) {

					// only keep if we're already tracking this frequency or if the SNR is significant
					for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
						if ((snr > _param_imu_gyro_fft_snr.get())
						    || (fabsf(freq_adjusted - peak_frequencies_publish[axis][peak_prev]) < (resolution_hz * 0.5f))) {
							// keep
							peak_frequencies[num_peaks_found] = freq_adjusted;
							peak_snr[num_peaks_found] = snr;
							num_peaks_found++;
							break;
						}
					}
				}
			}
		}
	}

	if (num_peaks_found > 0) {
		UpdateOutput(timestamp_sample, axis, peak_frequencies, peak_snr, num_peaks_found);
	}
}

void GyroFFT::UpdateOutput(const hrt_abstime &timestamp_sample, int axis, float peak_frequencies[MAX_NUM_PEAKS],
			   float peak_snr[MAX_NUM_PEAKS], int num_peaks_found)
{
	float *peak_frequencies_publish[] { _sensor_gyro_fft.peak_frequencies_x, _sensor_gyro_fft.peak_frequencies_y, _sensor_gyro_fft.peak_frequencies_z };
	float *peak_snr_publish[]         { _sensor_gyro_fft.peak_snr_x,         _sensor_gyro_fft.peak_snr_y,         _sensor_gyro_fft.peak_snr_z };

	// new peak: r, old peak: c
	float peak_frequencies_diff[MAX_NUM_PEAKS][MAX_NUM_PEAKS];

	for (int peak_new = 0; peak_new < MAX_NUM_PEAKS; peak_new++) {
		// compute distance to previous peaks
		for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
			if ((peak_frequencies[peak_new] > 0)
			    && (peak_frequencies_publish[axis][peak_prev] > 0) && PX4_ISFINITE(peak_frequencies_publish[axis][peak_prev])
			   ) {
				peak_frequencies_diff[peak_new][peak_prev] = fabsf(peak_frequencies[peak_new] -
						peak_frequencies_publish[axis][peak_prev]);

			} else {
				peak_frequencies_diff[peak_new][peak_prev] = INFINITY;
			}
		}
	}

	// go through peak_frequencies_diff and find smallest diff (closest peaks)
	//  - copy new peak to old peak slot
	//  - exclude new peak (row) and old peak (column) in search
	//  - repeat
	//
	//  - finally copy unmatched peaks to empty slots
	bool peak_new_copied[MAX_NUM_PEAKS] {};
	bool peak_out_filled[MAX_NUM_PEAKS] {};
	int peaks_copied = 0;

	for (int new_peak = 0; new_peak < num_peaks_found; new_peak++) {

		float smallest_diff = INFINITY;
		int closest_new_peak = -1;
		int closest_prev_peak = -1;

		// find new peak with smallest difference to old peak
		for (int peak_new = 0; peak_new < num_peaks_found; peak_new++) {
			for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
				if (!peak_new_copied[peak_new] && !peak_out_filled[peak_prev]
				    && (peak_frequencies_diff[peak_new][peak_prev] < smallest_diff)) {

					smallest_diff = peak_frequencies_diff[peak_new][peak_prev];
					closest_new_peak = peak_new;
					closest_prev_peak = peak_prev;
				}
			}
		}

		if (PX4_ISFINITE(smallest_diff) && (smallest_diff > 0)) {
			// smallest diff found, copy newly found peak into same slot previously published
			float peak_frequency = _median_filter[axis][closest_prev_peak].apply(peak_frequencies[closest_new_peak]);

			if (peak_frequency > 0) {
				peak_frequencies_publish[axis][closest_prev_peak] = peak_frequency;
				peak_snr_publish[axis][closest_prev_peak] = peak_snr[closest_new_peak];
				peaks_copied++;

				_last_update[axis][closest_prev_peak] = timestamp_sample;
				_sensor_gyro_fft.timestamp_sample = timestamp_sample;
				_publish = true;

				// clear
				peak_frequencies[closest_new_peak] = NAN;
				peak_frequencies_diff[closest_new_peak][closest_prev_peak] = NAN;
				peak_new_copied[closest_new_peak] = true;
				peak_out_filled[closest_prev_peak] = true;

				if (peaks_copied == num_peaks_found) {
					break;
				}
			}
		}
	}

	// clear any stale entries
	for (int peak_out = 0; peak_out < MAX_NUM_PEAKS; peak_out++) {
		if (timestamp_sample - _last_update[axis][peak_out] > 500_ms) {
			peak_frequencies_publish[axis][peak_out] = NAN;
			peak_snr_publish[axis][peak_out] = NAN;

			_last_update[axis][peak_out] = 0;
		}
	}

	// copy any remaining new (unmatched) peaks to overwrite old or empty slots
	if (peaks_copied != num_peaks_found) {
		for (int peak_new = 0; peak_new < num_peaks_found; peak_new++) {
			if (PX4_ISFINITE(peak_frequencies[peak_new]) && (peak_frequencies[peak_new] > 0)) {
				int oldest_slot = -1;
				hrt_abstime oldest = timestamp_sample;

				// find oldest slot and replace with new peak frequency
				for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
					if (_last_update[axis][peak_prev] < oldest) {
						oldest_slot = peak_prev;
						oldest = _last_update[axis][peak_prev];
					}
				}

				if (oldest_slot >= 0) {
					// copy peak to output slot
					float peak_frequency = _median_filter[axis][oldest_slot].apply(peak_frequencies[peak_new]);

					if (peak_frequency > 0) {
						peak_frequencies_publish[axis][oldest_slot] = peak_frequency;
						peak_snr_publish[axis][oldest_slot] = peak_snr[peak_new];

						_last_update[axis][oldest_slot] = timestamp_sample;
						_sensor_gyro_fft.timestamp_sample = timestamp_sample;
						_publish = true;
					}
				}
			}
		}
	}
}

void GyroFFT::Publish()
{
	_sensor_gyro_fft.device_id = _selected_sensor_device_id;
	_sensor_gyro_fft.sensor_sample_rate_hz = _gyro_sample_rate_hz;
	_sensor_gyro_fft.resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;
	_sensor_gyro_fft.timestamp = hrt_absolute_time();
	_sensor_gyro_fft_pub.publish(_sensor_gyro_fft);
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
	PX4_INFO("gyro sample rate: %.3f Hz", (double)_gyro_sample_rate_hz);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	perf_print_counter(_gyro_generation_gap_perf);
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
