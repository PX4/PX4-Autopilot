/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (int i = 0; i < MAX_NUM_PEAKS; i++) {
		_sensor_gyro_fft.peak_frequencies_x[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z[i] = NAN;

		_sensor_gyro_fft.peak_frequencies_x_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z_raw[i] = NAN;

		_sensor_gyro_fft.peak_magnitude_x[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_y[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_z[i] = NAN;
	}

	_sensor_gyro_fft_pub.advertise();
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_generation_gap_perf);
}

bool GyroFFT::init()
{
	_imu_gyro_fft_len = 64;

	if (!SensorSelectionUpdate(true)) {
		ScheduleDelayed(500_ms);
	}

	return true;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_selected_sensor_device_id != sensor_selection.gyro_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						//_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH - 1);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
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
				_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
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

float GyroFFT::EstimatePeakFrequencyBin(int axis, int32_t k)
{
	if (k > 2) {
		// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
		const auto &dft = _sliding_dft[axis];

		const float divider = (dft.dft(k).real() * dft.dft(k).real() + dft.dft(k).imag() * dft.dft(k).imag());

		// ap = (X[k + 1].r * X[k].r + X[k+1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float ap = (dft.dft(k + 1).real() * dft.dft(k).real() + dft.dft(k + 1).imag() * dft.dft(k).imag()) / divider;

		// dp = -ap / (1 – ap)
		float dp = -ap  / (1.f - ap);

		// am = (X[k - 1].r * X[k].r + X[k – 1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float am = (dft.dft(k - 1).real() * dft.dft(k).real() + dft.dft(k - 1).imag() * dft.dft(k).imag()) / divider;

		// dm = am / (1 – am)
		float dm = am / (1.f - am);

		// d = (dp + dm) / 2 + tau(dp * dp) – tau(dm * dm)
		float d = (dp + dm) / 2.f + tau(dp * dp) - tau(dm * dm);

		// k’ = k + d
		float adjusted_bin = k + d;

		return adjusted_bin;
	}

	return NAN;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
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

	// run on sensor gyro updates
	sensor_gyro_s sensor_gyro;

	while (_sensor_gyro_sub.update(&sensor_gyro)) {
		if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
			// force reset if we've missed a sample
			perf_count(_gyro_generation_gap_perf);
		}

		_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

		perf_begin(_fft_perf);
		_sliding_dft[0].update(sensor_gyro.x);
		_sliding_dft[1].update(sensor_gyro.y);
		_sliding_dft[2].update(sensor_gyro.z);
		perf_end(_fft_perf);
	}

	Update(sensor_gyro.timestamp_sample);

	if (_publish) {
		Publish();
		_publish = false;
	}

	perf_end(_cycle_perf);
}

void GyroFFT::Update(const hrt_abstime &timestamp_sample)
{
	//float *peak_frequencies_raw[] {_sensor_gyro_fft.peak_frequencies_x_raw, _sensor_gyro_fft.peak_frequencies_y_raw, _sensor_gyro_fft.peak_frequencies_z_raw};
	//float *peak_magnitude_out[] {_sensor_gyro_fft.peak_magnitude_x, _sensor_gyro_fft.peak_magnitude_y, _sensor_gyro_fft.peak_magnitude_z};

	// wipe TODO: temporary
	memset(&_sensor_gyro_fft.peak_frequencies_x_raw, 0, sizeof(_sensor_gyro_fft.peak_frequencies_x_raw));
	memset(&_sensor_gyro_fft.peak_frequencies_y_raw, 0, sizeof(_sensor_gyro_fft.peak_frequencies_y_raw));
	memset(&_sensor_gyro_fft.peak_frequencies_z_raw, 0, sizeof(_sensor_gyro_fft.peak_frequencies_z_raw));

	memset(&_sensor_gyro_fft.peak_magnitude_x, 0, sizeof(_sensor_gyro_fft.peak_magnitude_x));
	memset(&_sensor_gyro_fft.peak_magnitude_y, 0, sizeof(_sensor_gyro_fft.peak_magnitude_y));
	memset(&_sensor_gyro_fft.peak_magnitude_z, 0, sizeof(_sensor_gyro_fft.peak_magnitude_z));


	for (int axis = 0; axis < 3; axis++) {
		// if we have enough samples begin processing
		if (_sliding_dft[axis].data_valid()) {
			_fft_updated = true;
			FindPeaks(timestamp_sample, axis);
		}
	}
}

void GyroFFT::FindPeaks(const hrt_abstime &timestamp_sample, int axis)
{

	float *peak_frequencies_raw[] {_sensor_gyro_fft.peak_frequencies_x_raw, _sensor_gyro_fft.peak_frequencies_y_raw, _sensor_gyro_fft.peak_frequencies_z_raw};
	float *peak_magnitude_out[] {_sensor_gyro_fft.peak_magnitude_x, _sensor_gyro_fft.peak_magnitude_y, _sensor_gyro_fft.peak_magnitude_z};



	const float resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;

	// sum total energy across all used buckets for SNR
	float bin_mag_sum = 0;

	// find raw peaks
	uint16_t raw_peak_index[MAX_NUM_PEAKS] {};
	float peak_magnitude[MAX_NUM_PEAKS] {};

	float peak_magnitudes_all[FFT_LEN] {};

	for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len / 2; bucket_index++) {

		const float real = _sliding_dft[axis].dft(bucket_index).real();
		const float imag = _sliding_dft[axis].dft(bucket_index).imag();

		const float fft_magnitude = sqrtf(real * real + imag * imag);

		peak_magnitudes_all[bucket_index] = fft_magnitude;

		bin_mag_sum += fft_magnitude;
	}

	for (int i = 0; i < MAX_NUM_PEAKS; i++) {

		float largest_peak = 0;
		int largest_peak_index = 0;

		for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len / 2; bucket_index++) {

			const float freq = (_gyro_sample_rate_hz * bucket_index / _imu_gyro_fft_len);

			if ((peak_magnitudes_all[bucket_index] > largest_peak)
			    && (freq >= _param_imu_gyro_fft_min.get())
			    && (freq <= _param_imu_gyro_fft_max.get())) {

				largest_peak = peak_magnitudes_all[bucket_index];
				largest_peak_index = bucket_index;
			}
		}

		if (largest_peak_index != 0) {
			raw_peak_index[i] = largest_peak_index;
			peak_magnitude[i] = peak_magnitudes_all[largest_peak_index];

			// remove peak + sides (included in frequency estimate later)
			peak_magnitudes_all[largest_peak_index - 1] = 0;
			peak_magnitudes_all[largest_peak_index]     = 0;
			peak_magnitudes_all[largest_peak_index + 1] = 0;

			// tmp logging
			peak_frequencies_raw[axis][i] = largest_peak_index * resolution_hz;
			peak_magnitude_out[axis][i] = peak_magnitude[i];
		}
	}




	_sensor_gyro_fft.total_energy[axis] = bin_mag_sum;

	// keep if peak has been previously seen and SNR > MIN_SNR
	//   or
	// peak has SNR > MIN_SNR_INITIAL
	static constexpr float MIN_SNR = 1.f; // TODO: configurable?

	int num_peaks_found = 0;
	float peak_frequencies[MAX_NUM_PEAKS] {};
	float peak_snr[MAX_NUM_PEAKS] {};

	float *peak_frequencies_publish[] { _sensor_gyro_fft.peak_frequencies_x, _sensor_gyro_fft.peak_frequencies_y, _sensor_gyro_fft.peak_frequencies_z };

	float peak_frequencies_prev[MAX_NUM_PEAKS];

	for (int i = 0; i < MAX_NUM_PEAKS; i++) {
		peak_frequencies_prev[i] = peak_frequencies_publish[axis][i];
	}

	for (int peak_new = 0; peak_new < MAX_NUM_PEAKS; peak_new++) {
		if (raw_peak_index[peak_new] > 0) {

			// estimate adjusted frequency bin, magnitude, and SNR for the largest peaks found
			const float adjusted_bin = EstimatePeakFrequencyBin(axis, raw_peak_index[peak_new]);

			if (PX4_ISFINITE(adjusted_bin)) {
				const float freq_adjusted = (_gyro_sample_rate_hz * adjusted_bin / _imu_gyro_fft_len);

				// PX4_INFO("bin: %.1f adjusted: %.1f freq adjusted: %.1f", (double)raw_peak_index[peak_new], (double)adjusted_bin,
				// 	 (double)freq_adjusted);

				const float snr = 10.f * log10f((_imu_gyro_fft_len - 1) * peak_magnitude[peak_new] /
								(bin_mag_sum - peak_magnitude[peak_new]));

				if (PX4_ISFINITE(freq_adjusted)
				    && (snr > MIN_SNR)
				    && (freq_adjusted >= _param_imu_gyro_fft_min.get())
				    && (freq_adjusted <= _param_imu_gyro_fft_max.get())) {

					// only keep if we're already tracking this frequency or if the SNR is significant
					for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
						bool snr_acceptable = (snr > _param_imu_gyro_fft_snr.get());
						bool peak_close = (fabsf(freq_adjusted - peak_frequencies_prev[peak_prev]) < (resolution_hz * 0.5f));

						if (snr_acceptable || peak_close) {
							// keep
							peak_frequencies[num_peaks_found] = freq_adjusted;
							peak_snr[num_peaks_found] = snr;

							// remove
							if (peak_close) {
								peak_frequencies_prev[peak_prev] = NAN;
							}

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
			//float peak_frequency = peak_frequencies[closest_new_peak];

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
					//float peak_frequency = peak_frequencies[peak_new];

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
