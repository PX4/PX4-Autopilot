/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "VehicleAngularVelocity.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);

#if !defined(CONSTRAINED_FLASH)
	perf_free(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_reset_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_free(_dynamic_notch_filter_fft_disable_perf);
	perf_free(_dynamic_notch_filter_fft_reset_perf);
	perf_free(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		ScheduleNow();
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_fifo_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

bool VehicleAngularVelocity::UpdateSampleRate()
{
	float sample_rate_hz = NAN;
	float publish_rate_hz = NAN;

	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().gyro_device_id == _selected_sensor_device_id) {
			sample_rate_hz = imu_status.get().gyro_raw_rate_hz;
			publish_rate_hz = imu_status.get().gyro_rate_hz;
			break;
		}
	}

	// calculate sensor update rate
	if ((sample_rate_hz > 0) && PX4_ISFINITE(sample_rate_hz) && (publish_rate_hz > 0) && PX4_ISFINITE(publish_rate_hz)) {
		// check if sample rate error is greater than 1%
		const bool sample_rate_changed = (fabsf(sample_rate_hz - _filter_sample_rate_hz) / sample_rate_hz) > 0.01f;

		if (_update_sample_rate || sample_rate_changed
		    || (_filter_sample_rate_hz <= FLT_EPSILON) || !PX4_ISFINITE(_filter_sample_rate_hz)) {

			PX4_DEBUG("updating sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate_hz, (double)sample_rate_hz);

			if (sample_rate_changed || !PX4_ISFINITE(_filter_sample_rate_hz)) {
				_reset_filters = true;
			}

			_filter_sample_rate_hz = sample_rate_hz;
			_update_sample_rate = false;

			if (_param_imu_gyro_ratemax.get() > 0.f) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
				const float publish_interval_us = 1e6f / publish_rate_hz;

				const uint8_t samples = roundf(configured_interval_us / publish_interval_us);

				if (_fifo_available) {
					_sensor_fifo_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_gyro_fifo_s::ORB_QUEUE_LENGTH));

				} else {
					_sensor_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_gyro_s::ORB_QUEUE_LENGTH));
				}

				// publish interval (constrained 100 Hz - 8 kHz)
				_publish_interval_min_us = math::constrain((int)roundf(configured_interval_us - (publish_interval_us * 0.5f)), 125,
							   10000);

			} else {
				_sensor_sub.set_required_updates(1);
				_sensor_fifo_sub.set_required_updates(1);
				_publish_interval_min_us = 0;
			}
		}
	}

	return PX4_ISFINITE(_filter_sample_rate_hz) && (_filter_sample_rate_hz > 0);
}

void VehicleAngularVelocity::ResetFilters()
{
	if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

		const Vector3f angular_velocity_uncalibrated{GetResetAngularVelocity()};
		const Vector3f angular_acceleration_uncalibrated{GetResetAngularAcceleration()};

		for (int axis = 0; axis < 3; axis++) {
			// angular velocity low pass
			_lp_filter_velocity[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_gyro_cutoff.get());
			_lp_filter_velocity[axis].reset(angular_velocity_uncalibrated(axis));

			// angular velocity notch
			_notch_filter_velocity[axis].setParameters(_filter_sample_rate_hz, _param_imu_gyro_nf_freq.get(),
					_param_imu_gyro_nf_bw.get());
			_notch_filter_velocity[axis].reset();

			// angular acceleration low pass
			if ((_param_imu_dgyro_cutoff.get() > 0.f)
			    && (_lp_filter_acceleration[axis].setCutoffFreq(_filter_sample_rate_hz, _param_imu_dgyro_cutoff.get()))) {
				_lp_filter_acceleration[axis].reset(angular_acceleration_uncalibrated(axis));

			} else {
				// disable filtering
				_lp_filter_acceleration[axis].setAlpha(1.f);
			}
		}

		// force reset notch filters on any scale change
		UpdateDynamicNotchEscRpm(true);
		UpdateDynamicNotchFFT(true);

		_angular_velocity_raw_prev = angular_velocity_uncalibrated;

		_reset_filters = false;
		perf_count(_filter_reset_perf);
	}
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias) && (bias.gyro_device_id == _selected_sensor_device_id)) {
			_bias = Vector3f{bias.gyro_bias};

		} else {
			_bias.zero();
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		bool selected_device_id_valid = false;
		uint32_t device_id = sensor_selection.gyro_device_id;
		uint32_t device_id_first_valid_imu = 0;

		// use vehicle_imu_status to do basic sensor selection validation
		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};
			bool imu_status_gyro_valid = false;

			if ((imu_status.get().gyro_device_id != 0) && (hrt_elapsed_time(&imu_status.get().timestamp) < 1_s)) {
				imu_status_gyro_valid = true;
			}

			if ((device_id != 0) && (imu_status.get().gyro_device_id == device_id) && imu_status_gyro_valid) {
				selected_device_id_valid = true;
			}

			// record first valid IMU as a backup option
			if ((device_id_first_valid_imu == 0) && imu_status_gyro_valid) {
				device_id_first_valid_imu = imu_status.get().gyro_device_id;
			}
		}

		// if no gyro selected or healthy then use fallback
		if ((device_id == 0) || !selected_device_id_valid) {
			device_id = device_id_first_valid_imu;
		}

		if ((_selected_sensor_device_id != device_id) || force) {

			const bool device_id_valid = (device_id != 0);

			// see if the selected sensor publishes sensor_gyro_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if (sensor_gyro_fifo_sub.get().device_id != 0) {
					// if no gyro was selected use the first valid sensor_gyro_fifo
					if (!device_id_valid && (hrt_elapsed_time(&sensor_gyro_fifo_sub.get().timestamp) < 1_s)) {
						device_id = sensor_gyro_fifo_sub.get().device_id;
					}

					if ((sensor_gyro_fifo_sub.get().device_id == device_id)
					    && _sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {

						// make sure non-FIFO sub is unregistered
						_sensor_sub.unregisterCallback();

						_calibration.set_device_id(sensor_gyro_fifo_sub.get().device_id);

						if (_calibration.enabled()) {
							_selected_sensor_device_id = sensor_gyro_fifo_sub.get().device_id;

							_timestamp_sample_last = 0;
							_filter_sample_rate_hz = 1.f / (sensor_gyro_fifo_sub.get().dt * 1e-6f);
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = true;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_gyro_fifo:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							_selected_sensor_device_id = 0;
						}
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id != 0) {
					// if no gyro was selected use the first valid sensor_gyro
					if (!device_id_valid && (hrt_elapsed_time(&sensor_gyro_sub.get().timestamp) < 1_s)) {
						device_id = sensor_gyro_sub.get().device_id;
					}

					if ((sensor_gyro_sub.get().device_id == device_id)
					    && _sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						// make sure FIFO sub is unregistered
						_sensor_fifo_sub.unregisterCallback();

						_calibration.set_device_id(sensor_gyro_sub.get().device_id);

						if (_calibration.enabled()) {
							_selected_sensor_device_id = sensor_gyro_sub.get().device_id;

							_timestamp_sample_last = 0;
							_filter_sample_rate_hz = NAN;
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = false;

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_gyro:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							_selected_sensor_device_id = 0;
						}
					}
				}
			}

			if (device_id != 0) {
				PX4_ERR("unable to find or subscribe to selected sensor (%" PRIu32 ")", device_id);
			}

			_selected_sensor_device_id = 0;
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const bool nf_enabled_prev = (_param_imu_gyro_nf_freq.get() > 0.f) && (_param_imu_gyro_nf_bw.get() > 0.f);

		updateParams();

		const bool nf_enabled = (_param_imu_gyro_nf_freq.get() > 0.f) && (_param_imu_gyro_nf_bw.get() > 0.f);

		_calibration.ParametersUpdate();

		// gyro low pass cutoff frequency changed
		for (auto &lp : _lp_filter_velocity) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

		// gyro notch filter frequency or bandwidth changed
		for (auto &nf : _notch_filter_velocity) {
			const bool nf_freq_changed = (fabsf(nf.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f);
			const bool nf_bw_changed   = (fabsf(nf.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f);

			if ((nf_enabled_prev != nf_enabled) || (nf_enabled && (nf_freq_changed || nf_bw_changed))) {
				_reset_filters = true;
				break;
			}
		}

		// gyro derivative low pass cutoff changed
		for (auto &lp : _lp_filter_acceleration) {
			if (fabsf(lp.getCutoffFreq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

#if !defined(CONSTRAINED_FLASH)

		if (_param_imu_gyro_dnf_en.get() & DynamicNotch::EscRpm) {
			if (_dynamic_notch_filter_esc_rpm_disable_perf == nullptr) {
				_dynamic_notch_filter_esc_rpm_disable_perf = perf_alloc(PC_COUNT,
						MODULE_NAME": gyro dynamic notch filter ESC RPM disable");
				_dynamic_notch_filter_esc_rpm_reset_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter ESC RPM reset");
				_dynamic_notch_filter_esc_rpm_update_perf = perf_alloc(PC_COUNT,
						MODULE_NAME": gyro dynamic notch filter ESC RPM update");
			}

		} else {
			DisableDynamicNotchEscRpm();
		}

		if (_param_imu_gyro_dnf_en.get() & DynamicNotch::FFT) {
			if (_dynamic_notch_filter_fft_disable_perf == nullptr) {
				_dynamic_notch_filter_fft_disable_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT disable");
				_dynamic_notch_filter_fft_reset_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT reset");
				_dynamic_notch_filter_fft_update_perf = perf_alloc(PC_COUNT, MODULE_NAME": gyro dynamic notch filter FFT update");
			}

		} else {
			DisableDynamicNotchFFT();
		}

#endif // !CONSTRAINED_FLASH
	}
}

Vector3f VehicleAngularVelocity::GetResetAngularVelocity() const
{
	if (_last_publish != 0) {
		// angular velocity filtering is performed on raw unscaled data
		//  start with last valid vehicle body frame angular velocity and compute equivalent raw data (for current sensor selection)
		Vector3f angular_velocity_uncalibrated{_calibration.Uncorrect(_angular_velocity + _bias)};

		if (PX4_ISFINITE(angular_velocity_uncalibrated(0))
		    && PX4_ISFINITE(angular_velocity_uncalibrated(1))
		    && PX4_ISFINITE(angular_velocity_uncalibrated(2))) {

			return angular_velocity_uncalibrated;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

Vector3f VehicleAngularVelocity::GetResetAngularAcceleration() const
{
	if (_last_publish != 0) {
		// angular acceleration filtering is performed on unscaled angular velocity data
		//  start with last valid vehicle body frame angular acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f angular_acceleration{_calibration.rotation().I() *_angular_acceleration};

		if (PX4_ISFINITE(angular_acceleration(0))
		    && PX4_ISFINITE(angular_acceleration(1))
		    && PX4_ISFINITE(angular_acceleration(2))) {

			return angular_acceleration;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

void VehicleAngularVelocity::DisableDynamicNotchEscRpm()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_esc_rpm_available) {
		for (int axis = 0; axis < 3; axis++) {
			for (int esc = 0; esc < MAX_NUM_ESC_RPM; esc++) {
				for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
					_dynamic_notch_filter_esc_rpm[axis][esc][harmonic].setParameters(0, 0, 0);
				}

				_esc_available.set(esc, false);
				perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);
			}
		}

		_dynamic_notch_esc_rpm_available = false;
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::DisableDynamicNotchFFT()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_fft_available) {
		for (int axis = 0; axis < 3; axis++) {
			for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
				_dynamic_notch_filter_fft[axis][peak].setParameters(0, 0, 0);
			}
		}

		_dynamic_notch_fft_available = false;
		perf_count(_dynamic_notch_filter_fft_disable_perf);
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchEscRpm(bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_gyro_dnf_en.get() & DynamicNotch::EscRpm;

	if (enabled && (_esc_status_sub.updated() || force)) {

		if (!_dynamic_notch_esc_rpm_available) {
			// force update filters if previously disabled
			force = true;
		}

		esc_status_s esc_status;

		if (_esc_status_sub.copy(&esc_status) && (hrt_elapsed_time(&esc_status.timestamp) < DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

			static constexpr float ESC_RPM_MIN = 10.f * 60.f; // TODO: configurable
			const float ESC_RPM_MAX = roundf(_filter_sample_rate_hz / 3.f * 60.f); // upper bound safety (well below Nyquist)

			for (size_t esc = 0; esc < math::min(esc_status.esc_count, (uint8_t)MAX_NUM_ESC_RPM); esc++) {
				const esc_report_s &esc_report = esc_status.esc[esc];
				const float esc_rpm = abs(esc_report.esc_rpm);

				// only update if ESC RPM range seems valid
				if ((esc_report.esc_rpm != 0) && (esc_rpm > ESC_RPM_MIN) && (esc_rpm < ESC_RPM_MAX)
				    && (hrt_elapsed_time(&esc_report.timestamp) < DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

					// for each ESC check determine if enabled/disabled from first notch (x axis, harmonic 0)
					auto &nfx0 = _dynamic_notch_filter_esc_rpm[0][esc][0];

					bool reset = force || (nfx0.getNotchFreq() <= FLT_EPSILON); // notch was previously disabled

					const float esc_hz = esc_rpm / 60.f;
					const float notch_freq_diff = fabsf(nfx0.getNotchFreq() - esc_hz);

					// update filter parameters if frequency changed or forced
					if (reset || (notch_freq_diff > 0.1f)) {

						// force reset if the notch frequency jumps significantly
						if (notch_freq_diff > _param_imu_gyro_dnf_bw.get()) {
							reset = true;
						}

						for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
							const float frequency_hz = esc_hz * (harmonic + 1);

							for (int axis = 0; axis < 3; axis++) {
								_dynamic_notch_filter_esc_rpm[axis][esc][harmonic].setParameters(_filter_sample_rate_hz, frequency_hz,
										_param_imu_gyro_dnf_bw.get());
							}
						}

						perf_count(_dynamic_notch_filter_esc_rpm_update_perf);
					}

					if (reset) {
						for (int axis = 0; axis < 3; axis++) {
							for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
								_dynamic_notch_filter_esc_rpm[axis][esc][harmonic].reset();
							}
						}

						perf_count(_dynamic_notch_filter_esc_rpm_reset_perf);
					}

					_dynamic_notch_esc_rpm_available = true;
					_esc_available.set(esc, true);
					_last_esc_rpm_notch_update[esc] = esc_report.timestamp;

				} else if (_esc_available[esc]
					   && (force || (hrt_elapsed_time(&_last_esc_rpm_notch_update[esc]) >= DYNAMIC_NOTCH_FITLER_TIMEOUT))) {
					// if this ESC was previously available disable all notch filters if forced or timeout
					_esc_available.set(esc, false);

					perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);

					for (int axis = 0; axis < 3; axis++) {
						for (int harmonic = 0; harmonic < MAX_NUM_ESC_RPM_HARMONICS; harmonic++) {
							_dynamic_notch_filter_esc_rpm[axis][esc][harmonic].setParameters(0, 0, 0);
						}
					}
				}
			}

		} else {
			DisableDynamicNotchEscRpm();
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAngularVelocity::UpdateDynamicNotchFFT(bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_gyro_dnf_en.get() & DynamicNotch::FFT;

	if (enabled && (_sensor_gyro_fft_sub.updated() || force)) {

		if (!_dynamic_notch_fft_available) {
			// force update filters if previously disabled
			force = true;
		}

		sensor_gyro_fft_s sensor_gyro_fft;

		if (_sensor_gyro_fft_sub.copy(&sensor_gyro_fft)
		    && (sensor_gyro_fft.device_id == _selected_sensor_device_id)
		    && (hrt_elapsed_time(&sensor_gyro_fft.timestamp) < DYNAMIC_NOTCH_FITLER_TIMEOUT)
		    && ((fabsf(sensor_gyro_fft.sensor_sample_rate_hz - _filter_sample_rate_hz) / _filter_sample_rate_hz) < 0.02f)) {

			// ignore any peaks below half the gyro cutoff frequency
			const float peak_freq_min = 10.f; // lower bound TODO: configurable?
			const float peak_freq_max = _filter_sample_rate_hz / 3.f; // upper bound safety (well below Nyquist)

			const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f); // TODO: base on numerical limits?

			float *peak_frequencies[] {sensor_gyro_fft.peak_frequencies_x, sensor_gyro_fft.peak_frequencies_y, sensor_gyro_fft.peak_frequencies_z};

			for (int axis = 0; axis < 3; axis++) {
				for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
					auto &nf = _dynamic_notch_filter_fft[axis][peak];

					bool reset = (nf.getNotchFreq() <= FLT_EPSILON); // notch was previously disabled

					const float peak_freq = peak_frequencies[axis][peak];

					if (PX4_ISFINITE(peak_freq) && (peak_freq > peak_freq_min) && (peak_freq < peak_freq_max)) {

						const float notch_freq_diff = fabsf(nf.getNotchFreq() - peak_freq);

						// update filter parameters if frequency changed or forced
						if (force || reset || (notch_freq_diff > 0.1f)) {
							nf.setParameters(_filter_sample_rate_hz, peak_freq, bandwidth);
							perf_count(_dynamic_notch_filter_fft_update_perf);
						}

						// force reset if the notch frequency jumps significantly
						if (force || reset || (notch_freq_diff > bandwidth)) {
							nf.reset();
							perf_count(_dynamic_notch_filter_fft_reset_perf);
						}

						_dynamic_notch_fft_available = true;

					} else {
						// disable this notch filter (if it isn't already)
						if (force || !reset) {
							nf.setParameters(0, 0, 0);
							perf_count(_dynamic_notch_filter_fft_disable_perf);
						}
					}
				}
			}

		} else {
			DisableDynamicNotchFFT();
		}
	}

#endif // !CONSTRAINED_FLASH
}

float VehicleAngularVelocity::FilterAngularVelocity(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)

	// Apply dynamic notch filter from ESC RPM
	if (_dynamic_notch_esc_rpm_available) {

		for (int esc = 0; esc < MAX_NUM_ESC_RPM; esc++) {
			if (_esc_available[esc]) {
				// apply notch filters higher -> lowest frequency
				for (int harmonic = MAX_NUM_ESC_RPM_HARMONICS - 1; harmonic >= 0; harmonic--) {
					_dynamic_notch_filter_esc_rpm[axis][esc][harmonic].applyArray(data, N);
				}
			}
		}
	}

	// Apply dynamic notch filter from FFT
	if (_dynamic_notch_fft_available) {
		for (int peak = MAX_NUM_FFT_PEAKS - 1; peak >= 0; peak--) {
			if (_dynamic_notch_filter_fft[axis][peak].getNotchFreq() > 0.f) {
				_dynamic_notch_filter_fft[axis][peak].applyArray(data, N);
			}
		}
	}

#endif // !CONSTRAINED_FLASH

	// Apply general notch filter (IMU_GYRO_NF_FREQ)
	if (_notch_filter_velocity[axis].getNotchFreq() > 0.f) {
		_notch_filter_velocity[axis].applyArray(data, N);
	}

	// Apply general low-pass filter (IMU_GYRO_CUTOFF)
	_lp_filter_velocity[axis].applyArray(data, N);

	// return last filtered sample
	return data[N - 1];
}

float VehicleAngularVelocity::FilterAngularAcceleration(int axis, float inverse_dt_s, float data[], int N)
{
	// angular acceleration: Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
	float angular_acceleration_filtered = 0.f;

	for (int n = 0; n < N; n++) {
		const float angular_acceleration = (data[n] - _angular_velocity_raw_prev(axis)) * inverse_dt_s;
		angular_acceleration_filtered = _lp_filter_acceleration[axis].update(angular_acceleration);
		_angular_velocity_raw_prev(axis) = data[n];
	}

	return angular_acceleration_filtered;
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate();

	if (selection_updated || _update_sample_rate) {
		if (!UpdateSampleRate()) {
			// sensor sample rate required to run
			return;
		}
	}

	perf_begin(_cycle_perf);

	ParametersUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);

	if (_reset_filters) {
		ResetFilters();

		if (_reset_filters) {
			// not safe to run until filters configured
			return;
		}
	}

	UpdateDynamicNotchEscRpm();
	UpdateDynamicNotchFFT();

	if (_fifo_available) {
		// process all outstanding fifo messages
		sensor_gyro_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {
			const float inverse_dt_s = 1e6f / sensor_fifo_data.dt;
			const int N = sensor_fifo_data.samples;
			static constexpr int FIFO_SIZE_MAX = sizeof(sensor_fifo_data.x) / sizeof(sensor_fifo_data.x[0]);

			if ((sensor_fifo_data.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
				Vector3f angular_velocity_uncalibrated;
				Vector3f angular_acceleration_uncalibrated;

				int16_t *raw_data_array[] {sensor_fifo_data.x, sensor_fifo_data.y, sensor_fifo_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering
					float data[FIFO_SIZE_MAX];

					for (int n = 0; n < N; n++) {
						data[n] = sensor_fifo_data.scale * raw_data_array[axis][n];
					}

					// save last filtered sample
					angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data, N);
					angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data, N);
				}

				// Publish
				if (!_sensor_fifo_sub.updated()) {
					if (CalibrateAndPublish(sensor_fifo_data.timestamp_sample,
								angular_velocity_uncalibrated, angular_acceleration_uncalibrated)) {

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}

	} else {
		// process all outstanding messages
		sensor_gyro_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			if (PX4_ISFINITE(sensor_data.x) && PX4_ISFINITE(sensor_data.y) && PX4_ISFINITE(sensor_data.z)) {

				if (_timestamp_sample_last == 0 || (sensor_data.timestamp_sample <= _timestamp_sample_last)) {
					_timestamp_sample_last = sensor_data.timestamp_sample - 1e6f / _filter_sample_rate_hz;
				}

				const float inverse_dt_s = 1.f / math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) * 1e-6f),
							   0.00002f, 0.02f);
				_timestamp_sample_last = sensor_data.timestamp_sample;

				Vector3f angular_velocity_uncalibrated;
				Vector3f angular_acceleration_uncalibrated;

				float raw_data_array[] {sensor_data.x, sensor_data.y, sensor_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy sensor sample to float array for filtering
					float data[1] {raw_data_array[axis]};

					// save last filtered sample
					angular_velocity_uncalibrated(axis) = FilterAngularVelocity(axis, data);
					angular_acceleration_uncalibrated(axis) = FilterAngularAcceleration(axis, inverse_dt_s, data);
				}

				// Publish
				if (!_sensor_sub.updated()) {
					if (CalibrateAndPublish(sensor_data.timestamp_sample,
								angular_velocity_uncalibrated, angular_acceleration_uncalibrated)) {

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}
	}

	// force reselection on timeout
	if (hrt_elapsed_time(&_last_publish) > 500_ms) {
		SensorSelectionUpdate(true);
	}

	perf_end(_cycle_perf);
}

bool VehicleAngularVelocity::CalibrateAndPublish(const hrt_abstime &timestamp_sample,
		const Vector3f &angular_velocity_uncalibrated, const Vector3f &angular_acceleration_uncalibrated)
{
	if (timestamp_sample >= _last_publish + _publish_interval_min_us) {

		// Publish vehicle_angular_acceleration
		vehicle_angular_acceleration_s v_angular_acceleration;
		v_angular_acceleration.timestamp_sample = timestamp_sample;

		// Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
		_angular_acceleration = _calibration.rotation() * angular_acceleration_uncalibrated;
		_angular_acceleration.copyTo(v_angular_acceleration.xyz);

		v_angular_acceleration.timestamp = hrt_absolute_time();
		_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);


		// Publish vehicle_angular_velocity
		vehicle_angular_velocity_s v_angular_velocity;
		v_angular_velocity.timestamp_sample = timestamp_sample;

		// Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
		_angular_velocity = _calibration.Correct(angular_velocity_uncalibrated) - _bias;
		_angular_velocity.copyTo(v_angular_velocity.xyz);

		v_angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity_pub.publish(v_angular_velocity);


		// shift last publish time forward, but don't let it get further behind than the interval
		_last_publish = math::constrain(_last_publish + _publish_interval_min_us,
						timestamp_sample - _publish_interval_min_us, timestamp_sample);

		return true;
	}

	return false;
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %" PRIu32 ", rate: %.1f Hz %s, estimated bias: [%.5f %.5f %.5f]",
		 _calibration.device_id(), (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
		 (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_filter_reset_perf);
	perf_print_counter(_selection_changed_perf);
#if !defined(CONSTRAINED_FLASH)
	perf_print_counter(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_reset_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_print_counter(_dynamic_notch_filter_fft_disable_perf);
	perf_print_counter(_dynamic_notch_filter_fft_reset_perf);
	perf_print_counter(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

} // namespace sensors
