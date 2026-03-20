/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_acceleration_pub.advertise();
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();

	perf_free(_cycle_perf);
	perf_free(_filter_reset_perf);
	perf_free(_selection_changed_perf);

#if !defined(CONSTRAINED_FLASH)
	delete[] _dynamic_notch_filter_esc_rpm;
	perf_free(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_init_perf);
	perf_free(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_free(_dynamic_notch_filter_fft_disable_perf);
	perf_free(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(hrt_absolute_time(), true)) {
		ScheduleNow();
	}

	return true;
}

void VehicleAcceleration::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_accel_fifo_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

bool VehicleAcceleration::UpdateSampleRate()
{
	float sample_rate_hz = NAN;
	float publish_rate_hz = NAN;

	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		if (imu_status.get().accel_device_id == _selected_sensor_device_id) {
			sample_rate_hz = imu_status.get().accel_raw_rate_hz;
			publish_rate_hz = imu_status.get().accel_rate_hz;
			break;
		}
	}

	// calculate sensor update rate
	if (PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 10) && (sample_rate_hz < 10'000)
	    && PX4_ISFINITE(publish_rate_hz) && (publish_rate_hz > 0)
	   ) {
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

			if (_param_imu_integ_rate.get() > 0) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
				const float publish_interval_us = 1e6f / publish_rate_hz;

				const uint8_t samples = roundf(configured_interval_us / publish_interval_us);

				if (_fifo_available) {
					_sensor_accel_fifo_sub.set_required_updates(math::constrain(samples, (uint8_t)1,
							sensor_accel_fifo_s::ORB_QUEUE_LENGTH));

				} else {
					_sensor_sub.set_required_updates(math::constrain(samples, (uint8_t)1, sensor_accel_s::ORB_QUEUE_LENGTH));
				}

				// publish interval (constrained 100 Hz - 8 kHz)
				_publish_interval_min_us = math::constrain((int)roundf(configured_interval_us - (publish_interval_us * 0.5f)), 125,
							   10000);

			} else {
				_sensor_sub.set_required_updates(1);
				_sensor_accel_fifo_sub.set_required_updates(1);
				_publish_interval_min_us = 0;
			}
		}
	}

	return PX4_ISFINITE(_filter_sample_rate_hz) && (_filter_sample_rate_hz > 0);
}

void VehicleAcceleration::ResetFilters(const hrt_abstime &time_now_us)
{
	if ((_filter_sample_rate_hz > 0) && PX4_ISFINITE(_filter_sample_rate_hz)) {

		const Vector3f acceleration_uncalibrated{GetResetAcceleration()};

		for (int axis = 0; axis < 3; axis++) {
			// acceleration low pass
			_lp_filter[axis].set_cutoff_frequency(_filter_sample_rate_hz, _param_imu_accel_cutoff.get());
			_lp_filter[axis].reset(acceleration_uncalibrated(axis));

			// acceleration notch 0
			_notch_filter0[axis].setParameters(_filter_sample_rate_hz, _param_imu_acc_nf0_frq.get(),
							   _param_imu_acc_nf0_bw.get());
			_notch_filter0[axis].reset();

			// acceleration notch 1
			_notch_filter1[axis].setParameters(_filter_sample_rate_hz, _param_imu_acc_nf1_frq.get(),
							   _param_imu_acc_nf1_bw.get());
			_notch_filter1[axis].reset();
		}

		// force reset notch filters on any scale change
		UpdateDynamicNotchEscRpm(time_now_us, true);
		UpdateDynamicNotchFFT(time_now_us, true);

		_reset_filters = false;
		perf_count(_filter_reset_perf);
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
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

		if (_estimator_sensor_bias_sub.copy(&bias) && (bias.accel_device_id == _selected_sensor_device_id)) {
			_bias = Vector3f{bias.accel_bias};

		} else {
			_bias.zero();
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(const hrt_abstime &time_now_us, bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		bool selected_device_id_valid = false;
		uint32_t device_id = sensor_selection.accel_device_id;
		uint32_t device_id_first_valid_imu = 0;

		// use vehicle_imu_status to do basic sensor selection validation
		for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
			uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

			if (imu_status.advertised()
			    && (imu_status.get().timestamp != 0) && (time_now_us < imu_status.get().timestamp + 1_s)
			    && (imu_status.get().accel_device_id != 0)) {
				// vehicle_imu_status accel valid

				if ((device_id != 0) && (imu_status.get().accel_device_id == device_id)) {
					selected_device_id_valid = true;
				}

				// record first valid IMU as a backup option
				if (device_id_first_valid_imu == 0) {
					device_id_first_valid_imu = imu_status.get().accel_device_id;
				}
			}
		}

		// if no accel selected or healthy then use fallback
		if ((device_id == 0) || !selected_device_id_valid) {
			device_id = device_id_first_valid_imu;
		}

		if ((_selected_sensor_device_id != device_id) || force) {

			const bool device_id_valid = (device_id != 0);

			// see if the selected sensor publishes sensor_accel_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_fifo_s> sensor_accel_fifo_sub{ORB_ID(sensor_accel_fifo), i};

				if (sensor_accel_fifo_sub.advertised()
				    && (sensor_accel_fifo_sub.get().timestamp != 0)
				    && (sensor_accel_fifo_sub.get().device_id != 0)
				    && (time_now_us < sensor_accel_fifo_sub.get().timestamp + 1_s)) {

					// if no accel was selected use the first valid sensor_accel_fifo
					if (!device_id_valid) {
						device_id = sensor_accel_fifo_sub.get().device_id;
						PX4_DEBUG("no accel selected, using sensor_accel_fifo:%" PRIu8 " %" PRIu32, i,
							  sensor_accel_fifo_sub.get().device_id);
					}

					if (sensor_accel_fifo_sub.get().device_id == device_id) {
						if (_sensor_accel_fifo_sub.ChangeInstance(i) && _sensor_accel_fifo_sub.registerCallback()) {
							// make sure non-FIFO sub is unregistered
							_sensor_sub.unregisterCallback();

							_calibration.set_device_id(sensor_accel_fifo_sub.get().device_id);

							_selected_sensor_device_id = sensor_accel_fifo_sub.get().device_id;

							_filter_sample_rate_hz = 1.f / (sensor_accel_fifo_sub.get().dt * 1e-6f);
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = true;

							// look up corresponding gyro device_id for FFT matching
							_selected_gyro_device_id = 0;

							for (uint8_t j = 0; j < MAX_SENSOR_COUNT; j++) {
								uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), j};

								if (imu_status.get().accel_device_id == _selected_sensor_device_id) {
									_selected_gyro_device_id = imu_status.get().gyro_device_id;
									break;
								}
							}

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_accel_fifo:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							PX4_ERR("unable to register callback for sensor_accel_fifo:%" PRIu8 " %" PRIu32,
								i, sensor_accel_fifo_sub.get().device_id);
						}
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				if (sensor_accel_sub.advertised()
				    && (sensor_accel_sub.get().timestamp != 0)
				    && (sensor_accel_sub.get().device_id != 0)
				    && (time_now_us < sensor_accel_sub.get().timestamp + 1_s)) {

					// if no accel was selected use the first valid sensor_accel
					if (!device_id_valid) {
						device_id = sensor_accel_sub.get().device_id;
						PX4_DEBUG("no accel selected, using sensor_accel:%" PRIu8 " %" PRIu32, i,
							  sensor_accel_sub.get().device_id);
					}

					if (sensor_accel_sub.get().device_id == device_id) {
						if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
							// make sure FIFO sub is unregistered
							_sensor_accel_fifo_sub.unregisterCallback();

							_calibration.set_device_id(sensor_accel_sub.get().device_id);

							_selected_sensor_device_id = sensor_accel_sub.get().device_id;

							_filter_sample_rate_hz = NAN;
							_update_sample_rate = true;
							_reset_filters = true;
							_bias.zero();
							_fifo_available = false;

							// look up corresponding gyro device_id for FFT matching
							_selected_gyro_device_id = 0;

							for (uint8_t j = 0; j < MAX_SENSOR_COUNT; j++) {
								uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), j};

								if (imu_status.get().accel_device_id == _selected_sensor_device_id) {
									_selected_gyro_device_id = imu_status.get().gyro_device_id;
									break;
								}
							}

							perf_count(_selection_changed_perf);
							PX4_DEBUG("selecting sensor_accel:%" PRIu8 " %" PRIu32, i, _selected_sensor_device_id);
							return true;

						} else {
							PX4_ERR("unable to register callback for sensor_accel:%" PRIu8 " %" PRIu32,
								i, sensor_accel_sub.get().device_id);
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

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		const bool nf0_enabled_prev = (_param_imu_acc_nf0_frq.get() > 0.f) && (_param_imu_acc_nf0_bw.get() > 0.f);
		const bool nf1_enabled_prev = (_param_imu_acc_nf1_frq.get() > 0.f) && (_param_imu_acc_nf1_bw.get() > 0.f);

		updateParams();

		const bool nf0_enabled = (_param_imu_acc_nf0_frq.get() > 0.f) && (_param_imu_acc_nf0_bw.get() > 0.f);
		const bool nf1_enabled = (_param_imu_acc_nf1_frq.get() > 0.f) && (_param_imu_acc_nf1_bw.get() > 0.f);

		_calibration.ParametersUpdate();

		// accel low pass cutoff frequency changed
		for (auto &lp : _lp_filter) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.01f) {
				_reset_filters = true;
				break;
			}
		}

		// accel notch filter 0 frequency or bandwidth changed
		for (auto &nf : _notch_filter0) {
			const bool nf_freq_changed = (fabsf(nf.getNotchFreq() - _param_imu_acc_nf0_frq.get()) > 0.01f);
			const bool nf_bw_changed   = (fabsf(nf.getBandwidth() - _param_imu_acc_nf0_bw.get()) > 0.01f);

			if ((nf0_enabled_prev != nf0_enabled) || (nf0_enabled && (nf_freq_changed || nf_bw_changed))) {
				_reset_filters = true;
				break;
			}
		}

		// accel notch filter 1 frequency or bandwidth changed
		for (auto &nf : _notch_filter1) {
			const bool nf_freq_changed = (fabsf(nf.getNotchFreq() - _param_imu_acc_nf1_frq.get()) > 0.01f);
			const bool nf_bw_changed   = (fabsf(nf.getBandwidth() - _param_imu_acc_nf1_bw.get()) > 0.01f);

			if ((nf1_enabled_prev != nf1_enabled) || (nf1_enabled && (nf_freq_changed || nf_bw_changed))) {
				_reset_filters = true;
				break;
			}
		}

#if !defined(CONSTRAINED_FLASH)

		if (_param_imu_acc_dnf_en.get() & DynamicNotch::EscRpm) {

			const int32_t esc_rpm_harmonics = math::constrain(_param_imu_acc_dnf_hmc.get(), (int32_t)1, (int32_t)7);

			if (_dynamic_notch_filter_esc_rpm && (esc_rpm_harmonics != _esc_rpm_harmonics)) {
				delete[] _dynamic_notch_filter_esc_rpm;
				_dynamic_notch_filter_esc_rpm = nullptr;
				_esc_rpm_harmonics = 0;
			}

			if (_dynamic_notch_filter_esc_rpm == nullptr) {

				_dynamic_notch_filter_esc_rpm = new NotchFilterHarmonic[esc_rpm_harmonics];

				if (_dynamic_notch_filter_esc_rpm) {
					_esc_rpm_harmonics = esc_rpm_harmonics;

					if (_dynamic_notch_filter_esc_rpm_disable_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_disable_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": accel dynamic notch filter ESC RPM disable");
					}

					if (_dynamic_notch_filter_esc_rpm_init_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_init_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": accel dynamic notch filter ESC RPM init");
					}

					if (_dynamic_notch_filter_esc_rpm_update_perf == nullptr) {
						_dynamic_notch_filter_esc_rpm_update_perf = perf_alloc(PC_COUNT,
								MODULE_NAME": accel dynamic notch filter ESC RPM update");
					}

				} else {
					_esc_rpm_harmonics = 0;

					perf_free(_dynamic_notch_filter_esc_rpm_disable_perf);
					perf_free(_dynamic_notch_filter_esc_rpm_init_perf);
					perf_free(_dynamic_notch_filter_esc_rpm_update_perf);

					_dynamic_notch_filter_esc_rpm_disable_perf = nullptr;
					_dynamic_notch_filter_esc_rpm_init_perf = nullptr;
					_dynamic_notch_filter_esc_rpm_update_perf = nullptr;
				}
			}

		} else {
			DisableDynamicNotchEscRpm();
		}

		if (_param_imu_acc_dnf_en.get() & DynamicNotch::FFT) {
			if (_dynamic_notch_filter_fft_disable_perf == nullptr) {
				_dynamic_notch_filter_fft_disable_perf = perf_alloc(PC_COUNT, MODULE_NAME": accel dynamic notch filter FFT disable");
				_dynamic_notch_filter_fft_update_perf = perf_alloc(PC_COUNT, MODULE_NAME": accel dynamic notch filter FFT update");
			}

		} else {
			DisableDynamicNotchFFT();
		}

#endif // !CONSTRAINED_FLASH
	}
}

Vector3f VehicleAcceleration::GetResetAcceleration() const
{
	if (_last_publish != 0) {
		// acceleration filtering is performed on raw uncalibrated data
		//  start with last valid body frame acceleration and compute equivalent raw data (for current sensor selection)
		Vector3f acceleration_uncalibrated{_calibration.Uncorrect(_acceleration + _bias)};

		if (acceleration_uncalibrated.isAllFinite()) {
			return acceleration_uncalibrated;
		}
	}

	return Vector3f{0.f, 0.f, 0.f};
}

void VehicleAcceleration::DisableDynamicNotchEscRpm()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_filter_esc_rpm) {
		for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
			for (int axis = 0; axis < 3; axis++) {
				for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
					_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].disable();
					_esc_available.set(esc, false);
					perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAcceleration::DisableDynamicNotchFFT()
{
#if !defined(CONSTRAINED_FLASH)

	if (_dynamic_notch_fft_available) {
		for (int axis = 0; axis < 3; axis++) {
			for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {
				_dynamic_notch_filter_fft[axis][peak].disable();
				perf_count(_dynamic_notch_filter_fft_disable_perf);
			}
		}

		_dynamic_notch_fft_available = false;
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAcceleration::UpdateDynamicNotchEscRpm(const hrt_abstime &time_now_us, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _dynamic_notch_filter_esc_rpm && (_param_imu_acc_dnf_en.get() & DynamicNotch::EscRpm);

	if (enabled && (_esc_status_sub.updated() || force)) {

		bool axis_init[3] {false, false, false};

		esc_status_s esc_status;

		if (_esc_status_sub.copy(&esc_status) && (time_now_us < esc_status.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

			const float bandwidth_hz = _param_imu_acc_dnf_bw.get();
			const float freq_min = math::max(_param_imu_acc_dnf_min.get(), bandwidth_hz);

			for (size_t esc = 0; esc < math::min(esc_status.esc_count, (uint8_t)MAX_NUM_ESCS); esc++) {
				const esc_report_s &esc_report = esc_status.esc[esc];

				const bool esc_connected = (esc_status.esc_online_flags & (1 << esc)) || (esc_report.esc_rpm != 0);

				// only update if ESC RPM range seems valid
				if (esc_connected && (time_now_us < esc_report.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {

					const float esc_hz = abs(esc_report.esc_rpm) / 60.f;

					const bool force_update = force || !_esc_available[esc]; // force parameter update or notch was previously disabled

					for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
						// as RPM drops leave the notch filter "parked" at the minimum rather than disabling
						//  keep harmonics separated by half the notch filter bandwidth
						const float frequency_hz = math::max(esc_hz * (harmonic + 1), freq_min + (harmonic * 0.5f * bandwidth_hz));

						// update filter parameters if frequency changed or forced
						for (int axis = 0; axis < 3; axis++) {
							auto &nf = _dynamic_notch_filter_esc_rpm[harmonic][axis][esc];

							const float notch_freq_delta = fabsf(nf.getNotchFreq() - frequency_hz);

							const bool notch_freq_changed = (notch_freq_delta > 0.1f);

							// only allow initializing one new filter per axis each iteration
							const bool allow_update = !axis_init[axis] || (nf.initialized() && notch_freq_delta < nf.getBandwidth());

							if ((force_update || notch_freq_changed) && allow_update) {
								if (nf.setParameters(_filter_sample_rate_hz, frequency_hz, bandwidth_hz)) {
									perf_count(_dynamic_notch_filter_esc_rpm_update_perf);

									if (!nf.initialized()) {
										perf_count(_dynamic_notch_filter_esc_rpm_init_perf);
										axis_init[axis] = true;
									}
								}
							}
						}
					}

					_esc_available.set(esc, true);
					_last_esc_rpm_notch_update[esc] = esc_report.timestamp;
				}
			}
		}

		// check notch filter timeout
		for (size_t esc = 0; esc < MAX_NUM_ESCS; esc++) {
			if (_esc_available[esc] && (time_now_us > _last_esc_rpm_notch_update[esc] + DYNAMIC_NOTCH_FITLER_TIMEOUT)) {
				bool all_disabled = true;

				// disable notch filters from highest frequency to lowest
				for (int harmonic = _esc_rpm_harmonics - 1; harmonic >= 0; harmonic--) {
					for (int axis = 0; axis < 3; axis++) {
						auto &nf = _dynamic_notch_filter_esc_rpm[harmonic][axis][esc];

						if (nf.getNotchFreq() > 0.f) {
							if (nf.initialized() && !axis_init[axis]) {
								nf.disable();
								perf_count(_dynamic_notch_filter_esc_rpm_disable_perf);
								axis_init[axis] = true;
							}
						}

						if (nf.getNotchFreq() > 0.f) {
							all_disabled = false;
						}
					}
				}

				if (all_disabled) {
					_esc_available.set(esc, false);
				}
			}
		}
	}

#endif // !CONSTRAINED_FLASH
}

void VehicleAcceleration::UpdateDynamicNotchFFT(const hrt_abstime &time_now_us, bool force)
{
#if !defined(CONSTRAINED_FLASH)
	const bool enabled = _param_imu_acc_dnf_en.get() & DynamicNotch::FFT;

	if (enabled && (_sensor_gyro_fft_sub.updated() || force)) {

		if (!_dynamic_notch_fft_available) {
			// force update filters if previously disabled
			force = true;
		}

		sensor_gyro_fft_s sensor_gyro_fft;

		if (_sensor_gyro_fft_sub.copy(&sensor_gyro_fft)
		    && (sensor_gyro_fft.device_id == _selected_gyro_device_id)
		    && (_selected_gyro_device_id != 0)
		    && (time_now_us < sensor_gyro_fft.timestamp + DYNAMIC_NOTCH_FITLER_TIMEOUT)
		    && (sensor_gyro_fft.sensor_sample_rate_hz > 0)) {

			static constexpr float peak_freq_min = 10.f; // lower bound

			const float bandwidth = math::constrain(sensor_gyro_fft.resolution_hz, 8.f, 30.f);

			float *peak_frequencies[] {sensor_gyro_fft.peak_frequencies_x, sensor_gyro_fft.peak_frequencies_y, sensor_gyro_fft.peak_frequencies_z};

			for (int axis = 0; axis < 3; axis++) {
				for (int peak = 0; peak < MAX_NUM_FFT_PEAKS; peak++) {

					const float peak_freq = peak_frequencies[axis][peak];

					auto &nf = _dynamic_notch_filter_fft[axis][peak];

					if (peak_freq > peak_freq_min) {
						// update filter parameters if frequency changed or forced
						if (force || !nf.initialized() || (fabsf(nf.getNotchFreq() - peak_freq) > 0.1f)) {
							nf.setParameters(_filter_sample_rate_hz, peak_freq, bandwidth);
							perf_count(_dynamic_notch_filter_fft_update_perf);
						}

						_dynamic_notch_fft_available = true;

					} else {
						// disable this notch filter (if it isn't already)
						if (nf.getNotchFreq() > 0.f) {
							nf.disable();
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

float VehicleAcceleration::FilterAcceleration(int axis, float data[], int N)
{
#if !defined(CONSTRAINED_FLASH)

	// Apply dynamic notch filter from ESC RPM
	if (_dynamic_notch_filter_esc_rpm) {
		for (int esc = 0; esc < MAX_NUM_ESCS; esc++) {
			if (_esc_available[esc]) {
				for (int harmonic = 0; harmonic < _esc_rpm_harmonics; harmonic++) {
					if (_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].getNotchFreq() > 0.f) {
						_dynamic_notch_filter_esc_rpm[harmonic][axis][esc].applyArray(data, N);
					}
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

	// Apply general notch filter 0 (IMU_ACC_NF0_FRQ)
	if (_notch_filter0[axis].getNotchFreq() > 0.f) {
		_notch_filter0[axis].applyArray(data, N);
	}

	// Apply general notch filter 1 (IMU_ACC_NF1_FRQ)
	if (_notch_filter1[axis].getNotchFreq() > 0.f) {
		_notch_filter1[axis].applyArray(data, N);
	}

	// Apply general low-pass filter (IMU_ACCEL_CUTOFF)
	_lp_filter[axis].applyArray(data, N);

	// return last filtered sample
	return data[N - 1];
}

void VehicleAcceleration::Run()
{
	perf_begin(_cycle_perf);

	// backup schedule
	ScheduleDelayed(10_ms);

	const hrt_abstime time_now_us = hrt_absolute_time();

	ParametersUpdate();

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate(time_now_us);

	if (selection_updated || _update_sample_rate) {
		if (!UpdateSampleRate()) {
			// sensor sample rate required to run
			perf_end(_cycle_perf);
			return;
		}
	}

	_calibration.SensorCorrectionsUpdate(selection_updated);

	SensorBiasUpdate(selection_updated);

	if (_reset_filters) {
		ResetFilters(time_now_us);

		if (_reset_filters) {
			// not safe to run until filters configured
			perf_end(_cycle_perf);
			return;
		}
	}

	UpdateDynamicNotchEscRpm(time_now_us);
	UpdateDynamicNotchFFT(time_now_us);

	if (_fifo_available) {
		// process all outstanding fifo messages
		int sensor_sub_updates = 0;
		sensor_accel_fifo_s sensor_fifo_data;

		while ((sensor_sub_updates < sensor_accel_fifo_s::ORB_QUEUE_LENGTH) && _sensor_accel_fifo_sub.update(&sensor_fifo_data)) {
			sensor_sub_updates++;

			const int N = sensor_fifo_data.samples;
			static constexpr int FIFO_SIZE_MAX = sizeof(sensor_fifo_data.x) / sizeof(sensor_fifo_data.x[0]);

			if ((sensor_fifo_data.dt > 0) && (N > 0) && (N <= FIFO_SIZE_MAX)) {
				Vector3f acceleration_uncalibrated;

				int16_t *raw_data_array[] {sensor_fifo_data.x, sensor_fifo_data.y, sensor_fifo_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering
					float data[FIFO_SIZE_MAX];

					for (int n = 0; n < N; n++) {
						data[n] = sensor_fifo_data.scale * raw_data_array[axis][n];
					}

					// save last filtered sample
					acceleration_uncalibrated(axis) = FilterAcceleration(axis, data, N);
				}

				// Publish
				if (!_sensor_accel_fifo_sub.updated()) {
					if (sensor_fifo_data.timestamp_sample >= _last_publish + _publish_interval_min_us) {

						// Apply calibration, rotate to body frame, and subtract estimated bias
						_acceleration = _calibration.Correct(acceleration_uncalibrated) - _bias;

						vehicle_acceleration_s v_acceleration;
						v_acceleration.timestamp_sample = sensor_fifo_data.timestamp_sample;
						_acceleration.copyTo(v_acceleration.xyz);
						v_acceleration.timestamp = hrt_absolute_time();
						_vehicle_acceleration_pub.publish(v_acceleration);

						// shift last publish time forward, but don't let it get further behind than the interval
						_last_publish = math::constrain(_last_publish + _publish_interval_min_us,
										sensor_fifo_data.timestamp_sample - _publish_interval_min_us,
										sensor_fifo_data.timestamp_sample);

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}

	} else {
		// process all outstanding messages
		int sensor_sub_updates = 0;
		sensor_accel_s sensor_data;

		while ((sensor_sub_updates < sensor_accel_s::ORB_QUEUE_LENGTH) && _sensor_sub.update(&sensor_data)) {
			sensor_sub_updates++;

			if (Vector3f(sensor_data.x, sensor_data.y, sensor_data.z).isAllFinite()) {

				Vector3f acceleration_uncalibrated;

				float raw_data_array[] {sensor_data.x, sensor_data.y, sensor_data.z};

				for (int axis = 0; axis < 3; axis++) {
					// copy sensor sample to float array for filtering
					float data[1] {raw_data_array[axis]};

					// save last filtered sample
					acceleration_uncalibrated(axis) = FilterAcceleration(axis, data);
				}

				// Publish
				if (!_sensor_sub.updated()) {
					if (sensor_data.timestamp_sample >= _last_publish + _publish_interval_min_us) {

						// Apply calibration, rotate to body frame, and subtract estimated bias
						_acceleration = _calibration.Correct(acceleration_uncalibrated) - _bias;

						vehicle_acceleration_s v_acceleration;
						v_acceleration.timestamp_sample = sensor_data.timestamp_sample;
						_acceleration.copyTo(v_acceleration.xyz);
						v_acceleration.timestamp = hrt_absolute_time();
						_vehicle_acceleration_pub.publish(v_acceleration);

						// shift last publish time forward, but don't let it get further behind than the interval
						_last_publish = math::constrain(_last_publish + _publish_interval_min_us,
										sensor_data.timestamp_sample - _publish_interval_min_us,
										sensor_data.timestamp_sample);

						perf_end(_cycle_perf);
						return;
					}
				}
			}
		}
	}

	// force reselection on timeout
	if (time_now_us > _last_publish + 500_ms) {
		SensorSelectionUpdate(time_now_us, true);
	}

	perf_end(_cycle_perf);
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_acceleration] selected sensor: %" PRIu32
		     ", rate: %.1f Hz %s, estimated bias: [%.4f %.4f %.4f]\n",
		     _selected_sensor_device_id, (double)_filter_sample_rate_hz, _fifo_available ? "FIFO" : "",
		     (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();

	perf_print_counter(_cycle_perf);
	perf_print_counter(_filter_reset_perf);
	perf_print_counter(_selection_changed_perf);
#if !defined(CONSTRAINED_FLASH)
	perf_print_counter(_dynamic_notch_filter_esc_rpm_disable_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_init_perf);
	perf_print_counter(_dynamic_notch_filter_esc_rpm_update_perf);

	perf_print_counter(_dynamic_notch_filter_fft_disable_perf);
	perf_print_counter(_dynamic_notch_filter_fft_update_perf);
#endif // CONSTRAINED_FLASH
}

} // namespace sensors
