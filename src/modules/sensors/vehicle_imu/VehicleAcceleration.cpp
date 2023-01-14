/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

using namespace matrix;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr)
{
	_vehicle_acceleration_pub.advertise();
}

void VehicleAcceleration::ParametersUpdate()
{
	updateParams();

	_filter_config_check = true;
}

void VehicleAcceleration::updateAccel(IMU &imu, const matrix::Vector3f &accel_raw, const float sample_rate_hz)
{
	bool filter_reset = false;

	if ((_selected_sensor_device_id != imu.accel.calibration.device_id()) || _filter_config_check) {

		bool sample_rate_changed = false;

		if (imu.accel.mean_publish_interval_us.valid() && PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 0)) {
			// check if sample rate error is greater than 5%
			if (!PX4_ISFINITE(_filter_sample_rate) || (fabsf(sample_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.05f) {
				PX4_DEBUG("sample rate changed: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)sample_rate_hz);
				_filter_sample_rate = sample_rate_hz;
				sample_rate_changed = true;
			}
		}

		// update software low pass filters
		if (sample_rate_changed || (fabsf(_lp_filter.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.1f)) {
			filter_reset = true;
		}

		_selected_sensor_device_id = imu.accel.calibration.device_id();
		_filter_config_check = false;
	}

	if (filter_reset) {
		_lp_filter.set_cutoff_frequency(_filter_sample_rate, _param_imu_accel_cutoff.get());
	}

	// Apply calibration and filter
	//  - calibration offsets, scale factors, and thermal scale (if available)
	//  - estimated in run bias (if available)
	//  - biquad low-pass filter
	const Vector3f accel_corrected = imu.accel.calibration.Correct(accel_raw) - imu.accel.estimated_bias;

	const Vector3f accel_filtered = filter_reset ? _lp_filter.reset(accel_corrected) : _lp_filter.apply(accel_corrected);

	// Publish vehicle_acceleration
	vehicle_acceleration_s v_acceleration;
	v_acceleration.timestamp_sample = imu.accel.timestamp_sample_last;
	accel_filtered.copyTo(v_acceleration.xyz);
	v_acceleration.timestamp = hrt_absolute_time();
	_vehicle_acceleration_pub.publish(v_acceleration);
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_acceleration] selected sensor: %" PRIu32 ", rate: %.1f Hz\n",
		     _selected_sensor_device_id, (double)_filter_sample_rate);
}

} // namespace sensors
