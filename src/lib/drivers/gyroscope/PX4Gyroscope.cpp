/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	CDev(nullptr),
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_gyro), priority},
	_sensor_fifo_pub{ORB_ID(sensor_gyro_fifo), priority},
	_sensor_integrated_pub{ORB_ID(sensor_gyro_integrated), priority},
	_sensor_status_pub{ORB_ID(sensor_gyro_status), priority},
	_device_id{device_id},
	_rotation{rotation}
{
	_class_device_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	// set software low pass filter for controllers
	updateParams();
	ConfigureFilter(_param_imu_gyro_cutoff.get());
	ConfigureNotchFilter(_param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
}

PX4Gyroscope::~PX4Gyroscope()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int PX4Gyroscope::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case GYROIOCSSCALE: {
			// Copy offsets and scale factors in
			gyro_calibration_s cal{};
			memcpy(&cal, (gyro_calibration_s *) arg, sizeof(cal));

			_calibration_offset = Vector3f{cal.x_offset, cal.y_offset, cal.z_offset};
		}

		return PX4_OK;

	case DEVIOCGDEVICEID:
		return _device_id;

	default:
		return -ENOTTY;
	}
}

void PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Gyroscope::set_sample_rate(uint16_t rate)
{
	_sample_rate = rate;

	ConfigureFilter(_filter.get_cutoff_freq());
	ConfigureNotchFilter(_notch_filter.getNotchFreq(), _notch_filter.getBandwidth());
}

void PX4Gyroscope::set_update_rate(uint16_t rate)
{
	const uint32_t update_interval = 1000000 / rate;
	_integrator_reset_samples = 4000 / update_interval;
}

void PX4Gyroscope::update(hrt_abstime timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const Vector3f raw{x, y, z};

	// Clipping (check unscaled raw values)
	const float clip_limit = (_range / _scale) * 0.95f;

	for (int i = 0; i < 3; i++) {
		if (fabsf(raw(i)) > clip_limit) {
			_clipping[i]++;
			_integrator_clipping++;
		}
	}

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{((raw * _scale) - _calibration_offset)};

	// Filtered values: apply notch and then low-pass
	Vector3f val_filtered{_notch_filter.apply(val_calibrated)};
	val_filtered = _filter.apply(val_filtered);


	// publish control data (filtered) immediately
	{
		sensor_gyro_s report{};

		report.timestamp_sample = timestamp_sample;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);
		report.timestamp = hrt_absolute_time();

		_sensor_pub.publish(report);
	}

	// Integrated values
	Vector3f delta_angle;
	uint32_t integral_dt = 0;

	if (_integrator_samples == 0) {
		_integrator_timestamp_sample = timestamp_sample;
	}

	_integrator_samples++;

	if (_integrator.put(timestamp_sample, val_calibrated, delta_angle, integral_dt)) {

		// fill sensor_gyro_integrated and publish
		sensor_gyro_integrated_s report{};

		report.timestamp_sample = _integrator_timestamp_sample;
		report.error_count = _error_count;
		report.device_id = _device_id;
		report.temperature = _temperature;
		delta_angle.copyTo(report.delta_angle);
		report.dt = integral_dt;
		report.samples = _integrator_samples;
		report.clip_count = _integrator_clipping;
		report.timestamp = hrt_absolute_time();

		_sensor_integrated_pub.publish(report);


		// reset integrator
		ResetIntegrator();

		// update vibration metrics
		UpdateVibrationMetrics(delta_angle);
	}

	PublishStatus();
}

void PX4Gyroscope::updateFIFO(const FIFOSample &sample)
{
	// filtered data (control)
	float x_filtered = _filterArrayX.apply(sample.x, sample.samples);
	float y_filtered = _filterArrayY.apply(sample.y, sample.samples);
	float z_filtered = _filterArrayZ.apply(sample.z, sample.samples);

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x_filtered, y_filtered, z_filtered);

	const Vector3f raw{x_filtered, y_filtered, z_filtered};

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{(raw * _scale) - _calibration_offset};


	// publish control data (filtered) immediately
	{
		bool publish_control = true;

		if (_param_imu_gyro_rate_max.get() > 0) {
			const uint64_t interval = 1e6f / _param_imu_gyro_rate_max.get();

			if (hrt_elapsed_time(&_control_last_publish) < interval) {
				publish_control = false;
			}
		}

		if (publish_control) {
			sensor_gyro_s report{};

			report.timestamp_sample = sample.timestamp_sample + ((sample.samples - 1) * sample.dt); // timestamp of last sample
			report.device_id = _device_id;
			report.temperature = _temperature;
			report.x = val_calibrated(0);
			report.y = val_calibrated(1);
			report.z = val_calibrated(2);
			report.timestamp = hrt_absolute_time();

			_sensor_pub.publish(report);

			_control_last_publish = report.timestamp_sample;
		}
	}


	// clipping
	const int16_t clip_limit = (_range / _scale) * 0.95f;

	// x clipping
	for (int n = 0; n < sample.samples; n++) {
		if (abs(sample.x[n]) > clip_limit) {
			_clipping[0]++;
			_integrator_clipping++;
		}
	}

	// y clipping
	for (int n = 0; n < sample.samples; n++) {
		if (abs(sample.y[n]) > clip_limit) {
			_clipping[1]++;
			_integrator_clipping++;
		}
	}

	// z clipping
	for (int n = 0; n < sample.samples; n++) {
		if (abs(sample.z[n]) > clip_limit) {
			_clipping[2]++;
			_integrator_clipping++;
		}
	}


	// integrated data (INS)
	{
		// reset integrator if previous sample was too long ago
		if ((sample.timestamp_sample > _timestamp_sample_prev)
		    && ((sample.timestamp_sample - _timestamp_sample_prev) > (sample.samples * sample.dt * 2))) {

			ResetIntegrator();
		}

		if (_integrator_samples == 0) {
			_integrator_timestamp_sample = sample.timestamp_sample;
		}

		// integrate
		_integrator_samples += 1;
		_integrator_fifo_samples += sample.samples;

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[0] += sample.x[n];
		}

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[1] += sample.y[n];
		}

		for (int n = 0; n < sample.samples; n++) {
			_integrator_accum[2] += sample.z[n];
		}

		if (_integrator_fifo_samples > 0 && (_integrator_samples >= _integrator_reset_samples)) {

			const uint32_t integrator_dt_us = _integrator_fifo_samples * sample.dt; // time span in microseconds

			// average integrated values to apply calibration
			float x_int_avg = _integrator_accum[0] / _integrator_fifo_samples;
			float y_int_avg = _integrator_accum[1] / _integrator_fifo_samples;
			float z_int_avg = _integrator_accum[2] / _integrator_fifo_samples;

			// Apply rotation (before scaling)
			rotate_3f(_rotation, x_int_avg, y_int_avg, z_int_avg);

			const Vector3f raw_int{x_int_avg, y_int_avg, z_int_avg};

			// Apply range scale and the calibrating offset/scale
			Vector3f delta_angle{(raw_int * _scale) - _calibration_offset};
			delta_angle *= (_integrator_fifo_samples * sample.dt * 1e-6f);	// restore

			// fill sensor_gyro_integrated and publish
			sensor_gyro_integrated_s report{};

			report.timestamp_sample = _integrator_timestamp_sample;
			report.error_count = _error_count;
			report.device_id = _device_id;
			report.temperature = _temperature;
			delta_angle.copyTo(report.delta_angle);
			report.dt = integrator_dt_us;
			report.samples = _integrator_fifo_samples;
			report.clip_count = _integrator_clipping;

			report.timestamp = hrt_absolute_time();
			_sensor_integrated_pub.publish(report);

			// update vibration metrics
			UpdateVibrationMetrics(delta_angle);

			// reset integrator
			ResetIntegrator();
		}

		_timestamp_sample_prev = sample.timestamp_sample;
	}

	// publish sensor fifo
	sensor_gyro_fifo_s fifo{};

	fifo.device_id = _device_id;
	fifo.timestamp_sample = sample.timestamp_sample;
	fifo.dt = sample.dt;
	fifo.scale = _scale;
	fifo.samples = sample.samples;

	memcpy(fifo.x, sample.x, sizeof(sample.x[0]) * sample.samples);
	memcpy(fifo.y, sample.y, sizeof(sample.y[0]) * sample.samples);
	memcpy(fifo.z, sample.z, sizeof(sample.z[0]) * sample.samples);

	fifo.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(fifo);


	PublishStatus();
}

void PX4Gyroscope::PublishStatus()
{
	// publish sensor status
	if (hrt_elapsed_time(&_status_last_publish) >= 100_ms) {
		sensor_gyro_status_s status{};

		status.device_id = _device_id;
		status.error_count = _error_count;
		status.full_scale_range = _range;
		status.rotation = _rotation;
		status.measure_rate = _update_rate;
		status.sample_rate = _sample_rate;
		status.temperature = _temperature;
		status.vibration_metric = _vibration_metric;
		status.coning_vibration = _coning_vibration;
		status.clipping[0] = _clipping[0];
		status.clipping[1] = _clipping[1];
		status.clipping[2] = _clipping[2];
		status.timestamp = hrt_absolute_time();
		_sensor_status_pub.publish(status);

		_status_last_publish = status.timestamp;
	}
}

void PX4Gyroscope::ResetIntegrator()
{
	_integrator_samples = 0;
	_integrator_fifo_samples = 0;
	_integrator_accum[0] = 0;
	_integrator_accum[1] = 0;
	_integrator_accum[2] = 0;
	_integrator_clipping = 0;

	_integrator_timestamp_sample = 0;
	_timestamp_sample_prev = 0;
}

void PX4Gyroscope::ConfigureFilter(float cutoff_freq)
{
	_filter.set_cutoff_frequency(_sample_rate, cutoff_freq);

	_filterArrayX.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayY.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayZ.set_cutoff_frequency(_sample_rate, cutoff_freq);
}

void PX4Gyroscope::ConfigureNotchFilter(float notch_freq, float bandwidth)
{
	_notch_filter.setParameters(_sample_rate, notch_freq, bandwidth);
}

void PX4Gyroscope::UpdateVibrationMetrics(const Vector3f &delta_angle)
{
	// Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
	const Vector3f delta_angle_diff = delta_angle - _delta_angle_prev;
	_vibration_metric = 0.99f * _vibration_metric + 0.01f * delta_angle_diff.norm();

	// Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
	const Vector3f coning_metric = delta_angle % _delta_angle_prev;
	_coning_vibration = 0.99f * _coning_vibration + 0.01f * coning_metric.norm();

	_delta_angle_prev = delta_angle;
}

void PX4Gyroscope::print_status()
{
	PX4_INFO(GYRO_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());
	PX4_INFO("notch filter freq: %.3f Hz\tbandwidth: %.3f Hz", (double)_notch_filter.getNotchFreq(),
		 (double)_notch_filter.getBandwidth());

	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));
}
