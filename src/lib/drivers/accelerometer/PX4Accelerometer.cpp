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


#include "PX4Accelerometer.hpp"

#include <lib/drivers/device/Device.hpp>

using namespace time_literals;
using matrix::Vector3f;

PX4Accelerometer::PX4Accelerometer(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	CDev(nullptr),
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_accel), priority},
	_sensor_control_pub{ORB_ID(sensor_accel_control), priority},
	_sensor_fifo_pub{ORB_ID(sensor_accel_fifo), priority},
	_sensor_integrated_pub{ORB_ID(sensor_accel_integrated), priority},
	_sensor_status_pub{ORB_ID(sensor_accel_status), priority},
	_device_id{device_id},
	_rotation{rotation}
{
	_class_device_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	// set software low pass filter for controllers
	updateParams();
	ConfigureFilter(_param_imu_accel_cutoff.get());
}

PX4Accelerometer::~PX4Accelerometer()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int
PX4Accelerometer::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case ACCELIOCSSCALE: {
			// Copy offsets and scale factors in
			accel_calibration_s cal{};
			memcpy(&cal, (accel_calibration_s *) arg, sizeof(cal));

			_calibration_offset = Vector3f{cal.x_offset, cal.y_offset, cal.z_offset};
			_calibration_scale = Vector3f{cal.x_scale, cal.y_scale, cal.z_scale};
		}

		return PX4_OK;

	case DEVIOCGDEVICEID:
		return _device_id;

	default:
		return -ENOTTY;
	}
}

void
PX4Accelerometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_device_id = device_id.devid;
}

void
PX4Accelerometer::set_sample_rate(uint16_t rate)
{
	_sample_rate = rate;

	_filter.set_cutoff_frequency(_sample_rate, _filter.get_cutoff_freq());

	_filterArrayX.set_cutoff_frequency(_sample_rate, _filterArrayX.get_cutoff_freq());
	_filterArrayY.set_cutoff_frequency(_sample_rate, _filterArrayY.get_cutoff_freq());
	_filterArrayZ.set_cutoff_frequency(_sample_rate, _filterArrayZ.get_cutoff_freq());
}

void
PX4Accelerometer::set_update_rate(uint16_t rate)
{
	const uint32_t update_interval = 1000000 / rate;

	_integrator_reset_samples = 4000 / update_interval;
}

void
PX4Accelerometer::update(hrt_abstime timestamp, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{(((raw * _scale) - _calibration_offset).emult(_calibration_scale))};

	// Filtered values
	const Vector3f val_filtered{_filter.apply(val_calibrated)};


	// publish control data (filtered) immediately
	bool publish_control = true;
	sensor_accel_control_s control{};

	if (publish_control) {
		control.timestamp_sample = timestamp;
		control.device_id = _device_id;
		val_filtered.copyTo(control.xyz);
		control.timestamp = hrt_absolute_time();
		_sensor_control_pub.publish(control);

		_control_last_publish = control.timestamp_sample;
	}


	// Integrated values
	Vector3f integrated_value;
	uint32_t integral_dt = 0;

	if (_integrator.put(timestamp, val_calibrated, integrated_value, integral_dt)) {

		sensor_accel_s report{};
		report.timestamp = timestamp;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.scaling = _scale;
		report.error_count = _error_count;

		// Raw values (ADC units 0 - 65535)
		report.x_raw = x;
		report.y_raw = y;
		report.z_raw = z;

		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);

		report.integral_dt = integral_dt;
		report.x_integral = integrated_value(0);
		report.y_integral = integrated_value(1);
		report.z_integral = integrated_value(2);

		_sensor_pub.publish(report);
	}
}

void
PX4Accelerometer::updateFIFO(const FIFOSample &sample)
{
	// filtered data (control)
	float x_filtered = _filterArrayX.apply(sample.x, sample.samples);
	float y_filtered = _filterArrayY.apply(sample.y, sample.samples);
	float z_filtered = _filterArrayZ.apply(sample.z, sample.samples);

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x_filtered, y_filtered, z_filtered);

	const Vector3f raw{x_filtered, y_filtered, z_filtered};

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{(((raw * _scale) - _calibration_offset).emult(_calibration_scale))};


	// control
	{
		// publish control data (filtered) immediately
		bool publish_control = true;
		sensor_accel_control_s control{};

		if (publish_control) {
			control.timestamp_sample = sample.timestamp_sample + ((sample.samples - 1) * sample.dt); // timestamp of last sample
			control.device_id = _device_id;
			val_calibrated.copyTo(control.xyz);
			control.timestamp = hrt_absolute_time();
			_sensor_control_pub.publish(control);

			_control_last_publish = control.timestamp_sample;
		}
	}


	// status
	bool clipping = false;
	{
		sensor_accel_status_s &status = _sensor_status_pub.get();

		const int16_t clip_limit = (_range / _scale) * 0.9f;

		// x clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.x[n]) > clip_limit) {
				status.clipping[0]++;
				_error_count++;
				clipping = true;
			}
		}

		// y clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.y[n]) > clip_limit) {
				status.clipping[1]++;
				_error_count++;
				clipping = true;
			}
		}

		// z clipping
		for (int n = 0; n < sample.samples; n++) {
			if (abs(sample.z[n]) > clip_limit) {
				status.clipping[2]++;
				_error_count++;
				clipping = true;
			}
		}


		// // Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)
		// for (int n = 1; n < sample.samples; n++) {

		// 	// float delta_vel = (fifo.x[n] - fifo.x[n-1]) / fifo.dt;	// ADC units and time in microseconds
		// 	matrix::Vector3f delta_velocity;

		// 	// calculate a metric which indicates the amount of high frequency accelerometer vibration
		// 	const matrix::Vector3f temp = delta_velocity - _delta_velocity_prev;

		// 	_high_frequency_vibration = 0.99f * _high_frequency_vibration + 0.01f * temp.norm();

		// 	_delta_velocity_prev = delta_velocity;
		// }
		status.high_frequency_vibration = _high_frequency_vibration;

		status.device_id = _device_id;
		status.error_count = _error_count;
		status.full_scale_range = _range;
		status.rotation = _rotation;
		status.measure_rate = _update_rate;
		status.sample_rate = _sample_rate;
		status.temperature = _temperature;
		status.timestamp = hrt_absolute_time();
		_sensor_status_pub.publish(status);
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

		if (_integrator_samples >= _integrator_reset_samples) {

			const uint32_t integrator_dt_us = _integrator_fifo_samples * sample.dt; // time span in microseconds

			// average integrated values to apply calibration
			float x_int_avg = _integrator_accum[0] / _integrator_fifo_samples;
			float y_int_avg = _integrator_accum[1] / _integrator_fifo_samples;
			float z_int_avg = _integrator_accum[2] / _integrator_fifo_samples;

			// Apply rotation (before scaling)
			rotate_3f(_rotation, x_int_avg, y_int_avg, z_int_avg);

			const Vector3f raw_int{x_int_avg, y_int_avg, z_int_avg};

			// Apply range scale and the calibrating offset/scale
			Vector3f val_int_calibrated{(((raw_int * _scale) - _calibration_offset).emult(_calibration_scale))};
			val_int_calibrated *= (_integrator_fifo_samples * sample.dt * 1e-6f);	// restore

			// publish
			sensor_accel_integrated_s integrated{};
			integrated.device_id = _device_id;
			val_int_calibrated.copyTo(integrated.delta_velocity);
			integrated.timestamp_sample = _integrator_timestamp_sample;
			integrated.dt = integrator_dt_us;
			integrated.clipping = clipping;
			integrated.timestamp = hrt_absolute_time();
			_sensor_integrated_pub.publish(integrated);


			// legacy sensor_accel_s message
			sensor_accel_s report{};
			report.device_id = _device_id;
			report.temperature = _temperature;
			report.scaling = _scale;
			report.error_count = _error_count;

			// Raw values (ADC units 0 - 65535)
			report.x_raw = sample.x[0];
			report.y_raw = sample.y[0];
			report.z_raw = sample.z[0];

			report.x = val_calibrated(0);
			report.y = val_calibrated(1);
			report.z = val_calibrated(2);

			report.integral_dt = integrator_dt_us;
			report.x_integral = val_int_calibrated(0);
			report.y_integral = val_int_calibrated(1);
			report.z_integral = val_int_calibrated(2);

			report.timestamp = _integrator_timestamp_sample;
			_sensor_pub.publish(report);


			// reset integrator
			ResetIntegrator();
		}

		_timestamp_sample_prev = sample.timestamp_sample;
	}

	sensor_accel_fifo_s fifo{};

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
}

void
PX4Accelerometer::ResetIntegrator()
{
	_integrator_samples = 0;
	_integrator_fifo_samples = 0;
	_integrator_accum[0] = 0;
	_integrator_accum[1] = 0;
	_integrator_accum[2] = 0;

	_integrator_timestamp_sample = 0;
	_timestamp_sample_prev = 0;
}

void
PX4Accelerometer::ConfigureFilter(float cutoff_freq)
{
	_filter.set_cutoff_frequency(_sample_rate, cutoff_freq);

	_filterArrayX.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayY.set_cutoff_frequency(_sample_rate, cutoff_freq);
	_filterArrayZ.set_cutoff_frequency(_sample_rate, cutoff_freq);
}

void
PX4Accelerometer::print_status()
{
	PX4_INFO(ACCEL_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));

}
