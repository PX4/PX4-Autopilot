/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

static inline int32_t sum(const int16_t samples[16], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, ORB_PRIO priority, enum Rotation rotation) :
	CDev(nullptr),
	ModuleParams(nullptr),
	_sensor_pub{ORB_ID(sensor_gyro), priority},
	_sensor_fifo_pub{ORB_ID(sensor_gyro_fifo), priority},
	_device_id{device_id},
	_rotation{rotation}
{
	// register class and advertise immediately to keep instance numbering in sync
	_class_device_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);
	_sensor_pub.advertise();

	updateParams();
}

PX4Gyroscope::~PX4Gyroscope()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_device_instance);
	}

	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
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

void PX4Gyroscope::update(hrt_abstime timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const Vector3f val_calibrated{((raw * _scale) - _calibration_offset)};

	// publish
	sensor_gyro_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = val_calibrated(0);
	report.y = val_calibrated(1);
	report.z = val_calibrated(2);
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

void PX4Gyroscope::updateFIFO(const FIFOSample &sample)
{
	const uint8_t N = sample.samples;
	const float dt = sample.dt;

	{
		// trapezoidal integration (equally spaced, scaled by dt later)
		Vector3f integral{
			(0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)),
			(0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)),
			(0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)),
		};

		_last_sample[0] = sample.x[N - 1];
		_last_sample[1] = sample.y[N - 1];
		_last_sample[2] = sample.z[N - 1];

		// Apply rotation (before scaling)
		rotate_3f(_rotation, integral(0), integral(1), integral(2));

		// average
		const float x = integral(0) / (float)N;
		const float y = integral(1) / (float)N;
		const float z = integral(2) / (float)N;

		// Apply range scale and the calibration offset
		const Vector3f val_calibrated{(Vector3f{x, y, z} * _scale) - _calibration_offset};

		// publish
		sensor_gyro_s report;

		report.timestamp_sample = sample.timestamp_sample;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.error_count = _error_count;
		report.x = val_calibrated(0);
		report.y = val_calibrated(1);
		report.z = val_calibrated(2);
		report.timestamp = hrt_absolute_time();

		_sensor_pub.publish(report);
	}


	// publish fifo
	sensor_gyro_fifo_s fifo{};

	fifo.device_id = _device_id;
	fifo.timestamp_sample = sample.timestamp_sample;
	fifo.dt = dt;
	fifo.scale = _scale;
	fifo.samples = N;

	memcpy(fifo.x, sample.x, sizeof(sample.x[0]) * N);
	memcpy(fifo.y, sample.y, sizeof(sample.y[0]) * N);
	memcpy(fifo.z, sample.z, sizeof(sample.z[0]) * N);

	fifo.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(fifo);
}

void PX4Gyroscope::print_status()
{
#if !defined(CONSTRAINED_FLASH)
	PX4_INFO(GYRO_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);

	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));
#endif // !CONSTRAINED_FLASH
}
