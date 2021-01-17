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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

class PX4Gyroscope
{
public:
	PX4Gyroscope(uint32_t device_id, enum Rotation rotation = ROTATION_NONE) :
		_device_id{device_id},
		_rotation{rotation}
	{
		param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
	}

	~PX4Gyroscope()
	{
		_sensor_pub.unadvertise();
		_sensor_fifo_pub.unadvertise();
	}

	uint32_t get_device_id() const { return _device_id; }

	int32_t get_max_rate_hz() const { return _imu_gyro_rate_max; }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype)
	{
		union device::Device::DeviceId device_id;
		device_id.devid = _device_id;
		device_id.devid_s.devtype = devtype; // update to new device type
		_device_id = device_id.devid; // copy back
	}

	void set_error_count(uint32_t error_count) { _error_count = error_count; }
	void increase_error_count() { _error_count++; }
	void set_range(float range) { _range = range; }
	void set_scale(float scale) { _scale = scale; }
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(const hrt_abstime &timestamp_sample, float x, float y, float z)
	{
		// Apply rotation (before scaling)
		rotate_3f(_rotation, x, y, z);

		sensor_gyro_s report;

		report.timestamp_sample = timestamp_sample;
		report.device_id = _device_id;
		report.temperature = _temperature;
		report.error_count = _error_count;
		report.x = x * _scale;
		report.y = y * _scale;
		report.z = z * _scale;
		report.timestamp = hrt_absolute_time();

		_sensor_pub.publish(report);
	}

	void updateFIFO(sensor_gyro_fifo_s &sample)
	{
		// publish fifo
		sample.device_id = _device_id;
		sample.temperature = _temperature;
		sample.error_count = _error_count;
		sample.scale = _scale;

		for (int i = 0; i < sample.samples; i++) {
			rotate_3i(_rotation, sample.x[i], sample.y[i], sample.z[i]);
		}

		sample.timestamp = hrt_absolute_time();
		_sensor_fifo_pub.publish(sample);
	}
private:
	uORB::PublicationMulti<sensor_gyro_s> _sensor_pub{ORB_ID(sensor_gyro)};
	uORB::PublicationMulti<sensor_gyro_fifo_s>  _sensor_fifo_pub{ORB_ID(sensor_gyro_fifo)};

	uint32_t		_device_id{0};
	const enum Rotation	_rotation;

	int32_t			_imu_gyro_rate_max{0};

	float			_range{math::radians(2000.f)};
	float			_scale{1.f};
	float			_temperature{NAN};

	uint32_t		_error_count{0};

	int16_t			_last_sample[3] {};
};
