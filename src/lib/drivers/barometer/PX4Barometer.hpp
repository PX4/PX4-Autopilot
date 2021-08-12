/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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
#include <lib/drivers/device/Device.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>

class PX4Barometer
{
public:
	PX4Barometer(uint32_t device_id) : _device_id{device_id} {}
	~PX4Barometer() { _sensor_pub.unadvertise(); }

	bool external() { return _external; }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype)
	{
		// current DeviceStructure
		union device::Device::DeviceId device_id;
		device_id.devid = _device_id;

		// update to new device type
		device_id.devid_s.devtype = devtype;

		// copy back
		_device_id = device_id.devid;
	}

	void set_error_count(uint32_t error_count) { _error_count = error_count; }
	void increase_error_count() { _error_count++; }
	void set_temperature(float temperature) { _temperature = temperature; }
	void set_external(bool external) { _external = external; }

	void update(const hrt_abstime &timestamp_sample, float pressure_pa)
	{
		sensor_baro_s report;
		report.timestamp_sample = timestamp_sample;
		report.device_id = _device_id;
		report.pressure = pressure_pa;
		report.temperature = _temperature;
		report.error_count = _error_count;

		report.is_external = _external;

		report.timestamp = hrt_absolute_time();
		_sensor_pub.publish(report);
	}

	int get_instance() { return _sensor_pub.get_instance(); };

private:
	uORB::PublicationMulti<sensor_baro_s> _sensor_pub{ORB_ID(sensor_baro)};

	uint32_t		_device_id{0};

	float			_temperature{NAN};
	uint32_t		_error_count{0};

	bool _external{false};
};
