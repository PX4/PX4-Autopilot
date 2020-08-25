/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "PX4Anemometer.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Anemometer::PX4Anemometer(const uint32_t device_id, const ORB_PRIO priority, const uint8_t device_orientation) :
	_anemometer_pub{ORB_ID(windspeed), priority}
{
	_anemometer_pub.advertise();

	set_device_id(device_id);
	set_orientation(device_orientation);
}

PX4Anemometer::~PX4Anemometer()
{
	_anemometer_pub.unadvertise();
}

void PX4Anemometer::set_device_type(uint8_t device_type)
{
	// TODO: range finders should have device ids

	// // current DeviceStructure
	// union device::Device::DeviceId device_id;
	// device_id.devid = _anemometer_pub.get().device_id;

	// // update to new device type
	// device_id.devid_s.devtype = devtype;

	// // copy back to report
	// _anemometer_pub.get().device_id = device_id.devid;
}

void PX4Anemometer::set_orientation(const uint8_t device_orientation)
{
	_anemometer_pub.get().orientation = device_orientation;
}

void PX4Anemometer::update(const hrt_abstime &timestamp_sample, const float measurement[3], const float quality[3], const int orientation, const float air_temperature_celsius)
{
	windspeed_s &report = _anemometer_pub.get();

	report.timestamp = timestamp_sample;
	report.measurement_windspeed_x_m_s = measurement[0];
	report.measurement_windspeed_y_m_s = measurement[1];
	report.measurement_windspeed_x_m_s = measurement[2];
	report.confidence_x = quality[0];
	report.confidence_y = quality[1];
	report.confidence_z = quality[2];
	report.orientation = orientation;
	report.air_temperature_celsius =air_temperature_celsius;

	_anemometer_pub.update();
}
