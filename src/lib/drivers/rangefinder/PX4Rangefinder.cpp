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

#include "PX4Rangefinder.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Rangefinder::PX4Rangefinder(uint32_t device_id, uint8_t priority, uint8_t rotation) :
	CDev(nullptr),
	_distance_sensor_pub{ORB_ID(distance_sensor), priority}
{
	_class_device_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	// TODO: range finders should have device ids
	//_distance_sensor_pub.get().device_id = device_id;

	_distance_sensor_pub.get().orientation = rotation;
}

PX4Rangefinder::~PX4Rangefinder()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_device_instance);
	}
}

void
PX4Rangefinder::set_device_type(uint8_t devtype)
{
	// TODO: range finders should have device ids

	// // current DeviceStructure
	// union device::Device::DeviceId device_id;
	// device_id.devid = _distance_sensor_pub.get().device_id;

	// // update to new device type
	// device_id.devid_s.devtype = devtype;

	// // copy back to report
	// _distance_sensor_pub.get().device_id = device_id.devid;
}

void
PX4Rangefinder::update(hrt_abstime timestamp, float distance, int8_t quality)
{
	distance_sensor_s &report = _distance_sensor_pub.get();

	report.timestamp = timestamp;
	report.current_distance = distance;
	report.signal_quality = quality;

	_distance_sensor_pub.update();
}

void
PX4Rangefinder::print_status()
{
	PX4_INFO(RANGE_FINDER_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);

	print_message(_distance_sensor_pub.get());
}
