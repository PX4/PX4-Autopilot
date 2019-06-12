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


#include "PX4DifferentialPressure.hpp"

#include <lib/drivers/device/Device.hpp>

#include <math.h>
#include <string.h>

PX4DifferentialPressure::PX4DifferentialPressure(uint32_t device_id, uint8_t priority) :
	CDev(nullptr),
	_differential_pressure_pub{ORB_ID(differential_pressure), priority}
{
	_class_device_instance = register_class_devname(AIRSPEED_BASE_DEVICE_PATH);

	_differential_pressure_pub.get().device_id = device_id;
	_differential_pressure_pub.get().temperature = NAN;
}

PX4DifferentialPressure::~PX4DifferentialPressure()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(AIRSPEED_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int
PX4DifferentialPressure::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case AIRSPEEDIOCSSCALE: {
			// Copy offsets in
			airspeed_scale cal{};
			memcpy(&cal, (airspeed_scale *) arg, sizeof(cal));
			_calibration_offset = cal.offset_pa;
			_calibration_scale = cal.scale;
		}

		return PX4_OK;

	case DEVIOCGDEVICEID:
		return _differential_pressure_pub.get().device_id;

	default:
		return -ENOTTY;
	}
}

void
PX4DifferentialPressure::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _differential_pressure_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_differential_pressure_pub.get().device_id = device_id.devid;
}

void
PX4DifferentialPressure::set_sample_rate(unsigned rate)
{
	_sample_rate = rate;
	_filter.set_cutoff_frequency(_sample_rate, _filter.get_cutoff_freq());
}

void
PX4DifferentialPressure::update(hrt_abstime timestamp, float differential_pressure)
{
	differential_pressure_s &report = _differential_pressure_pub.get();

	const float val_calibrated = (differential_pressure - _calibration_offset) * _calibration_scale;

	report.timestamp = timestamp;
	report.differential_pressure_raw_pa = val_calibrated;
	report.differential_pressure_filtered_pa = _filter.apply(val_calibrated);

	poll_notify(POLLIN);

	_differential_pressure_pub.update();
}

void
PX4DifferentialPressure::print_status()
{
	PX4_INFO(AIRSPEED_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());

	PX4_INFO("calibration scale: %.5f", (double)_calibration_scale);
	PX4_INFO("calibration offset: %.5f", (double)_calibration_offset);

	print_message(_differential_pressure_pub.get());
}
