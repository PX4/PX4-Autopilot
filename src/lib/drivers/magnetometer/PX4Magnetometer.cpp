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


#include "PX4Magnetometer.hpp"

PX4Magnetometer::PX4Magnetometer(const char *path, device::Device  *interface, uint8_t dev_type, enum Rotation rotation,
				 float scale) :
	CDev(path),
	_interface(interface)
{
	_device_id.devid = _interface->get_device_id();
	// _device_id.devid_s.bus_type = (device::Device::DeviceBusType)_interface->get_device_bus_type();
	// _device_id.devid_s.bus = _interface->get_device_bus();
	// _device_id.devid_s.address = _interface->get_device_address();
	_device_id.devid_s.devtype = dev_type;

	CDev::init();

	_class_device_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	_rotation = rotation;
	_scale = scale;

	_cal.x_offset = 0;
	_cal.x_scale  = 1.0f;
	_cal.y_offset = 0;
	_cal.y_scale  = 1.0f;
	_cal.z_offset = 0;
	_cal.z_scale  = 1.0f;
}

PX4Magnetometer::~PX4Magnetometer()
{
	if (_topic != nullptr) {
		orb_unadvertise(_topic);
	}
}

int PX4Magnetometer::init()
{
	mag_report report{};
	report.device_id = _device_id.devid;

	if (_topic == nullptr) {
		_topic = orb_advertise_multi(ORB_ID(sensor_mag), &report, &_orb_class_instance, ORB_PRIO_HIGH - 1);

		if (_topic == nullptr) {
			PX4_ERR("Advertise failed.");
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

int PX4Magnetometer::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case MAGIOCSSCALE:
		// Copy scale in.
		memcpy(&_cal, (struct mag_calibration_s *) arg, sizeof(_cal));
		return OK;

	case MAGIOCGSCALE:
		// Copy scale out.
		memcpy((struct mag_calibration_s *) arg, &_cal, sizeof(_cal));
		return OK;

	case DEVIOCGDEVICEID:
		return _device_id.devid;

	default:
		// Give it to the superclass.
		return CDev::ioctl(filp, cmd, arg);
	}
}

void PX4Magnetometer::configure_filter(float sample_freq, float cutoff_freq)
{
	_filter_x.set_cutoff_frequency(sample_freq, cutoff_freq);
	_filter_y.set_cutoff_frequency(sample_freq, cutoff_freq);
	_filter_z.set_cutoff_frequency(sample_freq, cutoff_freq);
}

// @TODO: use fixed point math to reclaim CPU usage
int PX4Magnetometer::publish(float x, float y, float z, float temperature)
{
	sensor_mag_s report{};

	report.device_id   = _device_id.devid;
	report.error_count = 0;
	report.scaling 	   = _scale;
	report.timestamp   = hrt_absolute_time();
	report.temperature = temperature;
	report.is_external = false;


	// Raw values (ADC units 0 - 65535)
	report.x_raw = x;
	report.y_raw = y;
	report.z_raw = z;

	// Apply the rotation.
	rotate_3f(_rotation, x, y, z);

	// Apply FS range scale and the calibrating offset/scale
	x = ((x * _scale) - _cal.x_offset) * _cal.x_scale;
	y = ((y * _scale) - _cal.y_offset) * _cal.y_scale;
	z = ((z * _scale) - _cal.z_offset) * _cal.z_scale;

	// Filtered values
	report.x = _filter_x.apply(x);
	report.y = _filter_y.apply(y);
	report.z = _filter_z.apply(z);

	poll_notify(POLLIN);
	orb_publish(ORB_ID(sensor_mag), _topic, &report);

	return PX4_OK;
}