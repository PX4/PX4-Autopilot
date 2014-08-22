/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/**
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "mag.hpp"

const char *const UavcanMagnetometerBridge::NAME = "mag";

UavcanMagnetometerBridge::UavcanMagnetometerBridge(uavcan::INode& node) :
device::CDev("uavcan_mag", "/dev/uavcan/mag"),
_sub_mag(node)
{
	_scale.x_scale = 1.0F;
	_scale.y_scale = 1.0F;
	_scale.z_scale = 1.0F;
}

UavcanMagnetometerBridge::~UavcanMagnetometerBridge()
{
	if (_class_instance > 0) {
		(void)unregister_class_devname(MAG_DEVICE_PATH, _class_instance);
	}
}

int UavcanMagnetometerBridge::init()
{
	// Init the libuavcan subscription
	int res = _sub_mag.start(MagCbBinder(this, &UavcanMagnetometerBridge::mag_sub_cb));
	if (res < 0) {
		log("failed to start uavcan sub: %d", res);
		return res;
	}

	// Detect our device class
	_class_instance = register_class_devname(MAG_DEVICE_PATH);
	switch (_class_instance) {
	case CLASS_DEVICE_PRIMARY: {
		_orb_id = ORB_ID(sensor_mag0);
		break;
	}
	case CLASS_DEVICE_SECONDARY: {
		_orb_id = ORB_ID(sensor_mag1);
		break;
	}
	case CLASS_DEVICE_TERTIARY: {
		_orb_id = ORB_ID(sensor_mag2);
		break;
	}
	default: {
		log("invalid class instance: %d", _class_instance);
		(void)unregister_class_devname(MAG_DEVICE_PATH, _class_instance);
		return -1;
	}
	}

	log("inited with class instance %d", _class_instance);
	return 0;
}

int UavcanMagnetometerBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case MAGIOCSSCALE: {
		std::memcpy(&_scale, reinterpret_cast<const void*>(arg), sizeof(_scale));
		return 0;
	}
	case MAGIOCGSCALE: {
		std::memcpy(reinterpret_cast<void*>(arg), &_scale, sizeof(_scale));
		return 0;
	}
	case MAGIOCSELFTEST: {
		return 0;           // Nothing to do
	}
	case MAGIOCGEXTERNAL: {
		return 0;           // We don't want anyone to transform the coordinate frame, so we declare it onboard
	}
	case MAGIOCSSAMPLERATE: {
		return 0;           // Pretend that this stuff is supported to keep the sensor app happy
	}
	case MAGIOCCALIBRATE:
	case MAGIOCGSAMPLERATE:
	case MAGIOCSRANGE:
	case MAGIOCGRANGE:
	case MAGIOCSLOWPASS:
	case MAGIOCEXSTRAP:
	case MAGIOCGLOWPASS: {
		return -EINVAL;
	}
	default: {
		return CDev::ioctl(filp, cmd, arg);
	}
	}
}

void UavcanMagnetometerBridge::mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::Magnetometer> &msg)
{
	auto report = ::mag_report();

	report.range_ga = 1.3F;   // Arbitrary number, doesn't really mean anything

	report.timestamp = msg.getUtcTimestamp().toUSec();
	if (report.timestamp == 0) {
		report.timestamp = msg.getMonotonicTimestamp().toUSec();
	}

	report.x = (msg.magnetic_field[0] - _scale.x_offset) * _scale.x_scale;
	report.y = (msg.magnetic_field[1] - _scale.y_offset) * _scale.y_scale;
	report.z = (msg.magnetic_field[2] - _scale.z_offset) * _scale.z_scale;

	if (_orb_advert >= 0) {
		orb_publish(_orb_id, _orb_advert, &report);
	} else {
		_orb_advert = orb_advertise(_orb_id, &report);
		if (_orb_advert < 0) {
			log("ADVERT FAIL");
		} else {
			log("advertised");
		}
	}
}
