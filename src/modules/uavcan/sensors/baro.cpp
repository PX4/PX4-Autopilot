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

#include "baro.hpp"
#include <cmath>

static const orb_id_t BARO_TOPICS[2] = {
	ORB_ID(sensor_baro0),
	ORB_ID(sensor_baro1)
};

const char *const UavcanBarometerBridge::NAME = "baro";

UavcanBarometerBridge::UavcanBarometerBridge(uavcan::INode& node) :
UavcanCDevSensorBridgeBase("uavcan_baro", "/dev/uavcan/baro", BARO_DEVICE_PATH, BARO_TOPICS),
_sub_air_data(node)
{
}

int UavcanBarometerBridge::init()
{
	int res = device::CDev::init();
	if (res < 0) {
		return res;
	}

	res = _sub_air_data.start(AirDataCbBinder(this, &UavcanBarometerBridge::air_data_sub_cb));
	if (res < 0) {
		log("failed to start uavcan sub: %d", res);
		return res;
	}
	return 0;
}

int UavcanBarometerBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case BAROIOCSMSLPRESSURE: {
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		} else {
			log("new msl pressure %u", _msl_pressure);
			_msl_pressure = arg;
			return OK;
		}
	}
	case BAROIOCGMSLPRESSURE: {
		return _msl_pressure;
	}
	default: {
		return CDev::ioctl(filp, cmd, arg);
	}
	}
}

void UavcanBarometerBridge::air_data_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticAirData> &msg)
{
	auto report = ::baro_report();

	report.timestamp   = msg.getMonotonicTimestamp().toUSec();
	report.temperature = msg.static_temperature;
	report.pressure    = msg.static_pressure / 100.0F;  // Convert to millibar

	/*
	 * Altitude computation
	 * Refer to the MS5611 driver for details
	 */
	const double T1 = 15.0 + 273.15; // temperature at base height in Kelvin
	const double a  = -6.5 / 1000;   // temperature gradient in degrees per metre
	const double g  = 9.80665;       // gravity constant in m/s/s
	const double R  = 287.05;        // ideal gas constant in J/kg/K

	const double p1 = _msl_pressure / 1000.0;      // current pressure at MSL in kPa
	const double p = double(msg.static_pressure) / 1000.0; // measured pressure in kPa

	report.altitude = (((std::pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

	publish(msg.getSrcNodeID().get(), &report);
}
