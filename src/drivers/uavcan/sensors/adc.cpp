/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include "adc.hpp"

const char *const UavcanAdcBridge::NAME = "adc";

UavcanAdcBridge::UavcanAdcBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_airspeed", "/dev/uavcan/adc", "/dev/adc", ORB_ID(analog_voltage_current)),
	_sub_adc_data(node)
{ }

int UavcanAdcBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_adc_data.start(AdcCbBinder(this, &UavcanAdcBridge::adc_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanAdcBridge::adc_sub_cb(const
				 uavcan::ReceivedDataStructure<com::volansi::equipment::adc::Report> &msg)
{
	analog_voltage_current_s report{};

	static constexpr int numIndices = 2;
	static constexpr uint16_t mV = com::volansi::equipment::adc::Report::UNITS_MV;
	static constexpr uint16_t mA = com::volansi::equipment::adc::Report::UNITS_MA;

	for (int i = 0; i < numIndices; i++) {

		// TODO: do we want to publish raw ADC values? What about temperature?
		if (msg.unit_type[i] == mV) {
			report.voltage[i] = msg.values[i];

		} else if (msg.unit_type[i] == mA) {
			report.current[i] = msg.values[i];
		}
	}

	publish(msg.getSrcNodeID().get(), &report);
}
