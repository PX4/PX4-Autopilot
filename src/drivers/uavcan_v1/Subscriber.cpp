/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file Subscriber.cpp
 *
 * Implements basic functionality of UAVCAN v1 subscriber class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#include "Subscriber.hpp"

/** ----- GPS Position Subscription ----- */

void UavcanGpsSubscription::subscribe()
{
	// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
	canardRxSubscribe(&_canard_instance,
			  CanardTransferKindMessage,
			  _port_id,
			  reg_drone_physics_kinematics_geodetic_Point_0_1_EXTENT_BYTES_,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
			  &_canard_sub);
}

void UavcanGpsSubscription::callback(const CanardTransfer &receive)
{
	// Test with Yakut:
	// export YAKUT_TRANSPORT="pyuavcan.transport.can.CANTransport(pyuavcan.transport.can.media.slcan.SLCANMedia('/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_23002B000E514E413431302000000000-if00', 8, 115200), 42)"
	// yakut pub 1500.reg.drone.physics.kinematics.geodetic.Point.0.1 '{latitude: 1.234, longitude: 2.34, altitude: {meter: 0.5}}'
	PX4_INFO("GpsCallback");

	reg_drone_physics_kinematics_geodetic_Point_0_1 geo {};
	size_t geo_size_in_bits = receive.payload_size;
	reg_drone_physics_kinematics_geodetic_Point_0_1_deserialize_(&geo, (const uint8_t *)receive.payload, &geo_size_in_bits);

	double lat = geo.latitude;
	double lon = geo.longitude;
	double alt = geo.altitude.meter;
	PX4_INFO("Latitude: %f, Longitude: %f, Altitude: %f", lat, lon, alt);
	/// do something with the data
}

/** ----- Battery Status Subscription ----- */

void UavcanBmsSubscription::subscribe()
{
	// Subscribe to messages reg.drone.service.battery.Status.0.1
	canardRxSubscribe(&_canard_instance,
			  CanardTransferKindMessage,
			  _port_id,
			  reg_drone_service_battery_Status_0_1_EXTENT_BYTES_,
			  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
			  &_canard_sub);
}

void UavcanBmsSubscription::callback(const CanardTransfer &receive)
{
	PX4_INFO("BmsCallback");

	reg_drone_service_battery_Status_0_1 bat {};
	size_t bat_size_in_bits = receive.payload_size;
	reg_drone_service_battery_Status_0_1_deserialize_(&bat, (const uint8_t *)receive.payload, &bat_size_in_bits);

	uavcan_si_unit_voltage_Scalar_1_0 V_Min = bat.cell_voltage_min_max[0];
	uavcan_si_unit_voltage_Scalar_1_0 V_Max = bat.cell_voltage_min_max[1];
	double vmin = static_cast<double>(V_Min.volt);
	double vmax = static_cast<double>(V_Max.volt);
	PX4_INFO("Min voltage: %f, Max Voltage: %f", vmin, vmax);
	/// do something with the data
}
