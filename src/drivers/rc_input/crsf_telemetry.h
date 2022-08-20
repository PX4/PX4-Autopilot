/****************************************************************************
 *
 *	Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file crsf_telemetry.h
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <drivers/drv_hrt.h>

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace time_literals;

/**
 * High-level class that handles sending of CRSF telemetry data
 */
class CRSFTelemetry
{
public:
	/**
	 * @param uart_fd file descriptor for the UART to use. It is expected to be configured
	 * already.
	 */
	CRSFTelemetry(int uart_fd);

	~CRSFTelemetry() = default;

	/**
	 * Send telemetry data. Call this regularly (i.e. at 100Hz), it will automatically
	 * limit the sending rate.
	 * @return true if new data sent
	 */
	bool update(const hrt_abstime &now);

private:
	bool send_battery();
	bool send_gps();
	bool send_attitude();
	bool send_flight_mode();

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	hrt_abstime _last_update{0};

	static constexpr int num_data_types{4}; ///< number of different telemetry data types
	int _next_type{0};

	int _uart_fd;
};
