/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/failsafe_flags.h>

static constexpr int kHomePositionGPSRequiredFixType = 2;
static constexpr float kHomePositionGPSRequiredEPH = 5.f;
static constexpr float kHomePositionGPSRequiredEPV = 10.f;
static constexpr float kHomePositionGPSRequiredEVH = 1.f;
static constexpr float kMinHomePositionChangeEPH = 1.f;
static constexpr float kMinHomePositionChangeEPV = 1.5f;

class HomePosition
{
public:
	HomePosition(const failsafe_flags_s &failsafe_flags);
	~HomePosition() = default;

	bool setHomePosition(bool force = false);
	void setInAirHomePosition();
	bool setManually(double lat, double lon, float alt, float yaw);

	void update(bool set_automatically, bool check_if_changed);

	bool valid() const { return _valid; }

private:
	bool hasMovedFromCurrentHomeLocation();
	void setHomePosValid();
	void updateHomePositionYaw(float yaw);

	static void fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos);
	static void fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading);
	static void fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos);
	static void fillGlobalHomePos(home_position_s &home, double lat, double lon, double alt);

	uORB::Subscription					_vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionData<vehicle_global_position_s>	_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_local_position_s>	_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::PublicationData<home_position_s>			_home_position_pub{ORB_ID(home_position)};

	uint8_t							_heading_reset_counter{0};
	bool							_valid{false};
	const failsafe_flags_s					&_failsafe_flags;
	bool							_gps_position_for_home_valid{false};
	double							_gps_lat{0};
	double							_gps_lon{0};
	double							_gps_alt{0};
	float							_gps_eph{0.f};
	float							_gps_epv{0.f};
};
