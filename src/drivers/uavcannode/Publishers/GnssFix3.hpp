/****************************************************************************
 *
 *   Copyright (c) 2021-2024 PX4 Development Team. All rights reserved.
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

#include <cmath>

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/Fix3.hpp>

#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_gps.h>

namespace uavcannode
{

class GnssFix3 :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::Fix3>
{
public:
	GnssFix3(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::Fix3::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_gps)),
		uavcan::Publisher<uavcan::equipment::gnss::Fix3>(node)
	{
		this->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::Fix3::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::equipment::gnss::Fix3;

		// sensor_gps -> uavcan::equipment::gnss::Fix3
		sensor_gps_s gps;

		if (uORB::SubscriptionCallbackWorkItem::update(&gps)) {
			uavcan::equipment::gnss::Fix3 fix3{};

			// Time
			fix3.gnss_time_usec = gps.time_utc_usec;
			fix3.time_standard = Fix3::TIME_STANDARD_UTC;
			fix3.num_leap_seconds = 0; // Unknown

			// Position - scale to 1e8 and mm
			fix3.latitude_deg_1e8 = (int64_t)(gps.latitude_deg * 1e8);
			fix3.longitude_deg_1e8 = (int64_t)(gps.longitude_deg * 1e8);
			fix3.altitude_msl = (int32_t)(gps.altitude_msl_m * 1e3);        // m to mm
			fix3.altitude_ellipsoid = (int32_t)(gps.altitude_ellipsoid_m * 1e3);  // m to mm

			// Velocity
			fix3.vel_north = gps.vel_n_m_s;
			fix3.vel_east = gps.vel_e_m_s;
			fix3.vel_down = gps.vel_d_m_s;
			fix3.ground_speed = gps.vel_m_s;
			fix3.eph = gps.eph;
			fix3.epv = gps.epv;

			// Speed accuracy - convert variance to std dev
			fix3.speed_accuracy = (gps.s_variance_m_s > 0) ? sqrtf(gps.s_variance_m_s) : -1.0f;

			// Heading - convert rad to deg
			if (!std::isnan(gps.heading)) {
				fix3.heading = math::degrees(gps.heading);
				fix3.heading_accuracy = std::isnan(gps.heading_accuracy) ? NAN : math::degrees(gps.heading_accuracy);

			} else {
				fix3.heading = NAN;
				fix3.heading_accuracy = NAN;
			}

			// Course over ground - convert rad to deg
			fix3.cog = math::degrees(gps.cog_rad);

			// DOP
			fix3.hdop = gps.hdop;
			fix3.vdop = gps.vdop;
			fix3.pdop = (gps.hdop > gps.vdop) ? gps.hdop : gps.vdop; // Approximate PDOP

			// Fix type
			fix3.fix_type = gps.fix_type;

			// Fix quality - derive from fix type if not available
			switch (gps.fix_type) {
			case sensor_gps_s::FIX_TYPE_NONE:
				fix3.fix_quality = 0;
				break;

			case sensor_gps_s::FIX_TYPE_2D:
				fix3.fix_quality = 30;
				break;

			case sensor_gps_s::FIX_TYPE_3D:
				fix3.fix_quality = 60;
				break;

			case sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL:
				fix3.fix_quality = 75;
				break;

			case sensor_gps_s::FIX_TYPE_RTK_FLOAT:
				fix3.fix_quality = 85;
				break;

			case sensor_gps_s::FIX_TYPE_RTK_FIXED:
				fix3.fix_quality = 100;
				break;

			default:
				fix3.fix_quality = 0;
				break;
			}

			// Satellite counts
			fix3.sats_used = gps.satellites_used;
			fix3.sats_visible = gps.satellites_used; // We don't have visible count separately

			// Differential correction age - not available in sensor_gps
			fix3.diff_age = 0xFFFF; // Unavailable

			// Signal quality metrics
			fix3.noise = gps.noise_per_ms;
			fix3.agc = gps.automatic_gain_control;
			fix3.jamming_state = gps.jamming_state;
			fix3.jamming_indicator = gps.jamming_indicator;
			fix3.spoofing_state = gps.spoofing_state;
			fix3.auth_state = gps.authentication_state;

			// System errors
			fix3.system_errors = gps.system_error;

			// Flags
			fix3.flags = 0;

			if (gps.vel_ned_valid) {
				fix3.flags |= Fix3::FLAGS_VEL_NED_VALID;
			}

			if (!std::isnan(gps.heading)) {
				fix3.flags |= Fix3::FLAGS_HEADING_VALID;
			}

			if (gps.time_utc_usec != 0) {
				fix3.flags |= Fix3::FLAGS_TIME_VALID;
			}

			if (gps.fix_type >= sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL) {
				fix3.flags |= Fix3::FLAGS_DIFF_CORRECTIONS;
			}

			// Assume receiver is healthy if we're getting data
			fix3.flags |= Fix3::FLAGS_RECEIVER_HEALTHY;

			// Armable if we have at least a 3D fix
			if (gps.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
				fix3.flags |= Fix3::FLAGS_ARMABLE;
			}

			uavcan::Publisher<uavcan::equipment::gnss::Fix3>::broadcast(fix3);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}
};
} // namespace uavcannode
