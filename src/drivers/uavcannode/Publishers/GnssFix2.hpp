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

#pragma once

#include <cmath>

#include "UavcanPublisherBase.hpp"

#include <uavcan/equipment/gnss/Fix2.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/pps_capture.h>
#include <uORB/topics/sensor_gnss.h>

namespace uavcannode
{

class GnssFix2 :
	public UavcanPublisherBase,
	public uORB::SubscriptionCallbackWorkItem,
	private uavcan::Publisher<uavcan::equipment::gnss::Fix2>
{
public:
	GnssFix2(px4::WorkItem *work_item, uavcan::INode &node) :
		UavcanPublisherBase(uavcan::equipment::gnss::Fix2::DefaultDataTypeID),
		uORB::SubscriptionCallbackWorkItem(work_item, ORB_ID(sensor_gnss)),
		uavcan::Publisher<uavcan::equipment::gnss::Fix2>(node)
	{
		this->setPriority(uavcan::TransferPriority::OneLowerThanHighest);
	}

	void PrintInfo() override
	{
		if (uORB::SubscriptionCallbackWorkItem::advertised()) {
			printf("\t%s -> %s:%d\n",
			       uORB::SubscriptionCallbackWorkItem::get_topic()->o_name,
			       uavcan::equipment::gnss::Fix2::getDataTypeFullName(),
			       id());
		}
	}

	void BroadcastAnyUpdates() override
	{
		using uavcan::equipment::gnss::Fix2;

		// Track PPS-anchored offset (GPS UTC - local HRT) so the broadcast can
		// stamp fix2.timestamp with a UTC value coherent with fix2.gnss_timestamp.
		// FC-side decoders (UavcanGnssBridge) use (timestamp - gnss_timestamp) as
		// the receiver processing delay; that subtraction is only meaningful when
		// both endpoints are on the same clock, which is what PPS provides here.
		pps_capture_s pps;

		if (_pps_capture_sub.update(&pps) && pps.timestamp != 0 && pps.rtc_timestamp != 0) {
			_pps_offset_us = static_cast<int64_t>(pps.rtc_timestamp) - static_cast<int64_t>(pps.timestamp);
			_pps_last_update = pps.timestamp;
		}

		// sensor_gnss -> uavcan::equipment::gnss::Fix2
		sensor_gnss_s sensor_gnss;

		if (uORB::SubscriptionCallbackWorkItem::update(&sensor_gnss)) {
			uavcan::equipment::gnss::Fix2 fix2{};

			fix2.gnss_time_standard = fix2.GNSS_TIME_STANDARD_UTC;
			fix2.gnss_timestamp.usec = sensor_gnss.time_utc_usec;

			const hrt_abstime now = hrt_absolute_time();

			if (_pps_last_update != 0 && (now - _pps_last_update) < kPpsStaleTimeoutUs) {
				fix2.timestamp.usec = static_cast<uint64_t>(static_cast<int64_t>(now) + _pps_offset_us);
			}

			fix2.latitude_deg_1e8 = (int64_t)(sensor_gnss.latitude * 1e8);
			fix2.longitude_deg_1e8 = (int64_t)(sensor_gnss.longitude * 1e8);
			fix2.height_msl_mm = (int32_t)(sensor_gnss.altitude_msl * 1e3);
			fix2.height_ellipsoid_mm = (int32_t)(sensor_gnss.altitude_ellipsoid * 1e3);
			fix2.status = sensor_gnss.fix_type;
			fix2.ned_velocity[0] = sensor_gnss.vel_north;
			fix2.ned_velocity[1] = sensor_gnss.vel_east;
			fix2.ned_velocity[2] = sensor_gnss.vel_down;
			fix2.pdop = sensor_gnss.hdop > sensor_gnss.vdop ? sensor_gnss.hdop :
				    sensor_gnss.vdop; // Use pdop for both hdop and vdop since uavcan v0 spec does not support them
			fix2.sats_used = sensor_gnss.satellites_used;

			fix2.mode = Fix2::MODE_SINGLE;
			fix2.sub_mode = 0;

			switch (fix2.status) {
			case 4:
				fix2.mode = Fix2::MODE_DGPS;
				break;

			case 5:
				fix2.mode = Fix2::MODE_RTK;
				fix2.sub_mode = Fix2::SUB_MODE_RTK_FLOAT;
				break;

			case 6:
				fix2.mode = Fix2::MODE_RTK;
				fix2.sub_mode = Fix2::SUB_MODE_RTK_FIXED;
				break;
			}

			// Diagonal matrix
			// position variances -- Xx, Yy, Zz (eph/epv are std dev in meters, must square for variance)
			fix2.covariance.push_back(sensor_gnss.eph * sensor_gnss.eph);
			fix2.covariance.push_back(sensor_gnss.eph * sensor_gnss.eph);
			fix2.covariance.push_back(sensor_gnss.epv * sensor_gnss.epv);
			// velocity variance -- Vxx, Vyy, Vzz
			fix2.covariance.push_back(sensor_gnss.speed_accuracy);
			fix2.covariance.push_back(sensor_gnss.speed_accuracy);
			fix2.covariance.push_back(sensor_gnss.speed_accuracy);

			uavcan::equipment::gnss::ECEFPositionVelocity ecefpositionvelocity{};
			ecefpositionvelocity.velocity_xyz[0] = NAN;
			ecefpositionvelocity.velocity_xyz[1] = NAN;
			ecefpositionvelocity.velocity_xyz[2] = NAN;

			// Use ecef_position_velocity for now... There are no fields for these
			ecefpositionvelocity.position_xyz_mm[0] = sensor_gnss.noise;
			ecefpositionvelocity.position_xyz_mm[1] = sensor_gnss.jamming_indicator;
			ecefpositionvelocity.position_xyz_mm[2] = (sensor_gnss.jamming_state << 8) | sensor_gnss.spoofing_state;

			fix2.ecef_position_velocity.push_back(ecefpositionvelocity);

			uavcan::Publisher<uavcan::equipment::gnss::Fix2>::broadcast(fix2);

			// ensure callback is registered
			uORB::SubscriptionCallbackWorkItem::registerCallback();
		}
	}

private:
	static constexpr hrt_abstime kPpsStaleTimeoutUs{5'000'000};

	uORB::Subscription _pps_capture_sub{ORB_ID(pps_capture)};
	int64_t _pps_offset_us{0};
	hrt_abstime _pps_last_update{0};
};
} // namespace uavcannode
