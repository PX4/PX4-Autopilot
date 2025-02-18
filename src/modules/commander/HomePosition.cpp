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


#include "HomePosition.hpp"

#include <math.h>

#include <lib/geo/geo.h>
#include "commander_helper.h"

using namespace time_literals;

HomePosition::HomePosition(const failsafe_flags_s &failsafe_flags)
	: _failsafe_flags(failsafe_flags)
{
}

bool HomePosition::hasMovedFromCurrentHomeLocation()
{
	float home_dist_xy = -1.f;
	float home_dist_z = -1.f;
	float eph = 0.f;
	float epv = 0.f;

	if (_home_position_pub.get().valid_lpos && _local_position_sub.get().xy_valid && _local_position_sub.get().z_valid) {
		mavlink_wpm_distance_to_point_local(_home_position_pub.get().x, _home_position_pub.get().y, _home_position_pub.get().z,
						    _local_position_sub.get().x, _local_position_sub.get().y, _local_position_sub.get().z,
						    &home_dist_xy, &home_dist_z);

		eph = _local_position_sub.get().eph;
		epv = _local_position_sub.get().epv;

	} else if (_home_position_pub.get().valid_hpos && _home_position_pub.get().valid_alt) {
		if (_valid) {
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			get_distance_to_point_global_wgs84(_home_position_pub.get().lat, _home_position_pub.get().lon,
							   _home_position_pub.get().alt,
							   gpos.lat, gpos.lon, gpos.alt,
							   &home_dist_xy, &home_dist_z);

			eph = gpos.eph;
			epv = gpos.epv;

		} else if (_gps_position_for_home_valid) {

			get_distance_to_point_global_wgs84(_home_position_pub.get().lat, _home_position_pub.get().lon,
							   _home_position_pub.get().alt,
							   _gps_lat, _gps_lon, _gps_alt,
							   &home_dist_xy, &home_dist_z);

			eph = _gps_eph;
			epv = _gps_epv;
		}
	}

	return (home_dist_xy > fmaxf(eph * 2.f, kMinHomePositionChangeEPH))
	       || (home_dist_z > fmaxf(epv * 2.f, kMinHomePositionChangeEPV));
}

bool HomePosition::setHomePosition(bool force)
{
	if (_home_position_pub.get().manual_home && !force) {
		return false;
	}

	bool updated = false;
	home_position_s home{};

	if (!_failsafe_flags.local_position_invalid) {
		// Set home position in local coordinates
		const vehicle_local_position_s &lpos = _local_position_sub.get();
		_heading_reset_counter = lpos.heading_reset_counter; // TODO: should not be here

		fillLocalHomePos(home, lpos);
		updated = true;
	}

	if (!_failsafe_flags.global_position_invalid) {
		// Set home using the global position estimate (fused INS/GNSS)
		const vehicle_global_position_s &gpos = _global_position_sub.get();
		fillGlobalHomePos(home, gpos);
		setHomePosValid();
		updated = true;

	} else if (_gps_position_for_home_valid) {
		// Set home using GNSS position
		fillGlobalHomePos(home, _gps_lat, _gps_lon, _gps_alt);
		setHomePosValid();
		updated = true;

	} else if (_local_position_sub.get().z_global) {
		// handle special case where we are setting only altitude using local position reference
		// This might be overwritten by altitude from global or GNSS altitude
		home.alt = _local_position_sub.get().ref_alt;
		home.valid_alt = true;

		updated = true;
	}

	if (updated) {
		home.timestamp = hrt_absolute_time();
		home.manual_home = false;
		home.update_count = _home_position_pub.get().update_count + 1U;
		updated = _home_position_pub.update(home);
	}

	return updated;
}

void HomePosition::fillLocalHomePos(home_position_s &home, const vehicle_local_position_s &lpos)
{
	fillLocalHomePos(home, lpos.x, lpos.y, lpos.z, lpos.heading);
}

void HomePosition::fillLocalHomePos(home_position_s &home, float x, float y, float z, float heading)
{
	home.x = x;
	home.y = y;
	home.z = z;
	home.valid_lpos = true;

	home.yaw = heading;
}

void HomePosition::fillGlobalHomePos(home_position_s &home, const vehicle_global_position_s &gpos)
{
	fillGlobalHomePos(home, gpos.lat, gpos.lon, (double)gpos.alt);
}

void HomePosition::fillGlobalHomePos(home_position_s &home, double lat, double lon, double alt)
{
	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;
}

void HomePosition::setInAirHomePosition()
{
	home_position_s &home = _home_position_pub.get();

	if (home.manual_home || (home.valid_lpos && home.valid_hpos && home.valid_alt)) {
		return;
	}

	const bool global_home_valid = home.valid_hpos && home.valid_alt;
	const bool local_home_valid = home.valid_lpos;

	if (local_home_valid && !global_home_valid) {
		if (!_failsafe_flags.local_position_invalid && !_failsafe_flags.global_position_invalid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS fused) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			MapProjection ref_pos{gpos.lat, gpos.lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = gpos.alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, (double)home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			home.update_count++;
			_home_position_pub.update();

		} else if (!_failsafe_flags.local_position_invalid && _gps_position_for_home_valid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS raw) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();

			MapProjection ref_pos{_gps_lat, _gps_lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const double home_alt = _gps_alt + (double)home.z;
			fillGlobalHomePos(home, home_lat, home_lon, (double)home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			home.update_count++;
			_home_position_pub.update();
		}

	} else if (!local_home_valid && global_home_valid) {
		const vehicle_local_position_s &lpos = _local_position_sub.get();

		if (!_failsafe_flags.local_position_invalid && lpos.xy_global && lpos.z_global) {
			// Back-compute x, y and z of home position given the global home position
			// and the global reference of the local frame
			MapProjection ref_pos{lpos.ref_lat, lpos.ref_lon};

			float home_x;
			float home_y;
			ref_pos.project(home.lat, home.lon, home_x, home_y);

			const float home_z = -(home.alt - lpos.ref_alt);
			fillLocalHomePos(home, home_x, home_y, home_z, NAN);

			home.timestamp = hrt_absolute_time();
			home.update_count++;
			_home_position_pub.update();
		}

	} else if (!local_home_valid && !global_home_valid) {
		// Home position is not known in any frame, set home at current position
		setHomePosition();

	} else {
		// nothing to do
	}
}

bool HomePosition::setManually(double lat, double lon, float alt, float yaw)
{
	const vehicle_local_position_s &vehicle_local_position = _local_position_sub.get();

	if (!vehicle_local_position.xy_global || !vehicle_local_position.z_global) {
		return false;
	}

	home_position_s &home = _home_position_pub.get();
	home.manual_home = true;

	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;

	// update local projection reference including altitude
	MapProjection ref_pos{vehicle_local_position.ref_lat, vehicle_local_position.ref_lon};
	ref_pos.project(lat, lon, home.x, home.y);
	home.z = -(alt - vehicle_local_position.ref_alt);
	home.valid_lpos = vehicle_local_position.xy_valid && vehicle_local_position.z_valid;

	home.yaw = yaw;

	home.timestamp = hrt_absolute_time();
	home.update_count++;
	_home_position_pub.update();
	setHomePosValid();
	return true;
}


void HomePosition::setHomePosValid()
{
	// play tune first time we initialize HOME
	if (!_valid) {
		tune_home_set(true);
	}

	// mark home position as set
	_valid = true;
}

void HomePosition::updateHomePositionYaw(float yaw)
{
	home_position_s home = _home_position_pub.get();

	home.yaw = yaw;
	home.timestamp = hrt_absolute_time();

	_home_position_pub.update(home);
}

void HomePosition::update(bool set_automatically, bool check_if_changed)
{
	_local_position_sub.update();
	_global_position_sub.update();

	if (_vehicle_gps_position_sub.updated()) {
		sensor_gps_s vehicle_gps_position;
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		_gps_lat = vehicle_gps_position.latitude_deg;
		_gps_lon = vehicle_gps_position.longitude_deg;
		_gps_alt = vehicle_gps_position.altitude_msl_m;
		_gps_eph = vehicle_gps_position.eph;
		_gps_epv = vehicle_gps_position.epv;

		const hrt_abstime now = hrt_absolute_time();
		const bool time = (now < vehicle_gps_position.timestamp + 1_s);
		const bool fix = vehicle_gps_position.fix_type >= kHomePositionGPSRequiredFixType;
		const bool eph = vehicle_gps_position.eph < kHomePositionGPSRequiredEPH;
		const bool epv = vehicle_gps_position.epv < kHomePositionGPSRequiredEPV;
		const bool evh = vehicle_gps_position.s_variance_m_s < kHomePositionGPSRequiredEVH;
		_gps_position_for_home_valid = time && fix && eph && epv && evh;
	}

	const vehicle_local_position_s &lpos = _local_position_sub.get();
	const home_position_s &home = _home_position_pub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (_valid && set_automatically) {
			updateHomePositionYaw(home.yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	if (check_if_changed && set_automatically) {
		const bool can_set_home_lpos_first_time = !home.valid_lpos && !_failsafe_flags.local_position_invalid;
		const bool can_set_home_gpos_first_time = ((!home.valid_hpos || !home.valid_alt)
				&& (!_failsafe_flags.global_position_invalid || _gps_position_for_home_valid));
		const bool can_set_home_alt_first_time = (!home.valid_alt && lpos.z_global);

		if (can_set_home_lpos_first_time
		    || can_set_home_gpos_first_time
		    || can_set_home_alt_first_time
		    || hasMovedFromCurrentHomeLocation()) {
			setHomePosition();
		}
	}
}
