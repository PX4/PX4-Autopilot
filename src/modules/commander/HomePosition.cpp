/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <lib/geo/geo.h>
#include "commander_helper.h"

HomePosition::HomePosition(const vehicle_status_flags_s &vehicle_status_flags)
	: _vehicle_status_flags(vehicle_status_flags)
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

		} else if (!_vehicle_status_flags.gps_position_invalid) {
			sensor_gps_s gps;
			_vehicle_gps_position_sub.copy(&gps);
			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			get_distance_to_point_global_wgs84(_home_position_pub.get().lat, _home_position_pub.get().lon,
							   _home_position_pub.get().alt,
							   lat, lon, alt,
							   &home_dist_xy, &home_dist_z);

			eph = gps.eph;
			epv = gps.epv;
		}
	}

	return (home_dist_xy > eph * 2.f) || (home_dist_z > epv * 2.f);
}

bool HomePosition::setHomePosition(bool force)
{
	if (_home_position_pub.get().manual_home && !force) {
		return false;
	}

	bool updated = false;
	home_position_s home{};

	if (!_vehicle_status_flags.local_position_invalid) {
		// Set home position in local coordinates
		const vehicle_local_position_s &lpos = _local_position_sub.get();
		_heading_reset_counter = lpos.heading_reset_counter; // TODO: should not be here

		fillLocalHomePos(home, lpos);
		updated = true;
	}

	if (!_vehicle_status_flags.global_position_invalid) {
		// Set home using the global position estimate (fused INS/GNSS)
		const vehicle_global_position_s &gpos = _global_position_sub.get();
		fillGlobalHomePos(home, gpos);
		setHomePosValid();
		updated = true;

	} else if (!_vehicle_status_flags.gps_position_invalid) {
		// Set home using GNSS position
		sensor_gps_s gps_pos;
		_vehicle_gps_position_sub.copy(&gps_pos);
		const double lat = static_cast<double>(gps_pos.lat) * 1e-7;
		const double lon = static_cast<double>(gps_pos.lon) * 1e-7;
		const float alt = static_cast<float>(gps_pos.alt) * 1e-3f;
		fillGlobalHomePos(home, lat, lon, alt);
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
	fillGlobalHomePos(home, gpos.lat, gpos.lon, gpos.alt);
}

void HomePosition::fillGlobalHomePos(home_position_s &home, double lat, double lon, float alt)
{
	home.lat = lat;
	home.lon = lon;
	home.valid_hpos = true;
	home.alt = alt;
	home.valid_alt = true;
}

void HomePosition::setInAirHomePosition()
{
	auto &home = _home_position_pub.get();

	if (home.manual_home || (home.valid_lpos && home.valid_hpos && home.valid_alt)) {
		return;
	}

	const bool global_home_valid = home.valid_hpos && home.valid_alt;
	const bool local_home_valid = home.valid_lpos;

	if (local_home_valid && !global_home_valid) {
		if (!_vehicle_status_flags.local_position_invalid && !_vehicle_status_flags.global_position_invalid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS fused) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			const vehicle_global_position_s &gpos = _global_position_sub.get();

			MapProjection ref_pos{gpos.lat, gpos.lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = gpos.alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_position_pub.update();

		} else if (!_vehicle_status_flags.local_position_invalid && !_vehicle_status_flags.gps_position_invalid) {
			// Back-compute lon, lat and alt of home position given the local home position
			// and current positions in local and global (GNSS raw) frames
			const vehicle_local_position_s &lpos = _local_position_sub.get();
			sensor_gps_s gps;
			_vehicle_gps_position_sub.copy(&gps);

			const double lat = static_cast<double>(gps.lat) * 1e-7;
			const double lon = static_cast<double>(gps.lon) * 1e-7;
			const float alt = static_cast<float>(gps.alt) * 1e-3f;

			MapProjection ref_pos{lat, lon};

			double home_lat;
			double home_lon;
			ref_pos.reproject(home.x - lpos.x, home.y - lpos.y, home_lat, home_lon);

			const float home_alt = alt + home.z;
			fillGlobalHomePos(home, home_lat, home_lon, home_alt);

			setHomePosValid();
			home.timestamp = hrt_absolute_time();
			_home_position_pub.update();
		}

	} else if (!local_home_valid && global_home_valid) {
		const vehicle_local_position_s &lpos = _local_position_sub.get();

		if (!_vehicle_status_flags.local_position_invalid && lpos.xy_global && lpos.z_global) {
			// Back-compute x, y and z of home position given the global home position
			// and the global reference of the local frame
			MapProjection ref_pos{lpos.ref_lat, lpos.ref_lon};

			float home_x;
			float home_y;
			ref_pos.project(home.lat, home.lon, home_x, home_y);

			const float home_z = -(home.alt - lpos.ref_alt);
			fillLocalHomePos(home, home_x, home_y, home_z, NAN);

			home.timestamp = hrt_absolute_time();
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
	const auto &vehicle_local_position = _local_position_sub.get();

	if (!vehicle_local_position.xy_global || !vehicle_local_position.z_global) {
		return false;
	}

	auto &home = _home_position_pub.get();
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

	const vehicle_local_position_s &lpos = _local_position_sub.get();
	const auto &home = _home_position_pub.get();

	if (lpos.heading_reset_counter != _heading_reset_counter) {
		if (_valid && set_automatically) {
			updateHomePositionYaw(home.yaw + lpos.delta_heading);
		}

		_heading_reset_counter = lpos.heading_reset_counter;
	}

	if (check_if_changed && set_automatically) {
		const bool can_set_home_lpos_first_time = !home.valid_lpos && !_vehicle_status_flags.local_position_invalid;
		const bool can_set_home_gpos_first_time = ((!home.valid_hpos || !home.valid_alt)
				&& (!_vehicle_status_flags.global_position_invalid || !_vehicle_status_flags.gps_position_invalid));
		const bool can_set_home_alt_first_time = (!home.valid_alt && lpos.z_global);

		if (can_set_home_lpos_first_time
		    || can_set_home_gpos_first_time
		    || can_set_home_alt_first_time
		    || hasMovedFromCurrentHomeLocation()) {
			setHomePosition();
		}
	}
}
