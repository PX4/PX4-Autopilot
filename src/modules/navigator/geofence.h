/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file geofence.h
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#ifndef GEOFENCE_H_
#define GEOFENCE_H_

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/home_position.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>

#define GEOFENCE_FILENAME PX4_ROOTFSDIR"/fs/microsd/etc/geofence.txt"

class Navigator;

class Geofence : public control::SuperBlock
{
public:
	Geofence(Navigator *navigator);

	Geofence(const Geofence &) = delete;
	Geofence &operator=(const Geofence &) = delete;

	~Geofence();

	/* Altitude mode, corresponding to the param GF_ALTMODE */
	enum {
		GF_ALT_MODE_WGS84 = 0,
		GF_ALT_MODE_AMSL = 1
	};

	/* Source, corresponding to the param GF_SOURCE */
	enum {
		GF_SOURCE_GLOBALPOS = 0,
		GF_SOURCE_GPS = 1
	};

	/**
	 * update the geofence from dataman
	 */
	void updateFence();

	/**
	 * Return whether the system obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const struct vehicle_global_position_s &global_position,
		   const struct vehicle_gps_position_s &gps_position, float baro_altitude_amsl,
		   const struct home_position_s home_pos, bool home_position_set);

	/**
	 * Return whether a mission item obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const struct mission_item_s &mission_item);

	int clearDm();

	bool valid();

	/**
	 * Load a single inclusion polygon, replacing any already existing polygons.
	 * The file has one of the following formats:
	 * - Decimal Degrees:
	 * 0 900
	 * 47.475273548913222 8.52672100067138672
	 * 47.4608261578541359 8.53414535522460938
	 * 47.4613484218861217 8.56444358825683594
	 * 47.4830758091035534 8.53470325469970703
	 *
	 * - Degree-Minute-Second:
	 * 0 900
	 * DMS -26 -34 -10.4304 151 50 14.5428
	 * DMS -26 -34 -11.8416 151 50 21.8580
	 * DMS -26 -34 -36.5628 151 50 28.1112
	 * DMS -26 -34 -37.1640 151 50 24.1620
	 *
	 * Where the first line is min, max altitude in meters AMSL.
	 */
	int loadFromFile(const char *filename);

	bool isEmpty() { return _num_polygons == 0; }

	int getAltitudeMode() { return _param_altitude_mode.get(); }

	int getSource() { return _param_source.get(); }

	int getGeofenceAction() { return _param_action.get(); }

private:
	Navigator	*_navigator;

	home_position_s _home_pos;
	bool _home_pos_set;

	hrt_abstime _last_horizontal_range_warning;
	hrt_abstime _last_vertical_range_warning;

	float _altitude_min;
	float _altitude_max;

	struct PolygonInfo {
		uint16_t fence_type; ///< one of MAV_CMD_NAV_FENCE_*
		uint16_t dataman_index;
		uint16_t vertex_count;
	};
	PolygonInfo *_polygons;
	int _num_polygons;

	/* Params */
	control::BlockParamInt _param_action;
	control::BlockParamInt _param_altitude_mode;
	control::BlockParamInt _param_source;
	control::BlockParamInt _param_counter_threshold;
	control::BlockParamFloat _param_max_hor_distance;
	control::BlockParamFloat _param_max_ver_distance;

	int _outside_counter;

	/**
	 * Check if a point passes the Geofence test.
	 * This takes all polygons and minimum & maximum altitude into account
	 *
	 * The check passes if: (inside(polygon_inclusion_1) || inside(polygon_inclusion_2) || ... ) &&
	 *                       !inside(polygon_exclusion_1) && !inside(polygon_exclusion_2) && ...
	 *                       && (altitude within [min, max])
	 *                  or: no polygon configured
	 * @return result of the check above (false for a geofence violation)
	 */
	bool checkPolygons(double lat, double lon, float altitude);

	/**
	 * Check if a point passes the Geofence test.
	 * In addition to checkPolygons(), this takes all additional parameters into account.
	 *
	 * @return false for a geofence violation
	 */
	bool checkAll(double lat, double lon, float altitude);

	bool checkAll(const struct vehicle_global_position_s &global_position);
	bool checkAll(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl);

	/**
	 * Check if a single point is within a polygon
	 * @return true if within polygon
	 */
	bool insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude);
};


#endif /* GEOFENCE_H_ */
