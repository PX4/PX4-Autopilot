/****************************************************************************
 *
 *   Copyright (c) 2013,2017 PX4 Development Team. All rights reserved.
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
 * @file geofence.cpp
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */
#include "geofence.h"

#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <string.h>
#include <dataman/dataman.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <unistd.h>
#include <geo/geo.h>
#include <drivers/drv_hrt.h>
#include <v2.0/common/mavlink.h>

#include "navigator.h"

#define GEOFENCE_RANGE_WARNING_LIMIT 5000000

Geofence::Geofence(Navigator *navigator) :
	SuperBlock(navigator, "GF"),
	_navigator(navigator),
	_home_pos{},
	_home_pos_set(false),
	_last_horizontal_range_warning(0),
	_last_vertical_range_warning(0),
	_altitude_min(0.f),
	_altitude_max(0.f),
	_polygons(nullptr),
	_num_polygons(0),
	_param_action(this, "GF_ACTION", false),
	_param_altitude_mode(this, "GF_ALTMODE", false),
	_param_source(this, "GF_SOURCE", false),
	_param_counter_threshold(this, "GF_COUNT", false),
	_param_max_hor_distance(this, "GF_MAX_HOR_DIST", false),
	_param_max_ver_distance(this, "GF_MAX_VER_DIST", false),
	_outside_counter(0)
{
	/* Load initial params */
	updateParams();
	updateFence();
}

Geofence::~Geofence()
{
	if (_polygons) {
		delete[](_polygons);
	}
}

void Geofence::updateFence()
{
	// initialize fence points count
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_fence_items = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_fence_items = stats.num_items;
	}

	// iterate over all polygons and store their starting vertices
	_num_polygons = 0;
	int current_seq = 1;

	while (current_seq <= num_fence_items) {
		mission_fence_point_s mission_fence_point;

		if (dm_read(DM_KEY_FENCE_POINTS, current_seq, &mission_fence_point, sizeof(mission_fence_point_s)) !=
		    sizeof(mission_fence_point_s)) {
			PX4_ERR("dm_read failed");
			break;
		}

		switch (mission_fence_point.nav_cmd) {
		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			// TODO: do we need to store this?
			++current_seq;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
			if (mission_fence_point.vertex_count == 0) {
				++current_seq; // avoid endless loop
				PX4_ERR("Polygon with 0 vertices. Skipping");

			} else {
				if (_polygons) {
					// resize: this is somewhat inefficient, but we do not expect there to be many polygons
					PolygonInfo *new_polygons = new PolygonInfo[_num_polygons + 1];

					if (new_polygons) {
						memcpy(new_polygons, _polygons, sizeof(PolygonInfo) * _num_polygons);
					}

					delete[](_polygons);
					_polygons = new_polygons;

				} else {
					_polygons = new PolygonInfo[1];
				}

				if (!_polygons) {
					_num_polygons = 0;
					PX4_ERR("alloc failed");
					return;
				}

				PolygonInfo &polygon = _polygons[_num_polygons];
				polygon.dataman_index = current_seq;
				polygon.fence_type = mission_fence_point.nav_cmd;
				polygon.vertex_count = mission_fence_point.vertex_count;
				current_seq += mission_fence_point.vertex_count;
				++_num_polygons;
			}

			break;

		default:
			PX4_ERR("unhandled Fence command: %i", (int)mission_fence_point.nav_cmd);
			++current_seq;
			break;
		}

	}

}

bool Geofence::checkAll(const struct vehicle_global_position_s &global_position)
{
	return checkAll(global_position.lat, global_position.lon, global_position.alt);
}

bool Geofence::checkAll(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl)
{
	return checkAll(global_position.lat, global_position.lon, baro_altitude_amsl);
}


bool Geofence::check(const struct vehicle_global_position_s &global_position,
		     const struct vehicle_gps_position_s &gps_position, float baro_altitude_amsl,
		     const struct home_position_s home_pos, bool home_position_set)
{
	_home_pos = home_pos;
	_home_pos_set = home_position_set;

	if (getAltitudeMode() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position);

		} else {
			return checkAll((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
					(double)gps_position.alt * 1.0e-3);
		}

	} else {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position, baro_altitude_amsl);

		} else {
			return checkAll((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
					baro_altitude_amsl);
		}
	}
}

bool Geofence::check(const struct mission_item_s &mission_item)
{
	return checkAll(mission_item.lat, mission_item.lon, mission_item.altitude);
}

bool Geofence::checkAll(double lat, double lon, float altitude)
{
	bool inside_fence = true;

	float max_horizontal_distance = _param_max_hor_distance.get();
	float max_vertical_distance = _param_max_ver_distance.get();

	if (max_horizontal_distance > 1.0f || max_vertical_distance > 1.0f) {
		if (_home_pos_set) {
			float dist_xy = -1.0f;
			float dist_z = -1.0f;
			get_distance_to_point_global_wgs84(lat, lon, altitude,
							   _home_pos.lat, _home_pos.lon, _home_pos.alt,
							   &dist_xy, &dist_z);

			if (max_vertical_distance > 1.0f && (dist_z > max_vertical_distance)) {
				if (hrt_elapsed_time(&_last_vertical_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							     "Maximum altitude above home exceeded by %.1f m",
							     (double)(dist_z - max_vertical_distance));
					_last_vertical_range_warning = hrt_absolute_time();
				}

				inside_fence = false;
			}

			if (max_horizontal_distance > 1.0f && (dist_xy > max_horizontal_distance)) {
				if (hrt_elapsed_time(&_last_horizontal_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(),
							     "Maximum distance from home exceeded by %.1f m",
							     (double)(dist_xy - max_horizontal_distance));
					_last_horizontal_range_warning = hrt_absolute_time();
				}

				inside_fence = false;
			}
		}
	}

	// to be inside the geofence both fences have to report being inside
	// as they both report being inside when not enabled
	inside_fence = inside_fence && checkPolygons(lat, lon, altitude);

	if (inside_fence) {
		_outside_counter = 0;
		return inside_fence;

	} else {
		_outside_counter++;

		if (_outside_counter > _param_counter_threshold.get()) {
			return inside_fence;

		} else {
			return true;
		}
	}
}


bool Geofence::checkPolygons(double lat, double lon, float altitude)
{
	if (isEmpty()) {
		/* Empty fence -> accept all points */
		return true;
	}

	/* Vertical check */
	if (_altitude_max > _altitude_min) { // only enable vertical check if configured properly
		if (altitude > _altitude_max || altitude < _altitude_min) {
			return false;
		}
	}

	/* Horizontal check: iterate all polygons */
	bool outside_exclusion = true;
	bool inside_inclusion = false;
	bool had_inclusion_polygons = false;

	for (int polygon_idx = 0; polygon_idx < _num_polygons; ++polygon_idx) {
		bool inside = insidePolygon(_polygons[polygon_idx], lat, lon, altitude);

		if (_polygons[polygon_idx].fence_type == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION) {
			if (inside) {
				inside_inclusion = true;
			}

			had_inclusion_polygons = true;

		} else { // exclusion
			if (inside) {
				outside_exclusion = false;
			}
		}
	}

	return (!had_inclusion_polygons || inside_inclusion) && outside_exclusion;
}

bool Geofence::insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude)
{

	/* Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF)
	 * Only supports non-complex polygons (not self intersecting)
	 */

	mission_fence_point_s temp_vertex_i;
	mission_fence_point_s temp_vertex_j;
	bool c = false;

	for (unsigned i = 0, j = polygon.vertex_count - 1; i < polygon.vertex_count; j = i++) {
		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + i, &temp_vertex_i,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
			break;
		}

		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + j, &temp_vertex_j,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
			break;
		}

		if (temp_vertex_i.frame != MAV_FRAME_GLOBAL && temp_vertex_i.frame != MAV_FRAME_GLOBAL_INT
		    && temp_vertex_i.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT
		    && temp_vertex_i.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			// TODO: handle different frames
			PX4_ERR("Frame type %i not supported", (int)temp_vertex_i.frame);
			break;
		}

		if (((double)temp_vertex_i.lon >= lon) != ((double)temp_vertex_j.lon >= lon) &&
		    (lat <= (double)(temp_vertex_j.lat - temp_vertex_i.lat) * (lon - (double)temp_vertex_i.lon) /
		     (double)(temp_vertex_j.lon - temp_vertex_i.lon) + (double)temp_vertex_i.lat)) {
			c = !c;
		}
	}

	return c;
}

bool
Geofence::valid()
{
	return true; // always valid
}

int
Geofence::loadFromFile(const char *filename)
{
	FILE		*fp;
	char		line[120];
	int			pointCounter = 0;
	bool		gotVertical = false;
	const char commentChar = '#';
	int rc = PX4_ERROR;

	/* Make sure no data is left in the datamanager */
	clearDm();

	/* open the mixer definition file */
	fp = fopen(GEOFENCE_FILENAME, "r");

	if (fp == nullptr) {
		return PX4_ERROR;
	}

	/* create geofence points from valid lines and store in DM */
	for (;;) {
		/* get a line, bail on error/EOF */
		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* Trim leading whitespace */
		size_t textStart = 0;

		while ((textStart < sizeof(line) / sizeof(char)) && isspace(line[textStart])) { textStart++; }

		/* if the line starts with #, skip */
		if (line[textStart] == commentChar) {
			continue;
		}

		/* if there is only a linefeed, skip it */
		if (line[0] == '\n') {
			continue;
		}

		if (gotVertical) {
			/* Parse the line as a geofence point */
			mission_fence_point_s vertex;
			vertex.frame = MAV_FRAME_GLOBAL;
			vertex.nav_cmd = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
			vertex.vertex_count = 0; // this will be filled in a second pass
			vertex.alt = 0; // alt is not used

			/* if the line starts with DMS, this means that the coordinate is given as degree minute second instead of decimal degrees */
			if (line[textStart] == 'D' && line[textStart + 1] == 'M' && line[textStart + 2] == 'S') {
				/* Handle degree minute second format */
				double lat_d, lat_m, lat_s, lon_d, lon_m, lon_s;

				if (sscanf(line, "DMS %lf %lf %lf %lf %lf %lf", &lat_d, &lat_m, &lat_s, &lon_d, &lon_m, &lon_s) != 6) {
					PX4_ERR("Scanf to parse DMS geofence vertex failed.");
					goto error;
				}

//				PX4_INFO("Geofence DMS: %.5lf %.5lf %.5lf ; %.5lf %.5lf %.5lf", lat_d, lat_m, lat_s, lon_d, lon_m, lon_s);

				vertex.lat = lat_d + lat_m / 60.0 + lat_s / 3600.0;
				vertex.lon = lon_d + lon_m / 60.0 + lon_s / 3600.0;

			} else {
				/* Handle decimal degree format */
				if (sscanf(line, "%lf %lf", &vertex.lat, &vertex.lon) != 2) {
					PX4_ERR("Scanf to parse geofence vertex failed.");
					goto error;
				}
			}

			if (dm_write(DM_KEY_FENCE_POINTS, pointCounter + 1, DM_PERSIST_POWER_ON_RESET, &vertex,
				     sizeof(vertex)) != sizeof(vertex)) {
				goto error;
			}

			PX4_INFO("Geofence: point: %d, lat %.5lf: lon: %.5lf", pointCounter, vertex.lat, vertex.lon);

			pointCounter++;

		} else {
			/* Parse the line as the vertical limits */
			if (sscanf(line, "%f %f", &_altitude_min, &_altitude_max) != 2) {
				goto error;
			}

			PX4_INFO("Geofence: alt min: %.4f, alt_max: %.4f", (double)_altitude_min, (double)_altitude_max);
			gotVertical = true;
		}
	}


	/* Check if import was successful */
	if (gotVertical && pointCounter > 2) {
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Geofence imported");
		rc = PX4_OK;

		/* do a second pass, now that we know the number of vertices */
		for (int seq = 1; seq <= pointCounter; ++seq) {
			mission_fence_point_s mission_fence_point;

			if (dm_read(DM_KEY_FENCE_POINTS, seq, &mission_fence_point, sizeof(mission_fence_point_s)) ==
			    sizeof(mission_fence_point_s)) {
				mission_fence_point.vertex_count = pointCounter;
				dm_write(DM_KEY_FENCE_POINTS, seq, DM_PERSIST_POWER_ON_RESET, &mission_fence_point,
					 sizeof(mission_fence_point_s));
			}
		}

		mission_stats_entry_s stats;
		stats.num_items = pointCounter;
		rc = dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));

	} else {
		PX4_ERR("Geofence: import error");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence import error");
	}

	updateFence();

error:
	fclose(fp);
	return rc;
}

int Geofence::clearDm()
{
	dm_clear(DM_KEY_FENCE_POINTS);
	updateFence();
	return PX4_OK;
}
