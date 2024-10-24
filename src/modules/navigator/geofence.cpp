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
#include "navigator.h"

#include <ctype.h>

#include <dataman_client/DatamanClient.hpp>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>

#include "navigator.h"

#define GEOFENCE_RANGE_WARNING_LIMIT 5000000

Geofence::Geofence(Navigator *navigator) :
	ModuleParams(navigator),
	_navigator(navigator),
	_sub_airdata(ORB_ID(vehicle_air_data))
{
	if (_navigator != nullptr) {
		updateFence();
	}
}

Geofence::~Geofence()
{
	if (_polygons) {
		delete[](_polygons);
	}
}

void Geofence::run()
{
	bool success;

	switch (_dataman_state) {

	case DatamanState::UpdateRequestWait:

		if (_initiate_fence_updated) {
			_initiate_fence_updated = false;
			_dataman_state	= DatamanState::Read;
		}

		break;

	case DatamanState::Read:

		_dataman_state = DatamanState::ReadWait;
		success = _dataman_client.readAsync(DM_KEY_FENCE_POINTS, 0, reinterpret_cast<uint8_t *>(&_stats),
						    sizeof(mission_stats_entry_s));

		if (!success) {
			_error_state = DatamanState::Read;
			_dataman_state = DatamanState::Error;
		}

		break;

	case DatamanState::ReadWait:

		_dataman_client.update();

		if (_dataman_client.lastOperationCompleted(success)) {

			if (!success) {
				_error_state = DatamanState::ReadWait;
				_dataman_state = DatamanState::Error;

			} else if (_update_counter != _stats.update_counter) {

				_update_counter = _stats.update_counter;
				_fence_updated = false;

				_dataman_cache.invalidate();

				if (_dataman_cache.size() != _stats.num_items) {
					_dataman_cache.resize(_stats.num_items);
				}

				for (int index = 1; index <= _dataman_cache.size(); ++index) {
					_dataman_cache.load(DM_KEY_FENCE_POINTS, index);
				}

				_dataman_state = DatamanState::Load;

			} else {
				_dataman_state = DatamanState::UpdateRequestWait;
			}
		}

		break;

	case DatamanState::Load:

		_dataman_cache.update();

		if (!_dataman_cache.isLoading()) {
			_dataman_state = DatamanState::UpdateRequestWait;
			_updateFence();
			_fence_updated = true;
		}

		break;

	case DatamanState::Error:
		PX4_ERR("Geofence update failed! state: %" PRIu8, static_cast<uint8_t>(_error_state));
		_dataman_state = DatamanState::UpdateRequestWait;
		break;

	default:
		break;

	}
}

void Geofence::updateFence()
{
	_initiate_fence_updated = true;
}

void Geofence::_updateFence()
{
	mission_fence_point_s mission_fence_point;
	bool is_circle_area = false;

	// iterate over all polygons and store their starting vertices
	_num_polygons = 0;
	int current_seq = 1;

	while (current_seq <= _dataman_cache.size()) {

		bool success = _dataman_cache.loadWait(DM_KEY_FENCE_POINTS, current_seq,
						       reinterpret_cast<uint8_t *>(&mission_fence_point),
						       sizeof(mission_fence_point_s));

		if (!success) {
			PX4_ERR("loadWait failed, seq: %i", current_seq);
			break;
		}

		switch (mission_fence_point.nav_cmd) {
		case NAV_CMD_FENCE_RETURN_POINT:
			// TODO: do we need to store this?
			++current_seq;
			break;

		case NAV_CMD_FENCE_CIRCLE_INCLUSION:
		case NAV_CMD_FENCE_CIRCLE_EXCLUSION:
			is_circle_area = true;

		/* FALLTHROUGH */
		case NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION:
		case NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION:
			if (!is_circle_area && mission_fence_point.vertex_count == 0) {
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

				if (is_circle_area) {
					polygon.circle_radius = mission_fence_point.circle_radius;
					current_seq += 1;

				} else {
					polygon.vertex_count = mission_fence_point.vertex_count;
					current_seq += mission_fence_point.vertex_count;
				}

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

bool Geofence::checkAll(const struct vehicle_global_position_s &global_position, const float alt)
{
	return checkAll(global_position.lat, global_position.lon, alt);
}

bool Geofence::checkAll(double lat, double lon, float altitude)
{
	bool inside_fence = isCloserThanMaxDistToHome(lat, lon, altitude);

	inside_fence = inside_fence && isBelowMaxAltitude(altitude);

	// to be inside the geofence both fences have to report being inside
	// as they both report being inside when not enabled
	inside_fence = inside_fence && isInsidePolygonOrCircle(lat, lon, altitude);

	if (inside_fence) {
		_outside_counter = 0;
		return inside_fence;

	} else {
		_outside_counter++;

		if (_outside_counter > _param_gf_count.get()) {
			return inside_fence;

		} else {
			return true;
		}
	}
}

bool Geofence::check(const vehicle_global_position_s &global_position, const sensor_gps_s &gps_position)
{
	if (_param_gf_altmode.get() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position);

		} else {
			return checkAll(gps_position.lat * 1.0e-7, gps_position.lon * 1.0e-7, gps_position.alt * 1.0e-3);
		}

	} else {
		// get baro altitude
		_sub_airdata.update();
		const float baro_altitude_amsl = _sub_airdata.get().baro_alt_meter;

		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position, baro_altitude_amsl);

		} else {
			return checkAll(gps_position.lat * 1.0e-7, gps_position.lon * 1.0e-7, baro_altitude_amsl);
		}
	}
}

bool Geofence::check(const struct mission_item_s &mission_item)
{
	return checkAll(mission_item.lat, mission_item.lon, mission_item.altitude);
}

bool Geofence::isCloserThanMaxDistToHome(double lat, double lon, float altitude)
{
	bool inside_fence = true;

	if (isHomeRequired() && _navigator->home_global_position_valid()) {

		const float max_horizontal_distance = _param_gf_max_hor_dist.get();

		const double home_lat = _navigator->get_home_position()->lat;
		const double home_lon = _navigator->get_home_position()->lon;
		const float home_alt = _navigator->get_home_position()->alt;

		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		get_distance_to_point_global_wgs84(lat, lon, altitude, home_lat, home_lon, home_alt, &dist_xy, &dist_z);

		if (max_horizontal_distance > FLT_EPSILON && (dist_xy > max_horizontal_distance)) {
			if (hrt_elapsed_time(&_last_horizontal_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum distance from home reached (%.5f)\t",
						     (double)max_horizontal_distance);
				events::send<float>(events::ID("navigator_geofence_max_dist_from_home"), {events::Log::Critical, events::LogInternal::Warning},
						    "Geofence: maximum distance from home reached ({1:.0m})",
						    max_horizontal_distance);
				_last_horizontal_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}
	}

	return inside_fence;
}

bool Geofence::isBelowMaxAltitude(float altitude)
{
	bool inside_fence = true;

	if (isHomeRequired() && _navigator->home_alt_valid()) {

		const float max_vertical_distance = _param_gf_max_ver_dist.get();
		const float home_alt = _navigator->get_home_position()->alt;

		float dist_z = altitude - home_alt;

		if (max_vertical_distance > FLT_EPSILON && (dist_z > max_vertical_distance)) {
			if (hrt_elapsed_time(&_last_vertical_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Maximum altitude above home reached (%.5f)\t",
						     (double)max_vertical_distance);
				events::send<float>(events::ID("navigator_geofence_max_alt_from_home"), {events::Log::Critical, events::LogInternal::Warning},
						    "Geofence: maximum altitude above home reached ({1:.0m_v})",
						    max_vertical_distance);
				_last_vertical_range_warning = hrt_absolute_time();
			}

			inside_fence = false;
		}
	}

	return inside_fence;
}

bool Geofence::isInsidePolygonOrCircle(double lat, double lon, float altitude)
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

	/* Horizontal check: iterate all polygons & circles */
	bool outside_exclusion = true;
	bool inside_inclusion = false;
	bool had_inclusion_areas = false;

	for (int polygon_index = 0; polygon_index < _num_polygons; ++polygon_index) {
		if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION) {
			bool inside = insideCircle(_polygons[polygon_index], lat, lon, altitude);

			if (inside) {
				inside_inclusion = true;
			}

			had_inclusion_areas = true;

		} else if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			bool inside = insideCircle(_polygons[polygon_index], lat, lon, altitude);

			if (inside) {
				outside_exclusion = false;
			}

		} else { // it's a polygon
			bool inside = insidePolygon(_polygons[polygon_index], lat, lon, altitude);

			if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION) {
				if (inside) {
					inside_inclusion = true;
				}

				had_inclusion_areas = true;

			} else { // exclusion
				if (inside) {
					outside_exclusion = false;
				}
			}
		}
	}

	return (!had_inclusion_areas || inside_inclusion) && outside_exclusion;
}

bool Geofence::insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude)
{
	/**
	 * Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF)
	 * Only supports non-complex polygons (not self intersecting)
	 */

	mission_fence_point_s temp_vertex_i{};
	mission_fence_point_s temp_vertex_j{};
	bool c = false;

	for (unsigned i = 0, j = polygon.vertex_count - 1; i < polygon.vertex_count; j = i++) {

		bool success = _dataman_cache.loadWait(DM_KEY_FENCE_POINTS, polygon.dataman_index + i,
						       reinterpret_cast<uint8_t *>(&temp_vertex_i), sizeof(mission_fence_point_s));

		if (!success) {
			break;
		}

		success = _dataman_cache.loadWait(DM_KEY_FENCE_POINTS, polygon.dataman_index + j,
						  reinterpret_cast<uint8_t *>(&temp_vertex_j), sizeof(mission_fence_point_s));

		if (!success) {
			break;
		}

		if (temp_vertex_i.frame != NAV_FRAME_GLOBAL && temp_vertex_i.frame != NAV_FRAME_GLOBAL_INT
		    && temp_vertex_i.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT
		    && temp_vertex_i.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
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

bool Geofence::insideCircle(const PolygonInfo &polygon, double lat, double lon, float altitude)
{

	mission_fence_point_s circle_point{};
	bool success = _dataman_cache.loadWait(DM_KEY_FENCE_POINTS, polygon.dataman_index,
					       reinterpret_cast<uint8_t *>(&circle_point), sizeof(mission_fence_point_s));

	if (!success) {
		PX4_ERR("dm_read failed");
		return false;
	}

	if (circle_point.frame != NAV_FRAME_GLOBAL && circle_point.frame != NAV_FRAME_GLOBAL_INT
	    && circle_point.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT
	    && circle_point.frame != NAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
		// TODO: handle different frames
		PX4_ERR("Frame type %i not supported", (int)circle_point.frame);
		return false;
	}

	if (!_projection_reference.isInitialized()) {
		_projection_reference.initReference(lat, lon);
	}

	float x1, y1, x2, y2;
	_projection_reference.project(lat, lon, x1, y1);
	_projection_reference.project(circle_point.lat, circle_point.lon, x2, y2);
	float dx = x1 - x2, dy = y1 - y2;
	return dx * dx + dy * dy < circle_point.circle_radius * circle_point.circle_radius;
}

bool
Geofence::valid()
{
	return true; // always valid
}

int
Geofence::loadFromFile(const char *filename)
{
	FILE *fp;
	char line[120];
	int pointCounter = 0;
	bool gotVertical = false;
	const char commentChar = '#';
	int ret_val = PX4_ERROR;

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
			mission_fence_point_s vertex{};
			vertex.frame = NAV_FRAME_GLOBAL;
			vertex.nav_cmd = NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION;
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

			bool success = _dataman_client.writeSync(DM_KEY_FENCE_POINTS, pointCounter + 1, reinterpret_cast<uint8_t *>(&vertex),
					sizeof(vertex));

			if (!success) {
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
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Geofence imported\t");
		events::send(events::ID("navigator_geofence_imported"), events::Log::Info, "Geofence imported");
		ret_val = PX4_ERROR;

		/* do a second pass, now that we know the number of vertices */
		for (int seq = 1; seq <= pointCounter; ++seq) {
			mission_fence_point_s mission_fence_point;

			bool success = _dataman_client.readSync(DM_KEY_FENCE_POINTS, seq, reinterpret_cast<uint8_t *>(&mission_fence_point),
								sizeof(mission_fence_point_s));

			if (success) {
				mission_fence_point.vertex_count = pointCounter;
				_dataman_client.writeSync(DM_KEY_FENCE_POINTS, seq, reinterpret_cast<uint8_t *>(&mission_fence_point),
							  sizeof(mission_fence_point_s));
			}
		}

		mission_stats_entry_s stats;
		stats.num_items = pointCounter;
		stats.update_counter = _update_counter + 1;

		bool success = _dataman_client.writeSync(DM_KEY_FENCE_POINTS, 0, reinterpret_cast<uint8_t *>(&stats),
				sizeof(mission_stats_entry_s));

		if (success) {
			ret_val = PX4_OK;
		}

	} else {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence: import error\t");
		events::send(events::ID("navigator_geofence_import_failed"), events::Log::Error, "Geofence: import error");
	}

	updateFence();

error:
	fclose(fp);
	return ret_val;
}

int Geofence::clearDm()
{
	_dataman_client.clearSync(DM_KEY_FENCE_POINTS);
	updateFence();
	return PX4_OK;
}

bool Geofence::isHomeRequired()
{
	bool max_horizontal_enabled = (_param_gf_max_hor_dist.get() > FLT_EPSILON);
	bool max_vertical_enabled = (_param_gf_max_ver_dist.get() > FLT_EPSILON);
	bool geofence_action_rtl = (getGeofenceAction() == geofence_result_s::GF_ACTION_RTL);

	return max_horizontal_enabled || max_vertical_enabled || geofence_action_rtl;
}

void Geofence::printStatus()
{
	int num_inclusion_polygons = 0, num_exclusion_polygons = 0, total_num_vertices = 0;
	int num_inclusion_circles = 0, num_exclusion_circles = 0;

	for (int i = 0; i < _num_polygons; ++i) {
		total_num_vertices += _polygons[i].vertex_count;

		if (_polygons[i].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION) {
			++num_inclusion_polygons;
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION) {
			++num_exclusion_polygons;
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION) {
			++num_inclusion_circles;
		}

		if (_polygons[i].fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			++num_exclusion_circles;
		}
	}

	PX4_INFO("Geofence: %i inclusion, %i exclusion polygons, %i inclusion, %i exclusion circles, %i total vertices",
		 num_inclusion_polygons, num_exclusion_polygons, num_inclusion_circles, num_exclusion_circles,
		 total_num_vertices);
}
