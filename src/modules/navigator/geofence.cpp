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

#include <dataman/dataman.h>
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
	// we assume there's no concurrent fence update on startup
	if (_navigator != nullptr) {
		_updateFence();
	}
}

Geofence::~Geofence()
{
	if (_polygons) {
		delete[](_polygons);
	}
}

void Geofence::updateFence()
{
	// Note: be aware that when calling this, it can block for quite some time, the duration of a geofence transfer.
	// However this is currently not used
	if (dm_lock(DM_KEY_FENCE_POINTS) != 0) {
		PX4_ERR("lock failed");
		return;
	}

	_updateFence();
	dm_unlock(DM_KEY_FENCE_POINTS);
}

void Geofence::_updateFence()
{
	// initialize fence points count
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_fence_items = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_fence_items = stats.num_items;
		_update_counter = stats.update_counter;
	}

	// iterate over all polygons and store their starting vertices
	_num_polygons = 0;
	_has_rtl_action = false;
	_has_default_action = false;
	int current_seq = 1;

	while (current_seq <= num_fence_items) {
		mission_fence_point_s mission_fence_point;
		bool is_circle_area = false;

		if (dm_read(DM_KEY_FENCE_POINTS, current_seq, &mission_fence_point, sizeof(mission_fence_point_s)) !=
		    sizeof(mission_fence_point_s)) {
			PX4_ERR("dm_read failed");
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
				polygon.fence_action = mission_fence_point.fence_action;
				polygon.max_alt = mission_fence_point.alt;

				if (is_circle_area) {
					polygon.circle_radius = mission_fence_point.circle_radius;
					current_seq += 1;

				} else {
					polygon.vertex_count = mission_fence_point.vertex_count;
					current_seq += mission_fence_point.vertex_count;
				}

				if (polygon.fence_action == geofence_result_s::GF_ACTION_RTL) {
					_has_rtl_action = true;
				}

				if (polygon.fence_action == geofence_result_s::GF_ACTION_DEFAULT) {
					_has_default_action = true;
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

bool Geofence::checkAll(const struct vehicle_global_position_s &global_position, uint8_t *breach_action)
{
	return checkAll(global_position.lat, global_position.lon, global_position.alt, breach_action);
}

bool Geofence::checkAll(const struct vehicle_global_position_s &global_position, const float alt,
			uint8_t *breach_action)
{
	return checkAll(global_position.lat, global_position.lon, alt, breach_action);
}

bool Geofence::checkAll(double lat, double lon, float altitude, uint8_t *breach_action)
{
	bool max_altitude_exceeded = false;  // Not used in this function
	bool lateral_breach = false;  // Not used in this function
	bool inside_fence = isInsideFence(lat, lon, altitude, &lateral_breach, &max_altitude_exceeded, breach_action);

	if (!isCloserThanMaxDistToHome(lat, lon, altitude) || !isBelowMaxAltitude(altitude)) {
		inside_fence = false;
		// Update action if more severe than existing
		*breach_action = math::max(*breach_action, legacyActionTranslator(_param_gf_action.get()));
	}

	return inside_fence;
}

bool Geofence::check(const vehicle_global_position_s &global_position, const vehicle_gps_position_s &gps_position,
		     uint8_t *breach_action)
{
	if (_param_gf_altmode.get() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position, breach_action);

		} else {
			return checkAll(gps_position.lat * 1.0e-7, gps_position.lon * 1.0e-7, gps_position.alt * 1.0e-3, breach_action);
		}

	} else {
		// get baro altitude
		_sub_airdata.update();
		const float baro_altitude_amsl = _sub_airdata.get().baro_alt_meter;

		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return checkAll(global_position, baro_altitude_amsl, breach_action);

		} else {
			return checkAll(gps_position.lat * 1.0e-7, gps_position.lon * 1.0e-7, baro_altitude_amsl, breach_action);
		}
	}
}

bool Geofence::check(const struct mission_item_s &mission_item, uint8_t *breach_action)
{
	return checkAll(mission_item.lat, mission_item.lon, mission_item.altitude, breach_action);
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

bool Geofence::isInsideFence(double lat, double lon, float altitude, bool *lateral_breach, bool *max_altitude_exceeded,
			     uint8_t *breach_action)
{
	// Set default for these first, so we have defined values if e.g. dm is locked and we return early
	// These values will refere to the most severe breach only
	*breach_action = geofence_result_s::GF_ACTION_NONE;
	*max_altitude_exceeded = false;
	*lateral_breach = false;

	// the following uses dm_read, so first we try to lock all items. If that fails, it (most likely) means
	// the data is currently being updated (via a mavlink geofence transfer), and we do not check for a violation now
	if (dm_trylock(DM_KEY_FENCE_POINTS) != 0) {
		return true;
	}

	// we got the lock, now check if the fence data got updated
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s) && _update_counter != stats.update_counter) {
		_updateFence();
	}

	if (isEmpty()) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		/* Empty fence -> accept all points */
		return true;
	}

	for (int polygon_index = 0; polygon_index < _num_polygons; ++polygon_index) {

		bool fence_max_altitude_exceeded = false;
		bool fence_lateral_breached = false;

		if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_CIRCLE_INCLUSION) {
			fence_lateral_breached = !insideCircle(_polygons[polygon_index], lat, lon, altitude);
			fence_max_altitude_exceeded = ((static_cast<int32_t>(_polygons[polygon_index].max_alt) != DISABLED_MAX_ALTITUDE_CHECK)
						       && (altitude > _polygons[polygon_index].max_alt));

		} else if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_CIRCLE_EXCLUSION) {
			fence_lateral_breached = insideCircle(_polygons[polygon_index], lat, lon, altitude);

		} else if (_polygons[polygon_index].fence_type == NAV_CMD_FENCE_POLYGON_VERTEX_INCLUSION) {
			fence_lateral_breached = !insidePolygon(_polygons[polygon_index], lat, lon, altitude);
			fence_max_altitude_exceeded = ((static_cast<int32_t>(_polygons[polygon_index].max_alt) != DISABLED_MAX_ALTITUDE_CHECK)
						       && (altitude > _polygons[polygon_index].max_alt));

		} else { // NAV_CMD_FENCE_POLYGON_VERTEX_EXCLUSION
			fence_lateral_breached = insidePolygon(_polygons[polygon_index], lat, lon, altitude);
		}

		if (fence_max_altitude_exceeded || fence_lateral_breached) {
			uint8_t current_fence_action = geofence_result_s::GF_ACTION_NONE;

			if (geofence_result_s::GF_ACTION_DEFAULT == _polygons[polygon_index].fence_action) {
				current_fence_action = legacyActionTranslator(_param_gf_action.get());

			} else {
				current_fence_action = _polygons[polygon_index].fence_action;
			}

			if (*breach_action <= current_fence_action) {
				// Use breach action and type of breach if at least as severe as any other breach found this iteration
				*breach_action = current_fence_action;
				*max_altitude_exceeded = fence_max_altitude_exceeded;
				*lateral_breach = fence_lateral_breached;
			}
		}
	}

	dm_unlock(DM_KEY_FENCE_POINTS);

	return !(*max_altitude_exceeded || *lateral_breach);
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
		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + i, &temp_vertex_i,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
			break;
		}

		if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index + j, &temp_vertex_j,
			    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
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

	if (dm_read(DM_KEY_FENCE_POINTS, polygon.dataman_index, &circle_point,
		    sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s)) {
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

int Geofence::clearDm()
{
	dm_clear(DM_KEY_FENCE_POINTS);
	updateFence();
	return PX4_OK;
}

uint8_t Geofence::legacyActionTranslator(uint8_t param_action)
{
	uint8_t actual_action = geofence_result_s::GF_ACTION_NONE;

	switch (param_action) {
	case GF_PARAM_ACTION_NONE:
		actual_action = geofence_result_s::GF_ACTION_NONE;
		break;

	case GF_PARAM_ACTION_WARNING:
		actual_action = geofence_result_s::GF_ACTION_WARN;
		break;

	case GF_PARAM_ACTION_HOLD_MODE:
		actual_action = geofence_result_s::GF_ACTION_LOITER;
		break;

	case GF_PARAM_ACTION_RETURN_MODE:
		actual_action = geofence_result_s::GF_ACTION_RTL;
		break;

	case GF_PARAM_ACTION_TERMINATE:
		actual_action = geofence_result_s::GF_ACTION_TERMINATE;
		break;

	case GF_PARAM_ACTION_LAND_MODE:
		actual_action = geofence_result_s::GF_ACTION_LAND;
		break;

	default:
		break;
	}

	return actual_action;
}

bool Geofence::isHomeRequired()
{
	bool max_horizontal_enabled = (_param_gf_max_hor_dist.get() > FLT_EPSILON);
	bool max_vertical_enabled = (_param_gf_max_ver_dist.get() > FLT_EPSILON);

	bool has_default_rtl_actions = _has_default_action
				       && legacyActionTranslator(_param_gf_action.get()) == geofence_result_s::GF_ACTION_RTL;

	return max_horizontal_enabled || max_vertical_enabled || _has_rtl_action || has_default_rtl_actions;
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

bool Geofence::validateAction(uint8_t action)
{
	switch (action) {
	case geofence_result_s::GF_ACTION_DEFAULT:
	case geofence_result_s::GF_ACTION_NONE:
	case geofence_result_s::GF_ACTION_WARN:
	case geofence_result_s::GF_ACTION_LOITER:
	case geofence_result_s::GF_ACTION_RTL:
	case geofence_result_s::GF_ACTION_LAND:
	case geofence_result_s::GF_ACTION_TERMINATE:
		return true;

	default:
		return false;
	}
}
