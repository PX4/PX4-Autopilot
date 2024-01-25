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

#pragma once

#include <float.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_air_data.h>

#define GEOFENCE_FILENAME PX4_STORAGEDIR"/etc/geofence.txt"

class Navigator;

class Geofence : public ModuleParams
{
public:
	Geofence(Navigator *navigator);
	Geofence(const Geofence &) = delete;
	Geofence &operator=(const Geofence &) = delete;
	virtual ~Geofence();

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

	/* Legacy actions from parameters */
	enum {
		GF_PARAM_ACTION_NONE = 0,
		GF_PARAM_ACTION_WARNING = 1,
		GF_PARAM_ACTION_HOLD_MODE = 2,
		GF_PARAM_ACTION_RETURN_MODE = 3,
		GF_PARAM_ACTION_TERMINATE = 4,
		GF_PARAM_ACTION_LAND_MODE = 5
	};

	/**
	 * update the geofence from dataman.
	 * It's generally not necessary to call this as it will automatically update when the data is changed.
	 */
	void updateFence();

	/**
	 * Return whether the system obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const vehicle_global_position_s &global_position, const vehicle_gps_position_s &gps_position,
		   uint8_t *breach_action);

	/**
	 * Return whether a mission item obeys the geofence.
	 *
	 * @return true: system is obeying fence, false: system is violating fence
	 */
	bool check(const struct mission_item_s &mission_item, uint8_t *breach_action);

	/**
	 * Check if a point passes the Geofence test.
	 * In addition to checkPolygons(), this takes all additional parameters into account.
	 *
	 * @return false for a geofence violation
	 */
	bool checkAll(double lat, double lon, float altitude, uint8_t *breach_action);

	bool isCloserThanMaxDistToHome(double lat, double lon, float altitude);

	bool isBelowMaxAltitude(float altitude);

	virtual bool isInsideFence(double lat, double lon, float altitude, bool *lateral_breach, bool *max_altitude_exceeded,
				   uint8_t *breach_action);

	int clearDm();

	bool valid();

	bool isEmpty() { return _num_polygons == 0; }

	int getSource() { return _param_gf_source.get(); }

	float getMaxHorDistanceHome() { return _param_gf_max_hor_dist.get(); }
	float getMaxVerDistanceHome() { return _param_gf_max_ver_dist.get(); }
	bool getPredict() { return _param_gf_predict.get(); }

	uint8_t legacyActionTranslator(uint8_t param_action);
	uint8_t getDefaultAction() { return legacyActionTranslator(_param_gf_action.get()); }
	bool isHomeRequired();

	/**
	 * print Geofence status to the console
	 */
	void printStatus();

	/**
	 * Checks if the given argument is a valid geofence action (one of GF_ACTION_*).
	 *
	 * @return true if action is valid
	 */
	static bool validateAction(uint8_t action);

private:

	struct PolygonInfo {
		uint16_t fence_type;	///< one of MAV_CMD_NAV_FENCE_* (can also be a circular region)
		uint8_t fence_action;	///< fence action when this fence is breached
		float max_alt;		///< Maximum altitude (AMSL) for polygon
		uint16_t dataman_index;
		union {
			uint16_t vertex_count;
			float circle_radius;
		};
	};

	Navigator   *_navigator{nullptr};
	PolygonInfo *_polygons{nullptr};

	hrt_abstime _last_horizontal_range_warning{0};
	hrt_abstime _last_vertical_range_warning{0};

	int _num_polygons{0};
	bool _has_rtl_action{false}; ///< at least one of the fences has GF_ACTION_RTL
	bool _has_default_action{false}; ///< at least one of the fences has GF_ACTION_DEFAULT

	MapProjection _projection_reference{}; ///< class to convert (lon, lat) to local [m]

	uORB::SubscriptionData<vehicle_air_data_s> _sub_airdata;

	uint16_t _update_counter{0}; ///< dataman update counter: if it does not match, we polygon data was updated

	/**
	 * implementation of updateFence(), but without locking
	 */
	void _updateFence();

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



	bool checkAll(const vehicle_global_position_s &global_position, uint8_t *breach_action);
	bool checkAll(const vehicle_global_position_s &global_position, float baro_altitude_amsl, uint8_t *breach_action);

	/**
	 * Check if a single point is within a polygon
	 * @return true if within polygon
	 */
	bool insidePolygon(const PolygonInfo &polygon, double lat, double lon, float altitude);

	/**
	 * Check if a single point is within a circle
	 * @param polygon must be a circle!
	 * @return true if within polygon the circle
	 */
	bool insideCircle(const PolygonInfo &polygon, double lat, double lon, float altitude);

	static constexpr int32_t DISABLED_MAX_ALTITUDE_CHECK = 0;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::GF_ACTION>)         _param_gf_action,
		(ParamInt<px4::params::GF_ALTMODE>)        _param_gf_altmode,
		(ParamInt<px4::params::GF_SOURCE>)         _param_gf_source,
		(ParamFloat<px4::params::GF_MAX_HOR_DIST>) _param_gf_max_hor_dist,
		(ParamFloat<px4::params::GF_MAX_VER_DIST>) _param_gf_max_ver_dist,
		(ParamBool<px4::params::GF_PREDICT>)       _param_gf_predict
	)
};
