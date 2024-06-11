/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include <dataman_client/DatamanClient.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>

#define GEOFENCE_FILENAME PX4_STORAGEDIR"/etc/geofence.txt"

class Navigator;

class Geofence : public ModuleParams
{
public:
	Geofence(Navigator *navigator);
	Geofence(const Geofence &) = delete;
	Geofence &operator=(const Geofence &) = delete;
	virtual ~Geofence();

	/* Source, corresponding to the param GF_SOURCE */
	enum {
		GF_SOURCE_GLOBALPOS = 0,
		GF_SOURCE_GPS = 1
	};

	/**
	 * @brief function to call regularly to do background work
	 */
	void run();

	/**
	 * update the geofence from dataman.
	 */
	void updateFence();


	/**
	 * Check if a 3D point passes the Geofence test.
	 * Checks max distance, max altitude, inside polygon or circle.
	 * In addition to checkPolygons(), this takes all additional parameters into account.
	 *
	 * @return false for a geofence violation
	 */
	bool checkPointAgainstAllGeofences(double lat, double lon, float altitude);

	/**
	 * @brief check if the horizontal distance to Home is greater than the maximum allowed distance
	 *
	 * @return true if the horizontal distance to Home is smaller than the maximum allowed distance
	 */
	bool isCloserThanMaxDistToHome(double lat, double lon, float altitude);


	/**
	 * @brief check if the altitude above Home is greater than the maximum allowed altitude
	 *
	 * @return true if the altitude above Home is smaller than the maximum allowed altitude
	 */
	bool isBelowMaxAltitude(float altitude);

	virtual bool isInsidePolygonOrCircle(double lat, double lon, float altitude);

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

	bool isEmpty() { return (!_fence_updated || (_num_polygons == 0)); }

	int getSource() { return _param_gf_source.get(); }
	int getGeofenceAction() { return _param_gf_action.get(); }

	float getMaxHorDistanceHome() { return _param_gf_max_hor_dist.get(); }
	bool getPredict() { return _param_gf_predict.get(); }

	bool isHomeRequired();

	/**
	 * print Geofence status to the console
	 */
	void printStatus();

private:

	enum class DatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	struct PolygonInfo {
		uint16_t fence_type; ///< one of MAV_CMD_NAV_FENCE_* (can also be a circular region)
		uint16_t dataman_index;
		union {
			uint16_t vertex_count;
			float circle_radius;
		};
	};

	Navigator   *_navigator{nullptr};
	PolygonInfo *_polygons{nullptr};

	mission_stats_entry_s _stats;
	DatamanState _dataman_state{DatamanState::UpdateRequestWait};
	DatamanState _error_state{DatamanState::UpdateRequestWait};
	DatamanCache _dataman_cache{"geofence_dm_cache_miss", 0};
	DatamanClient	&_dataman_client = _dataman_cache.client();

	float _altitude_min{0.0f};
	float _altitude_max{0.0f};

	int _num_polygons{0};

	MapProjection _projection_reference{}; ///< class to convert (lon, lat) to local [m]

	uint32_t _opaque_id{0}; ///< dataman geofence id: if it does not match, the polygon data was updated
	bool _fence_updated{true};  ///< flag indicating if fence are updated to dataman cache
	bool _initiate_fence_updated{true}; ///< flag indicating if fence updated is needed

	uORB::Publication<geofence_status_s> _geofence_status_pub{ORB_ID(geofence_status)};

	/**
	 * implementation of updateFence()
	 */
	void _updateFence();


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

	/**
	 * Check if a single point is within a polygon or circle
	 * @return true if within polygon or circle
	 */

	bool checkPointAgainstPolygonCircle(const PolygonInfo &polygon, double lat, double lon, float altitude);

	/**
	 * Check polygon or circle geofence fullfills the requirements relative to Home.
	 * @return true if checks pass
	 */
	bool checkHomeRequirementsForGeofence(const PolygonInfo &polygon);

	/**
	 * Check polygon or circle geofence fullfills the requirements relative to the current vehicle position.
	 * @return true if checks pass
	 */
	bool checkCurrentPositionRequirementsForGeofence(const PolygonInfo &polygon);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::GF_ACTION>)         _param_gf_action,
		(ParamInt<px4::params::GF_SOURCE>)         _param_gf_source,
		(ParamFloat<px4::params::GF_MAX_HOR_DIST>) _param_gf_max_hor_dist,
		(ParamFloat<px4::params::GF_MAX_VER_DIST>) _param_gf_max_ver_dist,
		(ParamBool<px4::params::GF_PREDICT>)       _param_gf_predict
	)
};
