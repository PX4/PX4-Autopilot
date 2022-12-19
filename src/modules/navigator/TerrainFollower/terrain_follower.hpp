/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include "../navigation.h"
#include <px4_platform_common/defines.h>
#include <lib/terrain/terrain_provider.h>

class TerrainFollower : public ModuleParams
{
public:
	TerrainFollower();

	~TerrainFollower();

	void setCurrentMissionItem(const mission_item_s &current_mission_item);

	void setCurrentPosition(const matrix::Vector2<double> &pos_lat_lon, float alt_amsl) { _pos_lat_lon = pos_lat_lon; _alt_amsl_m = alt_amsl; }

	void setHomeAltitude(float home_alt_m) { _alt_home_amsl_m = home_alt_m; }

	void setTerrainProvider(terrain::TerrainProvider *terrain_provider) { _terrain_provider = terrain_provider; }

	mission_item_s getIntermediateMissionItem() { return _intermediate_mission_item; }

	bool updateIntermediateMissionItem(bool mission_item_reached);

	bool hasIntermediateMissionItem() { return _has_intermediate_mission_item; }

	mission_item_s getCurrentMissionItem() { return _current_mission_item; }

	bool isCurrentMissionItemValid() { return _current_mission_item_valid; }

	void reset();

	void setGroundSpeed(float ground_speed_m_s) { _ground_speed_m_s = ground_speed_m_s; }

	void setIsHoverCraft(bool is_hovercraft) { _is_hovercraft = is_hovercraft; }

	void setLoiterRadius(float radius) { _loiter_radius = radius; }

	void updateParams() override;


private:

	matrix::Vector2<double> _pos_lat_lon = {};	// current position of the vehicle
	float _alt_amsl_m{0.0f};
	float _alt_home_amsl_m{0.0f};
	float _ground_speed_m_s{0.0f};

	float _loiter_radius{0.0f};

	mission_item_s _current_mission_item = {}; // the current mission item from the mission plan
	struct mission_item_s _intermediate_mission_item = {};
	struct mission_item_s _previous_mission_item = {};

	bool _current_mission_item_valid = false;	// we have a target mission item from the navigator set
	bool _has_intermediate_mission_item =
		false;	// we are generating intermediate waypoints in between our location and _current_mission_item
	bool _had_first_mission_item = false;

	bool _is_hovercraft = true;

	float _dist_to_current_mission_item{0.0f};	// distance from the vehicle to _current_mission_item

	terrain::TerrainProvider *_terrain_provider = nullptr;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TF_CHECK_DIST>) _param_check_dist,
		(ParamFloat<px4::params::TF_TERR_DIST_MIN>) _param_terrain_dist_min,
		(ParamFloat<px4::params::TF_TERR_DIST_MAX>) _param_terrain_dist_max
	)

	param_t param_handle_fw_climbrate_max;
	float param_fw_climbrate_max{0.0f};

	bool getDistanceToTerrainAtLocation(const matrix::Vector2<double> &check_point_lat_lon, float &dist_to_terrain);

	float getAltitudeAtCheckPoint(const matrix::Vector2<double> &check_point_lat_lon);

	float getAltitudeGradient(const matrix::Vector2<double> &A, const matrix::Vector2<double> &B, const float alt_A,
				  const float altB);

	matrix::Vector2<double> getClosestPointOnMissionPath();

	matrix::Vector2<double> calculateCheckpoint();

	bool needAltitudeUpdate(const matrix::Vector2<double> &check_point_lat_lon, float &new_altitude);
};
