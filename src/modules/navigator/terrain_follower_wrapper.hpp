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

#include "TerrainFollower/terrain_follower.hpp"

class TerrainFollowerWrapper
{
public:
	TerrainFollowerWrapper() = default;

	~TerrainFollowerWrapper();

	void setCurrentMissionItem(const mission_item_s &current_mission_item);

	void setCurrentPosition(const matrix::Vector2<double> &pos_lat_lon, float alt_amsl);

	void setHomeAltitude(float home_alt_m);

	void setTerrainProvider(terrain::TerrainProvider *terrain_provider);

	struct mission_item_s getIntermediateMissionItem();

	bool updateIntermediateMissionItem(bool mission_item_reached);

	bool hasIntermediateMissionItem();

	mission_item_s getCurrentMissionItem();

	bool isCurrentMissionItemValid();

	void reset();

	void setGroundSpeed(float ground_speed_m_s);

	void setIsHoverCraft(bool is_hovercraft);

	void setLoiterRadius(float radius);

	void updateParams();

	void enable();

private:

	TerrainFollower *_terrain_follower = nullptr;

};