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

#include "terrain_follower_wrapper.hpp"

TerrainFollowerWrapper::~TerrainFollowerWrapper()
{
	delete _terrain_follower;
}

void TerrainFollowerWrapper::enable()
{
	if (!_terrain_follower) {
		_terrain_follower = new TerrainFollower();
	}
}

void TerrainFollowerWrapper::setCurrentMissionItem(const mission_item_s &current_mission_item)
{
	if (_terrain_follower) {
		_terrain_follower->setCurrentMissionItem(current_mission_item);
	}
}


void TerrainFollowerWrapper::setCurrentPosition(const matrix::Vector2<double> &pos_lat_lon, float alt_amsl)
{
	if (_terrain_follower) {
		_terrain_follower->setCurrentPosition(pos_lat_lon, alt_amsl);
	}
}

void TerrainFollowerWrapper::setHomeAltitude(float home_alt_m)
{
	if (_terrain_follower) {
		_terrain_follower->setHomeAltitude(home_alt_m);
	}
}

void TerrainFollowerWrapper::setTerrainProvider(terrain::TerrainProvider *terrain_provider)
{
	if (_terrain_follower) {
		_terrain_follower->setTerrainProvider(terrain_provider);
	}
}

mission_item_s TerrainFollowerWrapper::getIntermediateMissionItem()
{
	if (_terrain_follower) {
		return _terrain_follower->getIntermediateMissionItem();

	} else {
		mission_item_s empty = {};
		return empty;
	}
}

bool TerrainFollowerWrapper::updateIntermediateMissionItem(bool mission_item_reached)
{
	if (_terrain_follower) {
		return _terrain_follower->updateIntermediateMissionItem(mission_item_reached);

	} else {
		return false;
	}
}

bool TerrainFollowerWrapper::hasIntermediateMissionItem()
{
	if (_terrain_follower) {
		return _terrain_follower->hasIntermediateMissionItem();

	} else {
		return false;
	}
}

mission_item_s TerrainFollowerWrapper::getCurrentMissionItem()
{
	if (_terrain_follower) {
		return _terrain_follower->getCurrentMissionItem();

	} else {
		mission_item_s empty = {};
		return empty;
	}
}

bool TerrainFollowerWrapper::isCurrentMissionItemValid()
{
	if (_terrain_follower) {
		return _terrain_follower->isCurrentMissionItemValid();

	} else {
		return false;
	}
}

void TerrainFollowerWrapper::reset()
{
	if (_terrain_follower) {
		_terrain_follower->reset();
	}
}

void TerrainFollowerWrapper::setGroundSpeed(float ground_speed_m_s)
{
	if (_terrain_follower) {
		_terrain_follower->setGroundSpeed(ground_speed_m_s);
	}
}

void TerrainFollowerWrapper::setIsHoverCraft(bool is_hovercraft)
{
	if (_terrain_follower) {
		_terrain_follower->setIsHoverCraft(is_hovercraft);
	}
}

void TerrainFollowerWrapper::setLoiterRadius(float radius)
{
	if (_terrain_follower) {
		_terrain_follower->setLoiterRadius(radius);
	}
}
void TerrainFollowerWrapper::updateParams()
{
	if (_terrain_follower) {
		_terrain_follower->updateParams();
	}
}