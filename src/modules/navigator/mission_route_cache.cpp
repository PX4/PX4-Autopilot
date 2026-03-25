/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file mission_route_cache.cpp
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_cache.h"

#include <inttypes.h>

#include <px4_platform_common/log.h>

void MissionRouteCache::update(const mission_s &mission)
{
	updateMissionCache(mission);
	updateMissionLandItemCache(mission);
	updateSafePointCache(mission);
}

void MissionRouteCache::invalidate()
{
	_ready = false;
	_mission_too_large = false;
	_mission_id = 0;
	_mission_count = 0;
	_mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_dataman_cache_mission.invalidate();

	_mission_land_mission_id = 0;
	_mission_land_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
	_mission_land_index = -1;
	_dataman_cache_land_item.invalidate();

	_safe_point_dataman_state = SafePointDatamanState::UpdateRequestWait;
	_safe_point_error_state = SafePointDatamanState::UpdateRequestWait;
	_safe_point_stats = {};
	_safe_points_id = 0;
	_opaque_id = 0;
	_safe_points_ready = false;
	_initiate_safe_points_updated = true;
	_dataman_cache_safepoint.invalidate();
}

bool MissionRouteCache::missionExceedsCacheLimit(const mission_s &mission) const
{
	return mission.count > MAX_ROUTE_MISSION_CACHE_SIZE;
}

bool MissionRouteCache::isReady(const mission_s &mission) const
{
	return !_mission_too_large && _ready && missionMatchesCache(mission);
}

int MissionRouteCache::missionCount() const
{
	return (!_mission_too_large && _ready) ? _mission_count : 0;
}

bool MissionRouteCache::loadMissionItem(int index, mission_item_s &mission_item) const
{
	return !_mission_too_large
	       && _ready
	       && index >= 0
	       && index < _mission_count
	       && _dataman_cache_mission.loadWait(static_cast<dm_item_t>(_mission_dataman_id), index,
			       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
			       MAX_DATAMAN_LOAD_WAIT);
}

int MissionRouteCache::safePointCount() const
{
	return safePointsReady() ? _safe_point_stats.num_items : 0;
}

bool MissionRouteCache::loadSafePointItem(int index, mission_item_s &safe_point_item) const
{
	return safePointsReady()
	       && index >= 0
	       && index < _safe_point_stats.num_items
	       && _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point_stats.dataman_id), index,
			       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item),
			       MAX_DATAMAN_LOAD_WAIT);
}

bool MissionRouteCache::getMissionLandItem(int32_t &index, mission_item_s &land_item) const
{
	if (_mission_land_index < 0) {
		return false;
	}

	index = _mission_land_index;
	return _dataman_cache_land_item.loadWait(static_cast<dm_item_t>(_mission_land_dataman_id), index,
			reinterpret_cast<uint8_t *>(&land_item), sizeof(land_item),
			MAX_DATAMAN_LOAD_WAIT);
}

bool MissionRouteCache::loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const
{
	return isReady(mission) && loadMissionItem(index, mission_item);
}

bool MissionRouteCache::syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item)
{
	if (!isReady(mission) || index < 0 || index >= _mission_count) {
		return false;
	}

	bool mission_updated = _dataman_cache_mission.updateCachedItem(static_cast<dm_item_t>(_mission_dataman_id), index,
			       reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item));

	bool land_updated = true;

	if (index == _mission_land_index) {
		land_updated = _dataman_cache_land_item.updateCachedItem(static_cast<dm_item_t>(_mission_land_dataman_id), index,
				reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item));
	}

	return mission_updated && land_updated;
}

void MissionRouteCache::updateMissionCache(const mission_s &mission)
{
	const bool mission_changed = mission.mission_id != _mission_id
				     || mission.count != _mission_count
				     || mission.mission_dataman_id != _mission_dataman_id;

	if (mission_changed) {
		_mission_id = mission.mission_id;
		_mission_count = mission.count;
		_mission_dataman_id = mission.mission_dataman_id;
		_ready = mission.count == 0;
		_mission_too_large = missionExceedsCacheLimit(mission);
		_dataman_cache_mission.invalidate();

		if (!_mission_too_large && mission.count > 0) {
			if (static_cast<int32_t>(_dataman_cache_mission.size()) != mission.count) {
				_dataman_cache_mission.resize(mission.count);
			}

			const dm_item_t mission_dataman_id = static_cast<dm_item_t>(mission.mission_dataman_id);

			for (int32_t index = 0; index < mission.count; ++index) {
				_dataman_cache_mission.load(mission_dataman_id, index);
			}
		}
	}

	_dataman_cache_mission.update();

	if (!_ready && !_mission_too_large && mission.count > 0 && !_dataman_cache_mission.isLoading()) {
		_ready = true;
	}
}

void MissionRouteCache::updateMissionLandItemCache(const mission_s &mission)
{
	const bool mission_land_changed = mission.mission_id != _mission_land_mission_id
					  || mission.mission_dataman_id != _mission_land_dataman_id
					  || mission.land_index != _mission_land_index;

	if (mission_land_changed) {
		_mission_land_mission_id = mission.mission_id;
		_mission_land_dataman_id = mission.mission_dataman_id;
		_mission_land_index = mission.land_index;
		_dataman_cache_land_item.invalidate();

		if (_mission_land_index >= 0) {
			_dataman_cache_land_item.load(static_cast<dm_item_t>(_mission_land_dataman_id), _mission_land_index);
		}
	}

	_dataman_cache_land_item.update();
}

void MissionRouteCache::updateSafePointCache(const mission_s &mission)
{
	bool success = false;

	if (mission.safe_points_id != _safe_points_id) {
		_safe_points_id = mission.safe_points_id;
		_safe_points_ready = false;
		_initiate_safe_points_updated = true;
	}

	switch (_safe_point_dataman_state) {
	case SafePointDatamanState::UpdateRequestWait:
		if (_initiate_safe_points_updated) {
			_initiate_safe_points_updated = false;
			_safe_point_dataman_state = SafePointDatamanState::Read;
		}

		break;

	case SafePointDatamanState::Read:
		_safe_point_dataman_state = SafePointDatamanState::ReadWait;
		success = _dataman_client_safepoint.readAsync(DM_KEY_SAFE_POINTS_STATE, 0,
				reinterpret_cast<uint8_t *>(&_safe_point_stats), sizeof(mission_stats_entry_s));

		if (!success) {
			_safe_point_error_state = SafePointDatamanState::Read;
			_safe_point_dataman_state = SafePointDatamanState::Error;
		}

		break;

	case SafePointDatamanState::ReadWait:
		_dataman_client_safepoint.update();

		if (_dataman_client_safepoint.lastOperationCompleted(success)) {
			if (!success) {
				_safe_point_error_state = SafePointDatamanState::ReadWait;
				_safe_point_dataman_state = SafePointDatamanState::Error;

			} else if (_safe_point_stats.num_items > DM_KEY_SAFE_POINTS_MAX) {
				PX4_ERR("Safe points update failed! invalid count: %" PRIu16, _safe_point_stats.num_items);
				_safe_point_error_state = SafePointDatamanState::ReadWait;
				_safe_point_dataman_state = SafePointDatamanState::Error;

			} else if (_opaque_id != _safe_point_stats.opaque_id) {
				_opaque_id = _safe_point_stats.opaque_id;
				_safe_points_ready = false;
				_dataman_cache_safepoint.invalidate();

				if (_dataman_cache_safepoint.size() != _safe_point_stats.num_items) {
					_dataman_cache_safepoint.resize(_safe_point_stats.num_items);
				}

				for (int index = 0; index < _dataman_cache_safepoint.size(); ++index) {
					_dataman_cache_safepoint.load(static_cast<dm_item_t>(_safe_point_stats.dataman_id), index);
				}

				_safe_point_dataman_state = (_safe_point_stats.num_items > 0)
							    ? SafePointDatamanState::Load
							    : SafePointDatamanState::UpdateRequestWait;

				if (_safe_point_dataman_state == SafePointDatamanState::UpdateRequestWait) {
					_safe_points_ready = true;
				}

			} else {
				_safe_point_dataman_state = SafePointDatamanState::UpdateRequestWait;
				_safe_points_ready = true;
			}
		}

		break;

	case SafePointDatamanState::Load:
		_dataman_cache_safepoint.update();

		if (!_dataman_cache_safepoint.isLoading()) {
			_safe_point_dataman_state = SafePointDatamanState::UpdateRequestWait;
			_safe_points_ready = true;
		}

		break;

	case SafePointDatamanState::Error:
		PX4_ERR("Safe points update failed! state: %" PRIu8, static_cast<uint8_t>(_safe_point_error_state));
		_safe_point_dataman_state = SafePointDatamanState::UpdateRequestWait;
		break;
	}
}

bool MissionRouteCache::missionMatchesCache(const mission_s &mission) const
{
	return mission.mission_id == _mission_id
	       && mission.count == _mission_count
	       && mission.mission_dataman_id == _mission_dataman_id;
}
