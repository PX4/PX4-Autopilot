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
 * Shared dataman-backed cache for mission geometry, mission-land items, and
 * safe points consumed by MissionRoutePlanner.
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
	_mission = {};
	_dataman_cache_mission.invalidate();

	_mission_land = {};
	_dataman_cache_land_item.invalidate();

	resetSafePointCacheState(true);
}

bool MissionRouteCache::queueMissionCacheLoads(const mission_s &mission)
{
	if (_mission.too_large || mission.count <= 0) {
		return false;
	}

	// Rebuild the mission cache from scratch, callers retry the full queue on failure.
	_dataman_cache_mission.invalidate();

	if (static_cast<int32_t>(_dataman_cache_mission.size()) != mission.count) {
		_dataman_cache_mission.resize(mission.count);
	}

	if (static_cast<int32_t>(_dataman_cache_mission.size()) != mission.count) {
		PX4_ERR("Mission cache resize failed! requested: %d, actual: %d",
			static_cast<int>(mission.count), static_cast<int>(_dataman_cache_mission.size()));
		return false;
	}

	const dm_item_t mission_dataman_id = static_cast<dm_item_t>(mission.mission_dataman_id);

	for (int32_t index = 0; index < mission.count; ++index) {
		if (!_dataman_cache_mission.load(mission_dataman_id, index)) {
			PX4_ERR("Mission cache queue failed! item=%" PRIu8 ", index=%" PRIi32,
				static_cast<uint8_t>(mission_dataman_id), index);
			_dataman_cache_mission.invalidate();
			return false;
		}
	}

	return true;
}

hrt_abstime MissionRouteCache::nextRetryBackoff(uint8_t retry_count)
{
	const uint8_t backoff_shift = (retry_count < MAX_RETRY_BACKOFF_SHIFT) ? retry_count : MAX_RETRY_BACKOFF_SHIFT;
	return CACHE_RETRY_BACKOFF << backoff_shift;
}

bool MissionRouteCache::missionCacheFullyLoaded(const mission_s &mission) const
{
	if (!missionMatchesCache(mission)) {
		return false;
	}

	if (mission.count == 0) {
		return true;
	}

	mission_item_s mission_item{};

	// A miss means async preloading did not finish cleanly.
	for (int32_t index = 0; index < mission.count; ++index) {
		if (!_dataman_cache_mission.loadWait(static_cast<dm_item_t>(_mission.dataman_id), index,
						     reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item), 0)) {
			return false;
		}
	}

	return true;
}

bool MissionRouteCache::safePointCacheFullyLoaded() const
{
	if (_safe_point.stats.num_items == 0) {
		return true;
	}

	mission_item_s safe_point_item{};

	// A miss means async preloading did not finish cleanly.
	for (int32_t index = 0; index < _safe_point.stats.num_items; ++index) {
		if (!_dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point.stats.dataman_id), index,
						       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item), 0)) {
			return false;
		}
	}

	return true;
}

void MissionRouteCache::resetSafePointCacheState(bool clear_source_identity)
{
	// Drop any in-flight response before resetting the local state machine.
	_dataman_client_safepoint.abortCurrentOperation();
	_dataman_cache_safepoint.invalidate();

	const uint32_t source_id = clear_source_identity ? 0 : _safe_point.source_id;
	_safe_point = {};
	_safe_point.source_id = source_id;
}

bool MissionRouteCache::missionExceedsCacheLimit(const mission_s &mission) const
{
	return mission.count > MAX_ROUTE_MISSION_CACHE_SIZE;
}

bool MissionRouteCache::isReady(const mission_s &mission) const
{
	return !_mission.too_large && _mission.ready && missionMatchesCache(mission);
}

int MissionRouteCache::missionCount() const
{
	return (!_mission.too_large && _mission.ready) ? _mission.count : 0;
}

bool MissionRouteCache::loadMissionItem(int index, mission_item_s &mission_item) const
{
	return !_mission.too_large
	       && _mission.ready
	       && index >= 0 && index < _mission.count
	       && _dataman_cache_mission.loadWait(static_cast<dm_item_t>(_mission.dataman_id), index,
			       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
			       MAX_DATAMAN_LOAD_WAIT);
}

int MissionRouteCache::safePointCount() const
{
	return safePointsReady() ? _safe_point.stats.num_items : 0;
}

bool MissionRouteCache::loadSafePointItem(int index, mission_item_s &safe_point_item) const
{
	return safePointsReady()
	       && index >= 0 && index < _safe_point.stats.num_items
	       && _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point.stats.dataman_id), index,
			       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item),
			       MAX_DATAMAN_LOAD_WAIT);
}

bool MissionRouteCache::getMissionLandItem(int32_t &index, mission_item_s &land_item) const
{
	if (_mission_land.index < 0 || _mission_land.index >= _mission.count) {
		return false;
	}

	index = _mission_land.index;
	return _dataman_cache_land_item.loadWait(static_cast<dm_item_t>(_mission_land.dataman_id), index,
			reinterpret_cast<uint8_t *>(&land_item), sizeof(land_item),
			MAX_DATAMAN_LOAD_WAIT);
}

bool MissionRouteCache::loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const
{
	return isReady(mission) && loadMissionItem(index, mission_item);
}

bool MissionRouteCache::syncMissionItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item)
{
	if (!isReady(mission) || index < 0 || index >= _mission.count) {
		return false;
	}

	bool mission_updated = _dataman_cache_mission.updateCachedItem(static_cast<dm_item_t>(_mission.dataman_id), index,
			       reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item));

	if (!mission_updated) {
		return false;
	}

	if (index == _mission_land.index) {
		if (_mission_land.dataman_id != _mission.dataman_id) {
			PX4_ERR("Mission land cache dataman id mismatch! land=%" PRIu8 ", mission=%" PRIu8,
				_mission_land.dataman_id, _mission.dataman_id);
			return false;
		}

		return _dataman_cache_land_item.updateCachedItem(static_cast<dm_item_t>(_mission_land.dataman_id), index,
				reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item));
	}

	return true;
}

void MissionRouteCache::updateMissionCache(const mission_s &mission)
{
	MissionCacheState &state = _mission;
	const bool mission_changed = mission.mission_id != state.id
				     || mission.count != state.count
				     || mission.mission_dataman_id != state.dataman_id;

	if (mission_changed) {
		// A new mission id always starts a fresh cache/validation cycle.
		state = {};
		state.id = mission.mission_id;
		state.count = mission.count;
		state.dataman_id = mission.mission_dataman_id;
		state.ready = mission.count == 0;
		state.too_large = missionExceedsCacheLimit(mission);

		if (!state.too_large && mission.count > 0) {
			state.validation_pending = queueMissionCacheLoads(mission);

			if (!state.validation_pending) {
				PX4_WARN("Mission route cache queue failed, retrying mission_id=%" PRIu32, mission.mission_id);
				state.retry_at = hrt_absolute_time() + nextRetryBackoff(state.retry_count);

				if (state.retry_count < UINT8_MAX) {
					++state.retry_count;
				}
			}
		}
	}

	_dataman_cache_mission.update();

	// Cache ready, early return
	if (state.ready || state.too_large || mission.count <= 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Wait for cache load and validation
	if (state.validation_pending) {
		if (_dataman_cache_mission.isLoading()) {
			return;
		}

		// Loading it completed, clear pending flag
		state.validation_pending = false;

		// O(N) check on every item cached into the RAM
		if (missionCacheFullyLoaded(mission)) {
			state.ready = true;
			state.retry_at = 0;
			state.retry_count = 0;

		} else {
			PX4_WARN("Mission route cache incomplete, retrying mission_id=%" PRIu32, mission.mission_id);
			state.retry_at = now + nextRetryBackoff(state.retry_count);

			// Cap the retry count to prevent overflow in nextRetryBackoff
			if (state.retry_count < UINT8_MAX) {
				++state.retry_count;
			}
		}

		// Error handling

	} else if (state.retry_at != 0 && now >= state.retry_at) {
		state.validation_pending = queueMissionCacheLoads(mission);

		if (state.validation_pending) {
			state.retry_at = 0; // retry succeeded, clear error

		} else {
			PX4_WARN("Mission route cache queue failed, retrying mission_id=%" PRIu32, mission.mission_id);
			state.retry_at = now + nextRetryBackoff(state.retry_count);

			if (state.retry_count < UINT8_MAX) {
				++state.retry_count;
			}
		}
	}
}

void MissionRouteCache::updateMissionLandItemCache(const mission_s &mission)
{
	MissionLandState &state = _mission_land;
	// Trust the published land_index, no mission rescanning.
	const int32_t land_index = (mission.land_index >= 0 && mission.land_index < mission.count) ? mission.land_index : -1;
	const bool mission_land_changed = mission.mission_id != state.mission_id
					  || mission.mission_dataman_id != state.dataman_id
					  || land_index != state.index;

	if (mission_land_changed) {
		state = {};
		state.mission_id = mission.mission_id;
		state.dataman_id = mission.mission_dataman_id;
		state.index = land_index;
		_dataman_cache_land_item.invalidate();

		if (state.index >= 0) {
			if (!_dataman_cache_land_item.load(static_cast<dm_item_t>(state.dataman_id), state.index)) {
				PX4_ERR("Mission land cache queue failed! item=%" PRIu8 ", index=%" PRIi32,
					state.dataman_id, state.index);
			}
		}
	}

	_dataman_cache_land_item.update();
}

void MissionRouteCache::updateSafePointCache(const mission_s &mission)
{
	SafePointState &state = _safe_point;
	bool success = false;

	if (mission.safe_points_id != state.source_id) {
		// A new safe_points_id makes any in-flight read and cached items stale.
		state.source_id = mission.safe_points_id;
		resetSafePointCacheState(false);
	}

	switch (state.dataman_state) {
	case SafePointDatamanState::UpdateRequestWait:
		if (state.update_requested && hrt_absolute_time() >= state.retry_at) {
			state.update_requested = false;
			state.dataman_state = SafePointDatamanState::Read;
		}

		break;

	case SafePointDatamanState::Read:
		state.stats = {};
		state.dataman_state = SafePointDatamanState::ReadWait;
		success = _dataman_client_safepoint.readAsync(DM_KEY_SAFE_POINTS_STATE, 0,
				reinterpret_cast<uint8_t *>(&state.stats), sizeof(mission_stats_entry_s));

		if (!success) {
			state.error_state = SafePointDatamanState::Read;
			state.dataman_state = SafePointDatamanState::Error;
		}

		break;

	case SafePointDatamanState::ReadWait:
		_dataman_client_safepoint.update();

		// Wait for operation to complete
		if (!_dataman_client_safepoint.lastOperationCompleted(success)) {
			break;
		}

		// Read errors
		if (!success) {
			state.error_state = SafePointDatamanState::ReadWait;
			state.dataman_state = SafePointDatamanState::Error;
			break;
		}

		if (state.stats.num_items > DM_KEY_SAFE_POINTS_MAX) {
			PX4_ERR("Safe points update failed! invalid count: %" PRIu16, state.stats.num_items);
			state.error_state = SafePointDatamanState::ReadWait;
			state.dataman_state = SafePointDatamanState::Error;
			break;
		}

		// Data on disk matches RAM, skip loading
		if (state.opaque_id_valid && state.opaque_id == state.stats.opaque_id) {
			state.dataman_state = SafePointDatamanState::UpdateRequestWait;
			state.ready = true;
			state.retry_count = 0;
			break;
		}

		// Cache Miss: Rebuild the cache
		state.opaque_id = state.stats.opaque_id;
		state.opaque_id_valid = true;
		state.ready = false;

		_dataman_cache_safepoint.invalidate();

		if (_dataman_cache_safepoint.size() != state.stats.num_items) {
			_dataman_cache_safepoint.resize(state.stats.num_items);
		}

		if (_dataman_cache_safepoint.size() != state.stats.num_items) {
			PX4_ERR("Safe point cache resize failed! requested: %" PRIu16 ", actual: %d",
				state.stats.num_items, _dataman_cache_safepoint.size());
			state.error_state = SafePointDatamanState::ReadWait;
			state.dataman_state = SafePointDatamanState::Error;
			break;
		}

		for (int index = 0; index < state.stats.num_items; ++index) {
			if (!_dataman_cache_safepoint.load(static_cast<dm_item_t>(state.stats.dataman_id), index)) {
				PX4_ERR("Safe point cache queue failed! item=%" PRIu8 ", index=%d",
					state.stats.dataman_id, index);
				state.error_state = SafePointDatamanState::ReadWait;
				state.dataman_state = SafePointDatamanState::Error;
				break;
			}
		}

		if (state.dataman_state == SafePointDatamanState::Error) {
			break;
		}

		// Move to load
		if (state.stats.num_items > 0) {
			state.dataman_state = SafePointDatamanState::Load;

		} else {
			// Zero safe points is still a valid ready state.
			state.dataman_state = SafePointDatamanState::UpdateRequestWait;
			state.ready = true;
			state.retry_count = 0;
		}

		break;

	case SafePointDatamanState::Load:
		_dataman_cache_safepoint.update();

		if (_dataman_cache_safepoint.isLoading()) {
			break;
		}

		if (safePointCacheFullyLoaded()) {
			state.dataman_state = SafePointDatamanState::UpdateRequestWait;
			state.ready = true;
			state.retry_count = 0;

		} else {
			state.error_state = SafePointDatamanState::Load;
			state.dataman_state = SafePointDatamanState::Error;
		}

		break;

	case SafePointDatamanState::Error:
	default: {
			// resetSafePointCacheState() clears retry_count, so save it first.
			const uint8_t retry_count = state.retry_count;

			if (state.dataman_state == SafePointDatamanState::Error) {
				PX4_ERR("Safe points update failed! error state: %" PRIu8, static_cast<uint8_t>(state.error_state));

			} else {
				PX4_ERR("Safe points update failed! invalid dataman state: %" PRIu8, static_cast<uint8_t>(state.dataman_state));
			}

			resetSafePointCacheState(false);
			state.retry_at = hrt_absolute_time() + nextRetryBackoff(retry_count);
			state.retry_count = retry_count;

			if (state.retry_count < UINT8_MAX) {
				++state.retry_count;
			}

			break;
		}
	}
}

bool MissionRouteCache::missionMatchesCache(const mission_s &mission) const
{
	return mission.mission_id == _mission.id
	       && mission.count == _mission.count
	       && mission.mission_dataman_id == _mission.dataman_id;
}
