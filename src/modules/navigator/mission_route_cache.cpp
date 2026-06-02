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
 * Dataman-backed Provider for mission-route planning. It preloads the whole
 * uploaded mission, the published mission land item, and the safe-point so
 * the planner can scan route geometry without blocking Navigator on dataman or
 * SD-card reads. Normal mission execution can keep using smaller sliding caches;
 * use this cache when a caller needs access to the full route.
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

	if (static_cast<int32_t>(_dataman_cache_mission.size()) < mission.count) {
		PX4_ERR("Mission cache capacity too small! requested: %d, capacity: %d",
			static_cast<int>(mission.count), static_cast<int>(_dataman_cache_mission.size()));
		return false;
	}

	const dm_item_t mission_dataman_id = static_cast<dm_item_t>(mission.mission_dataman_id);

	for (int32_t index = 0; index < mission.count; ++index) {
		if (!_dataman_cache_mission.load(mission_dataman_id, index)) {
			PX4_WARN("Mission cache queue failed, retrying! item=%" PRIu8 ", index=%" PRIi32,
				 static_cast<uint8_t>(mission_dataman_id), index);
			_dataman_cache_mission.invalidate();
			return false;
		}
	}

	return true;
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
	// Validate against the set currently being loaded.
	if (_safe_point.read_stats.num_items == 0) {
		return true;
	}

	mission_item_s safe_point_item{};

	// A miss means async preloading did not finish cleanly.
	for (int32_t index = 0; index < _safe_point.read_stats.num_items; ++index) {
		if (!_dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point.read_stats.dataman_id), index,
						       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item), 0)) {
			return false;
		}
	}

	return true;
}

bool MissionRouteCache::safePointCacheMatchesReadStats() const
{
	const SafePointState &state = _safe_point;

	// Comparing the opaque id alone is not enough
	// because it can be reused across different safe-point sets
	return state.ready
	       && state.cached_source_id == state.source_id
	       && state.cached_stats.opaque_id == state.read_stats.opaque_id
	       && state.cached_stats.num_items == state.read_stats.num_items
	       && state.cached_stats.dataman_id == state.read_stats.dataman_id;
}

void MissionRouteCache::publishSafePointCache()
{
	// Only call after a successful (re)build.
	_safe_point.cached_stats = _safe_point.read_stats;
	_safe_point.cached_source_id = _safe_point.source_id;
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
	return mission.count > kMaxRouteMissionCacheSize;
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
	// Planner-facing reads are cache-only so route RTL falls back instead of stalling Navigator on SD-card misses.
	return !_mission.too_large
	       && _mission.ready
	       && index >= 0 && index < _mission.count
	       && _dataman_cache_mission.loadWait(static_cast<dm_item_t>(_mission.dataman_id), index,
			       reinterpret_cast<uint8_t *>(&mission_item), sizeof(mission_item),
			       kCacheOnlyLoadWait);
}

int MissionRouteCache::safePointCount() const
{
	return safePointsReady() ? _safe_point.cached_stats.num_items : 0;
}

bool MissionRouteCache::loadSafePointItem(int index, mission_item_s &safe_point_item) const
{
	return safePointsReady()
	       && index >= 0 && index < _safe_point.cached_stats.num_items
	       && _dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point.cached_stats.dataman_id), index,
			       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item),
			       kCacheOnlyLoadWait);
}

bool MissionRouteCache::getMissionLandItem(int32_t &index, mission_item_s &land_item) const
{
	if (_mission_land.index < 0 || _mission_land.index >= _mission.count) {
		return false;
	}

	index = _mission_land.index;
	return _dataman_cache_land_item.loadWait(static_cast<dm_item_t>(_mission_land.dataman_id), index,
			reinterpret_cast<uint8_t *>(&land_item), sizeof(land_item),
			kCacheOnlyLoadWait);
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

		if (!_dataman_cache_land_item.updateCachedItem(static_cast<dm_item_t>(_mission_land.dataman_id), index,
				reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item))) {
			_dataman_cache_land_item.invalidate();
			_mission_land.index = -1;
			return false;
		}
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
				state.retry.scheduleRetry(hrt_absolute_time());
			}
		}
	}

	_dataman_cache_mission.update();

	if (state.ready || state.too_large || mission.count <= 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (state.validation_pending) {
		if (_dataman_cache_mission.isLoading()) {
			return;
		}

		state.validation_pending = false;

		// Validate that every queued async read made it into the RAM cache.
		if (missionCacheFullyLoaded(mission)) {
			state.ready = true;
			state.retry.clear();

		} else {
			PX4_WARN("Mission route cache incomplete, retrying mission_id=%" PRIu32, mission.mission_id);
			state.retry.scheduleRetry(now);
		}

	} else if (state.retry.due(now)) {
		state.validation_pending = queueMissionCacheLoads(mission);

		if (state.validation_pending) {
			state.retry.retry_at = 0;

		} else {
			PX4_WARN("Mission route cache queue failed, retrying mission_id=%" PRIu32, mission.mission_id);
			state.retry.scheduleRetry(now);
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
			queueMissionLandItem();
		}

	} else if (state.index >= 0 && state.retry.due(hrt_absolute_time())) {
		// A transient queue failure must not permanently disable the mission-land fallback.
		_dataman_cache_land_item.invalidate();
		queueMissionLandItem();
	}

	_dataman_cache_land_item.update();
}

void MissionRouteCache::queueMissionLandItem()
{
	MissionLandState &state = _mission_land;

	if (_dataman_cache_land_item.load(static_cast<dm_item_t>(state.dataman_id), state.index)) {
		state.retry.clear();
		return;
	}

	PX4_WARN("Mission land cache queue failed, retrying! item=%" PRIu8 ", index=%" PRIi32, state.dataman_id, state.index);
	state.retry.scheduleRetry(hrt_absolute_time());
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
	case SafePointDatamanState::kUpdateRequestWait:
		if (state.update_requested && hrt_absolute_time() >= state.retry.retry_at) {
			state.update_requested = false;
			state.dataman_state = SafePointDatamanState::kRead;
		}

		break;

	case SafePointDatamanState::kRead:
		state.read_stats = {};
		state.dataman_state = SafePointDatamanState::kReadWait;
		success = _dataman_client_safepoint.readAsync(DM_KEY_SAFE_POINTS_STATE, 0,
				reinterpret_cast<uint8_t *>(&state.read_stats), sizeof(mission_stats_entry_s));

		if (!success) {
			state.error_state = SafePointDatamanState::kRead;
			state.dataman_state = SafePointDatamanState::kError;
		}

		break;

	case SafePointDatamanState::kReadWait:
		_dataman_client_safepoint.update();

		if (!_dataman_client_safepoint.lastOperationCompleted(success)) {
			break;
		}

		if (!success) {
			state.error_state = SafePointDatamanState::kReadWait;
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		if (state.read_stats.num_items > DM_KEY_SAFE_POINTS_MAX) {
			PX4_ERR("Safe points update failed! invalid count: %" PRIu16, state.read_stats.num_items);
			state.error_state = SafePointDatamanState::kReadWait;
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		if (state.read_stats.dataman_id != DM_KEY_SAFE_POINTS_0 && state.read_stats.dataman_id != DM_KEY_SAFE_POINTS_1) {
			PX4_ERR("Safe points update failed! invalid dataman id: %" PRIu8, state.read_stats.dataman_id);
			state.error_state = SafePointDatamanState::kReadWait;
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		// The RAM cache already holds this exact set, skip the rebuild.
		if (safePointCacheMatchesReadStats()) {
			state.dataman_state = SafePointDatamanState::kUpdateRequestWait;
			state.ready = true;
			state.retry.clear();
			break;
		}

		// Cache miss, rebuild the cache from the new read state.
		state.ready = false;

		_dataman_cache_safepoint.invalidate();

		if (_dataman_cache_safepoint.size() != state.read_stats.num_items) {
			_dataman_cache_safepoint.resize(state.read_stats.num_items);
		}

		if (_dataman_cache_safepoint.size() != state.read_stats.num_items) {
			PX4_WARN("Safe point cache resize failed, retrying! requested: %" PRIu16 ", actual: %d",
				 state.read_stats.num_items, _dataman_cache_safepoint.size());
			state.error_state = SafePointDatamanState::kReadWait;
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		for (int index = 0; index < state.read_stats.num_items; ++index) {
			if (!_dataman_cache_safepoint.load(static_cast<dm_item_t>(state.read_stats.dataman_id), index)) {
				PX4_WARN("Safe point cache queue failed, retrying! item=%" PRIu8 ", index=%d",
					 state.read_stats.dataman_id, index);
				state.error_state = SafePointDatamanState::kReadWait;
				state.dataman_state = SafePointDatamanState::kError;
				break;
			}
		}

		if (state.dataman_state == SafePointDatamanState::kError) {
			break;
		}

		if (state.read_stats.num_items > 0) {
			state.dataman_state = SafePointDatamanState::kLoad;

		} else {
			// Zero safe points is still a valid ready state.
			publishSafePointCache();
			state.dataman_state = SafePointDatamanState::kUpdateRequestWait;
			state.ready = true;
			state.retry.clear();
		}

		break;

	case SafePointDatamanState::kLoad:
		_dataman_cache_safepoint.update();

		if (_dataman_cache_safepoint.isLoading()) {
			break;
		}

		if (safePointCacheFullyLoaded()) {
			publishSafePointCache();
			state.dataman_state = SafePointDatamanState::kUpdateRequestWait;
			state.ready = true;
			state.retry.clear();

		} else {
			state.error_state = SafePointDatamanState::kLoad;
			state.dataman_state = SafePointDatamanState::kError;
		}

		break;

	case SafePointDatamanState::kError:
	default: {
			// resetSafePointCacheState() clears the backoff, so save it first.
			const RetryBackoff retry = state.retry;

			if (state.dataman_state == SafePointDatamanState::kError) {
				PX4_WARN("Safe points update failed, retrying! error state: %" PRIu8, static_cast<uint8_t>(state.error_state));

			} else {
				PX4_ERR("Safe points update failed! invalid dataman state: %" PRIu8, static_cast<uint8_t>(state.dataman_state));
			}

			resetSafePointCacheState(false);
			state.retry = retry;
			state.retry.scheduleRetry(hrt_absolute_time());

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
