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
 * Navigator-owned cache of the dataman-backed data RTL destination selection
 * needs. It preloads the rally (safe) points and the published mission land
 * item asynchronously, so RTL can evaluate destinations without blocking
 * Navigator on dataman or SD-card reads.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_cache.h"

#include "mission_route_types.h"

#include <inttypes.h>

#include <px4_platform_common/log.h>

void MissionRouteCache::update(const mission_s &mission)
{
	updateMissionLandItemCache(mission);
	updateSafePointCache(mission);
}

void MissionRouteCache::invalidate()
{
	_mission_land = {};
	_dataman_cache_land_item.invalidate();

	resetSafePointCacheState(true);
}

bool MissionRouteCache::missionLandItemCacheFullyLoaded() const
{
	if (_mission_land.index < 0 || _mission_land.index >= _mission_land.count) {
		return false;
	}

	mission_item_s land_item{};

	if (!_dataman_cache_land_item.loadWait(static_cast<dm_item_t>(_mission_land.dataman_id),
					       _mission_land.index,
					       reinterpret_cast<uint8_t *>(&land_item),
					       sizeof(land_item),
					       kCacheOnlyLoadWait)) {
		return false;
	}

	return mission_route::isLandingCmd(land_item.nav_cmd);
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
	if (!missionLandItemReady()) {
		return false;
	}

	mission_item_s cached_land_item{};

	if (!_dataman_cache_land_item.loadWait(static_cast<dm_item_t>(_mission_land.dataman_id), _mission_land.index,
					       reinterpret_cast<uint8_t *>(&cached_land_item), sizeof(cached_land_item),
					       kCacheOnlyLoadWait)) {
		return false;
	}

	if (!mission_route::isLandingCmd(cached_land_item.nav_cmd)) {
		return false;
	}

	index = _mission_land.index;
	land_item = cached_land_item;
	return true;
}

void MissionRouteCache::updateMissionLandItemCache(const mission_s &mission)
{
	MissionLandState &state = _mission_land;
	const hrt_abstime now = hrt_absolute_time();
	// Trust the published land_index, no mission rescanning.
	const int32_t land_index = (mission.land_index >= 0 && mission.land_index < mission.count) ? mission.land_index : -1;
	const bool mission_land_changed = mission.mission_id != state.mission_id
					  || mission.count != state.count
					  || mission.mission_dataman_id != state.dataman_id
					  || land_index != state.index;

	if (mission_land_changed) {
		state = {};
		state.mission_id = mission.mission_id;
		state.dataman_id = mission.mission_dataman_id;
		state.index = land_index;
		state.count = mission.count;
		_dataman_cache_land_item.invalidate();

		if (state.index >= 0) {
			state.validation_pending = queueMissionLandItem();

			if (!state.validation_pending) {
				state.retry.scheduleRetry(now);
			}
		}
	}

	_dataman_cache_land_item.update();

	if (state.index < 0 || state.ready) {
		return;
	}

	if (state.validation_pending) {
		if (_dataman_cache_land_item.isLoading()) {
			return;
		}

		state.validation_pending = false;

		if (missionLandItemCacheFullyLoaded()) {
			state.ready = true;
			state.retry.clear();

		} else {
			PX4_WARN("Mission land cache invalid or incomplete, retrying mission_id=%" PRIu32 ", index=%" PRIi32,
				 state.mission_id, state.index);
			_dataman_cache_land_item.invalidate();
			state.retry.scheduleRetry(now);
		}

	} else if (state.retry.due(now)) {
		_dataman_cache_land_item.invalidate();

		state.validation_pending = queueMissionLandItem();

		if (state.validation_pending) {
			state.retry.retry_at = 0;

		} else {
			state.retry.scheduleRetry(now);
		}
	}
}

bool MissionRouteCache::queueMissionLandItem()
{
	const MissionLandState &state = _mission_land;

	if (state.index < 0) {
		return false;
	}

	if (_dataman_cache_land_item.load(static_cast<dm_item_t>(state.dataman_id), state.index)) {
		return true;
	}

	PX4_WARN("Mission land cache queue failed, retrying! item=%" PRIu8 ", index=%" PRIi32, state.dataman_id, state.index);
	return false;
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
