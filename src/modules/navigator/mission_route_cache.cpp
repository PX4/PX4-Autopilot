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
 * uploaded mission, the published mission land item, and the safe points so
 * the planner can scan route geometry without blocking Navigator on dataman or
 * SD-card reads. Normal mission execution can keep using smaller sliding caches;
 * use this cache when a caller needs access to the full route.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "mission_route_cache.h"

#include "mission_route_types.h"

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

// The mission and land caches only accept these dataman storages as their source.
static bool isMissionDatamanIdValid(uint8_t dataman_id)
{
	return dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1;
}

MissionRouteCache::MissionRouteCache(orb_advert_t *mavlink_log_pub)
{
#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	_mavlink_log_pub = mavlink_log_pub;

	// Allocated once at Navigator start and kept for the module lifetime. Intentionally
	// not zero-filled: no index is exposed before a successful dataman read fills it.
	_mission_items = new mission_item_s[kMaxRouteMissionCacheSize];

	if (_mission_items == nullptr) {
		PX4_ERR("Mission cache allocation failed");
	}

#else
	(void)mavlink_log_pub;
#endif
}

MissionRouteCache::~MissionRouteCache()
{
#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	_dataman_client_mission.abortCurrentOperation();
	delete[] _mission_items;
#endif
}

void MissionRouteCache::update(const mission_s &mission, bool allow_blocking_load)
{
#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	updateMissionCache(mission, allow_blocking_load);
#else
	(void)allow_blocking_load;
#endif
	updateMissionLandItemCache(mission);
	updateSafePointCache(mission);
}

void MissionRouteCache::invalidate()
{
#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
	advanceMissionGeneration();
	_mission = {};
#endif

	_mission_land = {};
	_dataman_cache_land_item.invalidate();

	resetSafePointCacheState(true);
}

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
void MissionRouteCache::advanceMissionGeneration()
{
	++_mission_generation;

	// Zero identifies a view that has never been published.
	if (_mission_generation == 0) {
		++_mission_generation;
	}
}

// Start or restart the load for the current source; fall back to a backoff retry.
void MissionRouteCache::startMissionLoadOrRetry(const mission_s &mission, hrt_abstime now)
{
	if (!_mission.source_valid || _mission.too_large || mission.count <= 0 || _mission_items == nullptr) {
		_mission.validation_pending = false;
		PX4_WARN("Mission cache load start failed");
		_mission.retry.scheduleRetry(now);
		return;
	}

	// A rebuild immediately hides the old generation. An active older request
	// is allowed to finish, and its result is discarded before this generation
	// starts writing the shared typed buffer.
	_mission.ready = false;
	_mission.next_index = 0;
	_mission.validation_pending = true;
	_mission.retry.retry_at = 0;
}

bool MissionRouteCache::blockingLoadMissionItems(MissionCacheState &state)
{
	// Disarmed-only warm-up: synchronous reads with a bounded budget per update() call.
	// Worst case a single read overruns the budget by kBlockingLoadItemTimeout.
	const hrt_abstime start = hrt_absolute_time();

	while (state.next_index < state.count) {
		if (hrt_elapsed_time(&start) >= kBlockingLoadBudget) {
			return false; // Out of budget, resume on the next update() call.
		}

		const int32_t request_index = state.next_index;

		if (_dataman_client_mission.readSync(static_cast<dm_item_t>(state.dataman_id),
						     static_cast<uint32_t>(request_index),
						     reinterpret_cast<uint8_t *>(&_mission_items[request_index]),
						     sizeof(mission_item_s), kBlockingLoadItemTimeout)) {
			++state.next_index;

		} else {
			state.validation_pending = false;
			PX4_WARN("Mission cache retry");
			state.retry.scheduleRetry(hrt_absolute_time());
			return false;
		}
	}

	return true;
}
#endif

bool MissionRouteCache::missionLandItemCacheFullyLoaded() const
{
	if (_mission_land.index < 0 || _mission_land.index >= _mission_land.count) {
		return false;
	}

	mission_item_s land_item;

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

	mission_item_s safe_point_item;

	// A miss means async preloading did not finish cleanly.
	for (int32_t index = 0; index < _safe_point.read_stats.num_items; ++index) {
		if (!_dataman_cache_safepoint.loadWait(static_cast<dm_item_t>(_safe_point.read_stats.dataman_id), index,
						       reinterpret_cast<uint8_t *>(&safe_point_item), sizeof(safe_point_item),
						       kCacheOnlyLoadWait)) {
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
	const uint8_t source_dataman_id = clear_source_identity ? static_cast<uint8_t>(DM_KEY_SAFE_POINTS_0) :
					  _safe_point.source_dataman_id;
	_safe_point = {};
	_safe_point.source_id = source_id;
	_safe_point.source_dataman_id = source_dataman_id;
}

bool MissionRouteCache::missionExceedsCacheLimit(const mission_s &mission) const
{
	return mission.count > kMaxRouteMissionCacheSize;
}

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
// True while the current mission generation is fully loaded and readable.
bool MissionRouteCache::missionCacheAvailable() const
{
	return _mission.initialized && _mission.source_valid && !_mission.too_large && _mission.ready;
}

bool MissionRouteCache::isReady(const mission_s &mission) const
{
	return missionCacheAvailable() && missionMatchesCache(mission);
}

int MissionRouteCache::missionCount() const
{
	return missionCacheAvailable() ? _mission.count : 0;
}

bool MissionRouteCache::loadMissionItem(int index, mission_item_s &mission_item) const
{
	if (missionCacheAvailable() && index >= 0 && index < _mission.count) {
		mission_item = _mission_items[index];
		return true;
	}

	return false;
}

bool MissionRouteCache::getMissionView(const mission_s &mission, MissionView &view) const
{
	if (!isReady(mission)) {
		return false;
	}

	view.items = (_mission.count > 0) ? _mission_items : nullptr;
	view.count = _mission.count;
	view.mission_id = _mission.id;
	view.dataman_id = _mission.dataman_id;
	view.generation = _mission_generation;
	return true;
}

bool MissionRouteCache::missionViewStillValid(const MissionView &view) const
{
	// Generation equality pins source and content: every source change, invalidate(),
	// reload, and patch advances it, and it is only observable while ready.
	return view.generation != 0 && view.generation == _mission_generation && _mission.ready;
}

#else
bool MissionRouteCache::isReady(const mission_s &) const { return false; }
int MissionRouteCache::missionCount() const { return 0; }
bool MissionRouteCache::loadMissionItem(int, mission_item_s &) const { return false; }
bool MissionRouteCache::getMissionView(const mission_s &, MissionView &) const { return false; }
bool MissionRouteCache::missionViewStillValid(const MissionView &) const { return false; }
#endif

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

	mission_item_s cached_land_item;

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

bool MissionRouteCache::loadMissionItem(const mission_s &mission, int32_t index, mission_item_s &mission_item) const
{
	return isReady(mission) && loadMissionItem(index, mission_item);
}

MissionRouteCache::SyncResult MissionRouteCache::syncMissionItem(const mission_s &mission, int32_t index,
		const mission_item_s &mission_item)
{
	// The land-item cache exists on every board and validates against the published
	// mission, so keep it coherent independently of the full-mission cache outcome.
	syncMissionLandItem(mission, index, mission_item);

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0

	if (!_mission.source_valid || _mission.too_large
	    || !missionMatchesCache(mission) || index < 0 || index >= _mission.count) {
		return SyncResult::kRejected;
	}

	SyncResult result = SyncResult::kRestarted;

	if (_mission.ready) {
		_mission_items[index] = mission_item;
		advanceMissionGeneration();
		result = SyncResult::kPatched;

	} else if (_mission.validation_pending && index < _mission.next_index) {
		// Sequential loading makes every earlier index stable. An authoritative
		// write can patch it while later indices continue loading.
		_mission_items[index] = mission_item;
		result = SyncResult::kPatched;

	} else {
		// The item is unread or its old read is still in flight. Start a new
		// unpublished generation after that request has been safely drained.
		advanceMissionGeneration();
		_mission.retry.clear();
		startMissionLoadOrRetry(mission, hrt_absolute_time());
	}

	return result;
#else
	return SyncResult::kRejected;
#endif
}

void MissionRouteCache::syncMissionLandItem(const mission_s &mission, int32_t index, const mission_item_s &mission_item)
{
	// Only the published land item of the mission the land cache tracks is relevant.
	// If source mismatch: skip, update() rebuilds the land cache on the next source change.
	if (index < 0 || index != _mission_land.index || !missionLandMatchesCache(mission)) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_mission_land.ready) {
		const bool patched = _dataman_cache_land_item.updateCachedItem(
					     static_cast<dm_item_t>(_mission_land.dataman_id), index,
					     reinterpret_cast<const uint8_t *>(&mission_item), sizeof(mission_item));

		if (!patched || !mission_route::isLandingCmd(mission_item.nav_cmd)) {
			_dataman_cache_land_item.invalidate();
			_mission_land.ready = false;
			_mission_land.validation_pending = false;
			_mission_land.retry.scheduleRetry(now);
		}

	} else {
		_dataman_cache_land_item.invalidate();
		_mission_land.validation_pending = false;
		_mission_land.retry.clear();

		if (mission_route::isLandingCmd(mission_item.nav_cmd)) {
			_mission_land.validation_pending = queueMissionLandItem();

			if (!_mission_land.validation_pending) {
				_mission_land.retry.scheduleRetry(now);
			}

		} else {
			_mission_land.retry.scheduleRetry(now);
		}
	}
}

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
void MissionRouteCache::updateMissionCache(const mission_s &mission, bool allow_blocking_load)
{
	MissionCacheState &state = _mission;

	if (!missionMatchesCache(mission)) {
		// A new source immediately invalidates the published view. Do not abort
		// an older request: it must be consumed and discarded before the same
		// Dataman key can safely be requested for this generation.
		advanceMissionGeneration();
		state = {};
		state.id = mission.mission_id;
		state.count = mission.count;
		state.dataman_id = mission.mission_dataman_id;
		state.initialized = true;
		state.source_valid = mission.count == 0 || isMissionDatamanIdValid(mission.mission_dataman_id);
		state.ready = state.source_valid && mission.count == 0;
		state.too_large = missionExceedsCacheLimit(mission);

		if (!state.source_valid) {
			PX4_ERR("Mission cache: invalid dataman id");
		}

		if (state.source_valid && state.too_large) {
			// Runs once per source change; the route cache stays unavailable for this mission.
			if (_mavlink_log_pub != nullptr) {
				mavlink_log_critical(_mavlink_log_pub, "Mission with %u items exceeds route cache capacity %d\t",
						     mission.count, static_cast<int>(kMaxRouteMissionCacheSize));
			}

			events::send<uint16_t, int32_t>(events::ID("navigator_route_cache_too_large"),
			{events::Log::Error, events::LogInternal::Info},
			"Mission with {1} items exceeds the route cache capacity {2}",
			mission.count, kMaxRouteMissionCacheSize);
		}

		// The boot-time allocation failed, no recovery is possible for a non-empty mission.
		if (state.source_valid && !state.too_large && mission.count > 0 && _mission_items == nullptr) {
			PX4_ERR("Mission cache unavailable: boot-time allocation failed");
			state.too_large = true;
		}

		if (state.source_valid && !state.too_large && mission.count > 0) {
			startMissionLoadOrRetry(mission, hrt_absolute_time());
		}
	}

	_dataman_client_mission.update();

	if (_mission_request_pending) {
		bool request_success = false;

		if (!_dataman_client_mission.lastOperationCompleted(request_success)) {
			return;
		}

		const bool current_generation = _mission_request_generation == _mission_generation;
		const bool current_index = _mission_request_index == state.next_index;
		_mission_request_pending = false;

		if (current_generation && current_index && state.validation_pending) {
			if (request_success) {
				++state.next_index;

			} else {
				state.validation_pending = false;
				PX4_WARN("Mission cache retry");
				state.retry.scheduleRetry(hrt_absolute_time());
				return;
			}
		}
	}

	if (!state.source_valid || state.ready || state.too_large || mission.count <= 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (state.validation_pending) {
		if (_mission_request_pending) {
			return;
		}

		if (allow_blocking_load && state.next_index < state.count && !blockingLoadMissionItems(state)) {
			// Budget exhausted (resume next call) or a failed read scheduled a retry.
			return;
		}

		if (state.next_index < state.count) {
			const int32_t request_index = state.next_index;
			const bool request_started = _dataman_client_mission.readAsync(
							     static_cast<dm_item_t>(state.dataman_id),
							     static_cast<uint32_t>(request_index),
							     reinterpret_cast<uint8_t *>(&_mission_items[request_index]),
							     sizeof(mission_item_s));

			if (request_started) {
				_mission_request_generation = _mission_generation;
				_mission_request_index = request_index;
				_mission_request_pending = true;

			} else {
				state.validation_pending = false;
				PX4_WARN("Mission cache read start failed");
				state.retry.scheduleRetry(now);
			}

			return;
		}

		// Every failure path clears validation_pending, so reaching the end of the
		// sequential load means the complete generation was read successfully.
		state.validation_pending = false;
		state.ready = true;
		state.retry.clear();

	} else if (state.retry.due(now)) {
		advanceMissionGeneration();
		startMissionLoadOrRetry(mission, now);
	}
}
#endif

void MissionRouteCache::updateMissionLandItemCache(const mission_s &mission)
{
	MissionLandState &state = _mission_land;
	const hrt_abstime now = hrt_absolute_time();
	// Trust the published land_index, no mission rescanning.
	const bool valid_dataman_id = isMissionDatamanIdValid(mission.mission_dataman_id);
	const bool valid_land_index = mission.land_index >= 0 && mission.land_index < mission.count;
	const int32_t land_index = (valid_dataman_id && valid_land_index) ? mission.land_index : -1;

	if (!missionLandMatchesCache(mission) || land_index != state.index) {
		state = {};
		state.mission_id = mission.mission_id;
		state.dataman_id = mission.mission_dataman_id;
		state.index = land_index;
		state.count = mission.count;
		_dataman_cache_land_item.invalidate();

		if (valid_land_index && !valid_dataman_id) {
			PX4_ERR("Mission land cache: invalid dataman id");
		}

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
			PX4_WARN("Mission land cache retry");
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

	PX4_WARN("Mission land cache retry");
	return false;
}

void MissionRouteCache::updateSafePointCache(const mission_s &mission)
{
	SafePointState &state = _safe_point;
	bool success = false;

	if (mission.safe_points_id != state.source_id || mission.safepoint_dataman_id != state.source_dataman_id) {
		// A new safe-point source makes any in-flight read and cached items stale.
		state.source_id = mission.safe_points_id;
		state.source_dataman_id = mission.safepoint_dataman_id;
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
			state.dataman_state = SafePointDatamanState::kError;
		}

		break;

	case SafePointDatamanState::kReadWait:
		_dataman_client_safepoint.update();

		if (!_dataman_client_safepoint.lastOperationCompleted(success)) {
			break;
		}

		if (!success) {
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		if (!(state.read_stats.num_items <= DM_KEY_SAFE_POINTS_MAX
		      && state.read_stats.opaque_id == state.source_id
		      && (state.read_stats.num_items == 0
			  || (state.read_stats.dataman_id == state.source_dataman_id
			      && (state.read_stats.dataman_id == DM_KEY_SAFE_POINTS_0
				  || state.read_stats.dataman_id == DM_KEY_SAFE_POINTS_1))))) {
			PX4_ERR("Safe point cache metadata invalid");
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
			state.dataman_state = SafePointDatamanState::kError;
			break;
		}

		for (int index = 0; index < state.read_stats.num_items; ++index) {
			if (!_dataman_cache_safepoint.load(static_cast<dm_item_t>(state.read_stats.dataman_id), index)) {
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
			state.dataman_state = SafePointDatamanState::kError;
		}

		break;

	case SafePointDatamanState::kError:
	default: {
			// resetSafePointCacheState() clears the backoff, so save it first.
			const RetryBackoff retry = state.retry;

			if (state.dataman_state == SafePointDatamanState::kError) {
				PX4_WARN("Safe point cache retry");

			} else {
				PX4_ERR("Safe point cache state invalid");
			}

			resetSafePointCacheState(false);
			state.retry = retry;
			state.retry.scheduleRetry(hrt_absolute_time());

			break;
		}
	}
}

#if CONFIG_RTL_MISSION_CACHE_SIZE > 0
bool MissionRouteCache::missionMatchesCache(const mission_s &mission) const
{
	return _mission.initialized
	       && mission.mission_id == _mission.id
	       && mission.count == _mission.count
	       && mission.mission_dataman_id == _mission.dataman_id;
}
#endif

bool MissionRouteCache::missionLandMatchesCache(const mission_s &mission) const
{
	return _mission_land.mission_id == mission.mission_id
	       && _mission_land.count == mission.count
	       && _mission_land.dataman_id == mission.mission_dataman_id;
}
