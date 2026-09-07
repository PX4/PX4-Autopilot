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
 * @file full_mission_cache.cpp
 *
 * Optional RAM cache for a complete mission route.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "full_mission_cache.h"

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
namespace
{
bool isMissionDatamanIdValid(uint8_t dataman_id)
{
	return dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1;
}
} // namespace
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

FullMissionCache::FullMissionCache(orb_advert_t *mavlink_log_pub)
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	_mavlink_log_pub = mavlink_log_pub;

	// No zero-fill is needed because items are exposed only after a successful read.
	_mission_items = new mission_item_s[kMaxMissionCacheSize];

	if (_mission_items == nullptr) {
		PX4_ERR("Mission cache allocation failed");
	}

#else
	(void)mavlink_log_pub;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
}

FullMissionCache::~FullMissionCache()
{
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	_dataman_client_mission.abortCurrentOperation();
	perf_cancel(_load_perf);
	perf_free(_load_perf);
	delete[] _mission_items;
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
void FullMissionCache::advanceMissionGeneration()
{
	++_mission_generation;

	// Zero identifies a view that has never been published.
	if (_mission_generation == 0) {
		++_mission_generation;
	}
}

void FullMissionCache::invalidate()
{
	perf_cancel(_load_perf);
	advanceMissionGeneration();
	_mission = {};
}

void FullMissionCache::startMissionLoad()
{
	_mission.ready = false;
	_mission.next_index = 0;
	_mission.validation_pending = true;
	_mission.retry.retry_at = 0;
	perf_begin(_load_perf);
}

// Retry from the load front without clearing the prefix or retry count.
void FullMissionCache::resumeMissionLoad()
{
	_mission.validation_pending = true;
	_mission.retry.retry_at = 0;
}

bool FullMissionCache::missionCacheAvailable() const
{
	return _mission.initialized && _mission.source_valid && !_mission.too_large && _mission.ready;
}

bool FullMissionCache::missionItemsReady(const mission_s &mission) const
{
	return missionCacheAvailable() && missionMatchesCache(mission);
}

int FullMissionCache::missionCount() const
{
	return missionCacheAvailable() ? _mission.count : 0;
}

bool FullMissionCache::loadMissionItem(int index, mission_item_s &mission_item) const
{
	if (missionCacheAvailable() && index >= 0 && index < _mission.count) {
		mission_item = _mission_items[index];
		return true;
	}

	return false;
}

bool FullMissionCache::getMissionView(const mission_s &mission, MissionView &view) const
{
	if (!missionItemsReady(mission)) {
		return false;
	}

	view.items = (_mission.count > 0) ? _mission_items : nullptr;
	view.count = _mission.count;
	view.mission_id = _mission.id;
	view.dataman_id = _mission.dataman_id;
	view.generation = _mission_generation;
	return true;
}

bool FullMissionCache::missionViewStillValid(const MissionView &view) const
{
	// Pre-ready retries and patches can keep the generation because no view is exposed.
	return view.generation != 0 && view.generation == _mission_generation && _mission.ready;
}

FullMissionCache::SyncResult FullMissionCache::syncMissionItem(const mission_s &mission, int32_t index,
		const mission_item_s &mission_item)
{
	if (!_mission.source_valid || _mission.too_large
	    || !missionMatchesCache(mission) || index < 0 || index >= _mission.count) {
		return SyncResult::kRejected;
	}

	if (_mission.ready) {
		// A ready cache has no pending read. Patch it and invalidate borrowed views.
		_mission_items[index] = mission_item;
		advanceMissionGeneration();
		return SyncResult::kPatched;
	}

	if (index < _mission.next_index) {
		// Loaded items are not re-read, so patch the prefix in place.
		_mission_items[index] = mission_item;
		return SyncResult::kPatched;
	}

	if (_mission_request.pending && _mission_request.index == index) {
		// Discard the read started before the write and queue this index again.
		advanceMissionGeneration();
	}

	// Unread items pick up the Dataman write.
	return SyncResult::kDeferred;
}

void FullMissionCache::update(const mission_s &mission)
{
	MissionCacheState &state = _mission;

	if (!missionMatchesCache(mission)) {
		// Invalidate now, but drain any older read before issuing a new one.
		perf_cancel(_load_perf);
		advanceMissionGeneration();
		state = {};
		state.id = mission.mission_id;
		state.count = mission.count;
		state.dataman_id = mission.mission_dataman_id;
		state.initialized = true;
		state.source_valid = mission.count == 0 || isMissionDatamanIdValid(mission.mission_dataman_id);
		state.ready = state.source_valid && mission.count == 0;
		state.too_large = mission.count > kMaxMissionCacheSize;

		if (!state.source_valid) {
			PX4_ERR("Mission cache: invalid dataman id");
		}

		if (state.source_valid && state.too_large) {
			if (_mavlink_log_pub != nullptr) {
				mavlink_log_warning(_mavlink_log_pub, "Mission with %u items exceeds route cache capacity %d\t",
						    mission.count, static_cast<int>(kMaxMissionCacheSize));
			}

			events::send<uint16_t, int32_t>(events::ID("navigator_route_cache_too_large"),
			{events::Log::Warning, events::LogInternal::Info},
			"Mission with {1} items exceeds the route cache capacity {2}",
			mission.count, kMaxMissionCacheSize);
		}

		if (state.source_valid && !state.too_large && mission.count > 0 && _mission_items == nullptr) {
			PX4_ERR("Mission cache unavailable: boot-time allocation failed");
			state.too_large = true;
		}

		if (state.source_valid && !state.too_large && mission.count > 0) {
			startMissionLoad();
		}
	}

	_dataman_client_mission.update();

	if (_mission_request.pending) {
		bool request_success = false;

		if (!_dataman_client_mission.lastOperationCompleted(request_success)) {
			return;
		}

		const bool current_generation = _mission_request.generation == _mission_generation;
		const bool current_index = _mission_request.index == state.next_index;
		_mission_request.pending = false;

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
		if (_mission_request.pending) {
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
				_mission_request.generation = _mission_generation;
				_mission_request.index = request_index;
				_mission_request.pending = true;

			} else {
				state.validation_pending = false;
				PX4_WARN("Mission cache read start failed");
				state.retry.scheduleRetry(now);
			}

			return;
		}

		state.validation_pending = false;
		state.ready = true;
		state.retry.clear();
		perf_end(_load_perf);

	} else if (state.retry.due(now)) {
		resumeMissionLoad();
	}
}

bool FullMissionCache::missionMatchesCache(const mission_s &mission) const
{
	return _mission.initialized
	       && mission.mission_id == _mission.id
	       && mission.count == _mission.count
	       && mission.mission_dataman_id == _mission.dataman_id;
}

orb_sub_t FullMissionCache::responseSubscription() const
{
	return _mission_request.pending ? _dataman_client_mission.responseSubscription() : ORB_SUB_INVALID;
}

#else
void FullMissionCache::update(const mission_s &) {}
void FullMissionCache::invalidate() {}
bool FullMissionCache::missionItemsReady(const mission_s &) const { return false; }
int FullMissionCache::missionCount() const { return 0; }
bool FullMissionCache::loadMissionItem(int, mission_item_s &) const { return false; }
bool FullMissionCache::getMissionView(const mission_s &, MissionView &) const { return false; }
bool FullMissionCache::missionViewStillValid(const MissionView &) const { return false; }
FullMissionCache::SyncResult FullMissionCache::syncMissionItem(const mission_s &, int32_t, const mission_item_s &)
{
	return SyncResult::kRejected;
}
orb_sub_t FullMissionCache::responseSubscription() const { return ORB_SUB_INVALID; }
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
