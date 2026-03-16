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
 * @file detect_and_avoid.h
 *
 * Helper class to do detect and avoid traffic
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "../mission_block.h"

#include <pthread.h>

#include <matrix/math.hpp>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/detect_and_avoid.h>
#include <uORB/topics/detect_and_avoid_most_urgent.h>
#include <lib/adsb/AdsbConflict.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_command.h>
#include <commander/px4_custom_mode.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>

#include <systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>

#include <px4_platform_common/board_common.h>
#include <px4_platform_common/time.h>

#include <containers/Array.hpp>

using namespace time_literals;

static constexpr uint8_t kMaxLogMsgSize{128};

static constexpr uint8_t kIdEncodingNbBytes{8}; // Max 8.
static constexpr uint8_t kUtmGuidMsgLength{11}; // Number of chars used to display uas id: 10 + Null terminated. Max: 17 = 16 + null
static constexpr uint8_t kCallsignLength{9};	// 8 + Null terminated
static constexpr uint8_t kIcaoLength{7};	// 6 + Null terminated
static constexpr uint8_t kDaaMaxTraffic{5};
static constexpr uint64_t kIgnoredTrafficNotifTime{2_s};
static constexpr uint64_t kRemoveStaleConflictsTime{2_s};

// Internal action priority order used for escalation comparisons.
// User-facing parameter values are translated separately so DAA_LVL_*_ACT can
// share the NAV_TRAFF_AVOID convention without changing the priority ladder.
enum class DaaAction : uint8_t {
	kDisabled = 0,
	kWarnOnly = 1,
	kPositionHoldMode = 2,
	kReturnMode = 3,
	kLandMode = 4,
	kTerminate = 5,
	kMaxActionValue = 6
};

enum class RemoveBufferCause : uint8_t {
	kStaleConflict = 0,
	kBufferFull = 1
};

enum class NotifyLandedActCause : uint8_t {
	kConflictAndArmed = 0,
	kConflictAndDisarmed = 1
};

enum class IgnoreTrafficCause : uint8_t {
	kBufferFull = 0,
	kFailedUpdate = 1,
	kFailedRemoval = 2,
	kFailedInclusion = 3,
	kFailedToGetLvl = 4,
	kInvalidIndex = 5
};

struct unique_id_s {
	uint64_t id{0};
	uint8_t encoding{detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO};
};

struct conflict_info_s {
	unique_id_s unique_id;
	hrt_abstime latest_update_timestamp;
	uint8_t conflict_level;
	float aircraft_dist; // Distance to aircraft = sqrtf(dist_hor^2 + dist_vert^2)
};

class DetectAndAvoid : public MissionBlock, public ModuleParams
{
public:
	DetectAndAvoid(Navigator *navigator);
	~DetectAndAvoid() override = default;

	/** @brief Start DAA. Loads parameters, picks the standard, logs a critical event on failure. */
	void on_activation() override;

	/** @brief Run one DAA tick: pull new traffic, update the buffer, send any required action. */
	void on_active() override;

	/** @brief Stop DAA and publish an empty status. */
	void on_inactivation() override;

	/** @brief Clear all DAA state (buffer, last action, timestamps). */
	void reset();

	bool is_activated() const { return _is_activated;};

	/** @brief Refresh cached parameter values. */
	void updateParams() override;

#if !defined(CONSTRAINED_FLASH) && !defined(__PX4_NUTTX)
	/** @brief Print buffer size and most-urgent conflict to the shell. */
	void print_status() const;
#endif // !CONSTRAINED_FLASH && !__PX4_NUTTX

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	// Fake traffic scripts used for manual DAA validation from the navigator shell.
	enum class FakeTraffMode : uint8_t {
		kUniqueIds = 0,
		kEscalation = 1,
		kSpamSame = 2,
		kSpamNew = 3,
		kFlags = 4,
		kQueueFill = 5
	};
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	/* Unique ID conversion */

	/** @brief Pack a callsign string into a 64-bit key used for buffer lookups. */
	static uint64_t callsign_to_uint64(const char callsign[kCallsignLength]);

	/** @brief Pack the trailing bytes of a UAS-ID into a 64-bit key. */
	static uint64_t last_uas_id_bytes_to_uint64(const uint8_t uas_id[PX4_GUID_BYTE_LENGTH]);

	/** @brief Inverse of last_uas_id_bytes_to_uint64; unpack the key back to a byte array. */
	static void uint64_to_last_uas_id_bytes(const uint64_t uas_id_int, uint8_t uas_id[kIdEncodingNbBytes]);

	/** @brief Inverse of callsign_to_uint64; unpack the key back to a null-terminated callsign. */
	static void convert_uint64_callsign_to_str(uint64_t value, char callsign[kCallsignLength]);

	/** @brief Print the low 24 bits of an ICAO address as a 6-digit uppercase hex string. */
	static void convert_icao_uint32_to_hex_str(uint64_t value, char *buffer, size_t buffer_size);

	/** @brief True if the report's unique ID matches ownship (ICAO, callsign or UAS-ID). */
	bool is_self_detection(const unique_id_s &unique_id) const;

	/** @brief Pick the best available identifier from a report (ICAO first, then callsign, then UAS-ID). */
	bool get_unique_id(const transponder_report_s &report, unique_id_s &unique_id) const;

	/* Actions */

	/**
	 * @brief True if @p requested_action is more urgent than the current nav mode.
	 *
	 * Public so unit tests can drive the escalation logic directly.
	 */
	bool eval_conflict_escalation_action(const DaaAction requested_action) const;

	/** @brief True if the conflict level maps to something beyond a warn-only notification. */
	bool conflict_lvl_requires_action(const uint8_t conflict_level) const;

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	/**
	 * @brief Arm a fake-traffic script from the navigator shell for manual DAA testing.
	 *
	 * Uses the ownship position so the fake targets stay anchored to a fixed origin.
	 */
	void run_fake_traffic(FakeTraffMode mode, double lat_uav, double lon_uav, float alt_uav);

	/** @brief Cancel a running fake-traffic script. */
	void stop_fake_traffic();

	/** @brief Publish one synthetic transponder report at a relative bearing/distance from ownship. */
	void fake_traffic(uint32_t icao_address, const char *const callsign, float distance, float direction = 0,
			  float traffic_heading = 0,
			  float altitude_diff = 10,
			  float hor_velocity = 100, float ver_velocity = 10, int emitter_type = 1, double lat_uav = 45.35324098,
			  double lon_uav = 6.446453,
			  float alt_uav = 300, uint16_t flags = 63);
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	void get_most_urgent_conflict(conflict_info_s &conflict) const {conflict = _most_urgent_conflict;};

protected:
	px4::Array<conflict_info_s, kDaaMaxTraffic> _traffic_buffer{};

private:
	using new_conflicts_pending_notif_s = px4::Array<unique_id_s, transponder_report_s::ORB_QUEUE_LENGTH>;

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	struct fake_traffic_origin_s {
		double lat{0.0};
		double lon{0.0};
		float alt{0.f};
	};

	struct fake_traffic_state_s {
		// True while a shell-triggered fake traffic script is still being replayed.
		bool active{false};
		// Which deterministic scenario is currently being replayed.
		FakeTraffMode mode{FakeTraffMode::kUniqueIds};
		// Index of the next scripted report to publish for the active mode.
		uint8_t next_step_idx{0};
		// Absolute time when the next scripted report may be published.
		hrt_abstime next_publish_at{0};
		// Fixed ownship origin captured when the script was armed.
		fake_traffic_origin_s origin{};

		void clear()
		{
			active = false;
			mode = FakeTraffMode::kUniqueIds;
			next_step_idx = 0;
			next_publish_at = 0;
			origin = {};
		}
	};
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	uORB::Publication<detect_and_avoid_s>		_detect_and_avoid_pub{ORB_ID(detect_and_avoid)};
	uORB::Publication<detect_and_avoid_most_urgent_s>		_detect_and_avoid_most_urgent_pub{ORB_ID(detect_and_avoid_most_urgent)};

	/** @brief Publish the cached most-urgent conflict on the dedicated topic. */
	void publish_most_urgent_conflict();

	/** @brief Empty the buffer and reset all derived conflict state and timers. */
	void clear_conflicts();

	/* Activation */

	/** @brief Mark the module as failed, publish empty status and send a critical event. */
	void failed_activation();

	bool _is_activated{false};
	bool _daa_enabled{false};
	int32_t _daa_notif_state_s{20};

	/* Handle traffic buffer */

	/** @brief Rate-limited entry point that drops timed-out conflicts. */
	bool try_removing_stale_conflicts();

	/**
	 * @brief Pull queued transponder reports and update the conflict buffer.
	 *
	 * Returns true if the buffer changed. New-traffic warnings are deferred via
	 * @p new_conflicts_pending_notif so the final priority is settled before logging.
	 */
	bool update_conflict_buffer(new_conflicts_pending_notif_s &new_conflicts_pending_notif);

	/**
	 * @brief Update or remove an existing buffer entry from a fresh report.
	 *
	 * DAA_CONFLICT_LVL_NONE removes the entry; everything else overwrites it.
	 */
	bool handle_existing_conflict(const conflict_info_s &current_conflict, const int current_conflict_idx);

	/**
	 * @brief Insert a new conflict, evicting the least urgent entry when the buffer is full.
	 *
	 * No "removed" warning for entries that were only added in the same spin.
	 */
	bool handle_new_conflict(const conflict_info_s &current_conflict, const new_conflicts_pending_notif_s &new_conflicts_pending_notif);

	/** @brief Walk the buffer back-to-front and remove entries older than the timeout. */
	bool stale_conflicts_removed();

	bool try_removing_conflict_from_buffer(const int conflict_idx, conflict_info_s &removed_conflict);
	bool try_adding_conflict_to_buffer(const conflict_info_s &conflict);
	bool try_updating_conflict_in_buffer(const int conflict_idx, const conflict_info_s &conflict);

	/* Handle conflict level */

	/**
	 * @brief Return the buffer index that best matches @p comp_func, or -1 if the buffer is empty.
	 *
	 * @p comp_func returns true when its first argument is "better" than the second.
	 */
	template <typename Compare>
	int find_conflict_idx_in_buffer(Compare comp_func) const
	{
		const int buff_size = static_cast<int>(_traffic_buffer.size());

		if (buff_size == 0) {
			return -1;
		}

		int bestIndex = 0;

		for (int i = 1; i < buff_size; ++i) {
			if (comp_func(_traffic_buffer[i], _traffic_buffer[bestIndex])) {
				bestIndex = i;
			}
		}

		return bestIndex;
	}

	static bool is_conflict_more_important(const conflict_info_s &new_conflict, const conflict_info_s &base_conflict);
	static bool is_conflict_less_important(const conflict_info_s &new_conflict, const conflict_info_s &base_conflict);

	/** @brief Get the highest-priority conflict in the buffer, or false if empty. */
	bool find_most_urgent_conflict(conflict_info_s &most_urgent_conflict) const;

	/** @brief Get the lowest-priority conflict in the buffer (used to pick which conflict to remove from the buffer). */
	bool get_least_urgent_conflict(conflict_info_s &least_urgent_conflict, int &least_urgent_idx) const;

	/** @brief Recompute @c _most_urgent_conflict from the current buffer state. */
	void update_most_urgent_conflict();

	void reset_most_urgent_conflict();
	conflict_info_s _most_urgent_conflict{};
	uint8_t _prev_most_urgent_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;

	/* Handle actions */

	/**
	 * @brief Decide whether to publish a vehicle command for the current most-urgent conflict.
	 *
	 * Splits in-air escalation, in-air de-escalation, and on-ground warnings.
	 */
	void evaluate_and_publish_action();

	void publish_and_notify_action(const DaaAction requested_action);

	/** @brief Convert a DAA action into the matching vehicle_command and publish it. */
	void publish_action_command(const DaaAction requested_action);

	/**
	 * @brief Map a conflict level to a DAA action.
	 *
	 * For F3442, the zones are nested, so if the configured action is DISABLED the
	 * function falls back to the action of the next larger zone.
	 */
	DaaAction get_action_from_conflict_level(const uint8_t conflict_level) const;

	/** @brief Return the DAA action equivalent to the current nav state (used to gate escalation). */
	DaaAction nav_state_to_equivalent_daa_action(const uint8_t nav_state) const;

	DaaAction _previous_action = DaaAction::kDisabled;

	/* Process transponder data */
#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	uORB::Publication<transponder_report_s> _fake_traffic_pub {ORB_ID(transponder_report)};
	// The shell command and the navigator update loop can touch fake traffic state
	// from different threads, so protect the pending script with a mutex.
	pthread_mutex_t _fake_traffic_mutex = PTHREAD_MUTEX_INITIALIZER;
	fake_traffic_state_s _fake_traffic_state{};

	/** @brief Publish whichever scripted fake-traffic reports are due this spin. */
	void process_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	uORB::Subscription _traffic_sub {ORB_ID(transponder_report)};
	AdsbConflict _adsb_traffic;

	/**
	 * @brief Parse the ownship and transponder data, then hand them to the active DAA standard.
	 *
	 * Replaces non-finite UAV velocity components with zero so a partially-valid
	 * pose still produces a fixed-size F3442 check.
	 */
	bool analyse_transponder_report(transponder_report_s &transponder_report, detect_and_avoid_s &daa_output);

	/** @brief True if the latest global and local position fixes are finite and recent enough. */
	bool uav_pose_valid_and_updated() const;

	/** @brief True if the transponder flags carry every field the active standard needs. */
	bool transponder_flags_valid(const uint16_t flags) const;

	/** @brief True if the report has finite coords/altitude, the required flags, and a recent timestamp. */
	bool transponder_data_valid(const transponder_report_s &transponder_report) const;

	void process_daa_output(const detect_and_avoid_s &daa_output, const unique_id_s &unique_id, const uint64_t timestamp,
				conflict_info_s &current_conflict);

	/* Timers to avoid spamming */
	hrt_abstime _time_last_buffer_clean{0};
	hrt_abstime _time_last_status_notif{0};
	hrt_abstime _time_last_traffic_ignored{0};
	hrt_abstime _time_last_landed_warning{0};

	/* Notifications */
	orb_advert_t _mavlink_log_pub{nullptr};

	/** @brief Emit the per-spin conflict notifications. */
	void notify_if_needed(const new_conflicts_pending_notif_s &new_conflicts_pending_notif);

	/**
	 * @brief Decide whether to notify about a conflict level right now.
	 *
	 * Forces a notification on level transitions when either side requires a warning,
	 * otherwise rate-limits to one notification per @p interval.
	 */
	bool must_notify(const uint8_t current_conflict_level, const hrt_abstime time_last_notified,
			 const hrt_abstime interval, const uint8_t previous_conflict_level = 0) const;

	/** @brief Send the user-facing message and event for a newly-published DAA action. */
	void notify_new_action(const conflict_info_s &conflict_info, const DaaAction action);

	/** @brief Warn the operator that air traffic is present while the vehicle is on the ground. */
	void notify_action_on_ground(const NotifyLandedActCause cause);

	/**
	 * @brief Periodic status message for the most-urgent conflict.
	 *
	 * When @p new_and_main is true the severity is taken from the conflict level,
	 * otherwise it is routed through the generic info channel.
	 */
	void notify_daa_status(const conflict_info_s &conflict_info, const uint8_t previous_conflict_level,
			       bool new_and_main = false);

	void maybe_notify_ignored_traffic(const conflict_info_s &conflict, const IgnoreTrafficCause cause);

	/** @brief Warn the user that a traffic report was dropped, tagged with the underlying cause. */
	void notify_traffic_ignored(const conflict_info_s &conflict_info, const IgnoreTrafficCause cause);

	/** @brief Tell the user a previously-tracked conflict was removed (stale or buffer full). */
	void notify_traffic_removed(const conflict_info_s &conflict_info, const RemoveBufferCause cause);

	/** @brief Emit the first notification for a newly added conflict. */
	void notify_new_conflict(const conflict_info_s &conflict_info);

	/** @brief Emit escalation, de-escalation or "solved" notification for a secondary (non-main) conflict. */
	void notify_existing_traffic(const conflict_info_s &conflict_info, const uint8_t previous_conflict_level);

	/**
	 * @brief Send @p message_buffer over MAVLink with a severity derived from @p conflict_level.
	 *
	 * Returns false (and sends nothing) for NONE or out-of-range levels. Sets
	 * @p log_level so the caller can re-use the same severity for a matching event.
	 */
	bool mavlink_log_conflict_by_level(const uint8_t conflict_level, const char message_buffer[kMaxLogMsgSize],
					   events::Log &log_level);

	/* Helper functions */
	bool conflict_lvl_requires_warning(const uint8_t conflict_level) const;
	inline bool is_valid_buffer_idx(const int idx) const {return idx >= 0 && idx < static_cast<int>(_traffic_buffer.size());};
	inline bool is_buffer_empty() const {return _traffic_buffer.empty();};
	inline bool is_buffer_full() const {return static_cast<int>(_traffic_buffer.size()) >= kDaaMaxTraffic;};
	static inline bool is_icao_valid(const uint32_t icao) {return icao != 0;};
	static inline bool is_callsign_valid(const uint16_t flags) {return flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;};
	bool is_uas_id_valid(const uint8_t uas_id[PX4_GUID_BYTE_LENGTH]) const;

	int find_unique_id_in_buffer(const unique_id_s &target_unique_id) const;

	/** @brief True (and stamps @p last_time to now) if @p interval has passed since the previous trigger. */
	bool has_elapsed(hrt_abstime &last_time, const hrt_abstime interval);

	void convert_unique_id_to_string(const unique_id_s &unique_id, char *buffer, size_t buffer_size) const;
	static void convert_uas_id_uint64_to_str(const uint64_t uas_id_int, char uas_id_char_arr[kUtmGuidMsgLength]);
	static bool is_null_terminated(const char *array, size_t max_length);
	static bool same_unique_id(const unique_id_s &lhs, const unique_id_s &rhs);
	static bool pending_new_conflict_notification_exists(const unique_id_s &target_unique_id,
			const new_conflicts_pending_notif_s &new_conflicts_pending_notif);

	/* Debug functions */
#if defined(DEBUG_BUILD)
	void debug_print_transponder_report(const transponder_report_s &transponder_report);
	void debug_print_conflict_info(const conflict_info_s &conflict);
	void debug_print_buffer_status();
#endif

	/* Parameters */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	/** @brief Consume a pending parameter_update notification and re-arm DAA if needed. */
	void check_param_change();

	/** @brief Translate the user-facing action param value into the internal DaaAction enum. */
	static DaaAction action_param_to_daa_action(int32_t action_param);

	bool try_setting_lib_params() {return _adsb_traffic.try_updating_params();};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DAA_EN>) _param_daa_en,
		(ParamFloat<px4::params::DAA_DFLT_VEL>) _param_daa_dflt_vel,
		(ParamInt<px4::params::DAA_EN_DFLT_VEL>) _param_daa_en_dflt_vel,
		(ParamInt<px4::params::DAA_STANDARD>) _param_daa_standard,
		(ParamInt<px4::params::DAA_NOTIF_STATE>) _param_daa_notif_state,
		(ParamInt<px4::params::DAA_TRAFF_TOUT>) _param_daa_traff_tout,
		(ParamInt<px4::params::DAA_LVL_LOW_ACT>) _param_daa_lvl_low_act,
		(ParamInt<px4::params::DAA_LVL_MED_ACT>) _param_daa_lvl_med_act,
		(ParamInt<px4::params::DAA_LVL_HIGH_ACT>) _param_daa_lvl_high_act,
		(ParamInt<px4::params::DAA_LVL_CRIT_ACT>) _param_daa_lvl_crit_act,
		(ParamInt<px4::params::NAV_TRAFF_AVOID>) _param_nav_traff_avoid,
		(ParamInt<px4::params::ADSB_ICAO_ID>)		_vehicle_adsb_icao,
		(ParamInt<px4::params::ADSB_ICAO_ID_2>)		_vehicle_adsb_icao_2,
		(ParamInt<px4::params::ADSB_CALLSIGN_1>)		_vehicle_adsb_callsign_1,
		(ParamInt<px4::params::ADSB_CALLSIGN_2>)		_vehicle_adsb_callsign_2
	)

	uint8_t _active_daa_standard{detect_and_avoid_s::DAA_STANDARD_F3442};
	int32_t _daa_lvl_low_action_param{1};
	int32_t _daa_lvl_med_action_param{1};
	int32_t _daa_lvl_high_action_param{1};
	int32_t _daa_lvl_crit_action_param{1};
	int32_t _nav_traff_avoid_mode{1};
};
