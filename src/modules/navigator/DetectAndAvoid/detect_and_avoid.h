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
#include "ConflictNotifier.h"

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
#include <pthread.h>
#endif

#include <drivers/drv_hrt.h>
#include <lib/adsb/AdsbConflict.h>
#include <lib/adsb/ConflictTracker.h>
#include <lib/adsb/DaaEncodedId.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/detect_and_avoid.h>
#include <uORB/topics/detect_and_avoid_most_urgent.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/transponder_report.h>

using namespace time_literals;

class DetectAndAvoid : public MissionBlock, public ModuleParams
{
public:
	DetectAndAvoid(Navigator *navigator);
	~DetectAndAvoid() override = default;

	void on_activation() override;
	void on_active() override;
	void on_inactivation() override;

	void reset();

	bool is_activated() const { return _is_activated; }

#if !defined(CONSTRAINED_FLASH) && !defined(__PX4_NUTTX)
	void print_status() const;
#endif // !CONSTRAINED_FLASH && !__PX4_NUTTX

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	static constexpr uint16_t kFakeTrafficDefaultFlags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
			transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN |
			transponder_report_s::PX4_ADSB_FLAGS_SIMULATED;

	// Fake traffic scripts used for manual DAA validation from the navigator shell.
	enum class FakeTraffMode : uint8_t {
		kUniqueIds = 0,
		kEscalation = 1,
		kSpamSame = 2,
		kSpamNew = 3,
		kFlags = 4,
		kQueueFill = 5
	};

	struct SyntheticTrafficReport {
		uint32_t icao_address{0};
		const char *callsign{""};
		float distance{0.f};
		float direction{0.f};
		float traffic_heading{0.f};
		float altitude_diff{10.f};
		float hor_velocity{100.f};
		float ver_velocity{10.f};
		int emitter_type{transponder_report_s::ADSB_EMITTER_TYPE_LIGHT};
		double lat_uav{45.35324098};
		double lon_uav{6.446453};
		float alt_uav{300.f};
		uint16_t flags{kFakeTrafficDefaultFlags};
	};
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	DaaAction get_action_from_conflict_level(const uint8_t conflict_level) const;

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	/**
	 * @brief Arm a fake-traffic script from the navigator shell for manual DAA testing.
	 *
	 * Uses the ownship position so the fake targets stay anchored to a fixed origin.
	 */
	void run_fake_traffic(FakeTraffMode mode, double lat_uav, double lon_uav, float alt_uav);

	void stop_fake_traffic();

	void fake_traffic(const SyntheticTrafficReport &report);
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	conflict_info_s get_most_urgent_conflict() const { return _conflict_tracker.most_urgent(); }

private:
	static constexpr hrt_abstime kOwnshipPositionTimeout{2_s};

	// Change records collected by the conflict tracker over one detection cycle.
	// Static so the ~1.8 KB buffer lives in .bss (AXI_SRAM on FMU targets).
	static conflict_tracker_changes_s _cycle_changes;

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	struct fake_traffic_origin_s {
		double lat{0.0};
		double lon{0.0};
		float alt{0.f};
	};

	struct fake_traffic_state_s {
		bool active{false};
		FakeTraffMode mode{FakeTraffMode::kUniqueIds};
		uint8_t next_step_idx{0};
		hrt_abstime next_publish_at{0};
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

	void publish_most_urgent_conflict_if_changed();
	void clear_conflicts();
	void update_activation_status();
	void process_traffic();

	bool _is_activated{false};

	ConflictTracker _conflict_tracker{};
	ConflictNotifier _conflict_notifier{};

	ConflictNotifier::cycle_context_s notifier_cycle_context() const;
	void update_most_urgent_conflict();

	bool _most_urgent_conflict_changed{false};
	uint8_t _prev_most_urgent_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;

	void evaluate_and_publish_action();
	void publish_action_command(const DaaAction requested_action);
	daa_action_params_s action_params() const;

	DaaAction _previous_action = DaaAction::kDisabled;

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	uORB::Publication<transponder_report_s> _fake_traffic_pub {ORB_ID(transponder_report)};
	// The shell command and the navigator update loop can touch fake traffic state
	// from different threads, so protect the pending script with a mutex.
	pthread_mutex_t _fake_traffic_mutex = PTHREAD_MUTEX_INITIALIZER;
	fake_traffic_state_s _fake_traffic_state{};

	void process_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	uORB::Subscription _traffic_sub {ORB_ID(transponder_report)};
	AdsbConflict _adsb_conflict_detector;

	daa_ownship_ids_s _ownship_ids{};

	void refresh_ownship_ids();

	bool gather_ownship_input(daa_input_s &daa_input) const;

	static bool transponder_data_valid(const transponder_report_s &report, hrt_abstime now, hrt_abstime timeout_us);

	/**
	 * @brief Process queued transponder reports and update the conflict buffer.
	 *
	 * @p daa_input carries the ownship state (filled once per cycle); the traffic half is
	 * overwritten in place for each processed report, avoiding a copy for each report.
	 */
	bool process_transponder_queue(daa_input_s &daa_input);

	bool process_transponder_report(daa_input_s &daa_input, const transponder_report_s &transponder_report);

	hrt_abstime _time_last_buffer_clean{0};

	/* Debug functions */
#if defined(DEBUG_BUILD)
	void debug_print_transponder_report(const transponder_report_s &transponder_report);
	void debug_print_conflict_info(const conflict_info_s &conflict);
	void debug_print_buffer_status();
#endif

	/* Parameters */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DAA_EN>) _param_daa_en,
		(ParamInt<px4::params::DAA_NOTIF_STATE>) _param_daa_notif_state,
		(ParamInt<px4::params::DAA_TRAFF_TOUT>) _param_daa_traff_tout,
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
		(ParamFloat<px4::params::DAA_DFLT_VEL>) _param_daa_dflt_vel,
		(ParamInt<px4::params::DAA_EN_DFLT_VEL>) _param_daa_en_dflt_vel,
		(ParamInt<px4::params::DAA_LVL_LOW_ACT>) _param_daa_lvl_low_act,
		(ParamInt<px4::params::DAA_LVL_MED_ACT>) _param_daa_lvl_med_act,
		(ParamInt<px4::params::DAA_LVL_HIGH_ACT>) _param_daa_lvl_high_act,
		(ParamInt<px4::params::DAA_LVL_CRIT_ACT>) _param_daa_lvl_crit_act,
#else
		(ParamInt<px4::params::NAV_TRAFF_AVOID>) _param_nav_traff_avoid,
#endif // CONFIG_NAVIGATOR_ADSB_F3442
		(ParamInt<px4::params::ADSB_ICAO_ID>) _vehicle_adsb_icao,
		(ParamInt<px4::params::ADSB_ICAO_ID_2>) _vehicle_adsb_icao_2,
		(ParamInt<px4::params::ADSB_CALLSIGN_1>) _vehicle_adsb_callsign_1,
		(ParamInt<px4::params::ADSB_CALLSIGN_2>) _vehicle_adsb_callsign_2
	)
};
