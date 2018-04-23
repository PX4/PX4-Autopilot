/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#ifndef COMMANDER_HPP_
#define COMMANDER_HPP_

#include "state_machine_helper.h"

#include <controllib/blocks.hpp>
#include <px4_module.h>
#include <px4_module_params.h>

// publications
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using uORB::Publication;
using uORB::Subscription;

class Commander : public ModuleBase<Commander>, public ModuleParams
{
public:
	Commander();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Commander *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	void enable_hil();

	// TODO: only temporarily static until low priority thread is removed
	static bool preflight_check(bool report);

private:
	bool handle_command(vehicle_status_s *status_local, const vehicle_command_s &cmd,
			    actuator_armed_s *armed_local, home_position_s *home, const vehicle_global_position_s &global_pos,
			    const vehicle_local_position_s &local_pos, orb_advert_t *home_pub, orb_advert_t *command_ack_pub, bool *changed);

	bool set_home_position(orb_advert_t &homePub, home_position_s &home, const vehicle_local_position_s &localPosition,
			       const vehicle_global_position_s &globalPosition, bool set_alt_only_to_lpos_ref);

	// Set the main system state based on RC and override device inputs
	transition_result_t set_main_state(const vehicle_status_s &status, const vehicle_global_position_s &global_position,
					   const vehicle_local_position_s &local_position, bool *changed);

	// Enable override (manual reversion mode) on the system
	transition_result_t set_main_state_override_on(const vehicle_status_s &status_local, bool *changed);

	// Set the system main state based on the current RC inputs
	transition_result_t set_main_state_rc(const vehicle_status_s &status_local,
					      const vehicle_global_position_s &global_position, const vehicle_local_position_s &local_position, bool *changed);

	void check_valid(const hrt_abstime &timestamp, const hrt_abstime &timeout, const bool valid_in, bool *valid_out,
			 bool *changed);

	bool check_posvel_validity(const bool data_valid, const float data_accuracy, const float required_accuracy,
				   const hrt_abstime &data_timestamp_us, hrt_abstime *last_fail_time_us, hrt_abstime *probation_time_us, bool *valid_state,
				   bool *validity_changed);

	void reset_posvel_validity(const vehicle_global_position_s &global_position,
				   const vehicle_local_position_s &local_position, bool *changed);

	void mission_init();

	/**
	 * Update the telemetry status and the corresponding status variables.
	 * Perform system checks when new telemetry link connected.
	 */
	void poll_telemetry_status();

	/**
	 * Checks the status of all available data links and handles switching between different system telemetry states.
	 */
	void data_link_checks(int32_t highlatencydatalink_loss_timeout, int32_t highlatencydatalink_regain_timeout,
			      int32_t datalink_loss_timeout, int32_t datalink_regain_timeout, bool *status_changed);

	// telemetry variables
	struct telemetry_data {
		int subscriber = -1;
		uint64_t last_heartbeat = 0u;
		uint64_t last_dl_loss = 0u;
		bool preflight_checks_reported = false;
		bool lost = true;
		bool high_latency = false;
	} _telemetry[ORB_MULTI_MAX_INSTANCES];

	// publisher
	orb_advert_t _vehicle_cmd_pub = nullptr;

	// subscriptions
	Subscription<mission_result_s> _mission_result_sub;
};

#endif /* COMMANDER_HPP_ */
