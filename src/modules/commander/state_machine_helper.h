/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file state_machine_helper.h
 * State machine helper functions definitions
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef STATE_MACHINE_HELPER_H_
#define STATE_MACHINE_HELPER_H_

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_status_flags.h>

typedef enum {
    TRANSITION_DENIED = -1,
    TRANSITION_NOT_CHANGED = 0,
    TRANSITION_CHANGED
} transition_result_t;

enum class link_loss_actions_t {
    DISABLED = 0,
    AUTO_LOITER = 1,
    AUTO_RTL = 2,
    AUTO_LAND = 3,
    AUTO_RECOVER = 4,
    TERMINATE = 5,
    LOCKDOWN = 6,
};

typedef enum {
	ARM_REQ_NONE = 0,
	ARM_REQ_MISSION_BIT = (1 << 0),
	ARM_REQ_ARM_AUTH_BIT = (1 << 1),
	ARM_REQ_GPS_BIT = (1 << 2),
} arm_requirements_t;

extern const char *const arming_state_names[];

bool is_safe(const struct safety_s *safety, const struct actuator_armed_s *armed);

transition_result_t arming_state_transition(struct vehicle_status_s *status,
					    struct battery_status_s *battery,
					    const struct safety_s *safety,
					    arming_state_t new_arming_state,
					    struct actuator_armed_s *armed,
					    bool fRunPreArmChecks,
					    orb_advert_t *mavlink_log_pub,        ///< uORB handle for mavlink log
					    vehicle_status_flags_s *status_flags,
					    float avionics_power_rail_voltage,
					    uint8_t arm_requirements,
					    hrt_abstime time_since_boot);

transition_result_t
main_state_transition(struct vehicle_status_s *status, main_state_t new_main_state, uint8_t &main_state_prev,
		      vehicle_status_flags_s *status_flags, struct commander_state_s *internal_state);

transition_result_t hil_state_transition(hil_state_t new_state, orb_advert_t status_pub, struct vehicle_status_s *current_status, orb_advert_t *mavlink_log_pub);

void enable_failsafe(struct vehicle_status_s *status, bool old_failsafe,
		     orb_advert_t *mavlink_log_pub, const char *reason);

bool set_nav_state(struct vehicle_status_s *status,
		   struct actuator_armed_s *armed,
		   struct commander_state_s *internal_state,
		   orb_advert_t *mavlink_log_pub,
		   const link_loss_actions_t data_link_loss_act,
		   const bool mission_finished,
		   const bool stay_in_failsafe,
		   vehicle_status_flags_s *status_flags,
		   bool landed,
		   const link_loss_actions_t rc_loss_act,
		   const int offb_loss_act,
		   const int offb_loss_rc_act,
		   const int posctl_nav_loss_act);

/*
 * Checks the validty of position data aaainst the requirements of the current navigation
 * mode and switches mode if position data required is not available.
 */
bool check_invalid_pos_nav_state(struct vehicle_status_s *status,
			       bool old_failsafe,
			       orb_advert_t *mavlink_log_pub,
			       vehicle_status_flags_s *status_flags,
			       const bool use_rc, // true if a mode using RC control can be used as a fallback
			       const bool using_global_pos); // true when the current mode requires a global position estimate

void set_rc_loss_nav_state(vehicle_status_s *status, actuator_armed_s *armed, vehicle_status_flags_s *status_flags,
						commander_state_s *internal_state, const link_loss_actions_t link_loss_act);

void set_data_link_loss_nav_state(vehicle_status_s *status, actuator_armed_s *armed, vehicle_status_flags_s *status_flags,
						commander_state_s *internal_state, const link_loss_actions_t link_loss_act);

int prearm_check(struct vehicle_status_s *status, orb_advert_t *mavlink_log_pub, bool prearm,
		    bool force_report, vehicle_status_flags_s *status_flags, battery_status_s *battery,
		    uint8_t arm_requirements, hrt_abstime time_since_boot);

#endif /* STATE_MACHINE_HELPER_H_ */
