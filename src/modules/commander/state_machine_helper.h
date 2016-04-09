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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>

typedef enum {
	TRANSITION_DENIED = -1,
	TRANSITION_NOT_CHANGED = 0,
	TRANSITION_CHANGED

} transition_result_t;

bool is_safe(const struct vehicle_status_s *current_state, const struct safety_s *safety, const struct actuator_armed_s *armed);

transition_result_t arming_state_transition(struct vehicle_status_s *current_state, const struct safety_s *safety,
		arming_state_t new_arming_state, struct actuator_armed_s *armed, bool fRunPreArmChecks, orb_advert_t *mavlink_log_pub);

transition_result_t main_state_transition(struct vehicle_status_s *current_state, main_state_t new_main_state);

transition_result_t hil_state_transition(hil_state_t new_state, orb_advert_t status_pub, struct vehicle_status_s *current_state, orb_advert_t *mavlink_log_pub);

bool set_nav_state(struct vehicle_status_s *status, const bool data_link_loss_enabled, const bool mission_finished, const bool stay_in_failsafe);

int preflight_check(struct vehicle_status_s *status, orb_advert_t *mavlink_log_pub, bool prearm, bool force_report=false);

#endif /* STATE_MACHINE_HELPER_H_ */
