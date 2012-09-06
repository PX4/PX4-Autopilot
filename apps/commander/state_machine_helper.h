/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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
 */

#ifndef STATE_MACHINE_HELPER_H_
#define STATE_MACHINE_HELPER_H_

#define GPS_NOFIX_COUNTER_LIMIT 4 //need GPS_NOFIX_COUNTER_LIMIT gps packets with a bad fix to call an error (if outdoor)
#define GPS_GOTFIX_COUNTER_REQUIRED 4 //need GPS_GOTFIX_COUNTER_REQUIRED gps packets with a good fix to obtain position lock

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>

/**
 * Switch to new state with no checking.
 *
 * do_state_update: this is the functions that all other functions have to call in order to update the state.
 * the function does not question the state change, this must be done before
 * The function performs actions that are connected with the new state (buzzer, reboot, ...)
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 *
 * @return 0 (macro OK) or 1 on error (macro ERROR)
 */
int do_state_update(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, commander_state_machine_t new_state);

/* These functions decide if an emergency exits and then switch to SYSTEM_STATE_MISSION_ABORT or SYSTEM_STATE_GROUND_ERROR */
// void update_state_machine_subsystem_present(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);
// void update_state_machine_subsystem_notpresent(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);

// void update_state_machine_subsystem_enabled(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);
// void update_state_machine_subsystem_disabled(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);

// void update_state_machine_subsystem_healthy(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);
// void update_state_machine_subsystem_unhealthy(int status_pub, struct vehicle_status_s *current_status, subsystem_type_t *subsystem_type);


/**
 * Handle state machine if got position fix
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_got_position_fix(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if position fix lost
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_no_position_fix(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if user wants to arm
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_arm(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if user wants to disarm
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_disarm(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if mode switch is manual
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_mode_manual(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if mode switch is stabilized
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_mode_stabilized(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Handle state machine if mode switch is auto
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void update_state_machine_mode_auto(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Publish current state information
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void state_machine_publish(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);


/*
 *  Functions that handle incoming requests to change the state machine or a parameter (probably from the mavlink app).
 *  If the request is obeyed the functions return 0
 *
 */

/**
 * Handles *incoming request* to switch to a specific state, if state change is successful returns 0
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
uint8_t update_state_machine_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t mode);

/**
 * Handles *incoming request* to switch to a specific custom state, if state change is successful returns 0
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
uint8_t update_state_machine_custom_mode_request(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd, uint8_t custom_mode);

/**
 * Always switches to critical mode under any circumstances.
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void state_machine_emergency_always_critical(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Switches to emergency if required.
 *
 * @param status_pub file descriptor for state update topic publication
 * @param current_status pointer to the current state machine to operate on
 * @param mavlink_fd file descriptor for MAVLink statustext messages
 */
void state_machine_emergency(int status_pub, struct vehicle_status_s *current_status, const int mavlink_fd);

/**
 * Publish the armed state depending on the current system state
 *
 * @param current_status the current system status
 */
void publish_armed_status(const struct vehicle_status_s *current_status);



#endif /* STATE_MACHINE_HELPER_H_ */
