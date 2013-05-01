/*
 * mission_helper.c
 *
 *  Created on: Apr 12, 2013
 *      Author: samuezih
 */

#include "mission_helper.h"
#include "mission_sounds.h"
#include "codegen/radarControl.h"
#include <mavlink/mavlink_log.h>


void init_state(struct mission_state_s *state) {
	/* reset all state variables */
	state->state = MISSION_RESETED;
	state->final_sequence = false;
	state->initialized = false;

	/* reset front situation */
	state->sonar = SONAR_NO_STATE;
	for (int i = 0; i < 4; i++) {
		state->front_situation[i] = 0;
	}
	state->free_to_go = false;
	state->waypoint_set = false;

	/*DEBUG*/
	state->debug_value1 = 0.0f;
	state->debug_value2 = 0.0f;
	state->debug_value3 = 0;
	state->debug_value4 = 0;

	/* reset step */
	state->step.x = 0.0f;
	state->step.y = 0.0f;
	state->step.yaw = 0.0f;

}

void do_state_update(struct mission_state_s *current_state, int mavlink_fd, mission_state_t new_state) {

	/* reset all state variables */
	init_state(current_state);
	current_state->state = new_state;

	if (new_state == MISSION_READY) {

		printf("[mission commander] mission ready.\n");
		//mavlink_log_info(mavlink_fd, "[mission commander] mission ready");

	} else if (new_state == MISSION_STARTED) {

		printf("[mission commander] mission started.\n");
		mavlink_log_info(mavlink_fd, "[mission commander] mission started");
		tune_mission_started();

	} else if (new_state == MISSION_ACCOMPLISHED) {

		printf("[mission commander] mission accomplished.\n");
		mavlink_log_info(mavlink_fd, "[mission commander] mission accomplished.");
		tune_mission_accomplished();

	} else if (new_state == MISSION_ABORTED) {

		printf("[mission commander] mission aborted.\n");
		mavlink_log_info(mavlink_fd, "[mission commander] mission aborted.");
		tune_mission_aborted();

	} else if (new_state == MISSION_RESETED) {

		printf("[mission commander] mission reseted.\n");
		mavlink_log_info(mavlink_fd, "[mission commander] mission reseted.");

	}
}

void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params, int mavlink_fd,
		struct discrete_radar_s *new_radar, float x_update, float y_update, float yaw_update) {

	/* test if enough space */
	if ((new_radar->sonar * 1000.0f) < params->mission_min_front_dist) {
		/* abord mission */
		do_state_update(current_state, mavlink_fd, MISSION_ABORTED);
		return;
	}

	/* load current control values */
	float yaw_control = current_state->step.yaw;
	float x_control = current_state->step.x;
	float y_control = current_state->step.y;
	bool free_environment = true;

	free_environment = radarControl(new_radar->distances, new_radar->sonar, current_state->front_situation,
			params->radarControlSettings, x_update, y_update, yaw_update, &x_control, &y_control, &yaw_control);

	current_state->step.x = x_control;
	current_state->step.y = y_control;
	current_state->step.yaw = yaw_control;

	/* log state changes */
	if (current_state->free_to_go) {
		if (!free_environment) {
			mavlink_log_info(mavlink_fd, "[mission commander] radar controlled...");
		}
	} else {
		if (free_environment) {
			mavlink_log_info(mavlink_fd, "[mission commander] free to go...");
		}
	}

	current_state->free_to_go = free_environment;
}


