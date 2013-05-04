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

void convert_setpoint_bodyframe2local(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		)
{
	static float wp_bodyframe_offset[3] = {0.0f, 0.0f, 0.0f};
	static float wp_local_offset[3] = {0.0f, 0.0f, 0.0f};

	wp_bodyframe_offset[0] = bodyframe_pos_sp->x - bodyframe_pos->x;
	wp_bodyframe_offset[1] = bodyframe_pos_sp->y - bodyframe_pos->y;
	wp_bodyframe_offset[2] = 0; // no influence of z...

	/* calc current waypoint cooridnates in local */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + wp_bodyframe_offset[j] * att->R[i][j];
		}
		wp_local_offset[i] = sum;
	}

	local_pos_sp->x = local_pos->x + wp_local_offset[0];
	local_pos_sp->y = local_pos->y + wp_local_offset[1];
	local_pos_sp->z = bodyframe_pos_sp->z; // let z as it is...
	local_pos_sp->yaw = bodyframe_pos_sp->yaw;
}

void convert_setpoint_local2bodyframe(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_local_position_setpoint_s *local_pos_sp,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp
		)
{
	static float wp_local_offset[3] = {0.0f, 0.0f, 0.0f}; // x,y
	static float wp_bodyframe_offset[3] = {0.0f, 0.0f, 0.0f};

	wp_local_offset[0] = local_pos_sp->x - local_pos->x;
	wp_local_offset[1] = local_pos_sp->y - local_pos->y;
	wp_local_offset[2] = 0; // no influence of z...

	/* calc current waypoint cooridnates in bodyframe */
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + wp_local_offset[j] * att->R[j][i];
		}
		wp_bodyframe_offset[i] = sum;
	}

	bodyframe_pos_sp->x = bodyframe_pos->x + wp_bodyframe_offset[0];
	bodyframe_pos_sp->y = bodyframe_pos->y + wp_bodyframe_offset[1];
	bodyframe_pos_sp->z = local_pos_sp->z; // let z as it is...
	bodyframe_pos_sp->yaw = local_pos_sp->yaw;

}

float get_yaw(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		)
{
	float dx = local_pos_sp->x - local_pos->x;
	float dy = local_pos_sp->y - local_pos->y;

	return atan2f(dy,dx);
}

void init_state(struct mission_state_s *state) {
	/* reset all state variables */
	state->state = MISSION_RESETED;
	state->final_sequence = false;
	state->initialized = false;

	/* reset front situation */
	state->sonar_obstacle.valid = false;
	state->sonar_obstacle.updated = false;
	state->free_to_go = false;

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

void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params,
		int mavlink_fd, struct discrete_radar_s *new_radar) {

	/* fill polar obstacle */
	float sonar_obstacle_polar[3];
	sonar_obstacle_polar[0] = current_state->sonar_obstacle.sonar_obst_polar_r;
	sonar_obstacle_polar[1] = current_state->sonar_obstacle.sonar_obst_polar_alpha;
	sonar_obstacle_polar[2] = current_state->sonar_obstacle.sonar_obst_pitch;

	/* fill sonar flags */
	bool sonar_flags[2];
	sonar_flags[0] = current_state->sonar_obstacle.valid;
	sonar_flags[1] = false;

	/* load current control values */
	float yaw_control = current_state->step.yaw;
	float x_control = current_state->step.x;
	float y_control = current_state->step.y;
	bool free_environment = true;

	free_environment = radarControl(new_radar->distances, new_radar->sonar, sonar_obstacle_polar, sonar_flags,
			params->radarControlSettings, &x_control, &y_control, &yaw_control);

	if(isfinite(x_control) && isfinite(y_control) && isfinite(yaw_control)) {
		current_state->step.x = x_control;
		current_state->step.y = y_control;
		current_state->step.yaw = yaw_control;

		current_state->sonar_obstacle.sonar_obst_pitch = sonar_obstacle_polar[2];
		current_state->sonar_obstacle.valid  = sonar_flags[0];
		current_state->sonar_obstacle.updated  = sonar_flags[1];

		if (current_state->sonar_obstacle.updated) {
			current_state->sonar_obstacle.sonar_obstacle_bodyframe.x = sonar_obstacle_polar[0];
			current_state->sonar_obstacle.sonar_obstacle_bodyframe.y = sonar_obstacle_polar[1];
		}

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

	} else {
		mavlink_log_critical(mavlink_fd, "[mission commander] NAN...");
		current_state->step.x = 0.0f;
		current_state->step.y = 0.0f;
		current_state->step.yaw = 0.0f;
	}

}


