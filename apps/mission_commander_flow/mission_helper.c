/*
 * mission_helper.c
 *
 *  Created on: Apr 12, 2013
 *      Author: samuezih
 */

#include "mission_helper.h"
#include "mission_sounds.h"
#include <mavlink/mavlink_log.h>


void init_state(struct mission_state_s *state) {
	/* reset all state variables */
	state->state = MISSION_RESETED;
	state->state_counter = 0;
	state->final_sequence = false;

	state->radar_previous = RADAR_CLEAR;
	state->radar_current = RADAR_CLEAR;
	state->radar_next = RADAR_CLEAR;
	state->front_free = true;
	state->front_react = false;
	state->wall_left = false;
	state->wall_left = false;

	state->react = REACT_TEST;
	state->reaction_counter = 0;

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

	} else if (new_state == MISSION_ABORTED) {

		printf("[mission commander] mission reseted.\n");
		mavlink_log_info(mavlink_fd, "[mission commander] mission reseted.");

	}
}

void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params, int mavlink_fd, struct discrete_radar_s *new_radar) {

	/* check new front condition */
	for(int i = 14; i<19; i++)
	{
		if (new_radar->distances[i] < params->mission_min_front_dist) {
			current_state->front_free = false;
		} else if(new_radar->distances[i] < params->mission_react_front_dist) {
			current_state->front_react = true;
		}
	}

	if(!current_state->front_free) {
		/* stand still and make nothing waiting for better weather */
		/* TODO add problem solving */
		printf("[mission commander] too close to wall.\n");
		do_state_update(&current_state, mavlink_fd, MISSION_ABORTED);
		return -1;
	}

	/* check new side-freeness */
	int left = (new_radar->distances[9] + new_radar->distances[10] + new_radar->distances[11]) / 3;
	int right = (new_radar->distances[21] + new_radar->distances[22] + new_radar->distances[23]) / 3;

	if (left < params->mission_react_side_dist) {
		current_state->wall_left = true;
	} else {
		current_state->wall_left = false;
	}
	if (right < params->mission_react_side_dist) {
		current_state->wall_right = true;
	} else {
		current_state->wall_right = false;
	}

	if(current_state->radar_current == RADAR_CLEAR) {
		/* TODO what if a wall appears? */
		if(current_state->front_react) {
			/* obstacle is detected */
			if (left > right) {
				current_state->radar_current = RADAR_REACT_LEFT;
				current_state->react = REACT_TURN;
				mavlink_log_info(mavlink_fd, "[mission commander] react left");
				current_state->reaction_counter = 0;
			} else {
				current_state->radar_current = RADAR_REACT_RIGHT;
				current_state->react = REACT_TURN;
				mavlink_log_info(mavlink_fd, "[mission commander] react right");
				current_state->reaction_counter = 0;
			}
		} else if (left < params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_WALL_L;
			mavlink_log_info(mavlink_fd, "[mission commander] follow left wall");
		} else if (right < params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_WALL_R;
			mavlink_log_info(mavlink_fd, "[mission commander] follow right wall");
		}

	} else if (current_state->radar_current == RADAR_FOLLOW_WALL_L ) {
		/* TODO what if a wall appears? */
		if(current_state->front_react) {
			/* obstacle is detected */
			current_state->radar_current = RADAR_REACT_RIGHT;
			current_state->react = REACT_TURN;
			mavlink_log_info(mavlink_fd, "[mission commander] react right");
			current_state->reaction_counter = 0;

		} else if (left > params->mission_react_side_dist) {
			current_state->radar_current = RADAR_CLEAR;
		} else if (right < params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_CORRIDOR;
		}

	} else if (current_state->radar_current == RADAR_FOLLOW_WALL_R) {
		/* TODO what if a wall appears? */
		if(current_state->front_react) {
			/* obstacle is detected */
			current_state->radar_current = RADAR_REACT_LEFT;
			current_state->react = REACT_TURN;
			mavlink_log_info(mavlink_fd, "[mission commander] react left");
			current_state->reaction_counter = 0;

		} else if (right > params->mission_react_side_dist) {
			current_state->radar_current = RADAR_CLEAR;
		} else if (left < params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_CORRIDOR;
		}

	} else if (current_state->radar_current == RADAR_FOLLOW_CORRIDOR) {
		/* TODO what if a front wall appears? */
		if (right > params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_WALL_L;
		} else if (left > params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_WALL_R;
		}

	} else if (	current_state->radar_current == RADAR_REACT_LEFT ||
				current_state->radar_current == RADAR_REACT_RIGHT ) {

		/* do we need a state change */
		int front_dist = (new_radar->distances[15] + new_radar->distances[16] + new_radar->distances[17]) / 3;

		if (current_state->wall_left && current_state->wall_right) {
			/* continue until both walls are too near */
			if (	(current_state->react == RADAR_REACT_LEFT && left < params->mission_min_side_dist) ||
					(current_state->radar_current == RADAR_REACT_RIGHT && right < params->mission_min_side_dist)
				) {
				/* stand still and make nothing waiting for better weather */
				/* TODO add problem solving */
				printf("[mission commander] too close to side walls.\n");
				do_state_update(&current_state, mavlink_fd, MISSION_ABORTED);
				return -1;
			}

		}

		if (current_state->react == REACT_TURN) {

			if (front_dist > params->mission_react_front_dist && current_state->reaction_counter > params->mission_min_reaction_steps) {
				current_state->react = REACT_PASS_OBJECT;

				if (current_state->radar_current == RADAR_REACT_LEFT) {
					mavlink_log_info(mavlink_fd, "[mission commander] pass right");

				} else {
					mavlink_log_info(mavlink_fd, "[mission commander] pass left");
				}
				/* reset counter to use it for pass counter */
				current_state->reaction_counter;
			}

		} else if (current_state->react == REACT_PASS_OBJECT) {

			if (current_state->reaction_counter > params->mission_min_free_steps) {
				/* if way is free change mission state */
				bool free_radar = true;
				for(int i = 0; i<32; i++) {

					if(new_radar->distances[i] < params->mission_react_side_dist) {
						free_radar = false;
					}
				}
				if (free_radar) {
					current_state->react = REACT_TEST;
					current_state->reaction_counter = 0;
					mavlink_log_info(mavlink_fd, "[mission commander] clear");
				}

			}
		} else if (current_state->react == REACT_PASS_OBJECT) {

			if (current_state->reaction_counter > params->mission_min_free_steps) {
				current_state->radar_current = RADAR_CLEAR;
			}
		}

	}

	return;
}


