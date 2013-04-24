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
	state->state_counter = 0;
	state->final_sequence = false;

	state->radar_previous = RADAR_NO_STATE;
	state->radar_current = RADAR_CLEAR;
	state->radar_next = RADAR_NO_STATE;
	state->wall_left = false;
	state->wall_left = false;

	state->react_current = REACT_TEST;
	state->react_next = REACT_NO_STATE;
	state->reaction_counter = 0;

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
		struct discrete_radar_s *new_radar, struct omnidirectional_flow_s *omni_flow) {

	/* if the next reaction state is overdue -> set it as current*/
	if (current_state->react_next != REACT_NO_STATE) {
		if (current_state->reaction_counter == 0) {
			current_state->react_current = current_state->react_next;
			current_state->react_next = REACT_NO_STATE;
		}
	}

	bool front_free = true;
	bool front_react = false;
	int minimum_front_distance = 5000;

	/* check new front condition */
	for(int i = 14; i<19; i++)
	{
		if (new_radar->distances[i] < params->mission_min_front_dist) {
			front_free = false;
		} else if(new_radar->distances[i] < params->mission_react_front_dist) {
			front_react = true;
		}

		if (new_radar->distances[i] < minimum_front_distance) {
			minimum_front_distance = new_radar->distances[i];
		}
	}

	/* check new frontal wall situation */


	if(!front_free) {
		/* stand still and make nothing waiting for better weather */
		/* TODO add problem solving */
		printf("[mission commander] too close to wall.\n");
		do_state_update(current_state, mavlink_fd, MISSION_ABORTED);
		return;
	}

	/* check new side-freeness */
	int left_full = (	new_radar->distances[9] + 	new_radar->distances[10] + new_radar->distances[11] +
						new_radar->distances[12] + 	new_radar->distances[13] + new_radar->distances[14]) / 6;
	int right_full = (	new_radar->distances[18] + 	new_radar->distances[19] + new_radar->distances[20] +
						new_radar->distances[21] + 	new_radar->distances[22] + new_radar->distances[23]) / 6;

	int left = 		(	new_radar->distances[9] + new_radar->distances[10] + new_radar->distances[11]) / 3;
	int right = 	(	new_radar->distances[21] + new_radar->distances[22] + new_radar->distances[23]) / 3;

	current_state->debug_value1 = (float) left_full;
	current_state->debug_value2 = (float) right_full;
	current_state->debug_value3 = new_radar->distances[9];
	current_state->debug_value4 = new_radar->distances[23];

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
		if(front_react) {
			/* obstacle is detected */

			/* TODO we have a BUG BUG HERE !!! */
			if (left_full > right_full) {
				current_state->radar_current = RADAR_REACT_LEFT;
				current_state->react_current = REACT_TURN;
				mavlink_log_info(mavlink_fd, "[mission commander] react left");
				current_state->reaction_counter = params->counter_react_angle;
			} else {
				current_state->radar_current = RADAR_REACT_RIGHT;
				current_state->react_current = REACT_TURN;
				mavlink_log_info(mavlink_fd, "[mission commander] react right");
				current_state->reaction_counter = params->counter_react_angle;
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
		if(front_react) {
			/* obstacle is detected */
			current_state->radar_current = RADAR_REACT_RIGHT;
			current_state->react_current = REACT_TURN;
			mavlink_log_info(mavlink_fd, "[mission commander] react right");
			current_state->reaction_counter = params->counter_react_angle;

		} else if (left > params->mission_react_side_dist) {
			current_state->radar_current = RADAR_CLEAR;
		} else if (right < params->mission_react_side_dist) {
			current_state->radar_current = RADAR_FOLLOW_CORRIDOR;
		}

	} else if (current_state->radar_current == RADAR_FOLLOW_WALL_R) {
		/* TODO what if a wall appears? */
		if(front_react) {
			/* obstacle is detected */
			current_state->radar_current = RADAR_REACT_LEFT;
			current_state->react_current = REACT_TURN;
			mavlink_log_info(mavlink_fd, "[mission commander] react left");
			current_state->reaction_counter = params->counter_react_angle;

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
			if (	(current_state->radar_current == RADAR_REACT_LEFT && left < params->mission_min_side_dist) ||
					(current_state->radar_current == RADAR_REACT_RIGHT && right < params->mission_min_side_dist)
				) {
				/* stand still and make nothing waiting for better weather */
				/* TODO add problem solving */
				printf("[mission commander] too close to side walls.\n");
				do_state_update(current_state, mavlink_fd, MISSION_ABORTED);
				return;
			}

		}

		if (current_state->react_current == REACT_TURN) {

			/* if front is free again plan next reaction */
			if (front_dist > params->mission_react_front_dist) {
//			if (!front_react) {
				if (current_state->react_next == REACT_NO_STATE) {
					// set next state
					current_state->react_next = REACT_PASS_OBJECT;
					current_state->reaction_counter = current_state->reaction_counter + params->counter_overreact_angle;

					if (current_state->radar_current == RADAR_REACT_LEFT) {
						// TODO add counter value
						mavlink_log_info(mavlink_fd, "[mission commander] pass left");

					} else {
						mavlink_log_info(mavlink_fd, "[mission commander] pass right");
					}
				}
			}

		} else if (current_state->react_current == REACT_PASS_OBJECT) {


			if (front_dist > params->mission_react_front_dist) {

				if (	(current_state->radar_current == RADAR_REACT_LEFT && !current_state->wall_right) ||
						(current_state->radar_current == RADAR_REACT_RIGHT && !current_state->wall_left)) {
					/* if there is no object on the side plan next reaction */
					if (current_state->react_next == REACT_NO_STATE) {
						current_state->react_next = REACT_TEST;
						current_state->reaction_counter = current_state->reaction_counter + params->counter_free_distance;
					}

				} else if (	(current_state->radar_current == RADAR_REACT_LEFT && current_state->wall_right) ||
							(current_state->radar_current == RADAR_REACT_RIGHT && current_state->wall_left)) {

					/* there is a wall reset next reaction if already planed */
					if (current_state->react_next != REACT_NO_STATE) {
						current_state->react_next = REACT_NO_STATE;
					}
				}
			} else {

				/* we again need a turn -> go one step back? */
				current_state->react_current = REACT_TURN;
				current_state->reaction_counter = params->counter_react_angle;
			}

		} else if (current_state->react_current == REACT_TEST) {

			if (	(current_state->radar_current == RADAR_REACT_LEFT && !current_state->wall_right) ||
					(current_state->radar_current == RADAR_REACT_RIGHT && !current_state->wall_left)) {

				current_state->radar_previous = current_state->radar_current;
				current_state->radar_current = RADAR_CLEAR;
				mavlink_log_info(mavlink_fd, "[mission commander] clear");

			} else {

				/* we need to go back */
				current_state->react_current = REACT_PASS_OBJECT;

			}

		}

	}

	return;
}

void do_radar_update2(struct mission_state_s *current_state, struct mission_commander_flow_params *params, int mavlink_fd,
		struct discrete_radar_s *new_radar, struct omnidirectional_flow_s *omni_flow) {

	float yaw_control = current_state->step.yaw;
	float x_control = current_state->step.x;
	float y_control = current_state->step.y;

	radarControl(new_radar->distances, omni_flow->front_distance_m, params->radarControlSettings, &yaw_control, &x_control, &y_control);

	current_state->step.x = x_control;
	current_state->step.y = y_control;
	current_state->step.yaw = yaw_control;

//	/* if the next reaction state is overdue -> set it as current*/
//	if (current_state->react_next != REACT_NO_STATE) {
//		if (current_state->reaction_counter == 0) {
//			current_state->react_current = current_state->react_next;
//			current_state->react_next = REACT_NO_STATE;
//		}
//	}
//
//	bool front_free = true;
//	bool front_react = false;
//	int minimum_front_distance = 5000;
//
//	/* check new front condition */
//	for(int i = 14; i<19; i++)
//	{
//		if (new_radar->distances[i] < params->mission_min_front_dist) {
//			front_free = false;
//		} else if(new_radar->distances[i] < params->mission_react_front_dist) {
//			front_react = true;
//		}
//
//		if (new_radar->distances[i] < minimum_front_distance) {
//			minimum_front_distance = new_radar->distances[i];
//		}
//	}
//
//	/* check new frontal wall situation */
//
//
//	if(!front_free) {
//		/* stand still and make nothing waiting for better weather */
//		/* TODO add problem solving */
//		printf("[mission commander] too close to wall.\n");
//		do_state_update(current_state, mavlink_fd, MISSION_ABORTED);
//		return;
//	}
//
//	/* check new side-freeness */
//	int left_full = (	new_radar->distances[9] + 	new_radar->distances[10] + new_radar->distances[11] +
//						new_radar->distances[12] + 	new_radar->distances[13] + new_radar->distances[14]) / 6;
//	int right_full = (	new_radar->distances[18] + 	new_radar->distances[19] + new_radar->distances[20] +
//						new_radar->distances[21] + 	new_radar->distances[22] + new_radar->distances[23]) / 6;
//
//	int left = 		(	new_radar->distances[9] + new_radar->distances[10] + new_radar->distances[11]) / 3;
//	int right = 	(	new_radar->distances[21] + new_radar->distances[22] + new_radar->distances[23]) / 3;
//
//	current_state->debug_value1 = (float) left_full;
//	current_state->debug_value2 = (float) right_full;
//	current_state->debug_value3 = new_radar->distances[9];
//	current_state->debug_value4 = new_radar->distances[23];
//
//	if (left < params->mission_react_side_dist) {
//		current_state->wall_left = true;
//	} else {
//		current_state->wall_left = false;
//	}
//	if (right < params->mission_react_side_dist) {
//		current_state->wall_right = true;
//	} else {
//		current_state->wall_right = false;
//	}
//
//	if(current_state->radar_current == RADAR_CLEAR) {
//		/* TODO what if a wall appears? */
//		if(front_react) {
//			/* obstacle is detected */
//
//			/* TODO we have a BUG BUG HERE !!! */
//			if (left_full > right_full) {
//				current_state->radar_current = RADAR_REACT_LEFT;
//				current_state->react_current = REACT_TURN;
//				mavlink_log_info(mavlink_fd, "[mission commander] react left");
//				current_state->reaction_counter = params->counter_react_angle;
//			} else {
//				current_state->radar_current = RADAR_REACT_RIGHT;
//				current_state->react_current = REACT_TURN;
//				mavlink_log_info(mavlink_fd, "[mission commander] react right");
//				current_state->reaction_counter = params->counter_react_angle;
//			}
//		} else if (left < params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_WALL_L;
//			mavlink_log_info(mavlink_fd, "[mission commander] follow left wall");
//		} else if (right < params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_WALL_R;
//			mavlink_log_info(mavlink_fd, "[mission commander] follow right wall");
//		}
//
//	} else if (current_state->radar_current == RADAR_FOLLOW_WALL_L ) {
//		/* TODO what if a wall appears? */
//		if(front_react) {
//			/* obstacle is detected */
//			current_state->radar_current = RADAR_REACT_RIGHT;
//			current_state->react_current = REACT_TURN;
//			mavlink_log_info(mavlink_fd, "[mission commander] react right");
//			current_state->reaction_counter = params->counter_react_angle;
//
//		} else if (left > params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_CLEAR;
//		} else if (right < params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_CORRIDOR;
//		}
//
//	} else if (current_state->radar_current == RADAR_FOLLOW_WALL_R) {
//		/* TODO what if a wall appears? */
//		if(front_react) {
//			/* obstacle is detected */
//			current_state->radar_current = RADAR_REACT_LEFT;
//			current_state->react_current = REACT_TURN;
//			mavlink_log_info(mavlink_fd, "[mission commander] react left");
//			current_state->reaction_counter = params->counter_react_angle;
//
//		} else if (right > params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_CLEAR;
//		} else if (left < params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_CORRIDOR;
//		}
//
//	} else if (current_state->radar_current == RADAR_FOLLOW_CORRIDOR) {
//		/* TODO what if a front wall appears? */
//		if (right > params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_WALL_L;
//		} else if (left > params->mission_react_side_dist) {
//			current_state->radar_current = RADAR_FOLLOW_WALL_R;
//		}
//
//	} else if (	current_state->radar_current == RADAR_REACT_LEFT ||
//				current_state->radar_current == RADAR_REACT_RIGHT ) {
//
//		/* do we need a state change */
//		int front_dist = (new_radar->distances[15] + new_radar->distances[16] + new_radar->distances[17]) / 3;
//
//		if (current_state->wall_left && current_state->wall_right) {
//			/* continue until both walls are too near */
//			if (	(current_state->radar_current == RADAR_REACT_LEFT && left < params->mission_min_side_dist) ||
//					(current_state->radar_current == RADAR_REACT_RIGHT && right < params->mission_min_side_dist)
//				) {
//				/* stand still and make nothing waiting for better weather */
//				/* TODO add problem solving */
//				printf("[mission commander] too close to side walls.\n");
//				do_state_update(current_state, mavlink_fd, MISSION_ABORTED);
//				return;
//			}
//
//		}
//
//		if (current_state->react_current == REACT_TURN) {
//
//			/* if front is free again plan next reaction */
//			if (front_dist > params->mission_react_front_dist) {
////			if (!front_react) {
//				if (current_state->react_next == REACT_NO_STATE) {
//					// set next state
//					current_state->react_next = REACT_PASS_OBJECT;
//					current_state->reaction_counter = current_state->reaction_counter + params->counter_overreact_angle;
//
//					if (current_state->radar_current == RADAR_REACT_LEFT) {
//						// TODO add counter value
//						mavlink_log_info(mavlink_fd, "[mission commander] pass left");
//
//					} else {
//						mavlink_log_info(mavlink_fd, "[mission commander] pass right");
//					}
//				}
//			}
//
//		} else if (current_state->react_current == REACT_PASS_OBJECT) {
//
//
//			if (front_dist > params->mission_react_front_dist) {
//
//				if (	(current_state->radar_current == RADAR_REACT_LEFT && !current_state->wall_right) ||
//						(current_state->radar_current == RADAR_REACT_RIGHT && !current_state->wall_left)) {
//					/* if there is no object on the side plan next reaction */
//					if (current_state->react_next == REACT_NO_STATE) {
//						current_state->react_next = REACT_TEST;
//						current_state->reaction_counter = current_state->reaction_counter + params->counter_free_distance;
//					}
//
//				} else if (	(current_state->radar_current == RADAR_REACT_LEFT && current_state->wall_right) ||
//							(current_state->radar_current == RADAR_REACT_RIGHT && current_state->wall_left)) {
//
//					/* there is a wall reset next reaction if already planed */
//					if (current_state->react_next != REACT_NO_STATE) {
//						current_state->react_next = REACT_NO_STATE;
//					}
//				}
//			} else {
//
//				/* we again need a turn -> go one step back? */
//				current_state->react_current = REACT_TURN;
//				current_state->reaction_counter = params->counter_react_angle;
//			}
//
//		} else if (current_state->react_current == REACT_TEST) {
//
//			if (	(current_state->radar_current == RADAR_REACT_LEFT && !current_state->wall_right) ||
//					(current_state->radar_current == RADAR_REACT_RIGHT && !current_state->wall_left)) {
//
//				current_state->radar_previous = current_state->radar_current;
//				current_state->radar_current = RADAR_CLEAR;
//				mavlink_log_info(mavlink_fd, "[mission commander] clear");
//
//			} else {
//
//				/* we need to go back */
//				current_state->react_current = REACT_PASS_OBJECT;
//
//			}
//
//		}
//
//	}
//
//	return;
}


