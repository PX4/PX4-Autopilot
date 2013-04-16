/*
 * mission_helper.h
 *
 *  Created on: Apr 12, 2013
 *      Author: samuezih
 */

#ifndef MISSION_HELPER_H_
#define MISSION_HELPER_H_

#include <stdbool.h>
#include <uORB/uORB.h>
#include <uORB/topics/discrete_radar.h>
#include "mission_commander_flow_params.h"

typedef enum mission_states {
	MISSION_READY,
	MISSION_STARTED,
	MISSION_ACCOMPLISHED,
	MISSION_ABORTED,
	MISSION_RESETED
} mission_state_t;

typedef enum radar_states {
	RADAR_NO_STATE,
	RADAR_CLEAR,
	RADAR_REACT_LEFT,
	RADAR_REACT_RIGHT,
	RADAR_FOLLOW_WALL_R,
	RADAR_FOLLOW_WALL_L,
	RADAR_FOLLOW_CORRIDOR,
	RADAR_STOP
} radar_state_t;

typedef enum reaction_states {
	REACT_NO_STATE,
	REACT_TURN,
	REACT_PASS_OBJECT,
	REACT_TEST
} reaction_state_t;

struct mission_state_s {
	mission_state_t state;
	int state_counter;
	bool final_sequence;

	radar_state_t radar_previous;
	radar_state_t radar_current;
	radar_state_t radar_next;
	bool wall_left;
	bool wall_right;

	reaction_state_t react_current;
	reaction_state_t react_next;
	int reaction_counter;
};

void init_state(struct mission_state_s *state);
void do_state_update(struct mission_state_s *current_state, int mavlink_fd, mission_state_t new_state);
void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params, int mavlink_fd, struct discrete_radar_s *new_radar);


#endif /* MISSION_HELPER_H_ */
