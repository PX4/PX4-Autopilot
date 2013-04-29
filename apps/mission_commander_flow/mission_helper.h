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
#include <uORB/topics/omnidirectional_flow.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include "mission_commander_flow_params.h"

typedef enum mission_states {
	MISSION_READY,
	MISSION_STARTED,
	MISSION_ACCOMPLISHED,
	MISSION_ABORTED,
	MISSION_RESETED
} mission_state_t;

typedef enum sonar_states {
	SONAR_NO_STATE,
	SONAR_CLEAR,
	SONAR_REACT,
	SONAR_STOP
} sonar_state_t;

struct mission_step_s {
	float x;
	float y;
	float yaw;
};

struct mission_state_s {
	/* mission states */
	mission_state_t state;
	struct mission_step_s step;
	bool initialized;
	bool final_sequence;

	/* sonar states */
	sonar_state_t sonar;
	int16_t front_situation[17];
	bool free_to_go;

	/* path states */
	struct vehicle_local_position_setpoint_s next_waypoint;
	bool waypoint_set;

	float debug_value1;
	float debug_value2;
	int debug_value3;
	int debug_value4;
};

void init_state(struct mission_state_s *state);
void do_state_update(struct mission_state_s *current_state, int mavlink_fd, mission_state_t new_state);
void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params, int mavlink_fd,
		struct discrete_radar_s *new_radar, struct omnidirectional_flow_s *omni_flow);

#endif /* MISSION_HELPER_H_ */
