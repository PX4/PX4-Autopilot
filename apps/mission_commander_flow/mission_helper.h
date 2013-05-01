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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_bodyframe_position.h>
#include <uORB/topics/vehicle_bodyframe_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
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

struct sonar_obstacle_s {
	/* set to final */
	struct vehicle_local_position_setpoint_s sonar_obstacle_local;
	/* always updated */
	struct vehicle_bodyframe_position_setpoint_s sonar_obstacle_bodyframe;

	/* polar coordinates */
	float sonar_obst_polar_r;
	float sonar_obst_polar_alpha;
	float sonar_obst_pitch;

	/* valid flag */
	bool valid;
	bool updated;
};

struct mission_state_s {
	/* mission states */
	mission_state_t state;
	struct mission_step_s step;
	bool initialized;
	bool final_sequence;

	/* sonar states */
	sonar_state_t sonar;

	struct sonar_obstacle_s sonar_obstacle;
	bool free_to_go;

	/* path states */
	struct vehicle_local_position_setpoint_s next_waypoint;
	bool waypoint_set;

	float debug_value1;
	float debug_value2;
	int debug_value3;
	int debug_value4;
};

void convert_setpoint_bodyframe2local(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp,
		struct vehicle_local_position_setpoint_s *local_pos_sp
		);
void convert_setpoint_local2bodyframe(
		struct vehicle_local_position_s *local_pos,
		struct vehicle_bodyframe_position_s *bodyframe_pos,
		struct vehicle_attitude_s *att,
		struct vehicle_local_position_setpoint_s *local_pos_sp,
		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp
		);
float get_yaw(struct vehicle_local_position_s *local_pos, struct vehicle_local_position_setpoint_s *local_pos_sp);
void init_state(struct mission_state_s *state);
void do_state_update(struct mission_state_s *current_state, int mavlink_fd, mission_state_t new_state);
void do_radar_update(struct mission_state_s *current_state, struct mission_commander_flow_params *params,
		int mavlink_fd, struct discrete_radar_s *new_radar);

#endif /* MISSION_HELPER_H_ */
