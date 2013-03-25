/*
 * setpoint_planer.c
 *
 *  Created on: Mar 21, 2013
 *      Author: samuezih
 */

#include "setpoint_planner.h"

//void get_next_bodyframe_setpoint(
//		struct discrete_radar_s *radar,
//		struct vehicle_local_position_s *local_pos,
//		struct vehicle_attitude_s *att,
//		struct vehicle_local_position_setpoint_s *current_waypoint,
//		struct flow_navigation_params *params,
//		struct vehicle_bodyframe_position_setpoint_s *bodyframe_pos_sp
//		)
//{
//	static float wp_local[3] = {0.0f, 0.0f, 0.0f}; // x,y
//	static float wp_bodyframe[3] = {0.0f, 0.0f, 0.0f};
//
//	wp_local[0] = current_waypoint->x;
//	wp_local[1] = current_waypoint->y;
//	wp_local[2] = current_waypoint->z;
//
//	/* calc current waypoint cooridnates in bodyframe */
//	for(uint8_t i = 0; i < 3; i++) {
//		float sum = 0.0f;
//		for(uint8_t j = 0; j < 3; j++) {
//			sum = sum + wp_local[j] * att->R[j][i];
//		}
//		wp_bodyframe[i] = sum;
//	}
//
//	bodyframe_pos_sp->x = wp_bodyframe[0];
//	bodyframe_pos_sp->y = wp_bodyframe[1];
//	bodyframe_pos_sp->z = current_waypoint->z; // let z as it is...
//
//}



