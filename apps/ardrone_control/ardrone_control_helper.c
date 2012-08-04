#include "ardrone_control_helper.h"
#include <unistd.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

// int read_sensors_raw(sensors_raw_t *sensors_raw)
// {
// 	static int ret;
// 	ret = global_data_wait(&global_data_sensors_raw->access_conf_rate_full);

// 	if (ret == 0) {
// 		memcpy(sensors_raw->gyro_raw, global_data_sensors_raw->gyro_raw, sizeof(sensors_raw->gyro_raw));
// //		printf("Timestamp %d\n", &global_data_sensors_raw->timestamp);

// 	} else {
// 		printf("Controller timeout, no new sensor values available\n");
// 	}

// 	global_data_unlock(&global_data_sensors_raw->access_conf_rate_full);
// 	return ret;
// }

// int read_attitude(global_data_attitude_t *attitude)
// {

// 	static int ret;
// 	ret = global_data_wait(&global_data_attitude->access_conf);

// 	if (ret == 0) {
// 		memcpy(&attitude->roll, &global_data_attitude->roll, sizeof(global_data_attitude->roll));
// 		memcpy(&attitude->pitch, &global_data_attitude->pitch, sizeof(global_data_attitude->pitch));
// 		memcpy(&attitude->yaw, &global_data_attitude->yaw, sizeof(global_data_attitude->yaw));
// 		memcpy(&attitude->rollspeed, &global_data_attitude->rollspeed, sizeof(global_data_attitude->rollspeed));
// 		memcpy(&attitude->pitchspeed, &global_data_attitude->pitchspeed, sizeof(global_data_attitude->pitchspeed));
// 		memcpy(&attitude->yawspeed, &global_data_attitude->yawspeed, sizeof(global_data_attitude->yawspeed));

// 	} else {
// 		printf("Controller timeout, no new attitude values available\n");
// 	}

// 	global_data_unlock(&global_data_attitude->access_conf);



// 	return ret;
// }

// void read_quad_motors_setpoint(quad_motors_setpoint_t *rate_setpoint)
// {

// 	if (0 == global_data_trylock(&global_data_quad_motors_setpoint->access_conf)) { //TODO: check if trylock is the right choice, maybe only lock?
// 		rate_setpoint->motor_front_nw = global_data_quad_motors_setpoint->motor_front_nw;
// 		rate_setpoint->motor_right_ne = global_data_quad_motors_setpoint->motor_right_ne;
// 		rate_setpoint->motor_back_se = global_data_quad_motors_setpoint->motor_back_se;
// 		rate_setpoint->motor_left_sw = global_data_quad_motors_setpoint->motor_left_sw;

// 		global_data_unlock(&global_data_quad_motors_setpoint->access_conf);
// 	}
// }
