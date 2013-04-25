/*
 * accelerometer_calibration.h
 *
 *  Created on: 25.04.2013
 *      Author: ton
 */

#ifndef ACCELEROMETER_CALIBRATION_H_
#define ACCELEROMETER_CALIBRATION_H_

#include <stdint.h>
#include <uORB/topics/vehicle_status.h>

void do_accel_calibration(int status_pub, struct vehicle_status_s *status, int mavlink_fd);
int detect_orientation(int mavlink_fd, int sub_sensor_combined);
int mat_invert3(float src[3][3], float dst[3][3]);
int calculate_calibration_values(int16_t accel_raw_ref[6][3], float accel_T[3][3], int16_t accel_offs[3], float g);

#endif /* ACCELEROMETER_CALIBRATION_H_ */
