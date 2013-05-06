/*
 * accelerometer_calibration.h
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#ifndef ACCELEROMETER_CALIBRATION_H_
#define ACCELEROMETER_CALIBRATION_H_

#include <stdint.h>
#include <uORB/topics/vehicle_status.h>

void do_accel_calibration(int status_pub, struct vehicle_status_s *status, int mavlink_fd);

#endif /* ACCELEROMETER_CALIBRATION_H_ */
