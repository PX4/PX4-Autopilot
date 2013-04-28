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

#endif /* ACCELEROMETER_CALIBRATION_H_ */
