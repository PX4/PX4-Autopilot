/*
 * acceleration_transform.h
 *
 *  Created on: 30.03.2013
 *      Author: ton
 */

#ifndef ACCELERATION_TRANSFORM_H_
#define ACCELERATION_TRANSFORM_H_

#include <unistd.h>

void acceleration_correct(float accel_corr[3], int16_t accel_raw[3], float accel_T[3][3], int16_t accel_offs[3]);

#endif /* ACCELERATION_TRANSFORM_H_ */
