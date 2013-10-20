/*
 * rotation.cpp
 *
 *  Created on: 20.10.2013
 *      Author: ton
 */

#include "math.h"
#include "rotation.h"

__EXPORT void
get_rot_matrix(enum Rotation rot, math::Matrix *rot_matrix)
{
	/* first set to zero */
	rot_matrix->Matrix::zero(3, 3);

	float roll  = M_DEG_TO_RAD_F * (float)rot_lookup[rot].roll;
	float pitch = M_DEG_TO_RAD_F * (float)rot_lookup[rot].pitch;
	float yaw   = M_DEG_TO_RAD_F * (float)rot_lookup[rot].yaw;

	math::EulerAngles euler(roll, pitch, yaw);

	math::Dcm R(euler);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			(*rot_matrix)(i, j) = R(i, j);
		}
	}
}
