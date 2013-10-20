/*
 * rotation.h
 *
 *  Created on: 20.10.2013
 *      Author: ton
 */

#ifndef ROTATION_H_
#define ROTATION_H_

#include <unistd.h>
#include <mathlib/mathlib.h>

/**
 * Enum for board and external compass rotations.
 * This enum maps from board attitude to airframe attitude.
 */
enum Rotation {
	ROTATION_NONE                = 0,
	ROTATION_YAW_45              = 1,
	ROTATION_YAW_90              = 2,
	ROTATION_YAW_135             = 3,
	ROTATION_YAW_180             = 4,
	ROTATION_YAW_225             = 5,
	ROTATION_YAW_270             = 6,
	ROTATION_YAW_315             = 7,
	ROTATION_ROLL_180            = 8,
	ROTATION_ROLL_180_YAW_45     = 9,
	ROTATION_ROLL_180_YAW_90     = 10,
	ROTATION_ROLL_180_YAW_135    = 11,
	ROTATION_PITCH_180           = 12,
	ROTATION_ROLL_180_YAW_225    = 13,
	ROTATION_ROLL_180_YAW_270    = 14,
	ROTATION_ROLL_180_YAW_315    = 15,
	ROTATION_ROLL_90             = 16,
	ROTATION_ROLL_90_YAW_45      = 17,
	ROTATION_ROLL_90_YAW_90      = 18,
	ROTATION_ROLL_90_YAW_135     = 19,
	ROTATION_ROLL_270            = 20,
	ROTATION_ROLL_270_YAW_45     = 21,
	ROTATION_ROLL_270_YAW_90     = 22,
	ROTATION_ROLL_270_YAW_135    = 23,
	ROTATION_PITCH_90            = 24,
	ROTATION_PITCH_270           = 25,
	ROTATION_MAX
};

typedef struct {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
} rot_lookup_t;

const rot_lookup_t rot_lookup[] = {
	{  0,   0,   0 },
	{  0,   0,  45 },
	{  0,   0,  90 },
	{  0,   0, 135 },
	{  0,   0, 180 },
	{  0,   0, 225 },
	{  0,   0, 270 },
	{  0,   0, 315 },
	{180,   0,   0 },
	{180,   0,  45 },
	{180,   0,  90 },
	{180,   0, 135 },
	{  0, 180,   0 },
	{180,   0, 225 },
	{180,   0, 270 },
	{180,   0, 315 },
	{ 90,   0,   0 },
	{ 90,   0,  45 },
	{ 90,   0,  90 },
	{ 90,   0, 135 },
	{270,   0,   0 },
	{270,   0,  45 },
	{270,   0,  90 },
	{270,   0, 135 },
	{  0,  90,   0 },
	{  0, 270,   0 }
};

/**
 * Get the rotation matrix
 */
__EXPORT void
get_rot_matrix(enum Rotation rot, math::Matrix *rot_matrix);

#endif /* ROTATION_H_ */
