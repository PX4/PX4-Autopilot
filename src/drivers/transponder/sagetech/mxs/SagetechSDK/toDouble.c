/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file toDouble.c
 * @author Peter Dorich
 *
 * @date Mar 10, 2022
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
double toDouble(const uint8_t *bufferPos)
{
	union
	{
		double val;
		uint8_t bytes[8];
	} db;

	for(int i = 0; i < 8; i++)
	{
		db.bytes[i] = bufferPos[i];
	}

	return db.val;
}
