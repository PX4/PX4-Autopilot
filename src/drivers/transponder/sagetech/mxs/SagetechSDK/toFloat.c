/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file toFloat.c
 * @author Peter Dorich
 *
 * @date Mar 10, 2022
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
float toFloat(const uint8_t *bufferPos)
{
	union
	{
		float val;
		uint8_t bytes[4];
	} fl;

	for(int i = 0; i < 4; i++)
	{
		fl.bytes[i] = bufferPos[i];
	}

	return fl.val;
}
