/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file latLon2Buf.c
 * @author Peter Dorich
 *
 * @date Mar 14, 2022
 *
 */

#include "sgUtil.h"

#define SV_RES_LATLON	180.0 / 8388608.0

/*
 * Documented in the header file.
 */
void latLon2Buf(uint8_t bytes[], double value)
{
   int32_t int24 = (int32_t)(value / (SV_RES_LATLON));
   int242Buf(bytes, int24);
}
