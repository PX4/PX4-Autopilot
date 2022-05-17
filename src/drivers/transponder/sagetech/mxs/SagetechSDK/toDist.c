/**
 * @copyright Copyright (c) 2020 Sagetech, Inc. All rights reserved.
 *
 * @file toDist.c
 * @author jim billmeyer
 *
 * @date Apr 1, 2021
 */

#include "sgUtil.h"

#define DIST_RES 0.00390625

/*
 * Documented in the header file.
 */
double toDist(const uint8_t *bytes)
{
   double value = toUint16(bytes);
   value *= DIST_RES;

   return value;
}

