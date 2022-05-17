/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file toUint24.c
 * @author Reese Lam
 *
 * @date Mar 28, 2022
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
uint32_t toUint24(const uint8_t bytes[])
{
   uint32_t uint32 = (0 << 24)                   |
                     ((uint32_t) bytes[0] << 16) |
                     ((uint32_t) bytes[1] << 8)  |
                     ((uint32_t) bytes[2]);

   return uint32;
}
