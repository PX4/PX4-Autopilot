/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file double2Buf.c
 * @author Reese Lam
 *
 * @date Mar 9, 2022
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void double2Buf(uint8_t *bufferPos, double value)
{
    const uint16_t DOUBLE_SIZE = 8;

    union
    {
       double val;
       unsigned char bytes[DOUBLE_SIZE];
    } conversion;

   conversion.val = value;

   for (int i = 0; i < DOUBLE_SIZE; ++i)
   {
     bufferPos[i] = conversion.bytes[i];
   }
}
