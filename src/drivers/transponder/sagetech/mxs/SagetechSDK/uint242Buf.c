/**
 * @copyright Copyright (c) 2022 Sagetech, Inc. All rights reserved.
 *
 * @file uint242Buf.c
 * @author Reese Lam
 *
 * @date Mar 7, 2022
 *
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void uint242Buf(uint8_t *bufferPos, uint32_t value)
{
    bufferPos[0] = (value & 0xFF0000) >> 16;
    bufferPos[1] = (value & 0x00FF00) >> 8;
    bufferPos[2] = (value & 0x0000FF);
}
