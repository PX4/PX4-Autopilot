/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file uint162Buf.c
 * @author Jacob.Garrison
 *
 * @date Mar 2, 2021
 *      
 */

#include "sgUtil.h"

/*
 * Documented in the header file.
 */
void int242Buf(uint8_t *bufferPos, int32_t value)
{
   bufferPos[0] = (value & 0xFF0000) >> 16;
   bufferPos[1] = (value & 0xFF00) >> 8;
   bufferPos[2] = (value & 0xFF);
}
