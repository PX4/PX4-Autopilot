/************************************************************************************
 * configs/ea3152/tools/lpchdr.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_EA3152_TOOLS_LPCHDR_H
#define __CONFIGS_EA3152_TOOLS_LPCHDR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct lpc31_header_s
{
                              /* OFFS DESCRIPTION */
  uint32_t vector;            /* 0x00    Valid ARM instruction. Usually this will be
                               *    a branch instruction to entry point of the
                               *    image. */
  uint32_t magic;             /* 0x04    This field is used by boot ROM to detect a
                               *    valid image header. This field should always
                               *    be set to 0x41676d69. */
  uint32_t execution_crc32;   /* 0x08    CRC32 value of execution part of the image. If
                               *    the ‘image_type’ is set to ‘0xA’, this field
                               *    is ignored by boot ROM. */
  uint32_t Reserved0[4];      /* 0x0c-0x18: Should be zero. */
  uint32_t imageType;         /* 0x1c Specifies whether CRC check should be done
                               *    on the image or not:
                               *      0xA – No CRC check required.
                               *      0xB – Do CRC32 check on both header and
                               *            execution part of the image. */
  uint32_t imageLength;       /* 0x20    Total image length including header rounded
                               *    up to the nearest 512 byte boundary. In C 
                               *    language the field can be computed as:
                               *    imageLength = (Actual length + 511) & ~0x1FF; */
  uint32_t releaseID;         /* 0x24    Release or version number of the image. Note,
                               *    this field is not used by boot ROM but is
                               *    provided to track the image versions. */
  uint32_t buildTime;         /* 0x28 Time (expressed in EPOC time format) at which
                               *    image is built. Note, this field is not used
                               *    by boot ROM but is provided to track the image
                               *    versions. */
  uint32_t sbzBootParameter;  /* 0x2c    hould be zero. */
  uint32_t cust_reserved[15]; /* 0x30-0x68: Reserved for customer use (60 bytes) */
  uint32_t header_crc32;      /* 0x6c CRC32 value of the header (bytes 0x00 to 0x6C
                               *    of the image). If the ‘image_type’ is set
                               *    to ‘0xA’, this field is ignored by boot ROM. */
  uint32_t Reserved1[4];      /* 0x70-0x7c: Should be zero. */
                              /* 0x80    Start of program code (128Kb max).  The final
                               *    image has to be padded to the nearest 512
                               *    byte boundary */
};

/************************************************************************************
 * Public data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

extern uint32_t crc32part(const uint8_t *src, size_t len, uint32_t crc32val);
extern uint32_t crc32(const uint8_t *src, size_t len);

#endif /* __CONFIGS_EA3152_TOOLS_LPCHDR_H */

