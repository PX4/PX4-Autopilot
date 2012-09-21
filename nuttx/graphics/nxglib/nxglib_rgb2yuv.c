/****************************************************************************
 * graphics/color/nxglib_rgb2yuv.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>
#include <fixedmath.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define b16_P0813 0x000014d0    /* 0.0813 */
#define b16_P1140 0x00001d2f    /* 0.1140 */
#define b16_P1687 0x00002b30    /* 0.1687 */
#define b16_P2990 0x00004c8b    /* 0.2990 */
#define b16_P3313 0x000054d0    /* 0.3313 */
#define b16_P4187 0x00006b30    /* 0.4187 */
#define b16_P5000 0x00008000    /* 0.5000 */
#define b16_P5870 0x00009646    /* 0.5870 */
#define b16_128P0 0x00800000    /* 128.0 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_rgb2yuv
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

void nxgl_rgb2yuv(uint8_t r, uint8_t g, uint8_t b,
                  uint8_t *y, uint8_t *u, uint8_t *v)
{
  /* Per the JFIF specification:
   *
   * Y =       (0.2990 * R) + (0.5870 * G) + (0.1140 * B)
   * U = 128 - (0.1687 * R) - (0.3313 * G) + (0.5000 * B)
   * V = 128 + (0.5000 * R) - (0.4187 * G) - (0.0813 * B);
   */

  *y = (uint8_t)b16toi(b16muli(b16_P2990, r) + b16muli(b16_P5870, g) + b16muli(b16_P1140, b));
  *u = (uint8_t)b16toi(b16_128P0 - b16muli(b16_P1687, r) - b16muli(b16_P3313, g) + b16muli(b16_P5000, b));
  *v = (uint8_t)b16toi(b16_128P0 + b16muli(b16_P5000, r) - b16muli(b16_P4187, g) - b16muli(b16_P0813, b));
}
