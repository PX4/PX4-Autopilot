/****************************************************************************
 * libc/fixedmath/lib_b16atan2.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define B16_C1     0x00000373 /* 0.013480470 */
#define B16_C2     0x00000eb7 /* 0.057477314 */
#define B16_C3     0x00001f0a /* 0.121239071 */
#define B16_C4     0x00003215 /* 0.195635925 */
#define B16_C5     0x0000553f /* 0.332994597 */
#define B16_C6     0x00010000 /* 0.999995630 */
#define B16_HALFPI 0x00019220 /* 1.570796327 */
#define B16_PI     0x00032440 /* 3.141592654 */

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef ABS
#  define ABS(a)   (a < 0 ? -a : a)
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: b16atan2
 *
 * Description:
 *   atan2 calculates the arctangent of y/x.  (Based on a algorithm I saw
 *   posted on the internet... now I have lost the link -- sorry).
 *
 ****************************************************************************/

b16_t b16atan2(b16_t y, b16_t x)
{
  b16_t t0;
  b16_t t1;
  b16_t t2;
  b16_t t3;

  t2 = ABS(x);
  t1 = ABS(y);
  t0 = MAX(t2, t1);
  t1 = MIN(t2, t1);
  t2 = ub16inv(t0);
  t2 = b16mulb16(t1, t2);

  t3 = b16mulb16(t2, t2); 
  t0 =                   - B16_C1;
  t0 = b16mulb16(t0, t3) + B16_C2;
  t0 = b16mulb16(t0, t3) - B16_C3;
  t0 = b16mulb16(t0, t3) + B16_C4;
  t0 = b16mulb16(t0, t3) - B16_C5;
  t0 = b16mulb16(t0, t3) + B16_C6;
  t2 = b16mulb16(t0, t2);

  t2 = (ABS(y) > ABS(x)) ? B16_HALFPI - t2 : t2;
  t2 = (x < 0) ?  B16_PI - t2 : t2;
  t2 = (y < 0) ? -t2 : t2;

  return t2;
}
