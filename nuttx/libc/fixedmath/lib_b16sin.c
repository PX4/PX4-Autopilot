/****************************************************************************
 * libc/fixedmath/lib_b16sin.c
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

#include <fixedmath.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define b16_P225       0x0000399a
#define b16_P405284735 0x000067c1
#define b16_1P27323954 0x000145f3

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: b16sin
 * Ref: http://lab.polygonal.de/2007/07/18/fast-and-accurate-sinecosine-approximation/
 ****************************************************************************/

b16_t b16sin(b16_t rad)
{
  b16_t tmp1;
  b16_t tmp2;
  b16_t tmp3;

  /* Force angle into the good range */

  if (rad < -b16PI)
    {
      rad += b16TWOPI;
    }
  else if (rad > b16PI)
   {
      rad -= b16TWOPI;
   }

  /* tmp1 = 1.27323954 * rad
   * tmp2 = .405284735 * rad * rad
   */


  tmp1 = b16mulb16(b16_1P27323954, rad);
  tmp2 = b16mulb16(b16_P405284735, b16sqr(rad));

  if (rad < 0)
    {
       /* tmp3 = 1.27323954 * rad + .405284735 * rad * rad */

       tmp3 = tmp1 + tmp2;
    }
  else
    {
       /* tmp3 = 1.27323954 * rad - 0.405284735 * rad * rad */

       tmp3 = tmp1 - tmp2;
    }

  /* tmp1 = tmp3*tmp3 */

  tmp1 = b16sqr(tmp3);
  if (tmp3 < 0)
    {
      /* tmp1 = tmp3 * -tmp3 */

      tmp1 = -tmp1;
    }

  /* Return sin = .225 * (tmp3 * (+/-tmp3) - tmp3) + tmp3 */

  return b16mulb16(b16_P225, (tmp1 - tmp3)) + tmp3;
}
