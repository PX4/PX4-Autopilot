/****************************************************************************
 * libc/math/lib_fixedmath.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <fixedmath.h>

#ifndef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: fixsign
 ****************************************************************************/

static void fixsign(b16_t *parg1, b16_t *parg2, bool *pnegate)
{
  bool negate = false;
  b16_t arg;

  arg = *parg1;
  if (arg < 0)
    {
      *parg1 = -arg;
      negate = true;
    }

  arg = *parg2;
  if (arg < 0)
    {
      *parg2 = -arg;
      negate ^= true;
    }

  *pnegate = negate;
}

/****************************************************************************
 * Name: adjustsign
 ****************************************************************************/

static b16_t adjustsign(b16_t result, bool negate)
{
  /* If the product is negative, then we overflowed */

  if (result < 0)
    {
      if (result)
        {
          return b16MIN;
        }
      else
        {
          return b16MAX;
        }
    }

  /* correct the sign of the result */

  if (negate)
    {
      return -result;
    }
  return result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: b16mulb16
 ****************************************************************************/

b16_t b16mulb16(b16_t m1, b16_t m2)
{
  bool negate;
  b16_t product;

  fixsign(&m1, &m2, &negate);
  product = (b16_t)ub16mulub16((ub16_t)m1, (ub16_t)m2);
  return adjustsign(product, negate);
}

/****************************************************************************
 * Name: ub16mulub16
 **************************************************************************/

ub16_t ub16mulub16(ub16_t m1, ub16_t m2)
{
 /* Let:
 *
  *   m1 = m1i*2**16 + m1f                                            (b16)
  *   m2 = m2i*2**16 + m2f                                            (b16)
  *
  * Then:
  *
  *  m1*m2 = (m1i*m2i)*2**32 + (m1i*m2f + m2i*m1f)*2**16 + m1f*m2f     (b32)
  *        = (m1i*m2i)*2**16 + (m1i*m2f + m2i*m1f) + m1f*m2f*2**-16    (b16)
  *        = a*2**16 + b + c*2**-16
  */

  uint32_t m1i = ((uint32_t)m1 >> 16);
  uint32_t m2i = ((uint32_t)m1 >> 16);
  uint32_t m1f = ((uint32_t)m1 & 0x0000ffff);
  uint32_t m2f = ((uint32_t)m2 & 0x0000ffff);

  return (m1i*m2i << 16) + m1i*m2f + m2i*m1f + (((m1f*m2f) + b16HALF) >> 16);
}

/****************************************************************************
 * Name: b16sqr
 **************************************************************************/

b16_t b16sqr(b16_t a)
{
  b16_t sq;

  /* The result is always positive.  Just take the absolute value */

  if (a < 0)
    {
      a = -a;
    }

  /* Overflow occurred if the result is negative */

  sq = (b16_t)ub16sqr(a);
  if (sq < 0)
    {
      sq = b16MAX;
    }
  return sq;
}

/****************************************************************************
 * Name: b16divb16
 **************************************************************************/

ub16_t ub16sqr(ub16_t a)
{
 /* Let:
 *
  *   m = mi*2**16 + mf                                               (b16)
  *
  * Then:
  *
  *  m*m = (mi*mi)*2**32 + 2*(m1*m2)*2**16 + mf*mf                    (b32)
  *      = (mi*mi)*2**16 + 2*(mi*mf)       + mf*mf*2**-16             (b16)
  */

  uint32_t mi = ((uint32_t)a >> 16);
  uint32_t mf = ((uint32_t)a & 0x0000ffff);

  return (mi*mi << 16) + (mi*mf << 1) + ((mf*mf + b16HALF) >> 16);
}

/****************************************************************************
 * Name: b16divb16
 **************************************************************************/

b16_t b16divb16(b16_t num, b16_t denom)
{
  bool  negate;
  b16_t quotient;

  fixsign(&num, &denom, &negate);
  quotient = (b16_t)ub16divub16((ub16_t)num, (ub16_t)denom);
  return adjustsign(quotient, negate);
}

/****************************************************************************
 * Name: ub16divub16
 **************************************************************************/

ub16_t ub16divub16(ub16_t num, ub16_t denom)
{
  uint32_t term1;
  uint32_t numf;
  uint32_t product;

 /* Let:
  *
  *   num = numi*2**16 + numf                                         (b16)
  *   den = deni*2**16 + denf                                         (b16)
  *
  * Then:
  *
  *  num/den = numi*2**16 / den + numf / den                          (b0)
  *          = numi*2**32 / den + numf*2**16 /den                     (b16)
  */

  /* Check for overflow in the first part of the quotient */

  term1 = ((uint32_t)num & 0xffff0000) / denom;
  if (term1 >= 0x00010000)
    {
        return ub16MAX; /* Will overflow */
    }

  /* Finish the division */

  numf    = num - term1 * denom;
  term1 <<= 16;
  product = term1 + (numf + (denom >> 1)) / denom;

  /* Check for overflow */

  if (product < term1)
    {
        return ub16MAX; /* Overflowed */
    }
  return product;
}

#endif
