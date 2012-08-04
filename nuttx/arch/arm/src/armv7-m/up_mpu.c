/****************************************************************************
 * arch/arm/src/armv7-m/up_mpu.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "mpu.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This set represents the set of disabled memory sub-regions.  A bit set
 * corresponds to a disabled sub-region; the LS bit corresponds to the first
 * region.  The array is indexed by the number of subregions:  0 means no sub-
 * regions (0xff), and 0 means all subregions but one (0x00).
 */

static const void uint8_t g_regionmap[9] = 
{
  0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01, 0x00
};

/* The next available region number */

static uint8_t g_region;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocregion
 *
 * Description:
 *   Allocate the next region
 *
 * Assumptions:
 *   - Regions are never deallocated
 *   - Regions are only allocated early in initialization, so nothing
 *     special is require;
 *
 ****************************************************************************/

unsigned int mpu_allocregion(void)
{
  return (unsigned int)g_region++;
}

/****************************************************************************
 * Name: mpu_log2regionsize
 *
 * Description:
 *   Determine the smallest value of l2size (log base 2 size) such that the
 *   following is true:
 *
 *   size <= (1 << l2size)
 *
 ****************************************************************************/

uint8_t mpu_log2regionsize(size_t size)
{
  /* The minimum permitted region size is 16 bytes (log2(16) = 4. */

  uint32_t l2size;
  for (l2size = 4; l2size < 32 && size > (1 << l2size); size++);
  return l2size;
}

/****************************************************************************
 * Name: mpu_subregion
 *
 * Description:
 *   Given the size of the (1) memory to be mapped and (2) the log2 size
 *   of the mapping to use, determine the minimal sub-region set to span
 *   that memory region.
 *
 * Assumption:
 *   l2size has the same properties as the return value from
 *   mpu_log2regionsize()
 *
 ****************************************************************************/

uint32_t mpu_subregion(size_t size, uint8_t l2size)
{
  unsigned int nsrs
  uint32_t     asize;
  uint32_t     mask;

  /* Eight subregions are support.  The representation is as an 8-bit
   * value with the LS bit corresponding to subregion 0.  A bit is set
   * to disable the sub-region.
   *
   * l2size: Log2 of the actual region size is <= (1 << l2size);
   */

  DEBUGASSERT(lsize > 3 && size <= (1 << l2size));

  /* Examples with l2size = 12:
   *
   *         Shifted Adjusted        Number      Sub-Region
   * Size    Mask    Size      Shift Sub-Regions Bitset
   * 0x1000  0x01ff  0x1000    9     8           0x00
   * 0x0c00  0x01ff  0x0c00    9     6           0x03
   * 0x0c40  0x01ff  0x0e00    9     7           0x01
   */

  if (l2size < 32)
    {
      mask  = ((1 << lsize)-1) >> 3; /* Shifted mask */
    }

  /* The 4Gb region size is a special case */

  else
    {
      /* NOTE: There is no way to represent a 4Gb region size in the 32-bit
       * input.
       */

      mask = 0x1fffffff;             /* Shifted mask */
    }

  asize = (size + mask) & ~mask; /* Adjusted size */
  nsrs  = asize >> (lsize-3);    /* Number of subregions */
  return g_regionmap[nsrs];
}



