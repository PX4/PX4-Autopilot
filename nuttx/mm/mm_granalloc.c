/****************************************************************************
 * mm/mm_granalloc.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/gran.h>

#include "mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_common_alloc
 *
 * Description:
 *   Allocate memory from the granule heap.
 *
 * Input Parameters:
 *   priv  - The granule heap state structure.
 *   alloc - The adress of the allocation.
 *   ngranules - The number of granules allocated
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gran_mark_allocated(FAR struct gran_s *priv, uintptr_t alloc, unsigned int ngranules)
{
  unsigned int granno;
  unsigned int gatidx;
  unsigned int gatbit;
  unsigned int avail;
  uint32_t mask;

  /* Determine the granule number of the allocation */

  granno = (alloc - priv->heapstart) >> priv->log2gran;

  /* Determine the GAT table index associated with the allocation */

  gatidx = granno >> 5;
  gatbit = granno & 31;

  /* Mark bits in the first GAT entry */

  avail = 32 - gatbit;
  if (ngranules > avail)
    {
      priv->gat[gatidx] |= (0xffffffff << gatbit);
      ngranules -= avail;
    }

  /* Handle the cae where where all of the granules come from one entry */

  else
    {
      mask = 0xffffffff >> (32 - ngranules);
      priv->gat[gatidx] |= (mask << gatbit);
      return;
    }

  /* Mark bits in the second GAT entry */

  mask = 0xffffffff >> (32 - ngranules);
  priv->gat[gatidx+1] |= (mask << gatbit);
}

/****************************************************************************
 * Name: gran_common_alloc
 *
 * Description:
 *   Allocate memory from the granule heap.
 *
 * Input Parameters:
 *   priv - The granule heap state structure.
 *   size - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the allocated memory is returned.
 *
 ****************************************************************************/

static inline FAR void *gran_common_alloc(FAR struct gran_s *priv, size_t size)
{
  unsigned int ngranules;
  size_t       tmpmask;
  uintptr_t     alloc;
  uint32_t      curr;
  uint32_t      next;
  uint32_t      mask;
  int           i;
  int           j;

  DEBUGASSERT(priv && size <= 32 * (1 << priv->log2gran));
  if (priv && size > 0)
    {
      /* How many contiguous granules we we need to find? */

      tmpmask   = (1 << priv->log2gran) - 1;
      ngranules = (size + tmpmask) >> priv->log2gran;

      /* Then create mask for that number of granules */

      DEBUGASSERT(ngranules <= 32);
      mask = 0xffffffff >> (32 - ngranules);

      /* Now search the granule allocation table for that number of contiguous */

      alloc = priv->heapstart;

      for (i = 0; i < priv->ngranules; i += 32)
        {
          /* Get the GAT index associated with the granule (i) */

          j = i >> 5;
          curr  = priv->gat[j];

          /* Get the next entry from the GAT to support a 64 bit shift */

          if (i < priv->ngranules)
            {
              next = priv->gat[j + 1];
            }

          /* Use all zeroes when are at the last entry in the GAT */

          else
            {
              next = 0;
            }

          for (j = 0; j < 32; j++)
            {
              /* Check if we have the allocation at this bit position (0
               * means unallocated).
               */

              if ((curr & mask) == 0)
                {
                  /* Yes.. mark these granules allocated */

                  gran_mark_allocated(priv, alloc, ngranules);

                  /* And return the allocation address */

                  return (FAR void *)alloc;
                }

              /* Set up for the next time through the loop.  Perform a 64
               * bit shift to move to the next gram position.
               */

              curr >>= 1;
              if ((next & 1) != 0)
                {
                  curr |= 0x80000000;
                }
              next >>= 1;

              /* Increment the next candidate allocation address */

              alloc += (1 << priv->log2gran);
            }
        }
    }

  return NULL;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_alloc
 *
 * Description:
 *   Allocate memory from the granule heap.
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granaules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, either a non-NULL pointer to the allocated memory (if
 *   CONFIG_GRAN_SINGLE) or zero  (if !CONFIG_GRAN_SINGLE) is returned.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
FAR void *gran_alloc(size_t size)
{
  return gran_common_alloc(g_graninfo, size);
}
#else
FAR void *gran_alloc(GRAN_HANDLE handle, size_t size)
{
  return gran_common_alloc((FAR struct gran_s *)handle, size);
}
#endif

#endif /* CONFIG_GRAN */
