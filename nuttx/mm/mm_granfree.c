/****************************************************************************
 * mm/mm_granfree.c
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
 * Name: gran_common_free
 *
 * Description:
 *   Return memory to the granule heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   memory - A pointer to memory previoiusly allocated by gran_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gran_common_free(FAR struct gran_s *priv,
                                    FAR void *memory, size_t size)
{
  unsigned int granno;
  unsigned int gatidx;
  unsigned int gatbit;
  unsigned int granmask;
  unsigned int ngranules;
  unsigned int avail;
  uint32_t     gatmask;

  DEBUGASSERT(priv && memory && size <= 32 * (1 << priv->log2gran));

  /* Get exclusive access to the GAT */

  gran_enter_critical(priv);

  /* Determine the granule number of the first granule in the allocation */

  granno = ((uintptr_t)memory - priv->heapstart) >> priv->log2gran;

  /* Determine the GAT table index and bit number associated with the
   * allocation.
   */

  gatidx = granno >> 5;
  gatbit = granno & 31;

  /* Determine the number of granules in the allocation */

  granmask =  (1 << priv->log2gran) - 1;
  ngranules = (size + granmask) >> priv->log2gran;

  /* Clear bits in the GAT entry or entries */

  avail = 32 - gatbit;
  if (ngranules > avail)
    {
      /* Clear bits in the first GAT entry */

      gatmask = (0xffffffff << gatbit);
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == gatmask);

      priv->gat[gatidx] &= ~gatmask;
      ngranules -= avail;

      /* Clear bits in the second GAT entry */

      gatmask = 0xffffffff >> (32 - ngranules);
      DEBUGASSERT((priv->gat[gatidx+1] & gatmask) == gatmask);

      priv->gat[gatidx+1] &= ~gatmask;
    }

  /* Handle the case where where all of the granules came from one entry */

  else
    {
      /* Clear bits in a single GAT entry */

      gatmask   = 0xffffffff >> (32 - ngranules);
      gatmask <<= gatbit;
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == gatmask);

      priv->gat[gatidx] &= ~gatmask;
    }

  gran_leave_critical(priv);
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_free
 *
 * Description:
 *   Return memory to the granule heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   memory - A pointer to memory previoiusly allocated by gran_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
void gran_free(FAR void *memory, size_t size)
{
  return gran_common_free(g_graninfo, memory, size);
}
#else
void gran_free(GRAN_HANDLE handle, FAR void *memory, size_t size)
{
  return gran_common_free((FAR struct gran_s *)handle, memory, size);
}
#endif

#endif /* CONFIG_GRAN */
