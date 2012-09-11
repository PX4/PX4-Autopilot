/****************************************************************************
 * mm/mm_graninit.c
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

#include <stdlib.h>

#include <nuttx/gran.h>

#include "mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of the single GRAN allocator */

#ifdef CONFIG_GRAN_SINGLE
FAR struct gran_s *g_graninfo;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_common_initialize
 *
 * Description:
 *   Perfrom common GRAN initialization.
 *
 * Input Parameters:
 *   info      - Private granule data structure pointer
 *   heapstart - Start of the granule allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2gran  - Log base 2 of the size of one granule.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8bytes, etc.
 *
 * Returned Value:
 *   On success, a non-NULL info structure is returned that may be used with
 *   other granule allocator interfaces.
 *
 ****************************************************************************/

static inline FAR struct gran_s *gran_common_initialize(FAR void *heapstart,
                                                        size_t heapsize,
                                                        uint8_t log2gran)
{
  FAR struct gran_s *priv;
  uintptr_t          heapend;
  uintptr_t          alignedstart;
  unsigned int       mask;
  unsigned int       alignedsize;
  unsigned int       ngranules;

  /* Determine the number of granules */

  mask         = (1 << log2gran) - 1;
  heapend      = (uintptr_t)heapstart + heapsize;
  alignedstart = ((uintptr_t)heapstart + mask) & ~mask;
  alignedsize  = (heapend - alignedstart) & ~mask;
  ngranules    = alignedsize >> log2gran;

  /* Allocate the information structure with a granule table of the
   * correct size.
   */

  priv = ( FAR struct gran_s *)zalloc(SIZEOF_GRAN_S(ngranules));
  if (priv)
    {
      /* Initialize non-zero elements of the granules heap info structure */

      priv->log2gran  = log2gran;
      priv->ngranules = ngranules;
      priv->heapstart = alignedstart;
    }

  return priv;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_initialize
 *
 * Description:
 *   Set up one granule allocator instance.  Allocations will be aligned to
 *   the granule size; allocations will be in units of the granule size.
 *   Larger granules will give better performance and less overhead but more
 *   losses of memory due to alignment and quantization waste.
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granaules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   heapstart - Start of the granule allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2gran  - Log base 2 of the size of one granule.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8bytes, etc.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   granual allocator interfaces.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
int gran_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2gran)
{
  g_graninfo = gran_common_initialize(heapstart, heapsize, log2gran);
  if (!g_granifo)
    {
      return -ENOMEM;
    }

  return OK;
}
#else
GRAN_HANDLE gran_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2gran)
{
  return (GRAN_HANDLE)gran_common_initialize(heapstart, heapsize, log2gran);
}
#endif

#endif /* CONFIG_GRAN */


