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
#include <assert.h>
#include <errno.h>

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
 *   heapstart - Start of the granule allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2gran  - Log base 2 of the size of one granule.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.
 *   log2align - Log base 2 of required alignment.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.  Note that
 *               log2gran must be greater than or equal to log2align
 *               so that all contiguous granules in memory will meet
 *               the minimum alignment requirement. A value of zero
 *               would mean that no alignment is required.
 *
 * Returned Value:
 *   On success, a non-NULL info structure is returned that may be used with
 *   other granule allocator interfaces.
 *
 ****************************************************************************/

static inline FAR struct gran_s *
gran_common_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2gran,
                       uint8_t log2align)
{
  FAR struct gran_s *priv;
  uintptr_t          heapend;
  uintptr_t          alignedstart;
  unsigned int       mask;
  unsigned int       alignedsize;
  unsigned int       ngranules;

  /* Check parameters if debug is on.  Note the the size of a granual is
   * limited to 2**31 bytes and that the size of the granule must be greater
   * than the alignment size.
   */

  DEBUGASSERT(heapstart && heapsize > 0 &&
              log2gran > 0 && log2gran < 32 &&
              log2gran > log2align);

  /* Get the aligned start of the heap */

  mask         = (1 << log2align) - 1;
  alignedstart = ((uintptr_t)heapstart + mask) & ~mask;
  
  /* Determine the number of granules */

  mask         = (1 << log2gran) - 1;
  heapend      = (uintptr_t)heapstart + heapsize;
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

      /* Initialize mutual exclusion support */

#ifndef CONFIG_GRAN_INTR
      sem_init(&priv->exclsem, 0, 1);
#endif
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
 *   the alignment size (log2align; allocations will be in units of the
 *   granule size (log2gran). Larger granules will give better performance
 *   and less overhead but more losses of memory due to quantization waste.
 *   Additional memory waste can occur from alignment; log2align should be
 *   set to 0 unless you are using the granule allocator to manage DMA memory
 *   and your hardware has specific memory alignment requirements.
 *
 *   Geneneral Usage Summary.  This is an example using the GCC section
 *   attribute to position a DMA heap in memory (logic in the linker script
 *   would assign the section .dmaheap to the DMA memory.
 *
 *     FAR uint32_t g_dmaheap[DMAHEAP_SIZE] __attribute__((section(.dmaheap)));
 *
 *   The heap is created by calling gran_initialize().  Here the granual size
 *   is set to 64 bytes (2**6) and the alignment to 16 bytes (2**4):
 *
 *     GRAN_HANDLE handle = gran_initialize(g_dmaheap, DMAHEAP_SIZE, 6, 4);
 *
 *   Then the GRAN_HANDLE can be used to allocate memory (There is no
 *   GRAN_HANDLE if CONFIG_GRAN_SINGLE=y):
 *
 *     FAR uint8_t *dma_memory = (FAR uint8_t *)gran_alloc(handle, 47);
 *
 *   The actual memory allocates will be 64 byte (wasting 17 bytes) and
 *   will be aligned at least to (1 << log2align).
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   heapstart - Start of the granule allocation heap
 *   heapsize  - Size of heap in bytes
 *   log2gran  - Log base 2 of the size of one granule.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.
 *   log2align - Log base 2 of required alignment.  0->1 byte,
 *               1->2 bytes, 2->4 bytes, 3-> 8 bytes, etc.  Note that
 *               log2gran must be greater than or equal to log2align
 *               so that all contiguous granules in memory will meet
 *               the minimum alignment requirement. A value of zero
 *               would mean that no alignment is required.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that may be used with other
 *   granual allocator interfaces.
 *
 ****************************************************************************/

#ifdef CONFIG_GRAN_SINGLE
int gran_initialize(FAR void *heapstart, size_t heapsize, uint8_t log2gran,
                    uint8_t log2align)
{
  g_graninfo = gran_common_initialize(heapstart, heapsize, log2gran,
                                      log2align);
  if (!g_graninfo)
    {
      return -ENOMEM;
    }

  return OK;
}
#else
GRAN_HANDLE gran_initialize(FAR void *heapstart, size_t heapsize,
                            uint8_t log2gran, uint8_t log2align)
{
  return (GRAN_HANDLE)gran_common_initialize(heapstart, heapsize,
                                             log2gran, log2align);
}
#endif

#endif /* CONFIG_GRAN */
