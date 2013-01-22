/****************************************************************************
 * mm/mm_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#include "mm_environment.h"
#include "mm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* This is the size of the heap provided to mm */

size_t g_heapsize;

/* This is the first and last nodes of the heap */

FAR struct mm_allocnode_s *g_heapstart[CONFIG_MM_REGIONS];
FAR struct mm_allocnode_s *g_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
int g_nregions;
#endif

/* All free nodes are maintained in a doubly linked list.  This array
 * provides some hooks into the list at various points to speed searches for
 * free nodes.
 */

FAR struct mm_freenode_s g_nodelist[MM_NNODES];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_initialize
 *
 * Description:
 *   This is an internal OS function called only at power-up boot time.
 *
 * Parameters:
 *   heapstart - Start of the initial heap region
 *   heapsize  - Size of the initial heap region
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_initialize(FAR void *heapstart, size_t heapsize)
{
  int i;

  mlldbg("Heap: start=%p size=%u\n", heapstart, heapsize);

  /* The following two lines have cause problems for some older ZiLog
   * compilers in the past (but not the more recent).  Life is easier if we
   * just the suppress them altogther for those tools.
   */

#ifndef __ZILOG__
  CHECK_ALLOCNODE_SIZE;
  CHECK_FREENODE_SIZE;
#endif

  /* Set up global variables */

  g_heapsize = 0;

#if CONFIG_MM_REGIONS > 1
  g_nregions = 0;
#endif

  /* Initialize the node array */

  memset(g_nodelist, 0, sizeof(struct mm_freenode_s) * MM_NNODES);
  for (i = 1; i < MM_NNODES; i++)
    {
      g_nodelist[i-1].flink = &g_nodelist[i];
      g_nodelist[i].blink   = &g_nodelist[i-1];
    }

  /* Initialize the malloc semaphore to one (to support one-at-
   * a-time access to private data sets).
   */

  mm_seminitialize();

  /* Add the initial region of memory to the heap */

  mm_addregion(heapstart, heapsize);
}

/****************************************************************************
 * Name: mm_addregion
 *
 * Description:
 *   This function gives a region of contiguous memory to the memory manager
 *
 * Parameters:
 *   heapstart - Start of the heap region
 *   heapsize  - Size of the heap region
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_addregion(FAR void *heapstart, size_t heapsize)
{
  FAR struct mm_freenode_s *node;
  uintptr_t heapbase;
  uintptr_t heapend;
#if CONFIG_MM_REGIONS > 1
  int IDX = g_nregions;
#else
# define IDX 0
#endif

  /* If the MCU handles wide addresses but the memory manager is configured
   * for a small heap, then verify that the caller is  not doing something
   * crazy.
   */

#if defined(CONFIG_MM_SMALL) && !defined(CONFIG_SMALL_MEMORY)
  DEBUGASSERT(heapsize <= MMSIZE_MAX+1);
#endif

  /* Adjust the provide heap start and size so that they are both aligned
   * with the MM_MIN_CHUNK size.
   */

  heapbase = MM_ALIGN_UP((uintptr_t)heapstart);
  heapend  = MM_ALIGN_DOWN((uintptr_t)heapstart + (uintptr_t)heapsize);
  heapsize = heapend - heapbase;

  mlldbg("Region %d: base=%p size=%u\n", IDX+1, heapstart, heapsize);

  /* Add the size of this region to the total size of the heap */

  g_heapsize += heapsize;

  /* Create two "allocated" guard nodes at the beginning and end of
   * the heap.  These only serve to keep us from allocating outside
   * of the heap.
   * 
   * And create one free node between the guard nodes that contains
   * all available memory.
   */

  g_heapstart[IDX]            = (FAR struct mm_allocnode_s *)heapbase;
  g_heapstart[IDX]->size      = SIZEOF_MM_ALLOCNODE;
  g_heapstart[IDX]->preceding = MM_ALLOC_BIT;

  node                        = (FAR struct mm_freenode_s *)(heapbase + SIZEOF_MM_ALLOCNODE);
  node->size                  = heapsize - 2*SIZEOF_MM_ALLOCNODE;
  node->preceding             = SIZEOF_MM_ALLOCNODE;

  g_heapend[IDX]              = (FAR struct mm_allocnode_s *)(heapend - SIZEOF_MM_ALLOCNODE);
  g_heapend[IDX]->size        = SIZEOF_MM_ALLOCNODE;
  g_heapend[IDX]->preceding   = node->size | MM_ALLOC_BIT;

#undef IDX

#if CONFIG_MM_REGIONS > 1
  g_nregions++;
#endif

  /* Add the single, large free node to the nodelist */

  mm_addfreechunk(node);
}
