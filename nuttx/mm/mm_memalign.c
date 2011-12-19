/************************************************************
 * mm/mm_memalign.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <assert.h>
#include "mm_environment.h"
#include "mm_internal.h"

/************************************************************
 * Pre-processor Definitions
 ************************************************************/

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc,
 *   finds a region within that chunk that meets the alignment
 *   request and then frees any leading or trailing space. 
 *
 *   The alignment argument must be a power of two (not
 *   checked).  8-byte alignment is guaranteed by normal
 *   malloc calls.
 *
 ************************************************************/

FAR void *memalign(size_t alignment, size_t size)
{
  FAR struct mm_allocnode_s *node;
  size_t rawchunk;
  size_t alignedchunk;
  size_t mask = (size_t)(alignment - 1);
  size_t allocsize;

  /* If this requested alignement less than or equal to the
   * natural alignment of malloc, then just let malloc do the
   * work.
   */

  if (alignment <= MM_MIN_CHUNK)
    {
      return malloc(size);
    }

  /* Adjust the size to account for (1) the size of the allocated
   * node, (2) to make sure that it is an even multiple of
   * our granule size, and to include the alignment amount.
   *
   * Notice that we increase the allocation size by twice the
   * the requested alignment.  We do this so that there will
   * be at least two valid alignment points within the allocated
   * memory.
   *
   * NOTE:  These are sizes given to malloc and not chunk sizes.
   * The do not include SIZEOF_MM_ALLOCNODE.
   */

  size      = MM_ALIGN_UP(size);   /* Make multiples of our granule size */
  allocsize = size + 2*alignment;  /* Add double full alignment size */

  /* Then malloc that size */

  rawchunk = (size_t)malloc(allocsize);
  if (rawchunk == 0)
    {
      return NULL;
    }

  /* We need to hold the MM semaphore while we muck with the
   * chunks and nodelist.
   */

  mm_takesemaphore();

  /* Get the node associated with the allocation and the next
   * node after the allocation.
   */

  node = (FAR struct mm_allocnode_s*)(rawchunk - SIZEOF_MM_ALLOCNODE);

  /* Find the aligned subregion */

  alignedchunk = (rawchunk + mask) & ~mask;

  /* Check if there is free space at the beginning of the aligned chunk */

  if (alignedchunk != rawchunk)
    {
      FAR struct mm_allocnode_s *newnode;
      FAR struct mm_allocnode_s *next;
      size_t precedingsize;

      /* Get the node the next node after the allocation. */

      next = (FAR struct mm_allocnode_s*)((char*)node + node->size);

      /* Make sure that there is space to convert the preceding mm_allocnode_s
       * into an mm_freenode_s.  I think that this should always be true
       */

      DEBUGASSERT(alignedchunk >= rawchunk + 8);

      newnode = (FAR struct mm_allocnode_s*)(alignedchunk - SIZEOF_MM_ALLOCNODE);

      /* Preceding size is full size of the new 'node,' including
       * SIZEOF_MM_ALLOCNODE
       */

      precedingsize = (size_t)newnode - (size_t)node;

      /* If we were unlucky, then the alignedchunk can lie in such
       * a position that precedingsize < SIZEOF_NODE_FREENODE.  We 
       * can't let that happen because we are going to cast 'node' to
       * struct mm_freenode_s below.  This is why we allocated memory
       * large enough to support two alignment points.  In this case,
       * we will simply use the second alignment point.
       */

      if (precedingsize < SIZEOF_MM_FREENODE)
        {
          alignedchunk += alignment;
          newnode       = (FAR struct mm_allocnode_s*)(alignedchunk - SIZEOF_MM_ALLOCNODE);
          precedingsize = (size_t)newnode - (size_t)node;
        }

      /* Set up the size of the new node */

      newnode->size = (size_t)next - (size_t)newnode;
      newnode->preceding = precedingsize | MM_ALLOC_BIT;

      /* Reduce the size of the original chunk and mark it not allocated, */

      node->size = precedingsize;
      node->preceding &= ~MM_ALLOC_BIT;

      /* Fix the preceding size of the next node */

      next->preceding = newnode->size | (next->preceding & MM_ALLOC_BIT);

      /* Convert the newnode chunk size back into malloc-compatible
       * size by subtracting the header size SIZEOF_MM_ALLOCNODE.
       */

      allocsize = newnode->size - SIZEOF_MM_ALLOCNODE;

      /* Add the original, newly freed node to the free nodelist */

      mm_addfreechunk((FAR struct mm_freenode_s *)node);

      /* Replace the original node with the newlay realloaced,
       * aligned node
       */

      node = newnode;
    }

  /* Check if there is free space at the end of the aligned chunk */

  if (allocsize > size)
    {
      /* Shrink the chunk by that much -- remember, mm_shrinkchunk
       * wants internal chunk sizes that include SIZEOF_MM_ALLOCNODE,
       * and not the malloc-compatible sizes that we have.
       */

      mm_shrinkchunk(node, size + SIZEOF_MM_ALLOCNODE);
    }

  mm_givesemaphore();
  return (FAR void*)alignedchunk;
}
