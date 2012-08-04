/****************************************************************************
 * mm/mm_shrinkchunk.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_shrinkchunk
 *
 * Description:
 *   Reduce the size of the chunk specified by the node structure to the
 *   specified size.  this internal logic is used both from memalign to
 *   dispose of any trailing memory in the aligned allocation and also by
 *   realloc when there is a request to reduce the size of an allocation.
 *
 *   NOTES:
 *     (1) size is the whole chunk size (payload and header)
 *     (2) the caller must hold the MM semaphore.
 *
 ****************************************************************************/

void  mm_shrinkchunk(FAR struct mm_allocnode_s *node, size_t size)
{
  FAR struct mm_freenode_s *next;

  /* Get a reference to the next node */

  next = (FAR struct mm_freenode_s*)((char*)node + node->size);

  /* Check if it is free */

  if ((next->preceding & MM_ALLOC_BIT) == 0)
    {
      FAR struct mm_allocnode_s *andbeyond;
      FAR struct mm_freenode_s *newnode;

      /* Get the chunk next the next node (which could be the tail chunk) */

      andbeyond = (FAR struct mm_allocnode_s*)((char*)next + next->size);

      /* Remove the next node.  There must be a predecessor, but there may
       * not be a successor node.
       */

      DEBUGASSERT(next->blink);
      next->blink->flink = next->flink;
      if (next->flink)
        {
          next->flink->blink = next->blink;
        }

      /* Create a new chunk that will hold both the next chunk and the
       * tailing memory from the aligned chunk.
       */

      newnode = (FAR struct mm_freenode_s*)((char*)node + size);

      /* Set up the size of the new node */

      newnode->size        = next->size + node->size - size;
      newnode->preceding   = size;
      node->size           = size;
      andbeyond->preceding = newnode->size | (andbeyond->preceding & MM_ALLOC_BIT);

      /* Add the new node to the freenodelist */

      mm_addfreechunk(newnode);
    }

  /* The next chunk is allocated.  Try to free the end portion at the end
   * chunk to be shrunk.
   */

  else if (node->size >= size + SIZEOF_MM_FREENODE)
    {
      FAR struct mm_freenode_s *newnode;

      /* Create a new chunk that will hold both the next chunk and the
       * tailing memory from the aligned chunk.
       */

      newnode = (FAR struct mm_freenode_s*)((char*)node + size);

      /* Set up the size of the new node */

      newnode->size        = node->size - size;
      newnode->preceding   = size;
      node->size           = size;
      next->preceding      = newnode->size | MM_ALLOC_BIT;

      /* Add the new node to the freenodelist */

      mm_addfreechunk(newnode);
    }
}
