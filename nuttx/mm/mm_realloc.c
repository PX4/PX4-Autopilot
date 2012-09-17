/****************************************************************************
 * mm/mm_realloc.c
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

#include <string.h>
#include "mm_environment.h"
#include <stdio.h>
#include "mm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: realloc
 *
 * Description:
 *   If the reallocation is for less space, then:
 *
 *     (1) the current allocation is reduced in size
 *     (2) the remainder at the end of the allocation is returned to the
 *         free list.
 *
 *  If the request is for more space and the current allocation can be
 *  extended, it will be extended by:
 *
 *     (1) Taking the additional space from the following free chunk, or
 *     (2) Taking the additional space from the preceding free chunk.
 *     (3) Or both
 *
 *  If the request is for more space but the current chunk cannot be
 *  extended, then malloc a new buffer, copy the data into the new buffer,
 *  and free the old buffer.
 *
 ****************************************************************************/

FAR void *realloc(FAR void *oldmem, size_t size)
{
  FAR struct mm_allocnode_s *oldnode;
  FAR struct mm_freenode_s  *prev;
  FAR struct mm_freenode_s  *next;
  size_t oldsize;
  size_t prevsize = 0;
  size_t nextsize = 0;
  FAR void *newmem;

  /* If oldmem is NULL, then realloc is equivalent to malloc */

  if (!oldmem)
    {
      return malloc(size);
    }

  /* If size is zero, then realloc is equivalent to free */

  if (size <= 0)
    {
      free(oldmem);
      return NULL;
    }

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is an even multiple of our granule size.
   */

  size = MM_ALIGN_UP(size + SIZEOF_MM_ALLOCNODE);

  /* Map the memory chunk into an allocated node structure */

  oldnode = (FAR struct mm_allocnode_s *)((FAR char*)oldmem - SIZEOF_MM_ALLOCNODE);

  /* We need to hold the MM semaphore while we muck with the nodelist. */

  mm_takesemaphore();

  /* Check if this is a request to reduce the size of the allocation. */

  oldsize = oldnode->size;
  if (size <= oldsize)
    {
      /* Handle the special case where we are not going to change the size
       * of the allocation.
       */

      if (size < oldsize)
        {
          mm_shrinkchunk(oldnode, size);
        }

      /* Then return the original address */

      mm_givesemaphore();
      return oldmem;
    }

  /* This is a request to increase the size of the allocation,  Get the
   * available sizes before and after the oldnode so that we can make the
   * best decision
   */

  next = (FAR struct mm_freenode_s *)((FAR char*)oldnode + oldnode->size);
  if ((next->preceding & MM_ALLOC_BIT) == 0)
    {
      nextsize = next->size;
    }

  prev = (FAR struct mm_freenode_s *)((FAR char*)oldnode - (oldnode->preceding & ~MM_ALLOC_BIT));
  if ((prev->preceding & MM_ALLOC_BIT) == 0)
    {
      prevsize = prev->size;
    }

  /* Now, check if we can extend the current allocation or not */

  if (nextsize + prevsize + oldsize >= size)
    {
      size_t needed   = size - oldsize;
      size_t takeprev = 0;
      size_t takenext = 0;

      /* Check if we can extend into the previous chunk and if the
       * previous chunk is smaller than the next chunk.
       */

      if (prevsize > 0 && (nextsize >= prevsize || nextsize <= 0))
        {
          /* Can we get everything we need from the previous chunk? */

          if (needed > prevsize)
            {
              /* No, take the whole previous chunk and get the
               * rest that we need from the next chunk.
               */

              takeprev = prevsize;
              takenext = needed - prevsize;
            }
          else
            {
              /* Yes, take what we need from the previous chunk */

              takeprev = needed;
              takenext = 0;
            }

          needed = 0;
        }

      /* Check if we can extend into the next chunk and if we still need
       * more memory.
       */

      if (nextsize > 0 && needed)
        {
          /* Can we get everything we need from the next chunk? */

          if (needed > nextsize)
            {
              /* No, take the whole next chunk and get the rest that we
               * need from the previous chunk.
               */

              takeprev = needed - nextsize;
              takenext = nextsize;
            }
          else
            {
              /* Yes, take what we need from the previous chunk */

              takeprev = 0;
              takenext = needed;
            }
        }

      /* Extend into the previous free chunk */

      newmem = oldmem;
      if (takeprev)
        {
          FAR struct mm_allocnode_s *newnode;

          /* Remove the previous node.  There must be a predecessor, but
           * there may not be a successor node.
           */

          DEBUGASSERT(prev->blink);
          prev->blink->flink = prev->flink;
          if (prev->flink)
            {
              prev->flink->blink = prev->blink;
            }

          /* Extend the node into the previous free chunk */

          newnode = (FAR struct mm_allocnode_s *)((FAR char*)oldnode - takeprev);

          /* Did we consume the entire preceding chunk? */

          if (takeprev < prevsize)
            {
              /* No.. just take what we need from the previous chunk and put
               * it back into the free list
               */

              prev->size        -= takeprev;
              newnode->size      = oldsize + takeprev;
              newnode->preceding = prev->size | MM_ALLOC_BIT;
              next->preceding    = newnode->size | (next->preceding & MM_ALLOC_BIT);

              /* Return the previous free node to the nodelist (with the new size) */

              mm_addfreechunk(prev);

             /* Now we want to return newnode */

              oldnode = newnode;
            }
          else
            {
              /* Yes.. update its size (newnode->preceding is already set) */

              newnode->size      += oldsize;
              newnode->preceding |= MM_ALLOC_BIT;
              next->preceding     = newnode->size | (next->preceding & MM_ALLOC_BIT);
            }

          oldnode = newnode;
          oldsize = newnode->size;

          /* Now we have to move the user contents 'down' in memory.  memcpy should
           * should be save for this.
           */

          newmem = (FAR void*)((FAR char*)newnode + SIZEOF_MM_ALLOCNODE);
          memcpy(newmem, oldmem, oldsize - SIZEOF_MM_ALLOCNODE);
        }

      /* Extend into the next free chunk */

      if (takenext)
        {
          FAR struct mm_freenode_s *newnode;
          FAR struct mm_allocnode_s *andbeyond;

          /* Get the chunk following the next node (which could be the tail
           * chunk)
           */

          andbeyond = (FAR struct mm_allocnode_s*)((char*)next + nextsize);

          /* Remove the next node.  There must be a predecessor, but there
           * may not be a successor node.
           */

          DEBUGASSERT(next->blink);
          next->blink->flink = next->flink;
          if (next->flink)
            {
              next->flink->blink = next->blink;
            }

          /* Extend the node into the next chunk */

          oldnode->size = oldsize + takenext;
          newnode       = (FAR struct mm_freenode_s *)((char*)oldnode + oldnode->size);

          /* Did we consume the entire preceding chunk? */

          if (takenext < nextsize)
            {
              /* No, take what we need from the next chunk and return it to
               * the free nodelist.
               */

              newnode->size        = nextsize - takenext;
              newnode->preceding   = oldnode->size;
              andbeyond->preceding = newnode->size | (andbeyond->preceding & MM_ALLOC_BIT);

              /* Add the new free node to the nodelist (with the new size) */

              mm_addfreechunk(newnode);
            }
          else
            {
              /* Yes, just update some pointers. */

              andbeyond->preceding = oldnode->size | (andbeyond->preceding & MM_ALLOC_BIT);
            }
        }

      mm_givesemaphore();
      return newmem;
    }

  /* The current chunk cannot be extended.  Just allocate a new chunk and copy */

  else
    {
      /* Allocate a new block.  On failure, realloc must return NULL but
       * leave the original memory in place.
       */

      mm_givesemaphore();
      newmem = (FAR void*)malloc(size);
      if (newmem)
        {
          memcpy(newmem, oldmem, oldsize);
          free(oldmem);
        }

      return newmem;
    }
}
