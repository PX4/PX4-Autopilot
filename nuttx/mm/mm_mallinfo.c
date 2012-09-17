/****************************************************************************
 * mm/mm_mallinfo.c
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

#include <assert.h>

#include "mm_environment.h"
#include "mm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current mallinfo.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
struct mallinfo mallinfo(void)
#else
int mallinfo(struct mallinfo *info)
#endif
{
  struct mm_allocnode_s *node;
  size_t mxordblk = 0; 
  int    ordblks  = 0;  /* Number of non-inuse chunks */
  size_t uordblks = 0;  /* Total allocated space */
  size_t fordblks = 0;  /* Total non-inuse space */
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif

#ifdef CONFIG_CAN_PASS_STRUCTS
  static struct mallinfo info;
#else
  if (!info)
    {
      return ERROR;
    }
#endif

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < g_nregions; region++)
#endif
    {
      /* Visit each node in the region */

      for (node = g_heapstart[region];
           node < g_heapend[region];
           node = (struct mm_allocnode_s *)((char*)node + node->size))
        {
          mvdbg("region=%d node=%p preceding=%p\n", region, node, node->preceding);
          if (node->preceding & MM_ALLOC_BIT)
            {
              uordblks += node->size;
            }
          else
            {
              ordblks++;
              fordblks += node->size;
              if (node->size > mxordblk)
                {
                  mxordblk = node->size;
                }
            }
        }

      mvdbg("region=%d node=%p g_heapend=%p\n", region, node, g_heapend[region]);
      DEBUGASSERT(node == g_heapend[region]);
      uordblks += SIZEOF_MM_ALLOCNODE; /* account for the tail node */
    }
#undef region

  DEBUGASSERT(uordblks + fordblks == g_heapsize);

#ifdef CONFIG_CAN_PASS_STRUCTS
  info.arena    = g_heapsize;
  info.ordblks  = ordblks;
  info.mxordblk = mxordblk;
  info.uordblks = uordblks;
  info.fordblks = fordblks;
  return info;
#else
  info->arena    = g_heapsize;
  info->ordblks  = ordblks;
  info->mxordblk = mxordblk;
  info->uordblks = uordblks;
  info->fordblks = fordblks;
  return OK;
#endif
}
