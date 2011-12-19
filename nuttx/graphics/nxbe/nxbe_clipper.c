/****************************************************************************
 * graphics/nxbe/nxbe_clipper.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdlib.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define NX_INITIAL_STACKSIZE (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is a container that retains an associated between a
 * window instance and a rectangle.
 */

struct nxbe_cliprect_s
{
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s        rect;
};

/* This is the stack of pending clip operations */

struct nxbe_clipstack_s
{
  uint16_t                npushed; /* Number of deferred rectangles in stack */
  uint16_t                mxrects; /* The capacity of the stack */
  struct nxbe_cliprect_s   *stack;   /* The stack of deferred rectangles */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_nxcliporder[4][4] =
{
  { NX_TOP_NDX,    NX_LEFT_NDX,  NX_RIGHT_NDX, NX_BOTTOM_NDX }, /* index = NX_CLIPORDER_TLRB */
  { NX_TOP_NDX,    NX_RIGHT_NDX, NX_LEFT_NDX,  NX_BOTTOM_NDX }, /*         NX_CLIPORDER_TRLB */
  { NX_BOTTOM_NDX, NX_LEFT_NDX,  NX_RIGHT_NDX, NX_TOP_NDX    }, /*         NX_CLIPORDER_BLRT */
  { NX_BOTTOM_NDX, NX_RIGHT_NDX, NX_LEFT_NDX,  NX_TOP_NDX    }  /*         NX_CLIPORDER_BRLT */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_pushrectangle
 ****************************************************************************/

static inline void nxbe_pushrectangle(FAR struct nxbe_clipstack_s *stack,
                                      FAR struct nxbe_window_s *wnd,
                                      FAR const struct nxgl_rect_s *rect)
{
  /* Check if there is room on the stack to hold another rectangle */

  if ((stack->npushed + 1) > stack->mxrects)
    {
      /* No then we will need to reallocate the stack to hole more */

      int mxrects = stack->mxrects ? 2 * stack->mxrects : NX_INITIAL_STACKSIZE;
      struct nxbe_cliprect_s *newstack;

      newstack = realloc(stack->stack, sizeof(struct nxbe_cliprect_s) * mxrects);
      if (!newstack)
        {
          gdbg("Failed to reallocate stack\n");
          return;
        }

      stack->stack   = newstack;
      stack->mxrects = mxrects;
    }

  /* Then push the new rectangle onto the stack */

  stack->stack[stack->npushed].wnd  = wnd;
  nxgl_rectcopy(&stack->stack[stack->npushed].rect, rect);
  stack->npushed++;
}

/****************************************************************************
 * Name: nxbe_poprectangle
 ****************************************************************************/

static inline bool nxbe_poprectangle(struct nxbe_clipstack_s *stack,
                                     FAR struct nxbe_window_s **wnd,
                                     struct nxgl_rect_s *rect)
{
  if(stack->npushed > 0)
    {
      stack->npushed--;
      *wnd = stack->stack[stack->npushed].wnd;
      nxgl_rectcopy(rect, &stack->stack[stack->npushed].rect);
      return true;
    }
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipper
 *
 * Descripton:
 *   Perform flexible clipping operations.  Callbacks are executed for
 *   each oscured and visible portions of the window.
 *
 * Input Parameters:
 *   wnd    - The window to be clipped.
 *   rect   - The region of concern within the window
 *   order  - Specifies the order to process the parts of the non-intersecting
 *            sub-rectangles.
 *   cops   - The callbacks to handle obscured and visible parts of the
 *            sub-rectangles.
 *   plane  - The raster operations to be used by the callback functions.
 *            These may vary with different color formats.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_clipper(FAR struct nxbe_window_s *wnd,
                  FAR const struct nxgl_rect_s *dest, uint8_t order,
                  FAR struct nxbe_clipops_s *cops,
                  FAR struct nxbe_plane_s *plane)
{
  struct nxbe_clipstack_s   stack;
  FAR struct nxbe_window_s *currw;
  struct nxgl_rect_s        rect;
  struct nxgl_rect_s        obscuredrect;
  struct nxgl_rect_s        currbounds;
  struct nxgl_rect_s        nonoverlapped[4];
  int                       i;

  /* Initialize the stack where we will keep deferred rectangle operations */

  stack.npushed = 0;
  stack.mxrects = 0;
  stack.stack   = NULL;

  /* Loop until there are no further pending operations */

  nxgl_rectcopy(&rect, dest); /* Start with the whole dest rectangle */
  do
    {
      /* Loop for every window from the current window and above.  Only windows
       * above the current window can obscure the current window
       */

      for (currw = wnd; currw; currw = currw->above)
        {
          /* Does the current window overlap the dest rectangle? */

          currbounds = currw->bounds;
          if (nxgl_rectoverlap(&rect, &currbounds))
            {
              /* Yes.. then it obscures all or part of the dest rectangle.
               * Divide the potentially visible, non-overlapping regions into 4
               * smaller rectangles and push them onto the stack for processing
               * on the next time through the outer loop.
               */

              nxgl_nonintersecting(nonoverlapped, &rect, &currbounds);
              for (i = 3; i >= 0; i--)
                {
                  /* Push the rectangles in the order specific by the input
                   * argument of that name.
                   */

                  struct nxgl_rect_s *candidate = &nonoverlapped[g_nxcliporder[order][i]];
                  if (!nxgl_nullrect(candidate))
                    {
                      nxbe_pushrectangle(&stack, currw->above, candidate);
                    }
                }

              /* Now performed any required processing on the obscurred,
               * overlapped region.
               */

              nxgl_rectintersect(&obscuredrect, &rect, &currbounds);
              cops->obscured(cops, plane, &obscuredrect);

              /* Break out of the loop to process the pushed rectangles */

              break;
            }
        }

      /* If there are no other windows overlapping this rectangle, then this
       * rectangular region must be visible.
       */

      if (!currw && !nxgl_nullrect(&rect))
        {
          cops->visible(cops, plane, &rect);
        }
    }
  while (nxbe_poprectangle(&stack, &wnd, &rect));

  /* Done! If any stack was allocated, then free it before exit-ting */

  if (stack.stack)
    {
      free(stack.stack);
    }
}

/****************************************************************************
 * Name: nxbe_clipnull
 *
 * Descripton:
 *   The do-nothing clipping callback function
 *
 ****************************************************************************/

void nxbe_clipnull(FAR struct nxbe_clipops_s *cops,
                   FAR struct nxbe_plane_s *plane,
                   FAR const struct nxgl_rect_s *rect)
{
}

