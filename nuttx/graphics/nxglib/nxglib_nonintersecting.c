/****************************************************************************
 * graphics/nxglib/nxsglib_rectnonintersecting.c
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_nonintersecting
 *
 * Description:
 *   Return the regions of rectangle rect 1 that do not intersect with
 *   rect2.  This may be up to founr rectangles some of which may be
 *   degenerate (and can be picked off with nxgl_nullrect)
 *
 ****************************************************************************/

void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4],
                          FAR const struct nxgl_rect_s *rect1,
                          FAR const struct nxgl_rect_s *rect2)
{
  struct nxgl_rect_s intersection;

  /* Get the intersection of the two rectangles */

  nxgl_rectintersect(&intersection, rect1, rect2);

  /* Then return the four rectangles representing the regions NON included
   * in the intersection.  Some of these rectangles may be invalid (zero
   * area), but those can be picked off using nxgl_nullrect()
   *
   *  rect1.pt1
   *   +-------------------------+
   *   |         rect2.pt1       |
   *   |         int.pt1         |
   *   |         +-------------------------+
   *   |         |               |         |
   *   |         |               |         |
   *   +-------------------------+         |
   *             |               rect1.pt2 |
   *             |               int.pt2   |
   *             +-------------------------+
   *                                       rect2.pt2
   *             rect1.pt1
   *             +-------------------------+
   *   rect2.pt1 |int.pt1                  |
   *   +---------+---------------+         |
   *   |         |               |         |
   *   |         |               |         |
   *   |         |               |int.pt2  |
   *   |         +---------------+---------+
   *   |                         |         rect1.pt2
   *   +-------------------------+
   *                             rect2.pt2
   *   rect2.pt1
   *   +-------------------------+
   *   |         rect1.pt1       |
   *   |         int.pt1         |
   *   |         +-------------------------+
   *   |         |               |         |
   *   |         |               |         |
   *   |         |               |         |
   *   +---------+---------------+         |
   *             |               rect2.pt2 |
   *             |               int.pt2   |
   *             +-------------------------+
   *                                       rect1.pt2
   */

  result[NX_TOP_NDX].pt1.x    = rect1->pt1.x;
  result[NX_TOP_NDX].pt1.y    = rect1->pt1.y;
  result[NX_TOP_NDX].pt2.x    = rect1->pt2.x;
  result[NX_TOP_NDX].pt2.y    = intersection.pt1.y - 1;

  result[NX_BOTTOM_NDX].pt1.x = rect1->pt1.x;
  result[NX_BOTTOM_NDX].pt1.y = intersection.pt2.y + 1;
  result[NX_BOTTOM_NDX].pt2.x = rect1->pt2.x;
  result[NX_BOTTOM_NDX].pt2.y = rect1->pt2.y;

  result[NX_LEFT_NDX].pt1.x   = rect1->pt1.x;
  result[NX_LEFT_NDX].pt1.y   = intersection.pt1.y;
  result[NX_LEFT_NDX].pt2.x   = intersection.pt1.x - 1;
  result[NX_LEFT_NDX].pt2.y   = intersection.pt2.y;

  result[NX_RIGHT_NDX].pt1.x  = intersection.pt2.x + 1;
  result[NX_RIGHT_NDX].pt1.y  = intersection.pt1.y;
  result[NX_RIGHT_NDX].pt2.x  = rect1->pt2.x;
  result[NX_RIGHT_NDX].pt2.y  = intersection.pt2.y;
}


