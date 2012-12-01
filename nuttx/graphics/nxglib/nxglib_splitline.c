/****************************************************************************
 * graphics/nxglib/nxglib_splitline.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define SMALL_SIN 1966 /* 1966/65536 = 0.03 */

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
 * Name: nxgl_splitline
 *
 * Description:
 *   In the general case, a line with width can be represented as a
 *   parallelogram with a triangle at the top and bottom.  Triangles and
 *   parallelograms are both degenerate versions of a trapeziod.  This
 *   function breaks a wide line into triangles and trapezoids.  This
 *   function also detects other degenerate cases:
 *
 *   1. If y1 == y2 then the line is horizontal and is better represented
 *      as a rectangle.
 *   2. If x1 == x2 then the line is vertical and also better represented
 *      as a rectangle.
 *   3. If the width of the line is 1, then there are no triangles at the
 *      top and bottome (this may also be the case if the width is narrow
 *      and the line is near vertical).
 *   4. If the line is oriented is certain angles, it may consist only of
 *      the upper and lower triangles with no trapezoid in between.  In
 *      this case, 3 trapezoids will be returned, but traps[1] will be
 *      degenerate.
 *
 * Input parameters:
 *   vector - A pointer to the vector described the line to be drawn.
 *   traps  - A pointer to a array of trapezoids (size 3).
 *   rect   - A pointer to a rectangle.
 *
 * Returned value:
 *   0: Line successfully broken up into three trapezoids.  Values in
 *      traps[0], traps[1], and traps[2] are valid.
 *   1: Line successfully represented by one trapezoid. Value in traps[1]
 *      is valid.
 *   2: Line successfully represented by one rectangle. Value in rect is
 *      valid
 *  <0: On errors, a negated errno value is returned.
 *
 ****************************************************************************/

int nxgl_splitline(FAR struct nxgl_vector_s *vector,
                   FAR struct nxgl_trapezoid_s *traps,
                   FAR struct nxgl_rect_s *rect,
                   nxgl_coord_t linewidth)
{
  struct nxgl_vector_s line;
  nxgl_coord_t iheight;
  nxgl_coord_t iwidth;
  nxgl_coord_t iy;
  nxgl_coord_t triheight;
  nxgl_coord_t halfheight;
  b16_t adjwidth;
  b16_t xoffset;
  b16_t halfoffset;
  b16_t angle;
  b16_t sinangle;
  b16_t b16x;

  /* First, check the linewidth */

  if (linewidth < 1)
    {
      return -EINVAL;
    }

  /* Then make sure that the start position of the line is above the end
   * position of the line... in raster order.
   */

  if (vector->pt1.y < vector->pt2.y)
    {
      /* Vector is already in raster order */

      memcpy(&line, vector, sizeof(struct nxgl_vector_s));
    }
  else if (vector->pt1.y > vector->pt2.y)
    {
      /* Swap the top and bottom */

      line.pt1.x = vector->pt2.x;
      line.pt1.y = vector->pt2.y;
      line.pt2.x = vector->pt1.x;
      line.pt2.y = vector->pt1.y;
    }
  else
    {
      /* First degenerate case:  The line is horizontal. */

      if (vector->pt1.x < vector->pt2.x)
        {
          rect->pt1.x = vector->pt1.x;
          rect->pt2.x = vector->pt2.x;
        }
      else
        {
          rect->pt1.x = vector->pt2.x;
          rect->pt2.x = vector->pt1.x;
        }

      /* The height of the rectangle is the width of the line, half above
       * and half below.
       */

      rect->pt1.y = vector->pt1.y - (linewidth >> 1);
      rect->pt2.y = rect->pt1.y + linewidth - 1;
      return 2;
    }

  /* Check if the line is vertical */

  if (line.pt1.x == line.pt2.x)
    {
      /* Second degenerate case:  The line is vertical. */

      rect->pt1.y = line.pt1.y;
      rect->pt2.y = line.pt2.y;

      rect->pt1.x = line.pt1.x - (linewidth >> 1);
      rect->pt2.x = rect->pt1.x + linewidth - 1;
      return 2;
    }

  /* The final degenerate case */

  if (linewidth == 1 &&
      abs(line.pt2.x - line.pt1.x) < (line.pt2.y - line.pt1.y))
    {
      /* A close to vertical line of width 1 is basically
       * a single parallelogram of width 1.
       */

      traps[1].top.x1 = itob16(line.pt1.x);
      traps[1].top.x2 = traps[1].top.x1;
      traps[1].top.y  = line.pt1.y;

      traps[1].bot.x1 = itob16(line.pt2.x);
      traps[1].bot.x2 = traps[1].bot.x1;
      traps[1].bot.y  = line.pt2.y;
      return 1;
    }

  /* Okay, then what remains is interesting.
   *
   * iheight = |y2 - y1|
   * iwidth  = |x2 - x1|
   */

  iheight = line.pt2.y - line.pt1.y + 1;
  if (line.pt1.x < line.pt2.x)
    {
      iwidth  = line.pt2.x - line.pt1.x + 1;
    }
  else
    {
      iwidth  = line.pt1.x - line.pt2.x + 1;
    }

  /* Triangle height: linewidth * cosA
   * Adjusted width:  triheight / sinA
   * X offset :  linewidth * linewidth / adjusted line width
   */

  angle        = b16atan2(itob16(iheight), itob16(iwidth));
  triheight    = b16toi(linewidth * b16cos(angle) + b16HALF);
  halfheight   = (triheight >> 1);

  /* If the sine of the angle is tiny (i.e., the line is nearly horizontal),
   * then we cannot compute the adjusted width.  In this case, just use
   * the width of the line bounding box.
   */

  sinangle     =  b16sin(angle);
  if (sinangle < SMALL_SIN)
    {
      adjwidth = itob16(iwidth);
      xoffset  = 0;
    }
  else
    {
      adjwidth = b16divb16(itob16(linewidth), sinangle);
      xoffset  = itob16(linewidth * linewidth);
      xoffset  = b16divb16(xoffset, adjwidth);
    }

  halfoffset   = (xoffset >> 1);

  /* Return the top triangle (if there is one).  NOTE that the horizontal
   * (z) positions are represented with 16 bits of fraction.  The vertical
   * (y) positions, on the other hand, are integer.
   */

  if (triheight > 0)
    {
      if (line.pt1.x < line.pt2.x)
        {
          /* Line is going "south east" */

          b16x = itob16(line.pt1.x) - halfoffset;
          iy   = line.pt1.y + halfheight;

          traps[0].top.x1 = b16x + xoffset;
          traps[0].top.x2 = traps[0].top.x1;
          traps[0].top.y  = iy - triheight + 1;
          traps[0].bot.x1 = b16x;
          traps[0].bot.x2 = b16x + adjwidth - b16ONE;
          traps[0].bot.y  = iy;

          b16x = itob16(line.pt2.x) + halfoffset;
          iy   = line.pt2.y - halfheight;

          traps[2].top.x1 = b16x - adjwidth + b16ONE;
          traps[2].top.x2 = b16x;
          traps[2].top.y  = iy;
          traps[2].bot.x1 = b16x - xoffset;
          traps[2].bot.x2 = traps[2].bot.x1;
          traps[2].bot.y  = iy + triheight - 1;
        }
      else
        {
          /* Line is going "south west" */

          b16x = itob16(line.pt1.x) + halfoffset;
          iy   = line.pt1.y + halfheight;

          traps[0].top.x1 = b16x - xoffset;
          traps[0].top.x2 = traps[0].top.x1;
          traps[0].top.y  = iy - triheight + 1;
          traps[0].bot.x1 = b16x - adjwidth + b16ONE;
          traps[0].bot.x2 = b16x;
          traps[0].bot.y  = iy;

          b16x = itob16(line.pt2.x) - halfoffset;
          iy   = line.pt2.y - halfheight;

          traps[2].top.x1 = b16x;
          traps[2].top.x2 = b16x + adjwidth - b16ONE;
          traps[2].top.y  = iy;
          traps[2].bot.x1 = b16x + xoffset;
          traps[2].bot.x2 = traps[2].bot.x1;
          traps[2].bot.y  = iy + triheight - 1;
        }

      /* The center parallelogram is the horizontal edge of each triangle.
       * Note the minor inefficency: that horizontal edges are drawn twice.
       */

      traps[1].top.x1 = traps[0].bot.x1;
      traps[1].top.x2 = traps[0].bot.x2;
      traps[1].top.y  = traps[0].bot.y;
 
      traps[1].bot.x1 = traps[2].top.x1;
      traps[1].bot.x2 = traps[2].top.x2;
      traps[1].bot.y  = traps[2].top.y;
 
      return 0;
    }

  /* The line is too vertical to have any significant triangular top or
   * bottom.  Just return the center parallelogram.
   */

  traps[1].top.x1 = itob16(line.pt1.x) - halfoffset;
  traps[1].top.x2 = traps[1].top.x1 + adjwidth - 1;
  traps[1].top.y  = line.pt1.y;
 
  traps[1].bot.x1 = itob16(line.pt2.x) -  halfoffset;
  traps[1].bot.x2 = traps[1].bot.x1 + adjwidth - 1;
  traps[1].bot.y  = line.pt2.y;
  return 1;
}
