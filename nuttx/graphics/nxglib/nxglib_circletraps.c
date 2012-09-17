/****************************************************************************
 * graphics/nxglib/nxglib_circletraps.c
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
#include <fixedmath.h>

#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Trigonometry */

#define SIN_0p0        0         /* sin(0) = 0 */
#define COS_0p0        1         /* cos(0) = 1 */
#define SIN_22p5       25080     /* sin(22.5) = 25080 / 65536 */
#define COS_22p5       60547     /* cos(22.5) = 60547 / 65536 */
#define SIN_45p0       46341     /* sin(45) = 46341 / 65536 */
#define COS_45p0       SIN_45p0  /* cos(45) = sin(45) */

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
 * Name: nxgl_circletraps
 *
 * Description:
 *   Given a description of a a circle, return 8 trapezoids that can be
 *   used to fill the circle by nx_fillcircle() and other interfaces.
 *
 * Input parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 8 trapezoids where
 *            the circle description will be returned.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void nxgl_circletraps(FAR const struct nxgl_point_s *center, nxgl_coord_t radius,
                      FAR struct nxgl_trapezoid_s *circle)
{
  nxgl_coord_t xoffs;
  nxgl_coord_t yoffs;

  circle[0].top.x1      = itob16(center->x);
  circle[0].top.x2      = circle[0].top.x1;
  circle[0].top.y       = center->y - radius;

  circle[7].bot.x1      = circle[0].top.x1;
  circle[7].bot.x2      = circle[0].top.x1;
  circle[7].bot.y       = center->y + radius;

  circle[3].bot.x1      = itob16(center->x - radius);
  circle[3].bot.x2      = itob16(center->x + radius);
  circle[3].bot.y       = center->y;

  circle[4].top.x1      = circle[3].bot.x1;
  circle[4].top.x2      = circle[3].bot.x2;
  circle[4].top.y       = circle[3].bot.y;

  /* Now 22.5, 67.5, 112.5, 157.5, 202.5, 247.5, 292.5, and 337.5 */

  xoffs = b16toi(b16muli(COS_22p5, radius) + b16HALF);
  yoffs = b16toi(b16muli(SIN_22p5, radius) + b16HALF);

  circle[2].bot.x1      = itob16(center->x - xoffs);
  circle[2].bot.x2      = itob16(center->x + xoffs);
  circle[2].bot.y       = center->y - yoffs;

  circle[3].top.x1      = circle[2].bot.x1;
  circle[3].top.x2      = circle[2].bot.x2;
  circle[3].top.y       = circle[2].bot.y;

  circle[4].bot.x1      = circle[2].bot.x1;
  circle[4].bot.x2      = circle[2].bot.x2;
  circle[4].bot.y       = center->y + yoffs;

  circle[5].top.x1      = circle[4].bot.x1;
  circle[5].top.x2      = circle[4].bot.x2;
  circle[5].top.y       = circle[4].bot.y;

  circle[0].bot.x1      = itob16(center->x - yoffs);
  circle[0].bot.x2      = itob16(center->x + yoffs);
  circle[0].bot.y       = center->y - xoffs;

  circle[1].top.x1      = circle[0].bot.x1;
  circle[1].top.x2      = circle[0].bot.x2;
  circle[1].top.y       = circle[0].bot.y;

  circle[6].bot.x1      = circle[1].top.x1;
  circle[6].bot.x2      = circle[1].top.x2;
  circle[6].bot.y       = center->y + xoffs;

  circle[7].top.x1      = circle[6].bot.x1;
  circle[7].top.x2      = circle[6].bot.x2;
  circle[7].top.y       = circle[6].bot.y;

  /* Finally, 45.0, 135.0, 225.0, 315.0 */

  xoffs = b16toi(b16muli(COS_45p0, radius) + b16HALF);

  circle[1].bot.x1      = itob16(center->x - xoffs);
  circle[1].bot.x2      = itob16(center->x + xoffs);
  circle[1].bot.y       = center->y - xoffs;

  circle[2].top.x1      = circle[1].bot.x1;
  circle[2].top.x2      = circle[1].bot.x2;
  circle[2].top.y       = circle[1].bot.y;
  
  circle[5].bot.x1      = circle[1].bot.x1;
  circle[5].bot.x2      = circle[1].bot.x2;
  circle[5].bot.y       = center->y + xoffs;

  circle[6].top.x1      = circle[5].bot.x1;
  circle[6].top.x2      = circle[5].bot.x2;
  circle[6].top.y       = circle[5].bot.y;
}
