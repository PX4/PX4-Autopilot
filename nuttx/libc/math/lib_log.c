/************************************************************************
 * libc/math/lib_log.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 * It derives from the Rhombs OS math library by Nick Johnson which has
 * a compatibile, MIT-style license:
 *
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
 * 
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>
#include <float.h>

/************************************************************************
 * Public Functions
 ************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double log(double x)
{
  double y, y_old, ey, epsilon;

  y = 0.0;
  y_old = 1.0;
  epsilon = DBL_EPSILON;

  while (y > y_old + epsilon || y < y_old - epsilon)
    {
      y_old = y;
      ey = exp(y);
      y -= (ey - x) / ey;

      if (y > 700.0)
        {
          y = 700.0;
        }

      if (y < -700.0)
        {
          y = -700.0;
        }

      epsilon = (fabs(y) > 1.0) ? fabs(y) * DBL_EPSILON : DBL_EPSILON;
    }

  if (y == 700.0)
    {
      return INFINITY;
    }

  if (y == -700.0)
    {
      return INFINITY;
    }

  return y;
}
#endif
