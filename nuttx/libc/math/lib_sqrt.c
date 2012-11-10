/************************************************************************
 * libc/math/lib_sqrt.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 * It derives from the Rhombs OS math library by Nick Johnson which has
 * a compatibile, MIT-style license:
 *
 * Copyright (C) 2009-2011 Nick Johnson <nickbjohnson4224 at gmail.com>
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
#include <errno.h>

#include "lib_internal.h"

/************************************************************************
 * Public Functions
 ************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double sqrt(double x)
{
  long double y, y1;

  if (x < 0.0)
    {
      errno = EDOM;
      return NAN;
    }

  if (isnan(x))
    {
      return NAN;
    }

  if (isinf(x))
    {
      return INFINITY;
    }

  if (x == 0.0)
    {
      return 0.0;
    }

  /* Guess square root (using bit manipulation) */

  y = lib_sqrtapprox(x);

  /* Perform four iterations of approximation.  This number (4) is
   * definitely optimal
   */

  y = 0.5 * (y + x / y);
  y = 0.5 * (y + x / y);
  y = 0.5 * (y + x / y);
  y = 0.5 * (y + x / y);

  /* If guess was terribe (out of range of float).  Repeat approximation
   * until convergence.
   */

  if (y * y < x - 1.0 || y * y > x + 1.0)
    {
      y1 = -1.0;
      while (y != y1)
        {
          y1 = y;
          y = 0.5 * (y + x / y);
        }
    }

  return y;
}
#endif
