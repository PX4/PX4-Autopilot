/************************************************************************
 * libc/math/lib_sinf.c
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

#include <sys/types.h>
#include <math.h>

/************************************************************************
 * Private Data
 ************************************************************************/

static float _flt_inv_fact[] =
{
  1.0 / 1.0,                    // 1 / 1!
  1.0 / 6.0,                    // 1 / 3!
  1.0 / 120.0,                  // 1 / 5!
  1.0 / 5040.0,                 // 1 / 7!
  1.0 / 362880.0,               // 1 / 9!
  1.0 / 39916800.0,             // 1 / 11!
};

/************************************************************************
 * Public Functions
 ************************************************************************/

float sinf(float x)
{
  float x_squared;
  float sin_x;
  size_t i;

  /* Move x to [-pi, pi) */

  x = fmodf(x, 2 * M_PI);
  if (x >= M_PI)
    {
      x -= 2 * M_PI;
    }

  if (x < -M_PI)
    {
      x += 2 * M_PI;
    }

  /* Move x to [-pi/2, pi/2) */

  if (x >= M_PI_2)
    {
      x = M_PI - x;
    }

  if (x < -M_PI_2)
    {
      x = -M_PI - x;
    }

  x_squared = x * x;
  sin_x = 0.0;

  /* Perform Taylor series approximation for sin(x) with six terms */

  for (i = 0; i < 6; i++)
    {
      if (i % 2 == 0)
        {
          sin_x += x * _flt_inv_fact[i];
        }
      else
        {
          sin_x -= x * _flt_inv_fact[i];
        }

      x *= x_squared;
    }

  return sin_x;
}
