/************************************************************************
 * libc/math/lib_libexpi.c
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

#include <sys/types.h>
#include <math.h>

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

#define M_E2    (M_E * M_E)
#define M_E4    (M_E2 * M_E2)
#define M_E8    (M_E4 * M_E4)
#define M_E16   (M_E8 * M_E8)
#define M_E32   (M_E16 * M_E16)
#define M_E64   (M_E32 * M_E32)
#define M_E128  (M_E64 * M_E64)
#define M_E256  (M_E128 * M_E128)
#define M_E512  (M_E256 * M_E256)
#define M_E1024 (M_E512 * M_E512)

/************************************************************************
 * Private Data
 ************************************************************************/

static double _expi_square_tbl[11] =
{
  M_E,                          // e^1
  M_E2,                         // e^2
  M_E4,                         // e^4
  M_E8,                         // e^8
  M_E16,                        // e^16
  M_E32,                        // e^32
  M_E64,                        // e^64
  M_E128,                       // e^128
  M_E256,                       // e^256
  M_E512,                       // e^512
  M_E1024,                      // e^1024
};

/************************************************************************
 * Public Functions
 ************************************************************************/

double lib_expi(size_t n)
{
  size_t i;
  double val;

  if (n > 1024)
    {
      return INFINITY;
    }

  val = 1.0;

  for (i = 0; n; i++)
    {
      if (n & (1 << i))
        {
          n   &= ~(1 << i);
          val *= _expi_square_tbl[i];
        }
    }

  return val;
}

