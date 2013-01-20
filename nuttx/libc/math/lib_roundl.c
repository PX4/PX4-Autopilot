/************************************************************************
 * lib/math/lib_round.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *             (C) 2012 Petteri Aimonen <jpa@nx.mail.kapsi.fi>
 *
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/************************************************************************
 * Public Functions
 ************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
long double roundl(long double x)
{
  long double f = modfl(x, &x);
  if (x <= 0.0 && f <= -0.5)
    {
      x -= 1.0;
    }
  
  if (x >= 0.0 && f >= 0.5)
    {
      x += 1.0;
    }

  return x;
}
#endif
