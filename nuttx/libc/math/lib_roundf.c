/************************************************************************
 * lib/math/lib_roundf.c
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

float roundf(float x)
{
  float f = modff(x, &x);
  if (x <= 0.0f && f <= -0.5f)
    {
      x -= 1.0f;
    }
  
  if (x >= 0.0f && f >= 0.5f)
    {
      x += 1.0f;
    }

  return x;
}
