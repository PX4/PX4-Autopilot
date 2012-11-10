/****************************************************************************
 * libc/string/lib_isbasedigit.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "lib_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_isbasedigit
 *
 * Description:
 *   Given an ASCII character, ch, and a base (1-36) do two
 *   things:  1) Determine if ch is a valid charcter, and 2)
 *   convert ch to its binary value.
 *
 ****************************************************************************/

bool lib_isbasedigit(int ch, int base, int *value)
{
  bool ret = false;
  int  tmp = 0;

  if (base <= 10)
    {
      if (ch >= '0' && ch <= base + '0' - 1)
        {
          tmp = ch - '0';
          ret = true;
        }
    }
  else if (base <= 36)
    {
      if (ch >= '0' && ch <= '9')
        {
          tmp = ch - '0';
          ret = true;
        }
      else if (ch >= 'a' && ch <= 'a' + base - 11)
        {
          tmp = ch - 'a' + 10;
          ret = true;
        }
      else if (ch >= 'A' && ch <= 'A' + base - 11)
        {
          tmp = ch - 'A' + 10;
          ret = true;
        }
    }

  if (value)
    {
      *value = tmp;
    }
  return ret;
}


