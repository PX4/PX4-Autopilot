/****************************************************************************
 * libc/string/lib_checkbase.c
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
 * Name: lib_checkbase
 *
 * Description:
 *   This is part of the strol() family implementation.  This function checks
 *   the initial part of a string to see if it can determine the numeric
 *   base that is represented.
 *
 * Assumptions:
 *   *ptr points to the first, non-whitespace character in the string.
 *
 ****************************************************************************/
 
int lib_checkbase(int base, const char **pptr)
{
   const char *ptr = *pptr;

  /* Check for unspecified base */

  if (!base)
    {
      /* Assume base 10 */

      base = 10;

      /* Check for leading '0' - that would signify octal or hex (or binary) */

      if (*ptr == '0')
        {
          /* Assume octal */

          base = 8;
          ptr++;

          /* Check for hexidecimal */

          if ((*ptr == 'X' || *ptr == 'x') && 
              lib_isbasedigit(ptr[1], 16, NULL))
            {
              base = 16;
              ptr++;
            }
        }
    }

  /* If it a hexidecimal representation, than discard any leading "0X" or "0x" */

  else if (base == 16)
    {
      if (ptr[0] == '0' && (ptr[1] == 'X' || ptr[1] == 'x'))
        {
          ptr += 2;
        }
    }

  /* Return the updated pointer and base */

  *pptr = ptr;
  return base;
}

