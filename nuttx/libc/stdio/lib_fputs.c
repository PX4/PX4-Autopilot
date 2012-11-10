/****************************************************************************
 * libc/stdio/lib_fputs.c
 *
 *   Copyright (C) 2007, 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "lib_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fputs
 *
 * Description:
 *   fputs() writes the string s to stream, without its trailing '\0'.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ROMGETC)
int fputs(FAR const char *s, FAR FILE *stream)
{
  int nput;
  int ret;
  char ch;

  /* Make sure that a string was provided. */

#ifdef CONFIG_DEBUG /* Most parameter checking is disabled if DEBUG is off */
  if (!s)
    {
      set_errno(EINVAL);
      return EOF;
    }
#endif

  /* Write the string.  Loop until the null terminator is encountered */

  for (nput = 0, ch = up_romgetc(s); ch; nput++, s++, ch = up_romgetc(s))
    {
      /* Write the next character to the stream buffer */

      ret = lib_fwrite(&ch, 1, stream);
      if (ret <= 0)
        {
          return EOF;
        }

      /* Flush the buffer if a newline was written to the buffer */

#ifdef CONFIG_STDIO_LINEBUFFER
      if (ch == '\n')
        {
          ret = lib_fflush(stream, true);
          if (ret < 0)
            {
              return EOF;
            }
        }
#endif
    }

  return nput;
}

#elif defined(CONFIG_STDIO_LINEBUFFER)
int fputs(FAR const char *s, FAR FILE *stream)
{
  int nput;
  int ret;

  /* Make sure that a string was provided. */

#ifdef CONFIG_DEBUG /* Most parameter checking is disabled if DEBUG is off */
  if (!s)
    {
      set_errno(EINVAL);
      return EOF;
    }
#endif

  /* Write the string.  Loop until the null terminator is encountered */

  for (nput = 0; *s; nput++, s++)
    {
      /* Write the next character to the stream buffer */

      ret = lib_fwrite(s, 1, stream);
      if (ret <= 0)
        {
          return EOF;
        }

      /* Flush the buffer if a newline was written to the buffer */

      if (*s == '\n')
        {
          ret = lib_fflush(stream, true);
          if (ret < 0)
            {
              return EOF;
            }
        }
    }

  return nput;
}

#else
int fputs(FAR const char *s, FAR FILE *stream)
{
  int ntowrite;
  int nput;

  /* Make sure that a string was provided. */

#ifdef CONFIG_DEBUG /* Most parameter checking is disabled if DEBUG is off */
  if (!s)
    {
      set_errno(EINVAL);
      return EOF;
    }
#endif

  /* Get the length of the string. */

  ntowrite = strlen(s);
  if (ntowrite == 0)
    {
      return 0;
    }

  /* Write the string */

  nput = lib_fwrite(s, ntowrite, stream);
  if (nput < 0)
    {
      return EOF;
    }
  return nput;
}
#endif
