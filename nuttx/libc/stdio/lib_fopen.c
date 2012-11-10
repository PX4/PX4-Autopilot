/****************************************************************************
 * libc/stdio/lib_fopen.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum open_mode_e
{
  MODE_NONE = 0, /* No access mode determined */
  MODE_R,        /* "r" or "rb" open for reading */
  MODE_W,        /* "w" or "wb" open for writing, truncating or creating file */
  MODE_A,        /* "a" or "ab" open for writing, appending to file */
  MODE_RPLUS,    /* "r+", "rb+", or "r+b" open for update (reading and writing) */
  MODE_WPLUS,    /* "w+", "wb+", or "w+b"  open for update, truncating or creating file */
  MODE_APLUS     /* "a+", "ab+", or "a+b" open for update, appending to file */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_mode2oflags
 ****************************************************************************/

static int lib_mode2oflags(FAR const char *mode)
{
  enum open_mode_e state;
  int oflags;

  /* Verify that a mode string was provided.  No error is  */

  if (!mode)
    {
      goto errout;
    }

  /* Parse the mode string to determine the corresponding open flags */

  state  = MODE_NONE;
  oflags = 0;

  for (; *mode; mode++)
    {
      switch (*mode)
        {
          /* Open for read access ("r", "r[+]", "r[b]",  "r[b+]", or "r[+b]") */

          case 'r' :
            if (state == MODE_NONE)
              {
                /* Open for read access */

                oflags = O_RDOK;
                state  = MODE_R;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for write access ("w", "w[+]", "w[b]",  "w[b+]", or "w[+b]") */

          case 'w' :
            if (state == MODE_NONE)
              {
                /* Open for write access, truncating any existing file */

                oflags = O_WROK|O_CREAT|O_TRUNC;
                state  = MODE_W;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for write/append access ("a", "a[+]", "a[b]", "a[b+]", or "a[+b]") */

          case 'a' :
            if (state == MODE_NONE)
              {
                /* Write to the end of the file */

                oflags = O_WROK|O_CREAT|O_APPEND;
                state  = MODE_A;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for update access ("[r]+", "[rb]+]", "[r]+[b]", "[w]+",
           * "[wb]+]", "[w]+[b]", "[a]+", "[ab]+]",  "[a]+[b]")
           */

          case '+' :
            switch (state)
              {
                case MODE_R:
                  {
                    /* Retain any binary mode selection */

                    oflags &= O_BINARY;

                    /* Open for read/write access */

                    oflags |= O_RDWR;
                    state   = MODE_RPLUS;
                 }
                 break;

                case MODE_W:
                  {
                    /* Retain any binary mode selection */

                    oflags &= O_BINARY;

                    /* Open for write read/access, truncating any existing file */

                    oflags |= O_RDWR|O_CREAT|O_TRUNC;
                    state   = MODE_WPLUS;
                  }
                  break;

                case MODE_A:
                  {
                    /* Retain any binary mode selection */

                    oflags &= O_BINARY;

                    /* Read from the beginning of the file; write to the end */

                    oflags |= O_RDWR|O_CREAT|O_APPEND;
                    state   = MODE_APLUS;
                  }
                  break;

                default:
                  goto errout;
                  break;
              }
            break;

          /* Open for binary access ("[r]b", "[r]b[+]", "[r+]b", "[w]b",
           * "[w]b[+]", "[w+]b", "[a]b", "[a]b[+]",  "[a+]b")
           */

          case 'b' :
            if (state != MODE_NONE)
              {
                /* The file is opened in binary mode */

                oflags |= O_BINARY;
              }
            else
              {
                goto errout;
              }
            break;

          /* Unrecognized or unsupported mode */

          default:
            goto errout;
            break;
        }
    }

  return oflags;

/* Both fopen and fdopen should fail with errno == EINVAL if the mode
 * string is invalid.
 */

errout:
  set_errno(EINVAL);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdopen
 ****************************************************************************/

FAR FILE *fdopen(int fd, FAR const char *mode)
{
  FAR FILE *ret = NULL;
  int oflags;

  /* Map the open mode string to open flags */

  oflags = lib_mode2oflags(mode);
  if (oflags >= 0)
    {
      ret = fs_fdopen(fd, oflags, NULL);
    }

  return ret;
}

/****************************************************************************
 * Name: fopen
 ****************************************************************************/

FAR FILE *fopen(FAR const char *path, FAR const char *mode)
{
  FAR FILE *ret = NULL;
  int oflags;
  int fd;

  /* Map the open mode string to open flags */

  oflags = lib_mode2oflags(mode);
  if (oflags < 0)
    {
      return NULL;
    }

  /* Open the file */

  fd = open(path, oflags, 0666);

  /* If the open was successful, then fdopen() the fil using the file
   * desciptor returned by open.  If open failed, then just return the
   * NULL stream -- open() has already set the errno.
   */

  if (fd >= 0)
    {
      ret = fs_fdopen(fd, oflags, NULL);
      if (!ret)
        {
          /* Don't forget to close the file descriptor if any other
           * failures are reported by fdopen().
           */

          (void)close(fd);
        }
    }
  return ret;
}
