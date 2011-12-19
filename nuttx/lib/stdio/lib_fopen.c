/****************************************************************************
 * lib/stdio/lib_fopen.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_mode2oflags
 ****************************************************************************/

static int lib_mode2oflags(FAR const char *mode)
{
  int oflags = 0;
  if (mode)
    {
      while(*mode)
        {
          switch (*mode)
            {
             /* Open for read access */

             case 'r' :
               if (*(mode + 1) == '+')
                 {
                   /* Open for read/write access */

                   oflags |= O_RDWR;
                   mode++;
                 }
               else
                 {
                   /* Open for read access */

                   oflags |= O_RDOK;
                 }
               break;

             /* Open for write access? */

             case 'w' :
               if (*(mode + 1) == '+')
                 {
                   /* Open for write read/access, truncating any existing file */

                   oflags |= O_RDWR|O_CREAT|O_TRUNC;
                   mode++;
                 }
               else
                 {
                   /* Open for write access, truncating any existing file */

                   oflags |= O_WROK|O_CREAT|O_TRUNC;
                 }
               break;

             /* Open for write/append access? */

             case 'a' :
               if (*(mode + 1) == '+')
                 {
                   /* Read from the beginning of the file; write to the end */

                   oflags |= O_RDWR|O_CREAT|O_APPEND;
                   mode++;
                 }
               else
                 {
                   /* Write to the end of the file */

                   oflags |= O_WROK|O_CREAT|O_APPEND;
                 }
               break;

             /* Open for binary access? */

             case 'b' :
             default:
               break;
            }
          mode++;
        }
    }
  return oflags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdopen
 ****************************************************************************/

FAR FILE *fdopen(int fd, FAR const char *mode)
{
  return fs_fdopen(fd, lib_mode2oflags(mode), NULL);
}

/****************************************************************************
 * Name: fopen
 ****************************************************************************/

FAR FILE *fopen(FAR const char *path, FAR const char *mode)
{
  FAR FILE *ret = NULL;
  int oflags;
  int fd;

  /* Open the file */

  oflags = lib_mode2oflags(mode);
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
