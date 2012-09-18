/****************************************************************************
 * netutils/webserver/httpd_mmap.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 * Included Header Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sendfile.h>

#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <apps/netutils/httpd.h>

#include "httpd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int httpd_sendfile_open(const char *name, struct httpd_fs_file *file)
{
  char path[PATH_MAX];
  struct stat st;

  if (sizeof path < snprintf(path, sizeof path, "%s%s",
      CONFIG_NETUTILS_HTTPD_PATH, name))
    {
      errno = ENAMETOOLONG;
      return ERROR;
    }

  /* XXX: awaiting fstat to avoid a race */

  if (-1 == stat(path, &st))
    {
       return ERROR;
    }

  if (S_ISDIR(st.st_mode))
    {
       errno = EISDIR;
       return ERROR;
    }

  if (!S_ISREG(st.st_mode))
    {
       errno = ENOENT;
       return ERROR;
    }

  if (st.st_size > INT_MAX || st.st_size > SIZE_MAX)
    {
       errno = EFBIG;
       return ERROR;
    }

  file->len = (int) st.st_size;

  file->fd = open(path, O_RDONLY);
  if (file->fd == -1)
    {
       return ERROR;
    }

  return OK;
}

int httpd_sendfile_close(struct httpd_fs_file *file)
{
  if (-1 == close(file->fd))
    {
      return ERROR;
    }

  return OK;
}

int httpd_sendfile_send(int outfd, struct httpd_fs_file *file)
{
  if (-1 == sendfile(outfd, file->fd, 0, file->len))
    {
      return ERROR;
    }

  return OK;
}

