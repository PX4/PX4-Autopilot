/****************************************************************************
 * examples/ostest/dev_null.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <fcntl.h>
#include "ostest.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0

static FAR char buffer[1024];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dev_null(void)
{
  int nbytes;
  int fd;

  fd = open("/dev/null", O_RDWR);
  if (fd < 0)
    {
      printf("dev_null: ERROR Failed to open /dev/null\n");
      return -1;
    }

  nbytes = read(fd, buffer, 1024);
  if (nbytes < 0)
    {
      printf("dev_null: ERROR Failed to read from /dev/null\n");
      close(fd);
      return -1;
    }
  printf("dev_null: Read %d bytes from /dev/null\n", nbytes);

  nbytes = write(fd, buffer, 1024);
  if (nbytes < 0)
    {
      printf("dev_null: ERROR Failed to write to /dev/null\n");
      close(fd);
      return -1;
    }
  printf("dev_null: Wrote %d bytes to /dev/null\n", nbytes);

  close(fd);
  return 0;
}

#endif /*CONFIG_NFILE_DESCRIPTORS */
