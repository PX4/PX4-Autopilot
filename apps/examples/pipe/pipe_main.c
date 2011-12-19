/****************************************************************************
 * examples/pipe/pipe_main.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <fcntl.h>
#include <errno.h>

#include "pipe.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  int filedes[2];
  int ret;

  /* Test FIFO logic */

  printf("\nuser_start: Performing FIFO test\n");
  ret = mkfifo(FIFO_PATH1, 0666);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: mkfifo failed with errno=%d\n", errno);
      return 1;
    }

  /* Open one end of the FIFO for reading and the other end for writing.  NOTE:
   * the following might not work on most FIFO implementations because the attempt
   * to open just one end of the FIFO for writing might block.  The NuttX FIFOs block
   * only on open for read-only (see interlock_test()).
   */

  filedes[1] = open(FIFO_PATH1, O_WRONLY);
  if (filedes[1] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for writing, errno=%d\n",
              FIFO_PATH1, errno);
      return 2;
    }

  filedes[0] = open(FIFO_PATH1, O_RDONLY);
  if (filedes[0] < 0)
    {
      fprintf(stderr, "user_start: Failed to open FIFO %s for reading, errno=%d\n",
              FIFO_PATH1, errno);
      if (close(filedes[1]) != 0)
        {
          fprintf(stderr, "user_start: close failed: %d\n", errno);
        }
      return 3;
    }

  /* Then perform the test using those file descriptors */

  ret = transfer_test(filedes[0], filedes[1]);
  if (close(filedes[0]) != 0)
    {
      fprintf(stderr, "user_start: close failed: %d\n", errno);
    }
  if (close(filedes[1]) != 0)
    {
      fprintf(stderr, "user_start: close failed: %d\n", errno);
    }
  /* unlink(FIFO_PATH1); fails */

  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO test FAILED (%d)\n", ret);
      return 4;
    }
  printf("user_start: FIFO test PASSED\n");

  /* Test PIPE logic */

  printf("\nuser_start: Performing pipe test\n");
  ret = pipe(filedes);
  if (ret < 0)
    {
      fprintf(stderr, "user_start: pipe failed with errno=%d\n", errno);
      return 5;
    }

  /* Then perform the test using those file descriptors */

  ret = transfer_test(filedes[0], filedes[1]);
  if (close(filedes[0]) != 0)
    {
      fprintf(stderr, "user_start: close failed: %d\n", errno);
    }
  if (close(filedes[1]) != 0)
    {
      fprintf(stderr, "user_start: close failed: %d\n", errno);
    }

  if (ret != 0)
    {
      fprintf(stderr, "user_start: PIPE test FAILED (%d)\n", ret);
      return 6;
    }
  printf("user_start: PIPE test PASSED\n");

  /* Perform the FIFO interlock test */

  printf("\nuser_start: Performing pipe interlock test\n");
  ret = interlock_test();
  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO interlock test FAILED (%d)\n", ret);
      return 7;
    }
  printf("user_start: PIPE interlock test PASSED\n");

  /* Perform the pipe redirection test */

  printf("\nuser_start: Performing redirection test\n");
  ret = redirection_test();
  if (ret != 0)
    {
      fprintf(stderr, "user_start: FIFO redirection test FAILED (%d)\n", ret);
      return 7;
    }
  printf("user_start: PIPE redirection test PASSED\n");

  fflush(stdout);
  return 0;
}
