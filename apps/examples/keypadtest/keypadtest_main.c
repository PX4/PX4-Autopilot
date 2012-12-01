/****************************************************************************
 * examples/keypadtest/keypadtest_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/input/keypad.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

/* Provide some default values for other configuration settings */

#ifndef CONFIG_EXAMPLES_KEYPAD_DEVNAME
#  define CONFIG_EXAMPLES_KEYPAD_DEVNAME "/dev/keypad"
#endif

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
 * Name: keypadtest_main
 ****************************************************************************/

int keypadtest_main(int argc, char *argv[])
{
  char buffer[256];
  ssize_t nbytes;
  int fd;
  int ret;

  /* First, register the keyboard device(s) */

  printf("keypadtest_main: Register keyboard device\n");
  ret = keypad_kbdinit();
  if (ret != OK)
    {
      printf("keypadtest_main: Failed to register the KBD class\n");
      fflush(stdout);
      return 1;
    }

  /* Open the configured keyboard device. */

  printf("keypadtest_main: Opening device %s\n", CONFIG_EXAMPLES_KEYPAD_DEVNAME);
  fd = open(CONFIG_EXAMPLES_KEYPAD_DEVNAME, O_RDONLY);
  if (fd < 0)
    {
      printf("keypadtest_main: open() failed: %d\n", errno);
      fflush(stdout);
      return 1;
    }

  printf("keypadtest_main: Device %s opened\n", CONFIG_EXAMPLES_KEYPAD_DEVNAME);
  fflush(stdout);

  /* Loop until there is a read failure */

  do
    {
      /* Read a buffer of data */

      nbytes = read(fd, buffer, 256);
      if (nbytes > 0)
        {
          /* On success, echo the buffer to stdout */

          (void)write(1, buffer, nbytes);
        }
    }
  while (nbytes >= 0);

  printf("keypadtest_main: Closing device %s: %d\n", CONFIG_EXAMPLES_KEYPAD_DEVNAME, (int)nbytes);
  fflush(stdout);
  close(fd);
  return 0;
}
