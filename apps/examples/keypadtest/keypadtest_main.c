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

#ifdef CONFIG_EXAMPLES_KEYPADTEST_ENCODED
#  include <nuttx/streams.h>
#  include <nuttx/input/kbd_codec.h>
#endif

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

#ifdef CONFIG_EXAMPLES_KEYPADTEST_ENCODED
struct keypad_instream_s
{
  struct lib_instream_s stream;
  FAR char *buffer;
  ssize_t nbytes;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: keypad_getstream
 *
 * Description:
 *   Get one character from the keyboard.
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_KEYPADTEST_ENCODED
static int keypad_getstream(FAR struct lib_instream_s *this)
{
  FAR struct keypad_instream_s *kbdstream = (FAR struct keypad_instream_s *)this;

  DEBUGASSERT(kbdstream && kbdstream->buffer);
  if (kbdstream->nbytes > 0)
    {
      kbdstream->nbytes--;
      kbdstream->stream.nget++;
      return (int)*kbdstream->buffer++;
    }

  return EOF;
}
#endif

/****************************************************************************
 * Name: keypad_decode
 *
 * Description:
 *   Decode encoded keyboard input
 *
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_KEYPADTEST_ENCODED
static void keypad_decode(FAR char *buffer, ssize_t nbytes)
{
  struct keypad_instream_s kbdstream;
  struct kbd_getstate_s state;
  uint8_t ch;
  int ret;

  /* Initialize */

  memset(&state, 0, sizeof(struct kbd_getstate_s));
  kbdstream.stream.get  = keypad_getstream;
  kbdstream.stream.nget = 0;
  kbdstream.buffer      = buffer;
  kbdstream.nbytes      = nbytes;

  /* Loop until all of the bytes have been consumed.  We implicitly assume
   * that the the escaped sequences do not cross buffer boundaries.  That
   * might be true if the read buffer were small or the data rates high.
   */

  for (;;)
    {
      /* Decode the next thing from the buffer */

      ret = kbd_decode((FAR struct lib_instream_s *)&kbdstream, &state, &ch);
      if (ret == KBD_ERROR)
        {
          break;
        }

      /* Normal data?  Or special key? */

      switch (ret)
        {
        case KBD_PRESS: /* Key press event */
          printf("Normal Press:    %c [%02x]\n", isprint(ch) ? ch : '.', ch);
          break;

        case KBD_RELEASE: /* Key release event */
          printf("Normal Release:  %c [%02x]\n", isprint(ch) ? ch : '.', ch);
          break;

        case KBD_SPECPRESS: /* Special key press event */
          printf("Special Press:   %d\n", ch);
          break;

        case KBD_SPECREL: /* Special key release event */
          printf("Special Release: %d\n", ch);
          break;

        case KBD_ERROR: /* Error or end-of-file */
          printf("EOF:             %d\n", ret);
          break;

        default:
          printf("Unexpected:      %d\n", ret);
          break;
        }
    }
}
#endif

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
#ifdef CONFIG_EXAMPLES_KEYPADTEST_ENCODED
          keypad_decode(buffer, nbytes);
#else
          (void)write(1, buffer, nbytes);
#endif
        }
    }
  while (nbytes >= 0);

  printf("keypadtest_main: Closing device %s: %d\n", CONFIG_EXAMPLES_KEYPAD_DEVNAME, (int)nbytes);
  fflush(stdout);
  close(fd);
  return 0;
}
