/****************************************************************************
 * lib/stdio/lib_perror.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* POSIX requires that perror provide its output on stderr.  This option may
 * be defined, however, to provide perror output that is serialized with
 * other stdout messages.
 */
 
#ifdef CONFIG_LIBC_PERROR_STDOUT
#  define PERROR_STREAM stdout
#  undef CONFIG_LIBC_PERROR_DEVNAME
#endif

/* Another non-standard option is to provide perror output to a logging
 * device or file. CONFIG_LIBC_PERROR_DEVNAME may be defined to be any write-
 * able, character device (or file).
 */

#ifndef CONFIG_LIBC_PERROR_DEVNAME
#  define PERROR_STREAM stderr
#else
#  define PERROR_STREAM perror_stream;
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_PERROR_DEVNAME
static FILE *perror_stream;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: perror
 ****************************************************************************/

void perror(FAR const char *s)
{
  /* If we are using a custom output device (something other than
   * /dev/console), then make sure that the device has been opened.
   */

#ifdef CONFIG_LIBC_PERROR_DEVNAME
  if (!perror_stream)
    {
      /* Not yet.. open it now */

      perror_stream = fopen(CONFIG_LIBC_PERROR_DEVNAME, "w");
      if (!perror_stream)
        {
          /* Oops... we couldn't open the device */

          return;
        }
    }
#endif

  /* If strerror() is not enabled, then just print the error number */

#ifdef CONFIG_LIBC_STRERROR
  (void)fprintf(PERROR_STREAM, "%s: %s\n", s, strerror(errno));
#else
  (void)fprintf(PERROR_STREAM, "%s: Error %d\n", s, errno);
#endif
}
