/****************************************************************************
 * include/nuttx/streams.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef _INCLUDE_NUTTX_STREAMS_H
#define _INCLUDE_NUTTX_STREAMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These are the generic representations of a streams used by the NuttX */

struct lib_instream_s;
struct lib_outstream_s;

typedef int  (*lib_getc_t)(FAR struct lib_instream_s *this);
typedef void (*lib_putc_t)(FAR struct lib_outstream_s *this, int ch);
typedef int  (*lib_flush_t)(FAR struct lib_outstream_s *this);

struct lib_instream_s
{
  lib_getc_t  get;                /* Pointer to function to get one character */
  int         nget;               /* Total number of characters gotten.  Written
                                   * by get method, readable by user */
};

struct lib_outstream_s
{
  lib_putc_t  put;                /* Pointer to function to put one character */
#ifdef CONFIG_STDIO_LINEBUFFER
  lib_flush_t flush;              /* Pointer to function flush buffered characters */
#endif
  int         nput;               /* Total number of characters put.  Written
                                   * by put method, readable by user */
};

/* These are streams that operate on a fixed-sized block of memory */

struct lib_meminstream_s
{
  struct lib_instream_s  public;
  FAR const char        *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

struct lib_memoutstream_s
{
  struct lib_outstream_s public;
  FAR char              *buffer;  /* Address of first byte in the buffer */
  int                    buflen;  /* Size of the buffer in bytes */
};

/* These are streams that operate on a FILE */

struct lib_stdinstream_s
{
  struct lib_instream_s  public;
  FAR FILE              *stream;
};

struct lib_stdoutstream_s
{
  struct lib_outstream_s public;
  FAR FILE              *stream;
};

/* These are streams that operate on a file descriptor */

struct lib_rawoutstream_s
{
  struct lib_outstream_s public;
  int                    fd;
};

struct lib_rawinstream_s
{
  struct lib_instream_s  public;
  int                    fd;
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lib_meminstream, lib_memoutstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *   Defined in lib/lib_meminstream.c and lib/lib_memoutstream.c
 *
 * Input parameters:
 *   meminstream  - User allocated, uninitialized instance of struct
 *                  lib_meminstream_s to be initialized.
 *   memoutstream - User allocated, uninitialized instance of struct
 *                  lib_memoutstream_s to be initialized.
 *   bufstart     - Address of the beginning of the fixed-size memory buffer
 *   buflen       - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

EXTERN void lib_meminstream(FAR struct lib_meminstream_s *meminstream,
                            FAR const char *bufstart, int buflen);
EXTERN void lib_memoutstream(FAR struct lib_memoutstream_s *memoutstream,
                             FAR char *bufstart, int buflen);

/****************************************************************************
 * Name: lib_stdinstream, lib_stdoutstream
 *
 * Description:
 *   Initializes a stream for use with a FILE instance.
 *   Defined in lib/lib_stdinstream.c and lib/lib_stdoutstream.c
 *
 * Input parameters:
 *   stdinstream  - User allocated, uninitialized instance of struct
 *                  lib_stdinstream_s to be initialized.
 *   stdoutstream - User allocated, uninitialized instance of struct
 *                  lib_stdoutstream_s to be initialized.
 *   stream       - User provided stream instance (must have been opened for
 *                  the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

EXTERN void lib_stdinstream(FAR struct lib_stdinstream_s *stdinstream,
                            FAR FILE *stream);
EXTERN void lib_stdoutstream(FAR struct lib_stdoutstream_s *stdoutstream,
                             FAR FILE *stream);

/****************************************************************************
 * Name: lib_rawinstream, lib_rawoutstream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *   Defined in lib/lib_rawinstream.c and lib/lib_rawoutstream.c
 *
 * Input parameters:
 *   rawinstream  - User allocated, uninitialized instance of struct
 *                  lib_rawinstream_s to be initialized.
 *   rawoutstream - User allocated, uninitialized instance of struct
 *                  lib_rawoutstream_s to be initialized.
 *   fd           - User provided file/socket descriptor (must have been opened
 *                  for the correct access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

EXTERN void lib_rawinstream(FAR struct lib_rawinstream_s *rawinstream,
                            int fd);
EXTERN void lib_rawoutstream(FAR struct lib_rawoutstream_s *rawoutstream,
                             int fd);

/****************************************************************************
 * Name: lib_lowinstream, lib_lowoutstream
 *
 * Description:
 *   Initializes a stream for use with low-level, architecture-specific I/O.
 *   Defined in lib/lib_lowinstream.c and lib/lib_lowoutstream.c
 *
 * Input parameters:
 *   lowinstream  - User allocated, uninitialized instance of struct
 *                  lib_lowinstream_s to be initialized.
 *   lowoutstream - User allocated, uninitialized instance of struct
 *                  lib_lowoutstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LOWGETC
EXTERN void lib_lowinstream(FAR struct lib_instream_s *lowinstream);
#endif
#ifdef CONFIG_ARCH_LOWPUTC
EXTERN void lib_lowoutstream(FAR struct lib_outstream_s *lowoutstream);
#endif

/****************************************************************************
 * Name: lib_zeroinstream, lib_nullinstream, lib_nulloutstream
 *
 * Description:
 *   Initializes NULL streams:
 *
 *   o The stream created by lib_zeroinstream will return an infinitely long
 *     stream of zeroes. Defined in lib/lib_zeroinstream.c
 *   o The stream created by lib_nullinstream will return only EOF.
 *     Defined in lib/lib_nullinstream.c
 *   o The stream created by lib_nulloutstream will write all data to the
 *     bit-bucket. Defined in lib/lib_nulloutstream.c
 *
 * Input parameters:
 *   zeroinstream  - User allocated, uninitialized instance of struct
 *                   lib_instream_s to be initialized.
 *   nullinstream  - User allocated, uninitialized instance of struct
 *                   lib_instream_s to be initialized.
 *   nulloutstream - User allocated, uninitialized instance of struct
 *                   lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

EXTERN void lib_zeroinstream(FAR struct lib_instream_s *zeroinstream);
EXTERN void lib_nullinstream(FAR struct lib_instream_s *nullinstream);
EXTERN void lib_nulloutstream(FAR struct lib_outstream_s *nulloutstream);

/****************************************************************************
 * Name: lib_sylogstream
 *
 * Description:
 *   Initializes a stream for use with the configured syslog interface.
 *
 * Input parameters:
 *   lowoutstream - User allocated, uninitialized instance of struct
 *                  lib_lowoutstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG
EXTERN void lib_syslogstream(FAR struct lib_outstream_s *stream);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* _INCLUDE_NUTTX_STREAMS_H */
