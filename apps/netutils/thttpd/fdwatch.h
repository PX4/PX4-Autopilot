/****************************************************************************
 * netutils/thttpd/fdwatch.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in THTTPD:
 *
 *   Copyright © 1999 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NETUTILS_THTTPD_FDWATCH_H
#define __NETUTILS_THTTPD_FDWATCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Define to enable detailed FDWATCH debug output */

#undef CONFIG_THTTPD_FDWATCH_DEBUG

#ifndef INFTIM
#  define INFTIM -1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fdwatch_s
{
  struct pollfd *pollfds;          /* Poll data (allocated) */
  void         **client;           /* Client data (allocated) */
  uint8_t       *ready;            /* The list of fds with activity (allocated) */
  uint8_t        nfds;             /* The configured maximum number of fds */
  uint8_t        nwatched;         /* The number of fds currently watched */
  uint8_t        nactive;          /* The number of fds with activity */
  uint8_t        next;             /* The index to the next client data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize the fdwatch data structures.  Returns NULL on failure. */

extern struct fdwatch_s *fdwatch_initialize(int nfds);

/* Uninitialize the fwdatch data structure */

extern void fdwatch_uninitialize(struct fdwatch_s *fw);

/* Add a descriptor to the watch list */

extern void fdwatch_add_fd(struct fdwatch_s *fw, int fd, void *client_data);

/* Delete a descriptor from the watch list. */

extern void fdwatch_del_fd(struct fdwatch_s *fw, int fd);

/* Do the watch.  Return value is the number of descriptors that are ready,
 * or 0 if the timeout expired, or -1 on errors.  A timeout of INFTIM means
 * wait indefinitely.
 */

extern int fdwatch(struct fdwatch_s *fw, long timeout_msecs);

/* Check if a descriptor was ready. */

extern int fdwatch_check_fd(struct fdwatch_s *fw, int fd);

/* Get the client data for the next returned event.  Returns -1 when there
 * are no more events.
 */

extern void *fdwatch_get_next_client_data(struct fdwatch_s *fw);

#endif /* __NETUTILS_THTTPD_FDWATCH_H */

