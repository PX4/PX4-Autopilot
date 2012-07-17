/****************************************************************************
 * include/poll.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_POLL_H
#define __INCLUDE_POLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Poll event definitions:
 *
 *   POLLIN
 *     Data other than high-priority data may be read without blocking.
 *   POLLRDNORM
 *     Normal data may be read without blocking.
 *   POLLRDBAND
 *     Priority data may be read without blocking.
 *   POLLPRI
 *     High priority data may be read without blocking.
 *   POLLOUT
 *     Normal data may be written without blocking.
 *   POLLWRNORM
 *     Equivalent to POLLOUT.
 *   POLLWRBAND
 *     Priority data may be written.
 *   POLLERR
 *     An error has occurred (revents only).
 *   POLLHUP
 *     Device has been disconnected (revents only).
 *   POLLNVAL
 *     Invalid fd member (revents only).
 */

#define POLLIN       (0x01)  /* NuttX does not make priority distinctions */
#define POLLRDNORM   (0x01)
#define POLLRDBAND   (0x01)
#define POLLPRI      (0x01)

#define POLLOUT      (0x02)  /* NuttX does not make priority distinctions */
#define POLLWRNORM   (0x02)
#define POLLWRBAND   (0x02)

#define POLLERR      (0x04)
#define POLLHUP      (0x08)
#define POLLNVAL     (0x10)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The number of poll descriptors (required by poll() specification */

typedef unsigned int nfds_t;

/* In the standard poll() definition, the size of the event set is 'short'.
 * Here we pick the smallest storage element that will contain all of the
 * poll events.
 */

typedef uint8_t pollevent_t;

/* This is the Nuttx variant of the standard pollfd structure. */

struct pollfd
{
  int         fd;       /* The descriptor being polled */
  sem_t      *sem;      /* Pointer to semaphore used to post output event */
  pollevent_t events;   /* The input event flags */
  pollevent_t revents;  /* The output event flags */
  FAR void   *priv;     /* For use by drivers */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN int poll(FAR struct pollfd *fds, nfds_t nfds, int timeout);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_POLL_H */
