/****************************************************************************
 * include/sys/select.h
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_SELECT_H
#define __INCLUDE_SYS_SELECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Get the total number of descriptors that we will have to support */

#define __SELECT_NDESCRIPTORS (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS)

/* We will use a 32-bit bitsets to represent the set of descriptors.  How
 * many uint32_t's do we need to span all descriptors?
 */

#if __SELECT_NDESCRIPTORS <= 32
#  define __SELECT_NUINT32 1
#elif __SELECT_NDESCRIPTORS <= 64
#  define __SELECT_NUINT32 2
#elif __SELECT_NDESCRIPTORS <= 96
#  define __SELECT_NUINT32 3
#elif __SELECT_NDESCRIPTORS <= 128
#  define __SELECT_NUINT32 4
#elif __SELECT_NDESCRIPTORS <= 160
#  define __SELECT_NUINT32 5
#elif __SELECT_NDESCRIPTORS <= 192
#  define __SELECT_NUINT32 6
#elif __SELECT_NDESCRIPTORS <= 224
#  define __SELECT_NUINT32 7
#elif __SELECT_NDESCRIPTORS <= 256
#  define __SELECT_NUINT32 8
#else
#  warning "Large fd_set needed"
#endif

/* These macros map a file descripto to an index and bit number */

#define _FD_NDX(fd) ((fd) >> 5)
#define _FD_BIT(fd) ((fd) & 0x1f)

/* Standard helper macros */

#define FD_CLR(fd,set)    (((uint32_t*)(set))[_FD_NDX(fd)] &= ~(1 << _FD_BIT(fd)))
#define FD_SET(fd,set)    (((uint32_t*)(set))[_FD_NDX(fd)] |= (1 << _FD_BIT(fd)))
#define FD_ISSET(fd,set)  ((((uint32_t*)(set))[_FD_NDX(fd)] & (1 << _FD_BIT(fd))) != 0)
#define FD_ZERO(set)      memset(set, 0, sizeof(fd_set))

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef uint32_t fd_set[__SELECT_NUINT32];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN int select(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
                  FAR fd_set *exceptfds, FAR struct timeval *timeout);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NFILE_DESCRIPTORS || CONFIG_NSOCKET_DESCRIPTORS */
#endif /* __INCLUDE_SYS_SELECT_H */
