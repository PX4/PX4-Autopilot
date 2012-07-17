/****************************************************************************
 * include/sys/mman.h
 *
 *   Copyright (C) 2008, 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_MMAN_H
#define __INCLUDE_SYS_MMAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Protections are chosen from these bits, OR'd together.  NuttX does not
 * support any of these, but are provided for source level compatibility
 */

#define PROT_NONE       0x0             /* Page may not be accessed */
#define PROT_READ       0x1             /* Page may be read */
#define PROT_WRITE      0x2             /* Page may be written */
#define PROT_EXEC       0x4             /* Page may be executed */

/* Sharing types -- ignored by NuttX. */

#define MAP_SHARED      0x00001         /* Share this mapping */
#define MAP_PRIVATE     0x00002         /* Create a private copy-on-write mapping */
#define MAP_TYPE        0x0000f         /* Mask for type of mapping */
#define MAP_FIXED       0x00010         /* Map to specified address exactly */
#define MAP_FILE        0x00000         /* The mapping is backed by a file */
#define MAP_ANONYMOUS   0x00020         /* The mapping is not backed by any file */
#define MAP_ANON        MAP_ANONYMOUS

/* These are Linux-specific.  */

#define MAP_GROWSDOWN   0x00100         /* Used to stack allocations */
#define MAP_DENYWRITE   0x00800         /* Do not permit writes to file */
#define MAP_EXECUTABLE  0x01000         /* Mark it as an executable */
#define MAP_LOCKED      0x02000         /* Lock pages mapped into memory */
#define MAP_NORESERVE   0x04000         /* Do not reserve swap space for this mapping */
#define MAP_POPULATE    0x08000         /* Populate (prefault) pagetables */
#define MAP_NONBLOCK    0x10000         /* Do not block on IO */

/* Failure return */

#define MAP_FAILED      ((void*)-1)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

EXTERN FAR void *mmap(FAR void *start, size_t length, int prot, int flags,
                      int fd, off_t offset);

#ifdef CONFIG_FS_RAMMAP
EXTERN int munmap(FAR void *start, size_t length);
#else
#  define munmap(start, length)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_MMAN_H */
