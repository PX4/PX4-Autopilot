/****************************************************************************
 * include/nuttx/kmalloc.h
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_KMALLOC_H
#define __INCLUDE_NUTTX_KMALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#ifndef CONFIG_NUTTX_KERNEL
#  include <stdlib.h>
#  include <nuttx/mm.h>
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
# define KMALLOC_EXTERN extern "C"
extern "C" {
#else
# define KMALLOC_EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* For a monolithic, kernel-mode NuttX build.  Special allocators must be
 * used.  Otherwise, the standard allocators prototyped in stdlib.h may
 * be used for both the kernel- and user-mode objects.
 */

#ifndef CONFIG_NUTTX_KERNEL

# define kmm_initialize(h,s)    mm_initialize(h,s)
# define kmm_addregion(h,s)     mm_addregion(h,s)
# define kmm_trysemaphore()     mm_trysemaphore()
# define kmm_givesemaphore()    mm_givesemaphore()

# define kmalloc(s)             malloc(s)
# define kzalloc(s)             zalloc(s)
# define krealloc(p,s)          realloc(p,s)
# define kfree(p)               free(p)

#else

KMALLOC_EXTERN void kmm_initialize(FAR void *heap_start, size_t heap_size);
KMALLOC_EXTERN void kmm_addregion(FAR void *heapstart, size_t heapsize);
KMALLOC_EXTERN int kmm_trysemaphore(void);
KMALLOC_EXTERN void kmm_givesemaphore(void);

KMALLOC_EXTERN FAR void *kmalloc(size_t);
KMALLOC_EXTERN FAR void *kzalloc(size_t);
KMALLOC_EXTERN FAR void *krealloc(FAR void*, size_t);
KMALLOC_EXTERN void kfree(FAR void*);

#endif

/* Functions defined in os_list.c *******************************************/

/* Handles memory freed from an interrupt handler */

KMALLOC_EXTERN void sched_free(FAR void *address);

#undef KMALLOC_EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_KMALLOC_H */
