/************************************************************************
 * sched/kmm_kmalloc.c
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_NUTTX_KERNEL

/* This logic is all tentatively and, hopefully, will grow in usability.
 * For now, the kernel-mode build uses the memory manager that is
 * provided in the user-space build.  That is awkward but reasonable for
 * the current level of support:  At present, only memory protection is
 * provided.  Kernel-mode code may call into user-mode code, but not
 * vice-versa.  So hosting the memory manager in user-space allows the
 * memory manager to be shared in both kernel- and user-mode spaces.
 *
 * In the longer run, if an MMU is support that can provide virtualized
 * memory, then some SLAB memory manager will be required in kernel-space
 * with some kind of brk() system call to obtain mapped heap space.
 *
 * In the current build model, the user-space module is built first. The
 * file user_map.h is generated in the first pass and contains the
 * addresses of the memory manager needed in this file:
 */

#include <arch/board/user_map.h>

/************************************************************************
 * Pre-processor definition
 ************************************************************************/

/* This value is obtained from user_map.h */

#define KMALLOC(s) ((kmalloc_t)CONFIG_USER_MALLOC)(s)

/************************************************************************
 * Private Types
 ************************************************************************/

typedef FAR void *(*kmalloc_t)(size_t);

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: kmalloc
 *
 * Description:
 *   This is a simple redirection to the user-space malloc() function.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Return Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 * Assumptions:
 *   1. malloc() resides in user-space
 *   2. The address of the user space malloc() is provided in user_map.h
 *   3. The user-space malloc() is callable from kernel-space.
 *
 ************************************************************************/

FAR void *kmalloc(size_t size)
{
  return KMALLOC(size);
}

#endif /* CONFIG_NUTTX_KERNEL */
