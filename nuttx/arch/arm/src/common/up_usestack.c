/****************************************************************************
 * arch/arm/src/common/up_usestack.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* ARM requires at least a 4-byte stack alignment.  For use with EABI and
 * floating point, the stack must be aligned to 8-byte addresses.
 */

#ifndef CONFIG_STACK_ALIGNMENT

/* The symbol  __ARM_EABI__ is defined by GCC if EABI is being used.  If you
 * are not using GCC, make sure that CONFIG_STACK_ALIGNMENT is set correctly!
 */

#  ifdef __ARM_EABI__
#    define CONFIG_STACK_ALIGNMENT 8
#  else
#    define CONFIG_STACK_ALIGNMENT 4
#  endif
#endif

/* Stack alignment macros */

#define STACK_ALIGN_MASK    (CONFIG_STACK_ALIGNMENT-1)
#define STACK_ALIGN_DOWN(a) ((a) & ~STACK_ALIGN_MASK)
#define STACK_ALIGN_UP(a)   (((a) + STACK_ALIGN_MASK) & ~STACK_ALIGN_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_use_stack
 *
 * Description:
 *   Setup up stack-related information in the TCB
 *   using pre-allocated stack memory
 *
 *   The following TCB fields must be initialized:
 *   adj_stack_size: Stack size after adjustment for hardware,
 *     processor, etc.  This value is retained only for debug
 *     purposes.
 *   stack_alloc_ptr: Pointer to allocated stack
 *   adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The
 *     initial value of the stack pointer.
 *
 * Inputs:
 *   tcb: The TCB of new task
 *   stack_size:  The allocated stack size.
 *
 ****************************************************************************/

int up_use_stack(_TCB *tcb, void *stack, size_t stack_size)
{
  size_t top_of_stack;
  size_t size_of_stack;

  if (tcb->stack_alloc_ptr)
    {
      sched_free(tcb->stack_alloc_ptr);
    }

  /* Save the stack allocation */

  tcb->stack_alloc_ptr = stack;

  /* The ARM uses a push-down stack:  the stack grows toward lower addresses
   * in memory.  The stack pointer register, points to the lowest, valid
   * work address (the "top" of the stack).  Items on the stack are
   * referenced as positive word offsets from sp.
   */

  top_of_stack = (uint32_t)tcb->stack_alloc_ptr + stack_size - 4;

  /* The ARM stack must be aligned; 4 byte alignment for OABI and 8-byte
   * alignment for EABI. If necessary top_of_stack must be rounded down
   * to the next boundary
   */

  top_of_stack = STACK_ALIGN_DOWN(top_of_stack);

  /* The size of the stack in bytes is then the difference between
   * the top and the bottom of the stack (+4 because if the top
   * is the same as the bottom, then the size is one 32-bit element).
   * The size need not be aligned.
   */

  size_of_stack = top_of_stack - (uint32_t)tcb->stack_alloc_ptr + 4;

  /* Save the adjusted stack values in the _TCB */

  tcb->adj_stack_ptr  = (uint32_t*)top_of_stack;
  tcb->adj_stack_size = size_of_stack;

  return OK;
}
