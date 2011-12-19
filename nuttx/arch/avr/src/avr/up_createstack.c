/****************************************************************************
 * arch/avr/src/avr/up_createstack.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

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
 * Name: up_create_stack
 *
 * Description:
 *   Allocate a stack for a new thread and setup up stack-related
 *  information in the TCB.
 *
 *   The following TCB fields must be initialized:
 *   adj_stack_size: Stack size after adjustment for hardware, processor,
 *     etc.  This value is retained only for debug purposes.
 *   stack_alloc_ptr: Pointer to allocated stack
 *   adj_stack_ptr: Adjusted stack_alloc_ptr for HW.  The initial value of
 *     the stack pointer.
 *
 * Input Parameters:
 *   tcb: The TCB of new task
 *   stack_size:  The requested stack size.  At least this how much must be
 *     allocated.
 *
 ****************************************************************************/

int up_create_stack(_TCB *tcb, size_t stack_size)
{
  /* Is there already a stack allocated of a different size? */

  if (tcb->stack_alloc_ptr && tcb->adj_stack_size != stack_size)
    {
      /* Yes.. free it */

      sched_free(tcb->stack_alloc_ptr);
      tcb->stack_alloc_ptr = NULL;
    }

   /* Do we need to allocate a stack? */
 
   if (!tcb->stack_alloc_ptr)
     {
       /* Allocate the stack.  If DEBUG is enabled (but not stack debug),
        * then create a zeroed stack to make stack dumps easier to trace.
        */

#if defined(CONFIG_DEBUG) && !defined(CONFIG_DEBUG_STACK)
       tcb->stack_alloc_ptr = (FAR void *)zalloc(stack_size);
#else
       tcb->stack_alloc_ptr = (FAR void *)malloc(stack_size);
#endif
     }

   /* Did we successfully allocate a stack? */

   if (tcb->stack_alloc_ptr)
     {
       size_t top_of_stack;

       /* Yes.. If stack debug is enabled, then fill the stack with a
        * recognizable value that we can use later to test for high
        * water marks.
        */

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_STACK)
       memset(tcb->stack_alloc_ptr, 0xaa, stack_size);
#endif

       /* The AVR uses a push-down stack:  the stack grows toward lower
        * addresses in memory.  The stack pointer register, points to the
        * lowest, valid work address (the "top" of the stack).  Items on the
        * stack are referenced as positive word offsets from sp.
        */

       top_of_stack = (size_t)tcb->stack_alloc_ptr + stack_size - 1;

       /* Save the adjusted stack values in the _TCB */

       tcb->adj_stack_ptr  = (FAR void *)top_of_stack;
       tcb->adj_stack_size = stack_size;

       up_ledon(LED_STACKCREATED);
       return OK;
     }

   return ERROR;
}
