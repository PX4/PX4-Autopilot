/************************************************************************
 * up_initialstate.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <sched.h>

#include "up_internal.h"

/************************************************************************
 * Private Definitions
 ************************************************************************/

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ************************************************************************/

void up_initial_state(FAR _TCB *tcb)
{
  FAR uint8_t *frame = tcb->xcp.stack;
  FAR uint8_t *regs  = tcb->xcp.regs;

  /* This is the form of initial stack frame
   *
   * This initial stack frame will be configured to hold.
   * (1) The 16-bit return address of either:
   *
   *     void task_start(void);
   *     void pthread_start(void)
   *
   *     The return address is stored at the top of stack.
   *     so that the RETI instruction will work:
   *
   *     PC15-8 <- ((SP))
   *     (SP)   <- (SP) -1
   *     PC7-0  <- ((SP))
   *     (SP)   <- (SP) -1
   */

   frame[FRAME_RETLS] = (((uint16_t)tcb->start) & 0xff);
   frame[FRAME_RETMS] = (((uint16_t)tcb->start) >> 8);

  /* The context save area for registers a, ie, and dpstr
   * follows the return address in the stack frame.
   */

  frame[FRAME_IE] = 0x80;

  /* Save the number of bytes in the frame (which will be used
   * to intialize the stack pointer when the task is started).
   */

  tcb->xcp.nbytes = FRAME_SIZE;

  /* Initialize the remaining register save area which is
   * outside of the stack save area.
   */

  tcb->xcp.regs[REGS_PSW] = 0;
}
