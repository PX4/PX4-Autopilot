/************************************************************************
 * up_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#include "up_internal.h"

/************************************************************************
 * Private Definitions
 ************************************************************************/

/************************************************************************
 * Private Data
 ************************************************************************/

/* This is the top of the stack containing the interrupt
 * stack frame.  It is set when processing an interrupt.  It
 * is also cleared when the interrupt returns so this can
 * also be used like a boolean indication that we are in an
 * interrupt.
 */

volatile uint8_t g_irqtos;

/* Registers are saved in the following global array during
 * interrupt processing.  If a context switch is performed
 * during the interrupt handling, these registers will be
 * copied into the TCB again (NOTE:  We could save a copy
 * if the interrupt handling logic saved the registers
 * directly into (_TCB*)g_readytorun.head->xcp.regs).
 */

uint8_t g_irqregs[REGS_SIZE];

/* If during execution of an interrup handler, a context
 * switch must be performed, the follwing will be set to
 * to that address of the relevant context structure.  The
 * actual switch will be deferred until the time that the
 * the interrupt exits.
 */

FAR struct xcptcontext *g_irqcontext;

/* It is faster to look up 8-bit shifts in this table than
 * to comput them.
 */

const uint8_t g_ntobit[8] = 
  { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS
 *   initialization after the basic OS services have been
 *   initialized.  The architecture specific details of
 *   initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the
 *   clock, and registering device drivers are some of the
 *   things that are different for each processor and hardware
 *   platform.
 *
 *   up_initialize is called after the OS initialized but
 *   before the init process has been started and before the
 *   libraries have been initialized.  OS services and driver
 *   services are available.
 *
 ************************************************************************/

void up_initialize(void)
{
 /* Initialize global variables */

  g_irqtos = 0;

  /* Add extra memory fragments to the memory manager */

#if CONFIG_MM_REGIONS > 1
  up_addregion();
#endif

  /* Initialize the interrupt subsystem */

  up_irqinitialize();

  /* Initialize the system timer interrupt */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_timerinit();
#endif

  up_ledon(LED_IRQSENABLED);
}

