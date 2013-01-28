/****************************************************************************
 * common/up_assert.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip/chip.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lowsyslog
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) /* noreturn_function */
{
  /* Are we in an interrupt handler or the idle task? */

  if (up_interrupt_context() || ((FAR _TCB*)g_readytorun.head)->pid == 0)
    {
       (void)irqsave();
        for(;;)
          {
#ifdef CONFIG_ARCH_LEDS
            up_ledon(LED_PANIC);
            up_mdelay(250);
            up_ledoff(LED_PANIC);
            up_mdelay(250);
#endif
          }
    }
  else
    {
      exit(errorcode);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

#ifdef CONFIG_HAVE_FILENAME
void up_assert(const uint8_t *filename, int lineno)
#else
void up_assert(void)
#endif
{
#if CONFIG_TASK_NAME_SIZE > 0
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);

#ifdef CONFIG_HAVE_FILENAME
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed at file:%s line: %d task: %s\n",
        filename, lineno, rtcb->name);
#else
  lldbg("Assertion failed at file:%s line: %d\n",
        filename, lineno);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed: task: %s\n", rtcb->name);
#else
  lldbg("Assertion failed\n");
#endif
#endif

  up_stackdump();
  REGISTER_DUMP();
 _up_assert(EXIT_FAILURE);
}

/****************************************************************************
 * Name: up_assert_code
 ****************************************************************************/

#ifdef CONFIG_HAVE_FILENAME
void up_assert_code(const uint8_t *filename, int lineno, int errorcode)
#else
void up_assert_code(int errorcode)
#endif
{
#if CONFIG_TASK_NAME_SIZE > 0
  _TCB *rtcb = (_TCB*)g_readytorun.head;
#endif

  up_ledon(LED_ASSERTION);

#ifdef CONFIG_HAVE_FILENAME
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed at file:%s line: %d task: %s error code: %d\n",
        filename, lineno, rtcb->name, errorcode);
#else
  lldbg("Assertion failed at file:%s line: %d error code: %d\n",
        filename, lineno, errorcode);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  lldbg("Assertion failed: task: %s error code: %d\n", rtcb->name, errorcode);
#else
  lldbg("Assertion failed: error code: %d\n", errorcode);
#endif
#endif

  up_stackdump();
  REGISTER_DUMP();
 _up_assert(errorcode);
}
