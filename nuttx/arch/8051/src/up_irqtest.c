/************************************************************************
 * up_irqtest.c
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>

#include <nuttx/arch.h>
#include <8052.h>

#include "up_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

#define up_extint0 ((vector_t)PM2_VECTOR_EXTINT0)
#define up_timer0  ((vector_t)PM2_VECTOR_TIMER0)
#define up_extint1 ((vector_t)PM2_VECTOR_EXTINT1)
#define up_timer1  ((vector_t)PM2_VECTOR_TIMER1)
#define up_uart    ((vector_t)PM2_VECTOR_UART)
#define up_timer2  ((vector_t)PM2_VECTOR_TIMER2)

/************************************************************************
 * Private Types
 ************************************************************************/

typedef void (*vector_t)(void);

/************************************************************************
 * Public Variables
 ************************************************************************/

bool g_irqtest;
volatile uint8_t g_irqtos;
uint8_t g_irqregs[REGS_SIZE];
int g_nirqs;
FAR struct xcptcontext *g_irqcontext; 

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: utility functions
 ************************************************************************/

static void _up_putc(uint8_t ch) __naked
{
  ch; /* To avoid unreferenced argument warning */
  _asm
        mov     a, dpl
        ljmp    PM2_ENTRY_COUT
  _endasm;
}

void _up_puthex(uint8_t hex) __naked
{
  hex; /* To avoid unreferenced argument warning */
  _asm
        mov     a, dpl
        ljmp    PM2_ENTRY_PHEX
  _endasm;
}

void _up_puthex16(int hex) __naked
{
  hex; /* To avoid unreferenced argument warning */
  _asm
        ljmp    PM2_ENTRY_PHEX16
  _endasm;
}

void _up_putnl(void) __naked
{
  _asm
	ljmp	PM2_ENTRY_NEWLINE
  _endasm;
}

void _up_puts(__code char *ptr)
{
  for (; *ptr; ptr++)
    {
       _up_putc(*ptr);
    }
}

void _up_delay(uint8_t milliseconds) __naked
{
  _asm
	mov	r0, dpl
00001$: mov	r1, #230
00002$: nop
	nop
	nop
	nop
	nop
	nop
	djnz	r1, 00002$
	djnz	r0, 00001$
	ret
  _endasm;
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: os_start
 *
 * Description:
 *   "Fake" OS entry point.
 *
 ************************************************************************/

void os_start(void)
{
  int i;

  /* Disable all interrupts */

  IE = 0;

  /* Then verify all of the interrupt */

  g_irqtest = false;

  up_extint0();
  up_timer0();
#ifndef CONFIG_8052_TIMER2
  up_timer0();
#endif
  up_extint1();
  up_timer1();
  up_uart();
  up_timer2();

  /* Now a real interrupt ... */

  /* Configure timer 0 */

  TR0   = 0;     /* Make sure timer 0 is stopped */
  TF0   = 0;     /* Clear the overflow flag */
  TMOD &= 0xF0;  /* Set to mode 0 (without changing timer1) */
  TL0   = 0;     /* Clear timer 0 value */
  TH0   = 0;
  TR0    = 1;    /* Start the timer */

  /* Start timer interrupts */

  g_irqtest = true;
  g_nirqs   = 0;
  IE        = 0x82;    /* Enable interrupts */

  /* Wait a about 500 MS */

  _up_delay(500);

  /* Disable the timer */

  TR0 = 0;     /* Stop timer 0 */
  IE  = 0;     /* Disable interrupts */

  _up_puts("IRQs in 500 MS=");
  _up_puthex16(g_nirqs);
  _up_putnl();

  /* end of test */

  _up_puts("Test complete");
  _up_putnl();
  for(;;);
}

/************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   "Fake" IRQ dispatcher
 *
 ***********************************************************************/

void irq_dispatch(int irq, FAR void *context)
{
  context;
  if (g_irqtest)
    {
      g_nirqs++;
    }
  else
    {
      _up_puts("Dispatch IRQ=");
      _up_puthex(irq);
      _up_putnl();
    }
}

/************************************************************************
 * Name: up_dumpstack / up_dumpframe
 *
 * Description:
 *   "Fake" debug routines if needed.
 *
 ************************************************************************/

void up_dumpstack(void)
{
}

void up_dumpframe(FAR struct xcptcontext *context)
{
}

/************************************************************************
 * Name: up_ledinit, up_ledon, up_ledoff
 *
 * Description:
 *   "Fake" LED routines if needed
 *
 ************************************************************************/

void up_ledinit(void)
{
}

void up_ledon(uint8_t led)
{
  led;
}

void up_ledoff(uint8_t led)
{
  led;
}
