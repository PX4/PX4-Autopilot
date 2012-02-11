/****************************************************************************
 * arch/z80/src/z8/z8_loweruart.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <ez8.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "chip/chip.h"
#include "common/up_internal.h"

#ifdef USE_LOWUARTINIT

extern uint32_t get_freq(void);

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_lowuartinit
 ****************************************************************************/

void up_lowuartinit(void)
{
  uint32_t freq = get_freq();
  uint16_t brg;
  uint8_t  val;

#ifdef CONFIG_UART0_SERIAL_CONSOLE

  brg = (freq +(uint32_t)CONFIG_UART0_BAUD * 8) /((uint32_t)CONFIG_UART0_BAUD * 16) ;

  /* Set the baudrate */

  putreg8(brg >> 8, U0BRH);
  putreg8(brg & 0xff, U0BRL);

  /* Configure GPIO Port A pins 4 & 5 for alternate function */

  putreg8(0x02, PAADDR);
  val = getreg8(PACTL) | 0x30;    /* Set bits in alternate function register */
  putreg8(val, PACTL);
  putreg8(0x07, PAADDR);
  val = getreg8(PACTL) & 0xcf;    /* Reset bits in alternate function set-1 register */
  putreg8(val, PACTL);
  putreg8(0x08, PAADDR);
  val = getreg8(PACTL) & 0xcf;    /* Reset bits in alternate function set-2 register */
  putreg8(val, PACTL);
  putreg8(0x00, PAADDR);

  putreg8(0x00, U0CTL1);          /* no multi-processor operation mode */
  putreg8(0xc0, U0CTL0);          /* Transmit enable, Receive enable, no Parity, 1 Stop bit */

#elif defined(EZ8_UART1) && defined(CONFIG_UART1_SERIAL_CONSOLE)
  /* Set the baudrate */

  putreg8(brg >> 8, U1BRH);
  putreg8(brg & 0xff, U1BRL);

  /* Configure GPIO Port D pins 4 & 5 for alternate function */
  
  putreg8(0x02, PAADDR);
  val = getreg8(PDCTL) | 0x30;    /* Set bits in alternate function register */
  putreg8(val, PDCTL);
  putreg8(0x07, PDADDR);
  val = getreg8(PDCTL) & 0xcf;    /* Reset bits in alternate function set-1 register */
  putreg8(val, PDCTL);
  putreg8(0x08, PDADDR);
  val = getreg8(PDCTL) & 0xcf;    /* Reset bits in alternate function set-2 register */
  putreg8(val, PDCTL);
  putreg8(0x00, PDADDR);

  putreg8(0x00, U1CTL1);          /* no multi-processor operation mode */
  putreg8(0xc0, U1CTL0);          /* Transmit enable, Receive enable, no Parity, 1 Stop bit */
#endif
}
#endif /* USE_LOWUARTINIT */