/****************************************************************************
 *  arch/x86/src/qemu/qemu_lowputc.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/io.h>
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 /* COM1 port addresses */
 
#define COM1_PORT 0x3f8   /* COM1: I/O port 0x3f8, IRQ 4 */
#define COM2_PORT 0x2f8   /* COM2: I/O port 0x2f8, IRQ 3 */
#define COM3_PORT 0x3e8   /* COM3: I/O port 0x3e8, IRQ 4 */
#define COM4_PORT 0x2e8   /* COM4: I/O port 0x2e8, IRQ 3 */

/* 16650 register offsets */

#define COM_RBR   0       /* DLAB=0, Receiver Buffer (read) */
#define COM_THR   0       /* DLAB=0, Transmitter Holding Register (write) */
#define COM_DLL   0       /* DLAB=1, Divisor Latch (least significant byte) */
#define COM_IER   1       /* DLAB=0, Interrupt Enable */
#define COM_DLM   1       /* DLAB=1, Divisor Latch(most significant byte) */
#define COM_IIR   2       /* Interrupt Identification (read) */
#define COM_FCR   2       /* FIFO Control (write) */
#define COM_LCR   3       /* Line Control */
#define COM_MCR   4       /* MODEM Control */
#define COM_LSR   5       /* Line Status */
#define COM_MSR   6       /* MODEM Status */
#define COM_SCR   7       /* Scratch */

/* 16650 register bit definitions */

#define LSR_THRE  (1 << 5) /* Bit 5: Transmitter Holding Register Empty */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 **************************************************************************/

void up_lowputc(char ch)
{
  /* Wait until the Transmitter Holding Register (THR) is empty. */

  while ((inb(COM1_PORT+COM_LSR) & LSR_THRE) == 0);

  /* Then output the character to the THR*/

  outb(ch, COM1_PORT+COM_THR);
}

