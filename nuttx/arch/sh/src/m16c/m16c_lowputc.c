/**************************************************************************
 * arch/sh/src/m16c/m16c_lowputc.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "up_internal.h"
#include "m16c_uart.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/* Configuration **********************************************************/

#ifndef M16C_XIN_PRESCALER
#  define M16C_XIN_PRESCALER 1
#endif

/* Is there any serial support?  This might be the case if the board does
 * not have serial ports but supports stdout through, say, an LCD.
 */

#if defined(CONFIG_M16C_UART0) || defined(CONFIG_M16C_UART1) || defined(CONFIG_M16C_UART2)
#  define HAVE_SERIAL
#else
#  undef HAVE_SERIAL
#endif

/* Is one of the serial ports a console? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART0)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART1)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART2)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#else
#  if defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE)|| defined(CONFIG_UART2_SERIAL_CONSOLE)
#    error "A serial console selected, but corresponding UART not enabled"
#  endif
#  undef HAVE_SERIALCONSOLE
#endif

#if defined(HAVE_SERIALCONSOLE) && defined(CONFIG_LCD_CONSOLE)
#  error "Both serial and LCD consoles are defined"
#elif !defined(HAVE_SERIALCONSOLE) && !defined(CONFIG_LCD_CONSOLE)
#  warning "No console is defined"
#endif

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIALCONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define M16C_UART_BASE     M16C_UART0_BASE
#  define M16C_UART_BAUD     CONFIG_UART0_BAUD
#  define M16C_UART_BITS     CONFIG_UART0_BITS
#  define M16C_UART_PARITY   CONFIG_UART0_PARITY
#  define M16C_UART_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define M16C_UART_BASE     M16C_UART1_BASE
#  define M16C_UART_BAUD     CONFIG_UART1_BAUD
#  define M16C_UART_BITS     CONFIG_UART1_BITS
#  define M16C_UART_PARITY   CONFIG_UART1_PARITY
#  define M16C_UART_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define M16C_UART_BASE     M16C_UART2_BASE
#  define M16C_UART_BAUD     CONFIG_UART2_BAUD
#  define M16C_UART_BITS     CONFIG_UART2_BITS
#  define M16C_UART_PARITY   CONFIG_UART2_PARITY
#  define M16C_UART_2STOP    CONFIG_UART2_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get mode setting */

#if M16C_UART_BITS == 7
#  define M16C_MR_SMDBITS UART_MR_SMD7BITS
#elif M16C_UART_BITS == 8
#  define M16C_MR_SMDBITS UART_MR_SMD8BITS
#elif M16C_UART_BITS == 8
#  define M16C_MR_SMDBITS UART_MR_SMD9BITS
#else
#  error "Number of bits not supported"
#endif

#if M16C_UART_PARITY == 0
#  define M16C_MR_PARITY (0)
#elif M16C_UART_PARITY == 1
#  define M16C_MR_PARITY UART_MR_PRYE
#elif M16C_UART_PARITY == 2
#  define M16C_MR_PARITY (UART_MR_PRYE|UART_MR_PRY)
#else
#  error "Invalid parity selection"
#endif

#if M16C_UART_2STOP != 0
#  define M16C_MR_STOPBITS UART_MR_STPS
#else
#  define M16C_MR_STOPBITS (0)
#endif

/* The full MR setting: */

#define M16C_MR_VALUE (M16C_MR_SMDBITS|M16C_MR_PARITY|M16C_MR_STOPBITS)

/* Clocking ***************************************************************/

/* The Bit Rate Generator (BRG) value can be calculated by:
 *
 * BRG = source-frequency / prescaler / 16 / baud rate - 1
 *
 * Example:
 *  source-frequency = 20,000,000 (20MHz)
 *  prescaler = 1
 *  baud rate = 19200
 *  BRG = 20,000,000/1/16/19200 - 1 = 64
 */

#define M16C_UART_BRG_VALUE \
	((M16C_XIN_FREQ / (16 * M16C_XIN_PRESCALER * M16C_UART_BAUD)) - 1)

#endif /* HAVE_SERIALCONSOLE */

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return TRUE of the Transmit Data Register is empty
 *
 **************************************************************************/

#ifdef HAVE_SERIALCONSOLE
static inline int up_txready(void)
{
  /* Check the TI bit in the CI register.  1=Transmit buffer empty */

  return ((getreg8(M16C_UART_BASE + M16C_UART_C1) & UART_C1_TI) != 0);
}
#endif /* HAVE_SERIALCONSOLE */

/**************************************************************************
 * Name: up_lowserialsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

#if defined(HAVE_SERIALCONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
static inline void up_lowserialsetup(void)
{
  uint8_t regval;

  /* Set the baud rate generator */

  putreg8(M16C_UART_BRG_VALUE, M16C_UART_BASE + M16C_UART_BRG);

  /* Disable CTS/RTS */

  putreg8(UART_C0_CRD, M16C_UART_BASE + M16C_UART_C0);

  /* Disable RX/TX interrupts */

#if 0
  putreg8(0, M16C_UCON);
#endif

/* Set interrupt cause=TX complete and continuous receive mode */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  regval  = getreg8(M16C_UCON);
  regval |= (UART_CON_U0IRS|UART_CON_U0RRM);
  putreg8(regval, M16C_UCON);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  regval = getreg8(M16C_UCON);
  regval |= (UART_CON_U1IRS|UART_CON_U1RRM);
  putreg8(regval, M16C_UCON);
#else
  regval = getreg8(M16C_U2C1);
  regval |= (UART_C1_U2IRS|UART_C1_U2RRM);
  putreg8(regval, M16C_U2C1);
#endif

  /* Set UART transmit/receive control register 1 to enable transmit and receive */

  putreg8(UART_C1_TE|UART_C1_RE, M16C_UART_BASE + M16C_UART_C1);
	
  /* Set UART transmit/receive mode register data bits, stop bits, parity */

  putreg8(M16C_MR_VALUE, M16C_UART_BASE + M16C_UART_MR);

  /* Set port direction registers for Rx/TX pins */

 #if defined(CONFIG_UART0_SERIAL_CONSOLE)
  regval  = getreg8(M16C_PD6);
  regval &= ~(1 << 2);                         /* PD6-2:RxD1 */
  regval |=  (1 << 3);                         /* PD6-3:TxD1 */
  putreg8(regval, M16C_PD6);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  regval = getreg8(M16C_PD6);
  regval &= ~(1 << 6);                         /* PD6-6:RxD1 */
  regval |=  (1 << 7);                         /* PD6-7:TxD1 */
  putreg8(regval, M16C_PD6);
#else
  regval = getreg8(M16C_PD7);
  regval &= ~(1 << 1);                         /* PD7-1:RxD1 */
  regval &=  (1 << 0);                         /* PD7-0:TxD1 */
  putreg8(regval, M16C_PD7);
#endif

  /* Read any data left in the RX fifo */

  regval = (uint8_t)getreg16(M16C_UART_BASE + M16C_UART_RB);
}
#endif /* HAVE_SERIALCONFIG && !CONFIG_SUPPRESS_UART_CONFIG */

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 **************************************************************************/

#if defined(HAVE_SERIAL) && !defined(CONFIG_LCD_CONSOLE)
void up_lowputc(char ch)
{
#ifdef HAVE_SERIALCONSOLE
  /* Wait until the transmit buffer is empty */

  while (!up_txready());

  /* Write the data to the transmit buffer */

  putreg16((uint16_t)ch, M16C_UART_BASE + M16C_UART_TB);
#endif
}
#endif

/**************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 **************************************************************************/

void up_lowsetup(void)
{
  /* Here we initialize the serial console early so that it can be used
   * for bring-up debugging.
   */

#if defined(HAVE_SERIALCONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  up_lowserialsetup()
#endif

  /* The LCD is initialized here to because it may be that the LCD is
   * used for console output.
   */

#ifdef CONFIG_ARCH_LCD
  up_lcdinit();
#endif
}
