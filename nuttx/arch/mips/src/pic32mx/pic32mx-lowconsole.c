/******************************************************************************
 * arch/mips/src/pic32mx/pic32mx-lowconsole.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "pic32mx-config.h"
#include "pic32mx-internal.h"

/******************************************************************************
 * Private Definitions
 ******************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART1_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART1_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART2_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART2_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART2_2STOP
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART3_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART3_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART3_2STOP
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART4_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART4_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART4_2STOP
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART5_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART5_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART5_2STOP
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define PIC32MX_CONSOLE_BASE     PIC32MX_UART6_K1BASE
#    define PIC32MX_CONSOLE_BAUD     CONFIG_UART6_BAUD
#    define PIC32MX_CONSOLE_BITS     CONFIG_UART6_BITS
#    define PIC32MX_CONSOLE_PARITY   CONFIG_UART6_PARITY
#    define PIC32MX_CONSOLE_2STOP    CONFIG_UART6_2STOP
#  else
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif

/******************************************************************************
 * Private Types
 ******************************************************************************/

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Global Variables
 ******************************************************************************/

/******************************************************************************
 * Private Variables
 ******************************************************************************/

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/******************************************************************************
 * Name: pic32mx_putreg
 *
 * Description:
 *   Write a value to a UART register
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline void pic32mx_putreg(uintptr_t uart_base, unsigned int offset,
                                      uint32_t value)
{
  putreg32(value, uart_base + offset);
}
#endif

/******************************************************************************
 * Name: pic32mx_getreg
 *
 * Description:
 *   Get a value from a UART register
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static inline uint32_t pic32mx_getreg(uintptr_t uart_base,
                                          unsigned int offset)
{
  return getreg32(uart_base + offset);
}
#endif

/******************************************************************************
 * Name: pic32mx_uartsetbaud
 *
 * Description:
 *   Configure the UART baud rate.
 *
 *   With BRGH=0
 *     BAUD = PBCLK / 16 / (BRG+1)
 *     BRG  = PBCLK / 16 / BAUD - 1
 *   With BRGH=1
 *     BAUD = PBCLK / 4 / (BRG+1)
 *     BRG  = PBCLK / 4 / BAUD - 1
 *
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
static void pic32mx_uartsetbaud(uintptr_t uart_base, uint32_t baudrate)
{
  uint32_t tmp;
  uint32_t brg;
  unsigned int mode;

  /* We want the largest value of BRG divisor possible (for the best accuracy).
   * Subject to BRG <= 65536.
   */

  tmp = BOARD_PBCLOCK / baudrate;

  /* Try BRGH=1 first.  This will select the 4x divisor and will produce the
   * larger BRG divisor, given all other things equal.
   */

  brg  = (tmp + 2) >> 2;
  mode = PIC32MX_UART_MODESET_OFFSET;

  if (brg > 65536)
    {
      /* Nope, too big.. try BRGH=0 */

      brg  = (tmp + 8) >> 4;
      mode = PIC32MX_UART_MODECLR_OFFSET;
    }
  DEBUGASSERT(brg <= 65536);

  /* Set the BRG divisor */

  pic32mx_putreg(uart_base, mode, UART_MODE_BRGH);
  pic32mx_putreg(uart_base, PIC32MX_UART_BRG_OFFSET, brg);
}
#endif

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: pic32mx_uartreset
 *
 * Description:
 *   Reset hardware and disable Rx and Tx.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartreset(uintptr_t uart_base)
{
  /* Doesn't reset the hardware... just shuts it down */

  pic32mx_putreg(uart_base, PIC32MX_UART_STACLR_OFFSET,
                 UART_STA_UTXEN | UART_STA_URXEN);
  pic32mx_putreg(uart_base, PIC32MX_UART_MODECLR_OFFSET, UART_MODE_ON);
}
#endif

/******************************************************************************
 * Name: pic32mx_uartconfigure
 *
 * Description:
 *   Configure a UART as a RS-232 UART.
 *
 ******************************************************************************/

#ifdef HAVE_UART_DEVICE
void pic32mx_uartconfigure(uintptr_t uart_base, uint32_t baudrate,
                           unsigned int parity, unsigned int nbits, bool stop2)
{
  /* Clear mode and sta bits */

  pic32mx_putreg(uart_base, PIC32MX_UART_MODECLR_OFFSET,
                 UART_MODE_STSEL    | UART_MODE_PDSEL_MASK  | UART_MODE_BRGH   |
                 UART_MODE_RXINV    | UART_MODE_WAKE        | UART_MODE_LPBACK |
                 UART_MODE_UEN_MASK | UART_MODE_RTSMD       | UART_MODE_IREN   |
                 UART_MODE_SIDL     | UART_MODE_ON);

  /* Configure the FIFOs:
   *
   *   RX: Interrupt at 6 of 8 (for 8-deep FIFO) or 3 o 4 (4-deep FIFO)
   *   TX: Interrupt on FIFO empty
   *   Invert transmit polarity.
   *
   * NOTE that there are not many options on trigger TX interrupts.  The FIFO not
   * full might generate better through-put but with a higher interrupt rate.  FIFO
   * empty should lower the interrupt rate but result in a burstier output.  If
   * you change this, please read the comment for acknowledging the interrupt in
   * pic32mx-serial.c
   */

#ifdef UART_STA_URXISEL_RXB6
  pic32mx_putreg(uart_base, PIC32MX_UART_STACLR_OFFSET,
                 UART_STA_UTXINV    | UART_STA_UTXISEL_TXBE | UART_STA_URXISEL_RXB6);
#else
  pic32mx_putreg(uart_base, PIC32MX_UART_STACLR_OFFSET,
                 UART_STA_UTXINV    | UART_STA_UTXISEL_TXBE | UART_STA_URXISEL_RXB3);
#endif

  /* Configure the FIFO interrupts */

  pic32mx_putreg(uart_base, PIC32MX_UART_STASET_OFFSET,
                 UART_STA_UTXISEL_TXBNF  | UART_STA_URXISEL_RECVD);

  /* Configure word size and parity */

  if (nbits == 9)
    {
      DEBUGASSERT(parity == 0);
      pic32mx_putreg(uart_base, PIC32MX_UART_MODESET_OFFSET,
                     UART_MODE_PDSEL_9NONE);
    }
  else
    {
      DEBUGASSERT(nbits == 8);
      if (parity == 1)
        {
          pic32mx_putreg(uart_base, PIC32MX_UART_MODESET_OFFSET,
                         UART_MODE_PDSEL_8ODD);
        }
      else if (parity == 2)
        {
          pic32mx_putreg(uart_base, PIC32MX_UART_MODESET_OFFSET,
                         UART_MODE_PDSEL_8EVEN);
        }
    }

  /* Configure 1 or 2 stop bits */

  if (stop2)
    {
      pic32mx_putreg(uart_base, PIC32MX_UART_MODESET_OFFSET,
                     UART_MODE_STSEL);
    }

  /* Set the BRG divisor */

  pic32mx_uartsetbaud(uart_base, baudrate);

  /* Enable the UART */

  pic32mx_putreg(uart_base, PIC32MX_UART_STASET_OFFSET,
                 UART_STA_UTXEN | UART_STA_URXEN);
  pic32mx_putreg(uart_base, PIC32MX_UART_MODESET_OFFSET,
                 UART_MODE_ON);
}
#endif

/******************************************************************************
 * Name: pic32mx_consoleinit
 *
 * Description:
 *   Initialize a low-level console for debug output.  This function is called
 *   very early in the intialization sequence to configure the serial console
 *   UART (only).
 *
 ******************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
void pic32mx_consoleinit(void)
{
  pic32mx_uartconfigure(PIC32MX_CONSOLE_BASE, PIC32MX_CONSOLE_BAUD,
                        PIC32MX_CONSOLE_PARITY, PIC32MX_CONSOLE_BITS,
                        PIC32MX_CONSOLE_2STOP);
}
#endif

/******************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console.
 *
 ******************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait for the transmit buffer not full */

  while ((pic32mx_getreg(PIC32MX_CONSOLE_BASE, PIC32MX_UART_STA_OFFSET) & UART_STA_UTXBF) != 0);

  /* Then write the character to the TX data register */

  pic32mx_putreg(PIC32MX_CONSOLE_BASE, PIC32MX_UART_TXREG_OFFSET, (uint32_t)ch);
#endif
}

