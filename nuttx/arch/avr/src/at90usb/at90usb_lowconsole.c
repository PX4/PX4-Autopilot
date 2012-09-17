/******************************************************************************
 * arch/avr/src/at90usb/at90usb_lowconsole.c
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
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include "at90usb_config.h"

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>
#include <avr/io.h>

#include "up_arch.h"
#include "up_internal.h"

#include "at90usb_internal.h"

/******************************************************************************
 * Private Definitions
 ******************************************************************************/

/* Baud rate settings for normal and double speed modes  */

#define AVR_NORMAL_UBRR1 \
  ((((BOARD_CPU_CLOCK / 16) + (CONFIG_USART1_BAUD / 2)) / (CONFIG_USART1_BAUD)) - 1)

#define AVR_DBLSPEED_UBRR1 \
  ((((BOARD_CPU_CLOCK / 8) + (CONFIG_USART1_BAUD / 2)) / (CONFIG_USART1_BAUD)) - 1)

/* Select normal or double speed baud settings.  This is a trade-off between the
 * sampling rate and the accuracy of the divisor for high baud rates.
 *
 * As examples, consider:
 *
 *   BOARD_CPU_CLOCK=8MHz and BAUD=115200:
 *     AVR_NORMAL_UBRR1 = 4 (rounded), actual baud = 125,000
 *     AVR_DBLSPEED_UBRR1 = 9 (rounded), actual baud = 111,111
 *
 *   BOARD_CPU_CLOCK=8MHz and BAUD=9600:
 *     AVR_NORMAL_UBRR1 = 52 (rounded), actual baud = 9615
 *     AVR_DBLSPEED_UBRR1 = 104 (rounded), actual baud = 9615
 */
 
#undef UART1_DOUBLE_SPEED
#if BOARD_CPU_CLOCK <= 4000000
#  if CONFIG_USART1_BAUD <= 9600
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define UART1_DOUBLE_SPEED 1
#  endif
#elif BOARD_CPU_CLOCK <= 8000000
#  if CONFIG_USART1_BAUD <= 19200
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define UART1_DOUBLE_SPEED 1
#  endif
#elif BOARD_CPU_CLOCK <= 12000000
#  if CONFIG_USART1_BAUD <= 28800
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define UART1_DOUBLE_SPEED 1
#  endif
#elif BOARD_CPU_CLOCK <= 16000000
#  if CONFIG_USART1_BAUD <= 38400
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define UART1_DOUBLE_SPEED 1
#  endif
#else
#  if CONFIG_USART1_BAUD <= 57600
#    define AVR_UBRR1 AVR_NORMAL_UBRR1
#  else
#    define AVR_UBRR1 AVR_DBLSPEED_UBRR1
#    define UART1_DOUBLE_SPEED 1
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
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: usart1_reset
 *
 * Description:
 *   Reset a USART.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart1_reset(void)
{
  /* Clear USART configuration */

  UCSR1A = 0;
  UCSR1B = 0;
  UCSR1C = 0;

  /* Unconfigure pins */

  DDRD  &= ~(1 << 3);
  PORTD &= ~(1 << 2);

  /* Unconfigure BAUD divisor */

  UBRR1 = 0;
}
#endif

/******************************************************************************
 * Name: usart1_configure
 *
 * Description:
 *   Configure USART1.
 *
 ******************************************************************************/

#ifdef HAVE_USART_DEVICE
void usart1_configure(void)
{
  uint8_t ucsr1b;
  uint8_t ucsr1c;

  /* Select normal or double speed. */

#ifdef UART1_DOUBLE_SPEED
  UCSR1A = (1 << U2X1);
#else
  UCSR1A = 0;
#endif

  /* Select baud, parity, nubmer of bits, stop bits, etc. */

  ucsr1b = ((1 << TXEN1)  | (1 << RXEN1));
  ucsr1c = 0;
 
  /* Select parity */

#if CONFIG_USART1_PARITY == 1
  ucsr1c |= ((1 << UPM11) | (1 << UPM10)); /* Odd parity */
#elif CONFIG_USART1_PARITY == 2
  ucsr1c |= (1 << UPM11);                  /* Even parity */
#endif

  /* 1 or 2 stop bits */

#if defined(CONFIG_USART1_2STOP) && CONFIG_USART1_2STOP > 0
  ucsr1c |= (1 << USBS1);                  /* Two stop bits */
#endif

  /* Word size */

#if CONFIG_USART1_BITS == 5
#elif CONFIG_USART1_BITS == 6
  ucsr1c |= (1 << UCSZ10);
#elif CONFIG_USART1_BITS == 7
  ucsr1c |= (1 << UCSZ11);
#elif CONFIG_USART1_BITS == 8
  ucsr1c |= ((1 << UCSZ10) | (1 << UCSZ11));
#elif CONFIG_USART1_BITS == 9
  ucsr1c |= ((1 << UCSZ10) | (1 << UCSZ11));
  ucsr1b |= (1 << UCSZ12);
#else
#  error "Unsupported word size"
#endif

  UCSR1B = ucsr1b;
  UCSR1C = ucsr1c;

  /* Pin Configuration: None necessary, Port D bits 2&3 are automatically
   * configured:
   *
   *   Port D, Bit 2: RXD1, Receive Data (Data input pin for the USART1). When
   *     the USART1 receiver is enabled this pin is configured as an input
   *     regardless of the value of DDD2. When the USART forces this pin to
   *     be an input, the pull-up can still be controlled by the PORTD2 bit.
   *   Port D, Bit 3: TXD1, Transmit Data (Data output pin for the USART1).
   *     When the USART1 Transmitter is enabled, this pin is configured as
   *     an output regardless of the value of DDD3.
   */

  DDRD  |= (1 << 3);   /* Force Port D pin 3 to be an output -- Shouldn't be necessary */
  PORTD |= (1 << 2);   /* Set pull-up on port D pin 2 */

  /* Set the baud rate divisor */

  UBRR1 = AVR_UBRR1;
}
#endif

/******************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the intialization sequence to configure the serial console uart
 *   (only).
 *
 ******************************************************************************/

void up_consoleinit(void)
{
#ifdef HAVE_SERIAL_CONSOLE
  usart1_configure();
#endif
}

/******************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ******************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  while ((UCSR1A & (1 << UDRE1)) == 0);
  UDR1 = ch;
#endif
}

