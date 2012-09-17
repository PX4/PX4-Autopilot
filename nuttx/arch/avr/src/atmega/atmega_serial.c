/****************************************************************************
 * arch/avr/src/atmega/atmega_serial.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "atmega_config.h"

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <avr/io.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "atmega_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one USART enabled and configured as a RS-232 device? */

#ifndef HAVE_USART_DEVICE
#  warning "No USARTs enabled as RS-232 devices"
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1? */

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart0port     /* USART0 is console */
#  define TTYS0_DEV       g_usart0port     /* USART0 is ttyS0 */
#  ifdef CONFIG_AVR_USART1
#    define TTYS1_DEV     g_usart1port     /* USART1 is ttyS1 */
#  else
#    undef TTYS1_DEV                       /* No ttyS1 */
#  endif
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port     /* USART1 is console */
#  define TTYS0_DEV       g_usart1port     /* USART1 is ttyS0 */
#  ifdef CONFIG_AVR_USART0
#    define TTYS1_DEV     g_usart0port     /* USART0 is ttyS1 */
#  else
#    undef TTYS1_DEV                       /* No ttyS1 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int  usart0_setup(struct uart_dev_s *dev);
static void usart0_shutdown(struct uart_dev_s *dev);
static int  usart0_attach(struct uart_dev_s *dev);
static void usart0_detach(struct uart_dev_s *dev);
static int  usart0_rxinterrupt(int irq, void *context);
static int  usart0_txinterrupt(int irq, void *context);
static int  usart0_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  usart0_receive(struct uart_dev_s *dev, FAR unsigned int *status);
static void usart0_rxint(struct uart_dev_s *dev, bool enable);
static bool usart0_rxavailable(struct uart_dev_s *dev);
static void usart0_send(struct uart_dev_s *dev, int ch);
static void usart0_txint(struct uart_dev_s *dev, bool enable);
static bool usart0_txready(struct uart_dev_s *dev);
static bool usart0_txempty(struct uart_dev_s *dev);
#endif

#ifdef CONFIG_AVR_USART1
static int  usart1_setup(struct uart_dev_s *dev);
static void usart1_shutdown(struct uart_dev_s *dev);
static int  usart1_attach(struct uart_dev_s *dev);
static void usart1_detach(struct uart_dev_s *dev);
static int  usart1_rxinterrupt(int irq, void *context);
static int  usart1_txinterrupt(int irq, void *context);
static int  usart1_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  usart1_receive(struct uart_dev_s *dev, FAR unsigned int *status);
static void usart1_rxint(struct uart_dev_s *dev, bool enable);
static bool usart1_rxavailable(struct uart_dev_s *dev);
static void usart1_send(struct uart_dev_s *dev, int ch);
static void usart1_txint(struct uart_dev_s *dev, bool enable);
static bool usart1_txready(struct uart_dev_s *dev);
static bool usart1_txempty(struct uart_dev_s *dev);
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USART0 operations */

#ifdef CONFIG_AVR_USART0
struct uart_ops_s g_usart0_ops =
{
  .setup          = usart0_setup,
  .shutdown       = usart0_shutdown,
  .attach         = usart0_attach,
  .detach         = usart0_detach,
  .ioctl          = usart0_ioctl,
  .receive        = usart0_receive,
  .rxint          = usart0_rxint,
  .rxavailable    = usart0_rxavailable,
  .send           = usart0_send,
  .txint          = usart0_txint,
  .txready        = usart0_txready,
  .txempty        = usart0_txempty,
};

/* USART0 I/O buffers */

static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];

/* This describes the state of the ATMega USART0 port. */

static uart_dev_t g_usart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_usart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_usart0txbuffer,
  },
  .ops      = &g_usart0_ops,
};
#endif

/* USART1 operations */

#ifdef CONFIG_AVR_USART1
struct uart_ops_s g_usart1_ops =
{
  .setup          = usart1_setup,
  .shutdown       = usart1_shutdown,
  .attach         = usart1_attach,
  .detach         = usart1_detach,
  .ioctl          = usart1_ioctl,
  .receive        = usart1_receive,
  .rxint          = usart1_rxint,
  .rxavailable    = usart1_rxavailable,
  .send           = usart1_send,
  .txint          = usart1_txint,
  .txready        = usart1_txready,
  .txempty        = usart1_txempty,
};

/* USART 1 I/O buffers */

static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];

/* This describes the state of the ATMega USART1 port. */

static uart_dev_t g_usart1port =
{
  .recv     =
  {
    .size   = CONFIG_USART1_RXBUFSIZE,
    .buffer = g_usart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART1_TXBUFSIZE,
    .buffer = g_usart1txbuffer,
   },
  .ops      = &g_usart1_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usart0/1_restoreusartint
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_restoreusartint(uint8_t imr)
{
  uint8_t regval;

  regval  = UCSR0B;
  regval &= ~((1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0));
  imr    &=  ((1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0));
  regval |= imr;
  UCSR0B  = regval;
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_restoreusartint(uint8_t imr)
{
  uint8_t regval;

  regval  = UCSR1B;
  regval &= ~((1 << RXCIE1) | (1 << TXCIE1) | (1 << UDRIE1));
  imr    &=  ((1 << RXCIE1) | (1 << TXCIE1) | (1 << UDRIE1));
  regval |= imr;
  UCSR1B  = regval;
}
#endif

/****************************************************************************
 * Name: usart0/1_disableusartint
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static inline void usart0_disableusartint(uint8_t *imr)
{
  uint8_t regval;

  regval  = UCSR0B;
  *imr    = regval;
  regval &= ~((1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0));
  UCSR0B  = regval;
}
#endif

#ifdef CONFIG_AVR_USART1
static inline void usart1_disableusartint(uint8_t *imr)
{
  uint8_t regval;

  regval  = UCSR1B;
  *imr    = regval;
  regval &= ~((1 << RXCIE1) | (1 << TXCIE1) | (1 << UDRIE1));
  UCSR0B  = regval;
}
#endif

/****************************************************************************
 * Name: usart0/1_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the USART as an RS-232 UART */

  usart0_configure();
#endif

  return OK;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  /* Configure the USART as an RS-232 UART */

  usart1_configure();
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: usart0/1_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  usart0_reset();
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_shutdown(struct uart_dev_s *dev)
{
  /* Reset, disable interrupts, and disable Rx and Tx */

  usart1_reset();
}
#endif

/****************************************************************************
 * Name: usart0/1_attach
 *
 * Description:
 *   Configure the USART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_attach(struct uart_dev_s *dev)
{
  /* Attach the USART0 IRQs:
   *
   * RX:  USART Receive Complete. Set when are unread data in the receive
   *      buffer and cleared when the receive buffer is empty.
   * TX:  USART Transmit Complete.  Set when the entire frame in the Transmit
   *      Shift Register has been shifted out and there are no new data
   *      currently present in the transmit buffer.
   * DRE: USART Data Register Empty.  Indicates if the transmit buffer is ready
   *      to receive new data: The buffer is empty, and therefore ready to be
   *      written.
   */

  (void)irq_attach(ATMEGA_IRQ_U0RX, usart0_rxinterrupt);
  (void)irq_attach(ATMEGA_IRQ_U0DRE, usart0_txinterrupt);
//  (void)irq_attach(ATMEGA_IRQ_U0TX, usart0_txinterrupt);
  return OK;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_attach(struct uart_dev_s *dev)
{
  /* Attach the USART1 IRQs:
   *
   * RX:  USART Receive Complete. Set when are unread data in the receive
   *      buffer and cleared when the receive buffer is empty.
   * TX:  USART Transmit Complete.  Set when the entire frame in the Transmit
   *      Shift Register has been shifted out and there are no new data
   *      currently present in the transmit buffer.
   * DRE: USART Data Register Empty.  Indicates if the transmit buffer is ready
   *      to receive new data: The buffer is empty, and therefore ready to be
   *      written.
   */

  (void)irq_attach(ATMEGA_IRQ_U1RX, usart1_rxinterrupt);
  (void)irq_attach(ATMEGA_IRQ_U1DRE, usart1_txinterrupt);
//  (void)irq_attach(ATMEGA_IRQ_U1TX, usart1_txinterrupt);
  return OK;
}
#endif

/****************************************************************************
 * Name: usart0/1_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_detach(struct uart_dev_s *dev)
{
  /* Disable all USART0 interrupts */

  usart0_disableusartint(NULL);

  /* Detach the USART0 IRQs */

  (void)irq_detach(ATMEGA_IRQ_U0RX);
  (void)irq_detach(ATMEGA_IRQ_U0DRE);
//  (void)irq_detach(ATMEGA_IRQ_U0TX);
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_detach(struct uart_dev_s *dev)
{
  /* Disable all USART1 interrupts */

  usart1_disableusartint(NULL);

  /* Detach the USART1 IRQs */

  (void)irq_detach(ATMEGA_IRQ_U1RX);
  (void)irq_detach(ATMEGA_IRQ_U1DRE);
//  (void)irq_detach(ATMEGA_IRQ_U1TX);
}
#endif

/****************************************************************************
 * Name: usart0/1_rxinterrupt
 *
 * Description:
 *   This is the USART RX interrupt handler.  It will be invoked when an
 *   RX interrupt received.  It will call uart_receivechar to perform the RX
 *   data transfers.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_rxinterrupt(int irq, void *context)
{
  uint8_t ucsr0a = UCSR0A;

  /* Handle incoming, receive bytes (with or without timeout) */

  if ((ucsr0a & (1 << RXC0)) != 0)
    {
       /* Received data ready... process incoming bytes */

       uart_recvchars(&g_usart0port);
    }

  return OK;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_rxinterrupt(int irq, void *context)
{
  uint8_t ucsr1a = UCSR1A;

  /* Handle incoming, receive bytes (with or without timeout) */

  if ((ucsr1a & (1 << RXC1)) != 0)
    {
       /* Received data ready... process incoming bytes */

       uart_recvchars(&g_usart1port);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usart0/1_txinterrupt
 *
 * Description:
 *   This is the USART TX interrupt handler.  It will be invoked when an
 *   TX or DRE interrupt received.  It will call uart_xmitchars to perform
 *   the TXdata transfers.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_txinterrupt(int irq, void *context)
{
  uint8_t ucsr0a = UCSR0A;

  /* Handle outgoing, transmit bytes when the transmit data buffer is empty.
   * (There may still be data in the shift register).
   */

  if ((ucsr0a & (1 << UDRE0)) != 0)
    {
       /* Transmit data regiser empty ... process outgoing bytes */

       uart_xmitchars(&g_usart0port);
    }

  return OK;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_txinterrupt(int irq, void *context)
{
  uint8_t ucsr1a = UCSR1A;

  /* Handle outgoing, transmit bytes when the transmit data buffer is empty.
   * (There may still be data in the shift register).
   */

  if ((ucsr1a & (1 << UDRE1)) != 0)
    {
       /* Transmit data regiser empty ... process outgoing bytes */

       uart_xmitchars(&g_usart1port);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usart0/1_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  int ret = OK;

  switch (cmd)
    {
    case xxx: /* Add commands here */
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
#else
  return -ENOTTY;
#endif
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 0 /* Reserved for future growth */
  int ret = OK;

  switch (cmd)
    {
    case xxx: /* Add commands here */
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
#else
  return -ENOTTY;
#endif
}
#endif

/****************************************************************************
 * Name: usart0/1_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static int usart0_receive(struct uart_dev_s *dev, FAR unsigned int *status)
{
  /* Return status information */

  if (status)
    {
      *status = (FAR unsigned int)UCSR0A;
    }

  /* Then return the actual received byte */

  return UDR0;
}
#endif

#ifdef CONFIG_AVR_USART1
static int usart1_receive(struct uart_dev_s *dev, FAR unsigned int *status)
{
  /* Return status information */

  if (status)
    {
      *status = (FAR unsigned int)UCSR1A;
    }

  /* Then return the actual received byte */

  return UDR1;
}
#endif

/****************************************************************************
 * Name: usart0/1_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_rxint(struct uart_dev_s *dev, bool enable)
{
  /* Enable/disable RX interrupts:
   *
   * RX:  USART Receive Complete. Set when are unread data in the receive
   *      buffer and cleared when the receive buffer is empty.
   */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
       UCSR0B |= (1 << RXCIE0);
#endif
    }
  else
    {
       UCSR0B &= ~(1 << RXCIE0);
    }
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_rxint(struct uart_dev_s *dev, bool enable)
{
  /* Enable/disable RX interrupts:
   *
   * RX:  USART Receive Complete. Set when are unread data in the receive
   *      buffer and cleared when the receive buffer is empty.
   */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
       UCSR1B |= (1 << RXCIE1);
#endif
    }
  else
    {
       UCSR1B &= ~(1 << RXCIE1);
    }
}
#endif

/****************************************************************************
 * Name: usart0/1_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static bool usart0_rxavailable(struct uart_dev_s *dev)
{
  return (UCSR0A & (1 << RXC0)) != 0;
}
#endif

#ifdef CONFIG_AVR_USART1
static bool usart1_rxavailable(struct uart_dev_s *dev)
{
  return (UCSR1A & (1 << RXC1)) != 0;
}
#endif

/****************************************************************************
 * Name: usart0/1_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_send(struct uart_dev_s *dev, int ch)
{
  UDR0 = ch;
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_send(struct uart_dev_s *dev, int ch)
{
  UDR1 = ch;
}
#endif

/****************************************************************************
 * Name: usart0/1_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static void usart0_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  /* Enable/disable TX interrupts:
   *
   * TX:  USART Transmit Complete.  Set when the entire frame in the Transmit
   *      Shift Register has been shifted out and there are no new data
   *      currently present in the transmit buffer.
   * DRE: USART Data Register Empty.  Indicates if the transmit buffer is ready
   *      to receive new data: The buffer is empty, and therefore ready to be
   *      written.
   */

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      UCSR0B |= (1 << UDRIE0);
//    UCSR0B |= (1 << TXCIE0);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(&g_usart0port);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      UCSR0B &= ~((1 << UDRIE0) | (1 << TXCIE0));
    }

  irqrestore(flags);
}
#endif

#ifdef CONFIG_AVR_USART1
static void usart1_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  /* Enable/disable TX interrupts:
   *
   * TX:  USART Transmit Complete.  Set when the entire frame in the Transmit
   *      Shift Register has been shifted out and there are no new data
   *      currently present in the transmit buffer.
   * DRE: USART Data Register Empty.  Indicates if the transmit buffer is ready
   *      to receive new data: The buffer is empty, and therefore ready to be
   *      written.
   */

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      UCSR1B |= (1 << UDRIE1);
//    UCSR1B |= (1 << TXCIE1);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(&g_usart1port);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      UCSR1B &= ~((1 << UDRIE1) | (1 << TXCIE1));
    }

  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: usart0/1_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static bool usart0_txready(struct uart_dev_s *dev)
{
  return (UCSR0A & (1 << UDRE0)) != 0;
}
#endif

#ifdef CONFIG_AVR_USART1
static bool usart1_txready(struct uart_dev_s *dev)
{
  return (UCSR1A & (1 << UDRE1)) != 0;
}
#endif

/****************************************************************************
 * Name: usart0/1_txempty
 *
 * Description:
 *   Return true if the tranmsit data register and shift reqister are both
 *   empty
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_USART0
static bool usart0_txempty(struct uart_dev_s *dev)
{
  return (UCSR0A & (1 << TXC0)) != 0;
}
#endif

#ifdef CONFIG_AVR_USART1
static bool usart1_txempty(struct uart_dev_s *dev)
{
  return (UCSR1A & (1 << TXC1)) != 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable all USARTS */

#ifdef CONFIG_AVR_USART1
  usart0_disableusartint(NULL);
#endif
#ifdef CONFIG_AVR_USART1
  usart1_disableusartint(NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
  usart0_setup(&g_usart0port);
#  elif defined(CONFIG_USART1_SERIAL_CONSOLE)
  usart1_setup(&g_usart1port);
#  endif
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
#endif

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  uint8_t imr;

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
  usart0_disableusartint(&imr);
#else
  usart1_cdisableusartint(&imr);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
  usart0_restoreusartint(imr);
#else
  usart1_restoreusartint(imr);
#endif
#endif

  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */

