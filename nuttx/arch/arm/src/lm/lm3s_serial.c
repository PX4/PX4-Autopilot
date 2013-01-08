/****************************************************************************
 * arch/arm/src/lm/lm3s_serial.c
 *
 *   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
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

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

#if LM3S_NUARTS < 2
#  undef  CONFIG_LM3S_UART1
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#endif

#if LM3S_NUARTS < 3
#  undef  CONFIG_LM3S_UART2
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#endif

/* Is there a UART enabled? */

#if !defined(CONFIG_LM3S_UART0) && !defined(CONFIG_LM3S_UART1) && !defined(CONFIG_LM3S_UART2)
#  error "No UARTs enabled"
#endif

/* Is there a serial console? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_LM3S_UART0)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LM3S_UART1)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LM3S_UART2)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  warning "No valid CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we
 * still must provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#  define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#  ifdef CONFIG_LM3S_UART1
#    define TTYS1_DEV     g_uart1port     /* UART1 is ttyS1 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                    /* No ttyS2 */
#    endif
#  else
#    undef TTYS2_DEV                      /* No ttyS2 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS1_DEV   g_uart2port     /* UART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                    /* No ttyS1 */
#    endif
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#  define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#  ifdef CONFIG_LM3S_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is ttyS1 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                    /* No ttyS2 */
#    endif
#  else
#    undef TTYS2_DEV                      /* No ttyS2 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS1_DEV   g_uart2port     /* UART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                    /* No ttyS1 */
#    endif
#  endif
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port     /* UART2 is console */
#  define TTYS0_DEV       g_uart2port     /* UART2 is ttyS0 */
#  ifdef CONFIG_LM3S_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is ttyS1 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                    /* No ttyS2 */
#    endif
#  else
#    undef TTYS2_DEV                      /* No ttyS2 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS1_DEV   g_uart2port     /* UART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                    /* No ttyS1 */
#    endif
#  endif
#elifdefined(CONFIG_LM3S_UART0)
#  undef  CONSOLE_DEV                     /* No console device */
#  define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#  ifdef CONFIG_LM3S_UART1
#    define TTYS1_DEV     g_uart1port     /* UART1 is ttyS1 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                    /* No ttyS2 */
#    endif
#  else
#    undef TTYS2_DEV                      /* No ttyS2 */
#    ifdef CONFIG_LM3S_UART2
#      define TTYS1_DEV   g_uart2port     /* UART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                    /* No ttyS1 */
#    endif
#  endif
#elifdefined(CONFIG_LM3S_UART1)
#  undef  CONSOLE_DEV                     /* No console device */
#  define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#  undef TTYS2_DEV                        /* No ttyS2 */
#  ifdef CONFIG_LM3S_UART2
#    define TTYS1_DEV     g_uart2port     /* UART2 is ttyS1 */
#  else
#    undef TTYS1_DEV                      /* No ttyS1 */
#  endif
#elifdefined(CONFIG_LM3S_UART2)
#  undef  CONSOLE_DEV                     /* No console device */
#  define TTYS0_DEV       g_uart2port     /* UART2 is ttyS0 */
#  undef  TTYS1_DEV                       /* No ttyS1 */
#  undef  TTYS2_DEV                       /* No ttyS2 */
#else
#  error "No valid TTY devices"
#  undef  CONSOLE_DEV                     /* No console device */
#  undef  TTYS0_DEV                       /* No ttyS0 */
#  undef  TTYS1_DEV                       /* No ttyS1 */
#  undef  TTYS2_DEV                       /* No ttyS2 */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase; /* Base address of UART registers */
  uint32_t baud;     /* Configured baud */
  uint32_t im;       /* Saved IM value */
  uint8_t  irq;      /* IRQ associated with this UART */
  uint8_t  parity;   /* 0=none, 1=odd, 2=even */
  uint8_t  bits;     /* Number of bits (7 or 8) */
  bool    stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_LM3S_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_LM3S_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LM3S_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the LM3S uart0 port. */

#ifdef CONFIG_LM3S_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = LM3S_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = LM3S_IRQ_UART0,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the LM3S uart1 port. */

#ifdef CONFIG_LM3S_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LM3S_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = LM3S_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the LM3S uart1 port. */

#ifdef CONFIG_LM3S_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = LM3S_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = LM3S_IRQ_UART2,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *im)
{
  /* Return the current interrupt mask value */

  if (im)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
  up_serialout(priv, LM3S_UART_IM_OFFSET, 0);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t im)
{
  priv->im = im;
  up_serialout(priv, LM3S_UART_IM_OFFSET, im);
}

/****************************************************************************
 * Name: up_waittxnotfull
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline void up_waittxnotfull(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TX available condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check Tx FIFO is full */

      if ((up_serialin(priv, LM3S_UART_FR_OFFSET) & UART_FR_TXFF) == 0)
        {
          /* The Tx FIFO is not full... return */

          break;
        }
    }

  /* If we get here, then the wait has timed out and the Tx FIFO remains
   * full.
   */
}
#endif

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t lcrh;
  uint32_t ctl;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t den;
  uint32_t brdi;
  uint32_t remainder;
  uint32_t divfrac;

  /* Note:  The logic here depends on the fact that that the UART module
   * was enabled and the GPIOs were configured in up_lowsetup().
   */

  /* Disable the UART by clearing the UARTEN bit in the UART CTL register */

  ctl = up_serialin(priv, LM3S_UART_CTL_OFFSET);
  ctl &= ~UART_CTL_UARTEN;
  up_serialout(priv, LM3S_UART_CTL_OFFSET, ctl);

  /* Calculate BAUD rate from the SYS clock:
   *
   * "The baud-rate divisor is a 22-bit number consisting of a 16-bit integer
   *  and a 6-bit fractional part. The number formed by these two values is
   *  used by the baud-rate generator to determine the bit period. Having a
   *  fractional baud-rate divider allows the UART to generate all the standard
   *  baud rates.
   *
   * "The 16-bit integer is loaded through the UART Integer Baud-Rate Divisor
   *  (UARTIBRD) register ... and the 6-bit fractional part is loaded with the
   *  UART Fractional Baud-Rate Divisor (UARTFBRD) register... The baud-rate
   *  divisor (BRD) has the following relationship to the system clock (where
   *  BRDI is the integer part of the BRD and BRDF is the fractional part,
   *  separated by a decimal place.):
   *
   *    "BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
   *
   * "where UARTSysClk is the system clock connected to the UART. The 6-bit
   *  fractional number (that is to be loaded into the DIVFRAC bit field in the
   *  UARTFBRD register) can be calculated by taking the fractional part of the
   *  baud-rate divisor, multiplying it by 64, and adding 0.5 to account for
   *  rounding errors:
   *
   *    "UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
   *
   * "The UART generates an internal baud-rate reference clock at 16x the baud-
   *  rate (referred to as Baud16). This reference clock is divided by 16 to
   *  generate the transmit clock, and is used for error detection during receive
   *  operations.
   *
   * "Along with the UART Line Control, High Byte (UARTLCRH) register ..., the
   *  UARTIBRD and UARTFBRD registers form an internal 30-bit register. This
   *  internal register is only updated when a write operation to UARTLCRH is
   *  performed, so any changes to the baud-rate divisor must be followed by a
   *  write to the UARTLCRH register for the changes to take effect. ..."
   */

  den       = priv->baud << 4;
  brdi      = SYSCLK_FREQUENCY / den;
  remainder = SYSCLK_FREQUENCY - den * brdi;
  divfrac   = ((remainder << 6) + (den >> 1)) / den;

  up_serialout(priv, LM3S_UART_IBRD_OFFSET, brdi);
  up_serialout(priv, LM3S_UART_FBRD_OFFSET, divfrac);

  /* Set up the LCRH register */

  lcrh = 0;
  switch (priv->bits)
    {
      case 5:
        lcrh |= UART_LCRH_WLEN_5BITS;
        break;
      case 6:
        lcrh |= UART_LCRH_WLEN_6BITS;
        break;
      case 7:
        lcrh |= UART_LCRH_WLEN_7BITS;
        break;
      case 8:
      default:
        lcrh |= UART_LCRH_WLEN_8BITS;
        break;
    }

  switch (priv->parity)
    {
      case 0:
      default:
        break;
      case 1:
        lcrh |= UART_LCRH_PEN;
        break;
      case 2:
        lcrh |= UART_LCRH_PEN|UART_LCRH_EPS;
        break;
    }

  if (priv->stopbits2)
    {
      lcrh |= UART_LCRH_STP2;
    }

  up_serialout(priv, LM3S_UART_LCRH_OFFSET, lcrh);
#endif

  /* Set the UART to interrupt whenever the TX FIFO is almost empty or when
   * any character is received.
   */

  up_serialout(priv, LM3S_UART_IFLS_OFFSET, UART_IFLS_TXIFLSEL_18th|UART_IFLS_RXIFLSEL_18th);

  /* Flush the Rx and Tx FIFOs -- How do you do that?*/

  /* Enable Rx interrupts from the UART except for Tx interrupts.  We don't want
   * Tx interrupts until we have something to send.  We will check for serial
   * errors as part of Rx interrupt processing (no interrupts will be received
   * yet because the interrupt is still disabled at the interrupt controller.
   */

  up_serialout(priv, LM3S_UART_IM_OFFSET, UART_IM_RXIM|UART_IM_RTIM);

  /* Enable the FIFOs */

#ifdef CONFIG_SUPPRESS_UART_CONFIG
  lcrh = up_serialin(priv, LM3S_UART_LCRH_OFFSET);
#endif
  lcrh |= UART_LCRH_FEN;
  up_serialout(priv, LM3S_UART_LCRH_OFFSET, lcrh);

  /* Enable Rx, Tx, and the UART */

#ifdef CONFIG_SUPPRESS_UART_CONFIG
  ctl = up_serialin(priv, LM3S_UART_CTL_OFFSET);
#endif
  ctl |= (UART_CTL_UARTEN|UART_CTL_TXE|UART_CTL_RXE);
  up_serialout(priv, LM3S_UART_CTL_OFFSET, ctl);

  /* Set up the cache IM value */

  priv->im = up_serialin(priv, LM3S_UART_IM_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       up_enable_irq(priv->irq);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           mis;
  int                passes;
  bool               handled;

#ifdef CONFIG_LM3S_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_LM3S_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked UART status and clear the pending interrupts. */

       mis = up_serialin(priv, LM3S_UART_MIS_OFFSET);
       up_serialout(priv, LM3S_UART_ICR_OFFSET, mis);

      /* Handle incoming, receive bytes (with or without timeout) */

      if ((mis & (UART_MIS_RXMIS|UART_MIS_RTMIS)) != 0)
        {
           /* Rx buffer not empty ... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((mis & UART_MIS_TXMIS) != 0)
        {
           /* Tx FIFO not full ... process outgoing bytes */

           uart_xmitchars(dev);
           handled = true;
        }
    }
    return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t rxd;

  /* Get the Rx byte + 4 bits of error information.  Return those in status */

  rxd     = up_serialin(priv, LM3S_UART_DR_OFFSET);
  *status = rxd;

  /* The lower 8bits of the Rx data is the actual recevied byte */

  return rxd & 0xff;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx FIFO (or an Rx
       * timeout occurs.
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= (UART_IM_RXIM|UART_IM_RTIM);
#endif
    }
  else
    {
      priv->im &= ~(UART_IM_RXIM|UART_IM_RTIM);
    }
  up_serialout(priv, LM3S_UART_IM_OFFSET, priv->im);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LM3S_UART_FR_OFFSET) & UART_FR_RXFE) == 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, LM3S_UART_DR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX fifo is half emptied */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= UART_IM_TXIM;
      up_serialout(priv, LM3S_UART_IM_OFFSET, priv->im);

      /* The serial driver wants an interrupt here, but will not get get
       * one unless we "prime the pump."  I believe that this is because
       * behave like a level interrupt and the LM3S interrupts behave
       * (at least by default) like edge interrupts.
       *
       * In any event, faking a TX interrupt here solves the problem;
       * Call uart_xmitchars() just as would have been done if we recieved
       * the TX interrupt.
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~UART_IM_TXIM;
      up_serialout(priv, LM3S_UART_IM_OFFSET, priv->im);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LM3S_UART_FR_OFFSET) & UART_FR_TXFF) == 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LM3S_UART_FR_OFFSET) & UART_FR_TXFE) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * up_lowsetup
   */

  /* Disable all UARTS */

  up_disableuartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableuartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableuartint(TTYS2_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
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

#ifdef HAVE_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint32_t im;

  up_disableuartint(priv, &im);
  up_waittxnotfull(priv);
  up_serialout(priv, LM3S_UART_DR_OFFSET, (uint32_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxnotfull(priv);
      up_serialout(priv, LM3S_UART_DR_OFFSET, (uint32_t)'\r');
    }

  up_waittxnotfull(priv);
  up_restoreuartint(priv, im);
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
#ifdef HAVE_CONSOLE
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
