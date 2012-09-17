/****************************************************************************
 * arch/arm/src/str71x/str71x_serial.c
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

#include "str71x_internal.h"

/****************************************************************************
 * Pre-procesor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there a UART enabled? */

#if !defined(CONFIG_STR71X_UART0) && !defined(CONFIG_STR71X_UART1) && \
    !defined(CONFIG_STR71X_UART2) && !defined(CONFIG_STR71X_UART3)
#  error "No UARTs enabled"
#endif

/* Is there a serial console? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE) ||\
   defined(CONFIG_UART2_SERIAL_CONSOLE) || defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define HAVE_CONSOLE 1
#else
#  undef HAVE_CONSOLE
#endif

/* Did the user select a priority? */

#ifndef CONFIG_UART_PRI
#  define CONFIG_UART_PRI 1
#elif CONFIG_UART_PRI <= 1 || CONFIG_UART_PRI > 15
#  error "CONFIG_UART_PRI is out of range"
#endif

/* If we are not using the serial driver for the console, then we
 * still must provide some minimal implementation of up_putc().
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  tty2? tty3? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) || !defined(HAVE_CONSOLE)
#  ifdef HAVE_CONSOLE
#    ifndef CONFIG_STR71X_UART0
#      error "UART0 not selected, cannot be console"
#    endif
#    define CONSOLE_DEV   g_uart0port     /* UART0 is console */
#  endif
#  define TTYS0_DEV       g_uart0port     /* UART0 is tty0 */
#  if CONFIG_STR71X_UART1
#    define TTYS1_DEV     g_uart1port     /* UART1 is tty1 */
#    if CONFIG_STR71X_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#      if CONFIG_STR71X_UART3
#        define TTYS3_DEV g_uart3port     /* UART3 is tty3 */
#      endif
#    elif CONFIG_STR71X_UART3
#      define TTYS2_DEV   g_uart3port     /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART2
#    define TTYS1_DEV   g_uart2port       /* UART2 is tty1 */
#    if CONFIG_STR71X_UART3
#      define TTYS2_DEV g_uart3port       /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART3
#    define TTYS1_DEV   g_uart3port       /* UART3 is tty1 */
#  endif
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  ifndef CONFIG_STR71X_UART1
#    error "UART1 not selected, cannot be console"
#  endif
#  define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#  define TTYS0_DEV       g_uart1port     /* UART1 is tty0 */
#  if CONFIG_STR71X_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is tty1 */
#    if CONFIG_STR71X_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#      if CONFIG_STR71X_UART3
#        define TTYS3_DEV g_uart3port     /* UART3 is tty3 */
#      endif
#    elif CONFIG_STR71X_UART3
#      define TTYS2_DEV   g_uart3port     /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART2
#    define TTYS1_DEV   g_uart2port       /* UART2 is tty1 */
#    if CONFIG_STR71X_UART3
#      define TTYS2_DEV g_uart3port       /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART3
#    define TTYS1_DEV   g_uart3port       /* UART3 is tty1 */
#  endif
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  ifndef CONFIG_STR71X_UART2
#    error "UART2 not selected, cannot be console"
#  endif
#  define CONSOLE_DEV     g_uart2port     /* UART2 is console */
#  define TTYS0_DEV       g_uart2port     /* UART2 is tty0 */
#  if CONFIG_STR71X_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is tty1 */
#    if CONFIG_STR71X_UART1
#      define TTYS2_DEV   g_uart1port     /* UART1 is tty2 */
#      if CONFIG_STR71X_UART3
#        define TTYS3_DEV g_uart3port     /* UART3 is tty3 */
#      endif
#    elif CONFIG_STR71X_UART3
#      define TTYS2_DEV   g_uart3port     /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART1
#    define TTYS1_DEV   g_uart1port       /* UART1 is tty1 */
#    if CONFIG_STR71X_UART3
#      define TTYS2_DEV g_uart3port       /* UART3 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART3
#    define TTYS1_DEV   g_uart3port       /* UART3 is tty1 */
#  endif
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  ifndef CONFIG_STR71X_UART3
#    error "UART3 not selected, cannot be console"
#  endif
#  define CONSOLE_DEV     g_uart3port     /* UART3 is console */
#  define TTYS0_DEV       g_uart3port     /* UART3 is tty0 */
#  if CONFIG_STR71X_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is tty1 */
#    if CONFIG_STR71X_UART1
#      define TTYS2_DEV   g_uart1port     /* UART1 is tty2 */
#      if CONFIG_STR71X_UART2
#        define TTYS3_DEV g_uart2port     /* UART2 is tty3 */
#      endif
#    elif CONFIG_STR71X_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART1
#    define TTYS1_DEV   g_uart1port       /* UART1 is tty1 */
#    if CONFIG_STR71X_UART2
#      define TTYS2_DEV g_uart2port       /* UART2 is tty2 */
#    endif
#  elif CONFIG_STR71X_UART2
#    define TTYS1_DEV   g_uart2port       /* UART2 is tty1 */
#  endif
#else
#  warning "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Select RX interrupt enable bits.  There are two models:  (1) We interrupt
 * when each character is received. Or, (2) we interrupt when either the Rx
 * FIFO is half full, OR a timeout occurs with data in the RX FIFO.  The
 * later does not work because there seems to be a disconnect -- we can get
 * the FIFO half full interrupt with no data in the RX buffer.
 */

#if 1
#  define RXENABLE_BITS (STR71X_UARTIER_RHF|STR71X_UARTIER_TIMEOUTNE)
#else
#  define RXENABLE_BITS STR71X_UARTIER_RNE
#endif

/* Which ever model is used, there seems to be some timing disconnects between
 * Rx FIFO not full and Rx FIFO half full indications.  Best bet is to use
 * both.
 */

#define RXAVAILABLE_BITS (STR71X_UARTSR_RNE|STR71X_UARTSR_RHF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint16_t ier;       /* Saved IER value */
  uint16_t sr;        /* Saved SR value (only used during interrupt processing) */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal Helpers */

static inline uint16_t up_serialin(struct up_dev_s *priv, int offset);
static inline void up_serialout(struct up_dev_s *priv, int offset, uint16_t value);
static inline void up_disableuartint(struct up_dev_s *priv, uint16_t *ier);
static inline void up_restoreuartint(struct up_dev_s *priv, uint16_t ier);
#ifdef HAVE_CONSOLE
static inline void up_waittxnotfull(struct up_dev_s *priv);
#endif

/* Serial Driver Methods */

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

#ifdef CONFIG_STR71X_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_STR71X_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_STR71X_UART2
static char g_uart2rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_STR71X_UART3
static char g_uart3rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

/* This describes the state of the STR71X uart0 port. */

#ifdef CONFIG_STR71X_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = STR71X_UART0_BASE,
  .baud           = CONFIG_UART0_BAUD,
  .irq            = STR71X_IRQ_UART0,
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

/* This describes the state of the STR71X uart1 port. */

#ifdef CONFIG_STR71X_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = STR71X_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = STR71X_IRQ_UART1,
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

/* This describes the state of the STR71X uart2 port. */

#ifdef CONFIG_STR71X_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = STR71X_UART2_BASE,
  .baud           = CONFIG_UART2_BAUD,
  .irq            = STR71X_IRQ_UART2,
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

/* This describes the state of the STR71X uart3 port. */

#ifdef CONFIG_STR71X_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = STR71X_UART3_BASE,
  .baud           = CONFIG_UART3_BAUD,
  .irq            = STR71X_IRQ_UART3,
  .parity         = CONFIG_UART3_PARITY,
  .bits           = CONFIG_UART3_BITS,
  .stopbits2      = CONFIG_UART3_2STOP,
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_UART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint16_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg16(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint16_t value)
{
  putreg16(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint16_t *ier)
{
  if (ier)
    {
      *ier = priv->ier;
    }

  priv->ier = 0;
  up_serialout(priv, STR71X_UART_IER_OFFSET, 0);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint16_t ier)
{
  priv->ier = ier;
  up_serialout(priv, STR71X_UART_IER_OFFSET, ier);
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
      /* Check TX FIFO is full */

      if ((up_serialin(priv, STR71X_UART_SR_OFFSET) & STR71X_UARTSR_TF) == 0)
        {
          /* The TX FIFO is not full... return */
          break;
        }
    }
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
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t divisor;
  uint32_t baud;
  uint16_t cr;

  /* Set the BAUD rate */

  divisor = 16 * priv->baud;
  baud    =  (STR71X_PCLK1 + divisor/2) / divisor;
  up_serialout(priv, STR71X_UART_BR_OFFSET, baud);

  /* Get mode setting */

  cr = STR71X_UARTCR_RUN|STR71X_UARTCR_RXENABLE|STR71X_UARTCR_FIFOENABLE;

  if (priv->bits == 7)
    {
      DEBUGASSERT(priv->parity != 0);
      cr |= STR71X_UARTCR_MODE7BITP;
    }
  else if (priv->bits == 8)
    {
      if (priv->parity)
        {
          cr |= STR71X_UARTCR_MODE8BITP;
        }
      else
        {
          cr |= STR71X_UARTCR_MODE8BIT;
        }
    }
  else
    {
      DEBUGASSERT(priv->bits == 9 && priv->parity == 0);
      cr |= STR71X_UARTCR_MODE9BIT;
    }

  if (priv->parity == 1)
    {
      cr |= STR71X_UARTCR_PARITYODD;
    }

  if (priv->stopbits2)
    {
      cr |= STR71X_UARTCR_STOPBIT20;
    }
  else
    {
      cr |= STR71X_UARTCR_STOPBIT10;
    }

  up_serialout(priv, STR71X_UART_CR_OFFSET, cr);

  /* Clear FIFOs */

  up_serialout(priv, STR71X_UART_TXRSTR_OFFSET, 0xffff);
  up_serialout(priv, STR71X_UART_RXRSTR_OFFSET, 0xffff);

  /* We will take RX interrupts on either the FIFO half full or upon
   * a timeout.  The timeout is based upon BAUD rate ticks
   */

  up_serialout(priv, STR71X_UART_TOR_OFFSET, 50);

  /* Set up the IER */

  priv->ier = up_serialin(priv, STR71X_UART_IER_OFFSET);
#endif
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

       /* Set the uart interrupt priority (the default value is one) */

       up_prioritize_irq(priv->irq, CONFIG_UART_PRI);
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
  int                passes;
  bool               handled;

#ifdef CONFIG_STR71X_UART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_STR71X_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_STR71X_UART2
  if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
#endif
#ifdef CONFIG_STR71X_UART3
  if (g_uart3priv.irq == irq)
    {
      dev = &g_uart3port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }

  priv = (struct up_dev_s*)dev->priv;
  DEBUGASSERT(priv && dev);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the current UART status  */

       priv->sr = up_serialin(priv, STR71X_UART_SR_OFFSET);

      /* Handle incoming, receive bytes (with or without timeout) */

      if ((priv->sr  & RXAVAILABLE_BITS) != 0 && /* Data available in Rx FIFO */
          (priv->ier & RXENABLE_BITS)    != 0)   /* Rx FIFO interrupts enabled */
        {
           /* Rx buffer not empty ... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & STR71X_UARTSR_TF)    == 0 && /* Tx FIFO not full */
          (priv->ier & STR71X_UARTIER_THE) != 0)   /* Tx Half empty interrupt enabled */
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
  uint16_t rxbufr;

  rxbufr  = up_serialin(priv, STR71X_UART_RXBUFR_OFFSET);
  *status = (uint32_t)priv->sr << 16 | rxbufr;
  return rxbufr & 0xff;
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
      /* Receive an interrupt when the Rx FIFO is half full (or if a timeout
       * occurs while the Rx FIFO is not empty).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= RXENABLE_BITS;
#endif
    }
  else
    {
      priv->ier &= ~RXENABLE_BITS;
    }
  up_serialout(priv, STR71X_UART_IER_OFFSET, priv->ier);
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
  return ((up_serialin(priv, STR71X_UART_SR_OFFSET) & RXAVAILABLE_BITS) != 0);
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
  up_serialout(priv, STR71X_UART_TXBUFR_OFFSET, (uint16_t)ch);
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
  if (enable)
    {
      /* Set to receive an interrupt when the TX fifo is half emptied */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= STR71X_UARTSR_THE;
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->ier &= ~STR71X_UARTSR_THE;
    }
  up_serialout(priv, STR71X_UART_IER_OFFSET, priv->ier);
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
  return ((up_serialin(priv, STR71X_UART_SR_OFFSET) & STR71X_UARTSR_TF) == 0);
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
  return ((up_serialin(priv, STR71X_UART_SR_OFFSET) & STR71X_UARTSR_TE) != 0);
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
#ifdef TTYS3_DEV
  up_disableuartint(TTYS3_DEV.priv, NULL);
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
#ifdef TTYS3_DEV
  (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
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
  uint16_t ier;

  up_disableuartint(priv, &ier);
  up_waittxnotfull(priv);
  up_serialout(priv, STR71X_UART_TXBUFR_OFFSET, (uint16_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxnotfull(priv);
      up_serialout(priv, STR71X_UART_TXBUFR_OFFSET, (uint16_t)'\r');
    }

  up_waittxnotfull(priv);
  up_restoreuartint(priv, ier);
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
