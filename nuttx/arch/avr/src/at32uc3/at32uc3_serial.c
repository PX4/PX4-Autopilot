/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_serial.c
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "at32uc3_config.h"
#include "chip.h"
#include "at32uc3_usart.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "at32uc3_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there at least one USART enabled and configured as a RS-232 device? */

#ifndef HAVE_RS232_DEVICE
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
#  ifdef CONFIG_AVR32_USART1_RS232
#    define TTYS1_DEV     g_usart1port     /* USART1 is ttyS1 */
#    ifdef CONFIG_AVR32_USART2_RS232
#      define TTYS2_DEV   g_usart2port     /* USART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_AVR32_USART2_RS232
#      define TTYS1_DEV   g_usart2port     /* USART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port     /* USART1 is console */
#  define TTYS0_DEV       g_usart1port     /* USART1 is ttyS0 */
#  ifdef CONFIG_AVR32_USART0_RS232
#    define TTYS1_DEV     g_usart0port     /* USART0 is ttyS1 */
#    ifdef CONFIG_AVR32_USART2_RS232
#      define TTYS2_DEV   g_usart2port     /* USART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_AVR32_USART2_RS232
#      define TTYS1_DEV   g_usart2port     /* USART2 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart2port     /* USART2 is console */
#  define TTYS0_DEV       g_usart2port     /* USART2 is ttyS0 */
#  ifdef CONFIG_AVR32_USART0_RS232
#    define TTYS1_DEV     g_usart0port     /* USART0 is ttyS1 */
#    ifdef CONFIG_AVR32_USART1_RS232
#      define TTYS2_DEV   g_usart1port     /* USART1 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_AVR32_USART1_RS232
#      define TTYS1_DEV   g_usart1port     /* USART1 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t usartbase; /* Base address of USART registers */
  uint32_t  baud;      /* Configured baud */
  uint32_t  csr;       /* Saved channel status register contents */
  uint8_t   irq;       /* IRQ associated with this USART */
  uint8_t   parity;    /* 0=none, 1=odd, 2=even */
  uint8_t   bits;      /* Number of bits (5, 6, 7 or 8) */
  bool      stopbits2; /* true: Configure with 2 stop bits instead of 1 */
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
  .txempty        = up_txready,
};

/* I/O buffers */

#ifdef CONFIG_AVR32_USART0_RS232
static char g_usart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_usart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_AVR32_USART1_RS232
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_AVR32_USART2_RS232
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif

/* This describes the state of the AVR32 USART0 ports. */

#ifdef CONFIG_AVR32_USART0_RS232
static struct up_dev_s g_usart0priv =
{
  .usartbase      = AVR32_USART0_BASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = AVR32_IRQ_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
};

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
  .ops      = &g_uart_ops,
  .priv     = &g_usart0priv,
};
#endif

/* This describes the state of the AVR32 USART1 port. */

#ifdef CONFIG_AVR32_USART1_RS232
static struct up_dev_s g_usart1priv =
{
  .usartbase      = AVR32_USART1_BASE,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = AVR32_IRQ_USART1,
  .parity         = CONFIG_USART1_PARITY,
  .bits           = CONFIG_USART1_BITS,
  .stopbits2      = CONFIG_USART1_2STOP,
};

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
  .ops      = &g_uart_ops,
  .priv     = &g_usart1priv,
};
#endif

/* This describes the state of the AVR32 USART2 port. */

#ifdef CONFIG_AVR32_USART2_RS232
static struct up_dev_s g_usart2priv =
{
  .usartbase      = AVR32_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = AVR32_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
};

static uart_dev_t g_usart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_usart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_usart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart2priv,
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
  return getreg32(priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void up_restoreusartint(struct up_dev_s *priv, uint32_t imr)
{
  /* Re-enable interrupts as for each "1" bit in imr */

  up_serialout(priv, AVR32_USART_IER_OFFSET, imr);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static inline void up_disableusartint(struct up_dev_s *priv, uint32_t *imr)
{
  if (imr)
    {
      *imr = up_serialin(priv, AVR32_USART_IMR_OFFSET);
    }

  /* Disable all interrupts */

  up_serialout(priv, AVR32_USART_IDR_OFFSET, USART_INT_ALL);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Configure the USART as an RS-232 UART */

  usart_configure(priv->usartbase, priv->baud, priv->parity,
                  priv->bits, priv->stopbits2);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Reset, disable interrupts, and disable Rx and Tx */

  usart_reset(priv->usartbase);
}

/****************************************************************************
 * Name: up_attach
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

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Attach the IRQ */

  return irq_attach(priv->irq, up_interrupt);
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach USART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, AVR32_USART_IDR_OFFSET, 0xffffffff);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           csr;
  int                passes;
  bool               handled;

#ifdef CONFIG_AVR32_USART0_RS232
  if (g_usart0priv.irq == irq)
    {
      dev = &g_usart0port;
    }
  else
#endif
#ifdef CONFIG_AVR32_USART1_RS232
  if (g_usart1priv.irq == irq)
    {
      dev = &g_usart1port;
    }
  else
#endif
#ifdef CONFIG_AVR32_USART2_RS232
  if (g_usart2priv.irq == irq)
    {
      dev = &g_usart2port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;
  DEBUGASSERT(priv);

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the USART channel status register contents. */

      csr       = up_serialin(priv, AVR32_USART_CSR_OFFSET);
      priv->csr = csr;

      /* Handle incoming, receive bytes (with or without timeout) */

      if ((csr & (USART_CSR_RXRDY|USART_CSR_TIMEOUT)) != 0)
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((csr & USART_CSR_TXRDY) != 0)
        {
           /* Transmit data regiser empty ... process outgoing bytes */

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
#if 0 /* Reserved for future growth */
  struct inode      *inode;
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  int                ret = OK;

  DEBUGASSERT(filep, filep->f_inode);
  inode = filep->f_inode;
  dev   = inode->i_private;

  DEBUGASSERT(dev, dev->priv)
  priv = (struct up_dev_s*)dev->priv;

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

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t rhr;

  /* Get the Rx byte.  The USART Rx interrupt flag is cleared by side effect
   * when reading the received character.
   */

  rhr = up_serialin(priv, AVR32_USART_RHR_OFFSET);

  /* Return status information */

  if (status)
    {
      *status = priv->csr;
    }
  priv->csr = 0;

  /* Then return the actual received byte */

  return rhr & USART_RHR_RXCHR_MASK;
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
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#  ifdef CONFIG_USART_ERRINTS
        up_serialout(priv, AVR32_USART_IER_OFFSET,
                     USART_INT_RXRDY|USART_INT_TIMEOUT|
                     USART_INT_OVRE|USART_INT_FRAME|USART_INT_PARE);
#  else
        up_serialout(priv, AVR32_USART_IER_OFFSET,
                     USART_INT_RXRDY|USART_INT_TIMEOUT);
#  endif
#endif
    }
  else
    {
        up_serialout(priv, AVR32_USART_IDR_OFFSET,
                     USART_INT_RXRDY|USART_INT_TIMEOUT|
                     USART_INT_OVRE|USART_INT_FRAME|USART_INT_PARE);
    }
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t regval;

  /* Read the channel status register and check if character is available to
   * be read from the RHR.
   */

  regval = up_serialin(priv, AVR32_USART_CSR_OFFSET);
  return (regval & USART_CSR_RXRDY) != 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the USART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, AVR32_USART_THR_OFFSET, (uint32_t)ch);
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
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
       up_serialout(priv, AVR32_USART_IER_OFFSET, USART_INT_TXRDY);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

       up_serialout(priv, AVR32_USART_IDR_OFFSET, USART_INT_TXRDY);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t regval;

  /* Read the channel status register and check if THR is ready to accept
   * another character.
   */

  regval = up_serialin(priv, AVR32_USART_CSR_OFFSET);
  return (regval & USART_CSR_TXRDY) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Disable all USARTS */

  up_disableusartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableusartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableusartint(TTYS2_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
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

#ifdef HAVE_SERIAL_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */

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
#ifdef HAVE_SERIAL_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint32_t imr;

  up_disableusartint(priv, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, imr);
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

