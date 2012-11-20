/****************************************************************************
 * arch/arm/src/imx/imx_serial.c
 * arch/arm/src/chip/imx_serial.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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
#include "imx_gpio.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The i.MXL chip has only two UARTs */

#if defined(CONFIG_ARCH_CHIP_IMXL) && defined(CONFIG_IMX_UART3)
#  undef CONFIG_IMX_UART3
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;	/* Base address of UART registers */
  uint32_t baud;		/* Configured baud */
  uint32_t ucr1;		/* Saved UCR1 value */
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  uint8_t  rxirq;		/* Rx IRQ associated with this UART */
  uint8_t  txirq;		/* Tx IRQ associated with this UART */
#else
  uint8_t  irq;			/* IRQ associated with this UART */
#endif
  uint8_t  parity;		/* 0=none, 1=odd, 2=even */
  uint8_t  bits;		/* Number of bits (7 or 8) */
  uint8_t  stopbits2:1;	/* 1: Configure with 2 stop bits vs 1 */
  uint8_t  hwfc:1;		/* 1: Hardware flow control */
  uint8_t  reserved:6;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, uint32_t offset);
static inline void up_serialout(struct up_dev_s *priv, uint32_t offset, uint32_t value);
static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ucr1);
static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ucr1);
static inline void up_waittxready(struct up_dev_s *priv);

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static inline struct uart_dev_s *up_mapirq(int irq);
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

#ifdef CONFIG_IMX_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

#ifdef CONFIG_IMX_UART3
static char g_uart3rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the IMX uart1 port. */

#ifdef CONFIG_IMX_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = IMX_UART1_VBASE,
  .baud           = CONFIG_UART1_BAUD,
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  .rxirq          = IMX_IRQ_UART1RX,
  .txirq          = IMX_IRQ_UART1TX,
#else
  .irq            = IMX_IRQ_UART1,
#endif
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

/* This describes the state of the IMX uart2 port. */

#ifdef CONFIG_IMX_UART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = IMX_UART2_VBASE,
  .baud           = CONFIG_UART2_BAUD,
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  .rxirq          = IMX_IRQ_UART2RX,
  .txirq          = IMX_IRQ_UART2TX,
#else
  .irq            = IMX_IRQ_UART2,
#endif
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

#ifdef CONFIG_IMX_UART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = IMX_UART3_REGISTER_BASE,
  .baud           = IMX_UART3_VBASE,
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  .rxirq          = IMX_IRQ_UART3RX,
  .txirq          = IMX_IRQ_UART3TX,
#else
  .irq            = IMX_IRQ_UART3,
#endif
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

/* Now, which one with be tty0/console and which tty1 and tty2? */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_IMX_UART1)
# define CONSOLE_DEV     g_uart1port /* UART1 is /dev/console */
# undef  CONFIG_UART2_SERIAL_CONSOLE
# undef  CONFIG_UART3_SERIAL_CONSOLE
# define TTYS0_DEV       g_uart1port /* UART1 is /dev/ttyS0 */
# if defined(CONFIG_IMX_UART2)
#   define TTYS1_DEV     g_uart2port /* UART2 is /dev/ttyS1 */
#   if defined(CONFIG_IMX_UART3)
#     define TTYS2_DEV   g_uart3port /* UART3 is /dev/ttyS2 */
#   else
#     undef TTYS2_DEV                /* No /dev/ttyS2 */
#   endif
# elif defined(CONFIG_IMX_UART3)
#   define TTYS1_DEV     g_uart3port /* UART3 is /dev/ttyS1 */
#   undef  TTYS2_DEV                 /* No /dev/ttyS2 */
# else
#   undef  TTYS1_DEV                 /* No /dev/ttyS1 */
#   undef  TTYS2_DEV                 /* No /dev/ttyS2 */
# endif

#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_IMX_UART2)
# define CONSOLE_DEV     g_uart2port  /* UART2 is /dev/console */
# undef  CONFIG_UART1_SERIAL_CONSOLE
# undef  CONFIG_UART3_SERIAL_CONSOLE
# define TTYS0_DEV       g_uart2port  /* UART2 is /dev/ttyS0 */
# if defined(CONFIG_IMX_UART1)
#   define TTYS1_DEV     g_uart1port  /* UART1 is /dev/ttyS1 */
#   if defined(CONFIG_IMX_UART3)
#     define TTYS2_DEV   g_uart3port  /* UART3 is /dev/ttyS2 */
#   else
#     undef TTYS2_DEV                 /* No /dev/ttyS2 */
#   endif
# elif defined(CONFIG_IMX_UART3)
#   define TTYS1_DEV     g_uart3port  /* UART3 is /dev/ttyS1 */
# else
#   undef TTYS1_DEV                   /* No /dev/ttyS1 */
#   undef TTYS2_DEV                   /* No /dev/ttyS2 */
# endif

#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_IMX_UART3)
# define CONSOLE_DEV     g_uart3port  /* UART3 is /dev/console */
# undef CONFIG_UART1_SERIAL_CONSOLE
# undef CONFIG_UART2_SERIAL_CONSOLE
# define TTYS0_DEV       g_uart3port  /* UART3 is /dev/ttyS0 */
# if defined(CONFIG_IMX_UART1)
#   define TTYS1_DEV     g_uart1port  /* UART1 is /dev/ttyS1 */
#   if defined(CONFIG_IMX_UART2)
#     define TTYS2_DEV   g_uart2port  /* UART2 is /dev/ttyS2 */
#   else
#     undef TTYS2_DEV                 /* No /dev/ttyS2 */
#   endif
# elif defined(CONFIG_IMX_UART2)
#   define TTYS1_DEV     g_uart2port  /* UART2 is /dev/ttyS1 */
#   undef  TTYS2_DEV                  /* No /dev/ttyS2 */
# else
#   undef  TTYS1_DEV                  /* No /dev/ttyS1 */
#   undef  TTYS2_DEV                  /* No /dev/ttyS2 */
# endif

#else
# undef CONSOLE_DEV     g_uart1port   /* No /dev/console */
# undef CONFIG_UART1_SERIAL_CONSOLE
# undef CONFIG_UART2_SERIAL_CONSOLE
# undef CONFIG_UART3_SERIAL_CONSOLE

# if defined(CONFIG_IMX_UART1)
#  define TTYS0_DEV       g_uart1port /* UART1 is /dev/ttyS0 */
#  if defined(CONFIG_IMX_UART2)
#    define TTYS1_DEV     g_uart2port /* UART2 is /dev/ttyS1 */
#    if defined(CONFIG_IMX_UART3)
#     define TTYS2_DEV   g_uart3port  /* UART3 is /dev/ttyS2 */
#    else
#     undef TTYS2_DEV                 /* No /dev/ttyS2 */
#    endif
#  elif defined(CONFIG_IMX_UART3)
#    define TTYS1_DEV     g_uart3port /* UART3 is /dev/ttyS1 */
#    undef TTYS2_DEV                  /* No /dev/ttyS2 */
#  else
#    undef TTYS1_DEV                  /* No /dev/ttyS1 */
#    undef TTYS2_DEV                  /* No /dev/ttyS2 */
#  endif

# elif defined(CONFIG_IMX_UART2)
#  define TTYS0_DEV       g_uart2port /* UART2 is /dev/ttyS0 */
#  undef  TTYS2_DEV                   /* No /dev/ttyS2 */
#  if defined(CONFIG_IMX_UART3)
#    define TTYS1_DEV     g_uart2port /* UART2 is /dev/ttyS1 */
#  else
#    undef TTYS1_DEV                  /* No /dev/ttyS1 */
#  endif

# elif defined(CONFIG_IMX_UART3)
#  define TTYS0_DEV       g_uart3port /* UART3 is /dev/ttyS0 */
#  undef  TTYS1_DEV                   /* No /dev/ttyS1 */
#  undef  TTYS2_DEV                   /* No /dev/ttyS2 */

# else
#  error "No UARTs enabled"
#  undef  TTYS0_DEV                   /* No /dev/ttyS0 */
#  undef  TTYS1_DEV                   /* No /dev/ttyS1 */
#  undef  TTYS2_DEV                   /* No /dev/ttyS2 */
# endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ucr1)
{
  /* Return the current Rx and Tx interrupt state */

  if (ucr1)
    {
      *ucr1 = priv->ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
    }

  /* Then disable both Rx and Tx interrupts */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  up_serialout(priv, UART_UCR1, priv->ucr1);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ucr1)
{
  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  priv->ucr1 &= ~(UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  priv->ucr1 |= ucr1 & (UART_UCR1_RRDYEN | UART_UCR1_TXEMPTYEN);
  up_serialout(priv, UART_UCR1, priv->ucr1);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((up_serialin(priv, UART_UTS) & UART_UTS_TXFULL) == 0)
        {
          break;
        }
    }
}

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
  uint32_t regval;
  uint32_t ucr2;
  uint32_t div;
  uint32_t num;
  uint32_t den;

  /* Disable the UART */

  up_serialout(priv, UART_UCR1, 0);
  up_serialout(priv, UART_UCR2, 0);
  up_serialout(priv, UART_UCR3, 0);
  up_serialout(priv, UART_UCR4, 0);

  /* Set up UCR2 */

  ucr2  = up_serialin(priv, UART_UCR2);
  ucr2 |= (UART_UCR2_SRST | UART_UCR2_IRTS)

  /* Select the number of data bits */

  DEBUGASSERT(priv->bits == 7 || priv->bits == 8);
  if (priv->bits == 8)
    {
      ucr2 |= UART_UCR2_WS;
    }

  /* Select the number of stop bits */

  if (priv->stopbits2)
    {
      ucr2 |= UART_UCR2_STPB;
    }

  /* Select even/odd parity */

  if (priv->parity)
    {
      DEBUGASSERT(priv->parity == 1 || priv->parity == 2);
      ucr2 |= UART_UCR2_PREN;
      if (priv->parity == 1)
        {
          ucr2 |= UART_UCR2_PROE;
        }
    }

  /* Select RTS */

#if 0
  ucr2 &= ~UCR2_IRTS;
  ucr2 |= UCR2_CTSC;
#endif

  /* Setup hardware flow control */

  regval = 0;
#if 0
  if (priv->hwfc)
    {
      ucr2 |= UART_UCR2_IRTS;
 
      /* CTS controled by Rx FIFO */

      ucr2 |= UART_UCR2_CTSC;

      /* Set CTS trigger level */

      regval |= 30 << UART_UCR4_CTSTL_SHIFT;
   }
#endif

  /* i.MX reference clock (PERCLK1) is configured for 16MHz */

  up_serialout(priv, UART_UCR4, regval | UART_UCR4_REF16);

  /* Setup the new UART configuration */

  up_serialout(priv, UART_UCR2, ucr2);

  /* Set the baud.  
   *
   *   baud * 16 / REFFREQ = NUM/DEN
   *   UBIR    = NUM-1;
   *   UMBR    = DEN-1
   *   REFFREQ = PERCLK1 / DIV
   *   DIV     = RFDIV[2:0]
   *
   * First, select a closest value we can for the divider
   */

   div = (IMX_PERCLK1_FREQ >> 4) / priv->baud;
   if (div > 7)
     {
       div = 7;
     }
   else if (div < 1)
     {
       div = 1;
     }

  /* Now find the numerator and denominator.  These must have
   * the ratio baud/(PERCLK / div / 16), but the values cannot
   * exceed 16 bits
   */

  num = priv->baud;
  den = (IMX_PERCLK1_FREQ << 4) / div;

  if (num > den)
    {
      if (num > 0x00010000)
        {
          /* b16 is a scale such that b16*num = 0x10000 * 2**16 */

          uint32_t b16 = 0x100000000LL / num;
          num = 0x00010000;
          den = (b16 * den) >> 16;
        }
    }
  else
    {
      if (den > 0x0000ffff)
        {
          /* b16 is a scale such that b16*den = 0x10000 * 2**16 */

          uint32_t b16 = 0x100000000LL / den;
          num = (b16 * num) >> 16;
          den = 0x00010000;
        }
    }

  /* The actual values are we write to the registers need to be
   * decremented by 1.
   */

  if (num > 0)
    {
      num--;
    }

  if (den > 0)
    {
      den--;
    }

  /* The UBIR must be set before the UBMR register */

  up_serialout(priv, UART_UBIR, num);
  up_serialout(priv, UART_UBMR, den);

  /* Fixup the divisor, the value in the UFCR regiser is
   *
   *   000 = Divide input clock by 6
   *   001 = Divide input clock by 5
   *   010 = Divide input clock by 4
   *   011 = Divide input clock by 3
   *   100 = Divide input clock by 2
   *   101 = Divide input clock by 1
   *   110 = Divide input clock by 7
   */

  if (div == 7)
    {
      div = 6;
    }
  else
    {
      div = 6 - div;
    }
  regval = div << UART_UFCR_RFDIV_SHIFT;

  /* Set the TX trigger level to interrupt when the TxFIFO has 2 or fewer characters.
   * Set the RX trigger level to interrupt when the RxFIFO has 1 character.
   */
 
  regval |= ((2 << UART_UFCR_TXTL_SHIFT) | (1 << UART_UFCR_RXTL_SHIFT));
  up_serialout(priv, UART_UFCR, regval);

  /* Initialize the UCR1 shadow register */

  priv->ucr1 = up_serialin(priv, UART_UCR1);

  /* Enable the UART
   *
   *  UART_UCR1_UARTCLEN = Enable UART clocking
   */

  ucr2 |= (UART_UCR2_TXEN | UART_UCR2_RXEN);
  up_serialout(priv, UART_UCR1, ucr2);

  priv->ucr1 |= UART_UCR1_UARTCLEN;
  up_serialout(priv, UART_UCR1, priv->ucr1);
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

  /* Disable the UART */

  up_serialout(priv, UART_UCR1, 0);
  up_serialout(priv, UART_UCR2, 0);
  up_serialout(priv, UART_UCR3, 0);
  up_serialout(priv, UART_UCR4, 0);
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

#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  ret = irq_attach(priv->rxirq, up_interrupt);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(priv->txirq, up_interrupt);
  if (ret < 0)
    {
      irq_detach(priv->rxirq);
      return ret;
    }

  /* Enable the interrupts (interrupts are still disabled in the UART) */

  up_enable_irq(priv->rxirq);
  up_enable_irq(priv->txirq);

#else
  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       up_enable_irq(priv->irq);
    }
#endif
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

#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
  up_disable_irq(priv->rxirq);
  up_disable_irq(priv->txirq);
  irq_detach(priv->rxirq);
  irq_detach(priv->txirq);
#else
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif
}

/****************************************************************************
 * Name: up_mapirq
 *
 * Description:
 *   Map an IRQ number to internal UART state structure
 *
 ****************************************************************************/

static inline struct uart_dev_s *up_mapirq(int irq)
{
  struct uart_dev_s *dev;

  switch (irq)
    {
#ifdef CONFIG_IMX_UART1
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
      case IMX_IRQ_UART1RX:
      case IMX_IRQ_UART1TX:
#else
      case IMX_IRQ_UART1:
#endif
        dev = &g_uart1port;
        break;
#endif

#ifdef CONFIG_IMX_UART2
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
      case IMX_IRQ_UART2RX:
      case IMX_IRQ_UART2TX:
#else
      case IMX_IRQ_UART2:
#endif
        dev = &g_uart2port;
        break;
#endif

#ifdef CONFIG_IMX_UART3
#if defined(CONFIG_ARCH_CHIP_IMX1) || defined(CONFIG_ARCH_CHIP_IMXL)
      case IMX_IRQ_UART3RX:
      case IMX_IRQ_UART3TX:
#else
      case IMX_IRQ_UART3:
#endif
        dev = &g_uart3port;
        break;
#endif

      default:
        PANIC(OSERR_INTERNAL);
        break;
    }
  return dev;
} 

/****************************************************************************
 * Name: up_interrupt (and front-ends)
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
  struct uart_dev_s *dev;
  struct up_dev_s   *priv;
  uint32_t usr1;
  int    passes = 0;

  dev  = up_mapirq(irq);
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for(;;)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

      usr1 = up_serialin(priv, UART_USR1);
      usr1 &= (UART_USR1_RRDY | UART_USR1_TRDY);

      if (usr1 == 0 || passes > 256) 
        {
          return OK;
        }

      /* Handline incoming, receive bytes */

      if (usr1 & UART_USR1_RRDY)
        {
          uart_recvchars(dev);
        }

      /* Handle outgoing, transmit bytes */

      if (usr1 & UART_USR1_TRDY &&
          (up_serialin(priv, UART_UCR1) & UART_UCR1_TXEMPTYEN) != 0)
        {
          uart_xmitchars(dev);
        }

      /* Keep track of how many times we do this in case there
       * is some hardware failure condition.
       */

      passes++;
    }
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
  int                ret   = OK;

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

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
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
  uint32_t rxd0;

  rxd0    = up_serialin(priv, UART_RXD0);
  *status = rxd0;
  return (rxd0 & UART_RXD_DATA_MASK) >> UART_RXD_DATA_SHIFT;
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

  /* Enable interrupts for data availab at Rx FIFO */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ucr1 |= UART_UCR1_RRDYEN;
#endif
    }
  else
    {
      priv->ucr1 &= ~UART_UCR1_RRDYEN;
    }
  up_serialout(priv, UART_UCR1, priv->ucr1);
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
  
  /* Return true is data is ready in the Rx FIFO */

  return ((up_serialin(priv, UART_USR2) & UART_USR2_RDR) != 0);
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
  up_serialout(priv, UART_TXD0, (uint32_t)ch);
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

  /* We won't take an interrupt until the FIFO is completely empty (although
   * there may still be a transmission in progress).
   */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ucr1 |= UART_UCR1_TXEMPTYEN;
#endif
    }
  else
    {
      priv->ucr1 &= ~UART_UCR1_TXEMPTYEN;
    }
  up_serialout(priv, UART_UCR1, priv->ucr1);
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

  /* When TXFULL is set, there is no space in the Tx FIFO  */

  return ((up_serialin(priv, UART_UTS) & UART_UTS_TXFULL) == 0);
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

  /* When TXDC is set, the FIFO is empty and the transmission is complete */

  return ((up_serialin(priv, UART_USR2) & UART_USR2_TXDC) != 0);
}

/****************************************************************************
 * Public Funtions
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
   /* Configure and disable the UART1 */

#ifdef CONFIG_IMX_UART1
   up_serialout(&g_uart1priv, UART_UCR1, 0);
   up_serialout(&g_uart1priv, UART_UCR2, 0);

   /* Configure UART1 pins: RXD, TXD, RTS, and CTS */

   imxgpio_configpfoutput(GPIOC, 9);  /* Port C, pin  9: CTS */
   imxgpio_configpfinput(GPIOC, 10);  /* Port C, pin 10: RTS */
   imxgpio_configpfoutput(GPIOC, 11); /* Port C, pin 11: TXD */
   imxgpio_configpfinput(GPIOC, 12);  /* Port C, pin 12: RXD */
#endif

   /* Configure and disable the UART2 */

#ifdef CONFIG_IMX_UART2
   up_serialout(&g_uart2priv, UART_UCR1, 0);
   up_serialout(&g_uart2priv, UART_UCR2, 0);

   /* Configure UART2 pins: RXD, TXD, RTS, and CTS (only, also
    * supports DTR, DCD, RI, and DSR -- not configured)
    */

   imxgpio_configpfoutput(GPIOB, 28); /* Port B, pin 28: CTS */
   imxgpio_configpfinput(GPIOB, 29);  /* Port B, pin 29: RTS */
   imxgpio_configpfoutput(GPIOB, 30); /* Port B, pin 30: TXD */
   imxgpio_configpfinput(GPIOB, 31);  /* Port B, pin 31: RXD */
#endif

   /* Configure and disable the UART3 */

#ifdef CONFIG_IMX_UART3
   up_serialout(&g_uart3priv, UART_UCR1, 0);
   up_serialout(&g_uart3priv, UART_UCR2, 0);

   /* Configure UART2 pins: RXD, TXD, RTS, and CTS (only, also
    * supports DTR, DCD, RI, and DSR -- not configured)
    */

   imxgpio_configpfoutput(GPIOC, 28); /* Port C, pin 18: CTS */
   imxgpio_configpfinput(GPIOC, 29);  /* Port C, pin 29: RTS */
   imxgpio_configpfoutput(GPIOC, 30); /* Port C, pin 30: TXD */
   imxgpio_configpfinput(GPIOC, 31);  /* Port C, pin 31: RXD */
#endif

  /* Then enable the console UART.  The others will be initialized
   * if and when they are opened.
   */

#ifdef CONSOLE_DEV
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
#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
# ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#  ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#  endif
# endif
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint32_t ier;

  up_disableuartint(priv, &ier);
  up_waittxready(priv);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_serialout(priv, UART_TXD0, (uint32_t)'\r');
      up_waittxready(priv);
    }

  up_serialout(priv, UART_TXD0, (uint32_t)ch);
  up_waittxready(priv);
  up_restoreuartint(priv, ier);
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Definitions
 ****************************************************************************/

#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define IMX_REGISTER_BASE IMX_UART1_VBASE
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define IMX_REGISTER_BASE IMX_UART2_VBASE
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define IMX_REGISTER_BASE IMX_UART3_VBASE
#  endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void up_waittxready(void)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {

     /* Loop until TXFULL is zero -- meaning that there is space available
      * in the TX FIFO.
      */

     if ((getreg32(IMX_REGISTER_BASE + UART_UTS) & UART_UTS_TXFULL) == 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_putc(int ch)
{
  up_waittxready();

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      putreg32((uint16_t)'\r', IMX_REGISTER_BASE + UART_TXD0);
      up_waittxready();
    }

  putreg32((uint16_t)ch, IMX_REGISTER_BASE + UART_TXD0);
  return ch;
}

#endif /* USE_SERIALDRIVER */


