/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_serial.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *   With updates by: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
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
#include <arch/serial.h>

#include "chip.h"
#include "up_arch.h"
#include "os_internal.h"

#include "internal.h"
#include "lpc23xx_scb.h"
#include "lpc23xx_pinsel.h"
#include "lpc23xx_uart.h"
#include "lpc23xx_vic.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;          /* Base address of UART registers */
  uint32_t baud;              /* Configured baud */
  uint8_t ier;                /* Saved IER value */
  uint8_t irq;                /* IRQ associated with this UART */
  uint8_t parity;             /* 0=none, 1=odd, 2=even */
  uint8_t bits;               /* Number of bits (7 or 8) */
  bool stopbits2;             /* true: Configure with 2 stop bits instead of 1 */
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
static int  up_receive(struct uart_dev_s *dev, uint32_t * status);
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
  .setup       = up_setup,
  .shutdown    = up_shutdown,
  .attach      = up_attach,
  .detach      = up_detach,
  .ioctl       = up_ioctl,
  .receive     = up_receive,
  .rxint       = up_rxint,
  .rxavailable = up_rxavailable,
  .send        = up_send,
  .txint       = up_txint,
  .txready     = up_txready,
  .txempty     = up_txempty,
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];

/* This describes the state of the LPC214X uart0 port. */

#ifdef CONFIG_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase  = UART0_BASE_ADDR,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = UART0_IRQ,
  .parity    = CONFIG_UART0_PARITY,
  .bits      = CONFIG_UART0_BITS,
  .stopbits2 = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv =
    {
      .size   = CONFIG_UART0_RXBUFSIZE,
      .buffer = g_uart0rxbuffer,
    },
  .xmit =
    {
      .size   = CONFIG_UART0_TXBUFSIZE,
      .buffer = g_uart0txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};
#endif

#ifdef CONFIG_UART2

/* This describes the state of the LPC23XX uart2 port. */

static struct up_dev_s g_uart2priv =
{
  .uartbase  = UART2_BASE_ADDR,
  .baud      = CONFIG_UART2_BAUD,
  .irq       = UART2_IRQ,
  .parity    = CONFIG_UART2_PARITY,
  .bits      = CONFIG_UART2_BITS,
  .stopbits2 = CONFIG_UART2_2STOP,

};

static uart_dev_t g_uart2port =
{
  .recv =
    {
      .size = CONFIG_UART2_RXBUFSIZE,
      .buffer = g_uart2rxbuffer,
    },
  .xmit =
    {
      .size = CONFIG_UART2_TXBUFSIZE,
      .buffer = g_uart2txbuffer,
    },
  .ops  = &g_uart_ops,
  .priv = &g_uart2priv,
};

#endif

/* Now, which one with be tty0/console and which tty1? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port
#  define TTYS0_DEV       g_uart0port
#  define TTYS1_DEV       g_uart2port
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port
#  define TTYS0_DEV       g_uart2port
#  define TTYS1_DEV       g_uart0port
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t * ier)
{
  if (ier)
    {
      *ier = priv->ier & IER_ALLIE;
    }

  priv->ier &= ~IER_ALLIE;
  up_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier)
{
  priv->ier |= ier & IER_ALLIE;
  up_serialout(priv, UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TX available condition */

  for (tmp = 1000; tmp > 0; tmp--)
    {
      /* Check if the tranmitter holding register (THR) is empty */

      if ((up_serialin(priv, UART_LSR_OFFSET) & LSR_THRE) != 0)
        {
          /* The THR is empty, return */

          break;
        }
    }
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint8_t lcr = up_serialin(priv, UART_LCR_OFFSET);
  if (enable)
    {
      lcr |= LCR_BREAK_ENABLE;
    }
  else
    {
      lcr &= ~LCR_BREAK_ENABLE;
    }
  up_serialout(priv, UART_LCR_OFFSET, lcr);
}

/****************************************************************************
 * Name: up_configbaud
 ****************************************************************************/
static inline void up_configbaud(struct up_dev_s *priv)
{

  /* In a buckled-up, embedded system, there is no reason to constantly
   * calculate the following.  The calculation can be skipped if the MULVAL,
   * DIVADDVAL, and DIVISOR values are provided in the configuration file.
   */

#ifndef CONFIG_UART_MULVAL
  uint32_t qtrclk;

  /* Test values calculated for every multiplier/divisor combination */

  uint32_t tdiv;
  uint32_t terr;
  int tmulval;
  int tdivaddval;

  /* Optimal multiplier/divider values */

  uint32_t div = 0;
  uint32_t err = 100000;
  int mulval = 1;
  int divaddval = 0;

  /* Baud is generated using FDR and DLL-DLM registers
   *
   *   baud = clock * (mulval/(mulval+divaddval) / (16 * div)
   *
   * Or
   *
   *   div = (clock/16) * (mulval/(mulval+divaddval) / baud
   *
   * Where mulval    = Fractional divider multiplier
   *       divaddval = Fractional divider pre-scale div
   *       div       = DLL-DLM divisor
   */

  /* Get UART block clock divided by 16 */

  qtrclk = U0_PCLK >> 4;        /* TODO: Different Uart port with different clocking */

  /* Try every valid multiplier, tmulval (or until a perfect match is found). */

  for (tmulval = 1; tmulval <= 15 && err > 0; tmulval++)
    {
      /* Try every valid pre-scale div, tdivaddval (or until a perfect match is 
       * found).
       */

      for (tdivaddval = 0; tdivaddval <= 15 && err > 0; tdivaddval++)
        {
          /* Calculate the divisor with these fractional divider settings */

          uint32_t tmp = (tmulval * qtrclk) / ((tmulval + tdivaddval));
          tdiv = (tmp + (priv->baud >> 1)) / priv->baud;

          /* Check if this candidate divisor is within a valid range */

          if (tdiv > 2 && tdiv < 0x10000)
            {
              /* Calculate the actual baud and the error */

              uint32_t actualbaud = tmp / tdiv;

              if (actualbaud <= priv->baud)
                {
                  terr = priv->baud - actualbaud;
                }
              else
                {
                  terr = actualbaud - priv->baud;
                }

              /* Is this the smallest error we have encountered? */

              if (terr < err)
                {
                  /* Yes, save these settings as the new, candidate optimal
                   * settings
                   */

                  mulval = tmulval;
                  divaddval = tdivaddval;
                  div = tdiv;
                  err = terr;
                }
            }
        }
    }

  /* Configure the MS and LS DLAB registers */

  up_serialout(priv, UART_DLM_OFFSET, div >> 8);
  up_serialout(priv, UART_DLL_OFFSET, div & 0xff);

  /* Configure the Fractional Divider Register (FDR) */

  up_serialout(priv, UART_FDR_OFFSET, ((mulval << 4) | divaddval));

#else

  /* Configure the MS and LS DLAB registers */
  up_serialout(priv, UART_DLM_OFFSET, DLMVAL >> 8);
  up_serialout(priv, UART_DLL_OFFSET, DLLVAL & 0xff);

  /* Configure the Fractional Divider Register (FDR) */

  up_serialout(priv, UART_FDR_OFFSET, ((MULVAL << 4) | DIVADDVAL));

#endif
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
#ifndef CONFIG_SUPPRESS_LPC214X_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint8_t lcr;

  /* Clear fifos */

  up_serialout(priv, UART_FCR_OFFSET, (FCR_RX_FIFO_RESET | FCR_TX_FIFO_RESET));

  /* Set trigger */

  up_serialout(priv, UART_FCR_OFFSET, (FCR_FIFO_ENABLE | FCR_FIFO_TRIG14));

  /* Set up the IER */

  priv->ier = up_serialin(priv, UART_IER_OFFSET);

  /* Set up the Line Control Register */

  lcr = 0;

  lcr |= (priv->bits == 7) ? LCR_CHAR_7 : LCR_CHAR_8;

  if (priv->stopbits2)
    {
      lcr |= LCR_STOP_2;
    }

  if (priv->parity == 1)
    {
      lcr |= LCR_PAR_ODD;
    }
  else if (priv->parity == 2)
    {
      lcr |= LCR_PAR_EVEN;
    }

  /* Enable access to latch divisor DLAB=1 */

  up_serialout(priv, UART_LCR_OFFSET, (lcr | LCR_DLAB_ENABLE));

  /* find values for DLL, DLM, DIVADDVAL, MULVAL */

  up_configbaud(priv);

  /* Disable access to latch divisor Clear DLAB */

  up_serialout(priv, UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  up_serialout(priv, UART_FCR_OFFSET,
               (FCR_FIFO_TRIG8 | FCR_TX_FIFO_RESET |
                FCR_RX_FIFO_RESET | FCR_FIFO_ENABLE));

  /* The NuttX serial driver waits for the first THRE interrrupt before sending 
   * serial data... However, it appears that the LPC2378 hardware too does not
   * generate that interrupt until a transition from not-empty to empty.  So,
   * the current kludge here is to send one NULL at startup to kick things off.
   */

  up_serialout(priv, UART_THR_OFFSET, '\0');
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled in the
       * UART */

      up_enable_irq(priv->irq);

      /* Set the uart interrupt priority (the default value is one) */
      if (priv->uartbase == UART0_BASE_ADDR)
        {
          up_prioritize_irq(priv->irq, PRIORITY_LOWEST);
        }
      else if (priv->uartbase == UART2_BASE_ADDR)
        {
          up_prioritize_irq(priv->irq, 10);
        }

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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
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
  struct up_dev_s *priv;
  uint8_t status;
  int passes;

  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or, until we have
   * been looping for a long time. */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop termination conditions */

      status = up_serialin(priv, UART_IIR_OFFSET);

      /* The NO INTERRUPT should be zero if there are pending interrupts */

      if ((status & IIR_NO_INT) != 0)
        {
          /* Break out of the loop when there is no longer a pending interrupt */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & IIR_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

        case IIR_RDA_INT:
        case IIR_CTI_INT:
          {
            uart_recvchars(dev);
            break;
          }

          /* Handle outgoing, transmit bytes */

        case IIR_THRE_INT:
          {
            uart_xmitchars(dev);
            break;
          }

          /* Just clear modem status interrupts (UART1 only) */

        case IIR_MS_INT:
          {
            /* Read the modem status register (MSR) to clear */

            status = up_serialin(priv, UART_MSR_OFFSET);
            vdbg("MSR: %02x\n", status);
            break;
          }

          /* Just clear any line status interrupts */

        case IIR_RLS_INT:
          {
            /* Read the line status register (LSR) to clear */

            status = up_serialin(priv, UART_LSR_OFFSET);
            vdbg("LSR: %02x\n", status);
            break;
          }

          /* There should be no other values */

        default:
          {
            dbg("Unexpected IIR: %02x\n", status);
            break;
          }
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
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
        struct up_dev_s *user = (struct up_dev_s *)arg;
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

    case TIOCSBRK:             /* BSD compatibility: Turn break on,
                                 * unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(priv, true);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:             /* BSD compatibility: Turn break off,
                                 * unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_enablebreaks(priv, false);
        irqrestore(flags);
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

static int up_receive(struct uart_dev_s *dev, uint32_t * status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint8_t rbr;

  *status = up_serialin(priv, UART_LSR_OFFSET);
  rbr = up_serialin(priv, UART_RBR_OFFSET);
  return rbr;
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= IER_ERBFI;
#endif
    }
  else
    {
      priv->ier &= ~IER_ERBFI;
    }
  up_serialout(priv, UART_IER_OFFSET, priv->ier);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, UART_LSR_OFFSET) & LSR_RDR) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_serialout(priv, UART_THR_OFFSET, (uint8_t) ch);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= IER_ETBEI;
#endif
    }
  else
    {
      priv->ier &= ~IER_ETBEI;
    }
  up_serialout(priv, UART_IER_OFFSET, priv->ier);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, UART_LSR_OFFSET) & LSR_THRE) != 0);
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
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  return ((up_serialin(priv, UART_LSR_OFFSET) & LSR_THRE) != 0);
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
  /* Enable UART0 and 2 */

  uint32_t pinsel = getreg32(LPC23XX_PINSEL0);

  pinsel &= ~(UART0_PINMASK | UART2_PINMASK);
  pinsel |= (UART0_PINSEL | UART2_PINSEL);

  putreg32(pinsel, LPC23XX_PINSEL0);

  /* Set Uart PCLK divider */

  SCB_PCLKSEL0 = (SCB_PCLKSEL0 & ~U0_PCLKSEL_MASK) | U0_PCLKSEL;
  SCB_PCLKSEL1 = (SCB_PCLKSEL1 & ~U2_PCLKSEL_MASK) | U2_PCLKSEL;

  /* Disable both UARTS */

  up_disableuartint(TTYS0_DEV.priv, NULL);
  up_disableuartint(TTYS1_DEV.priv, NULL);

  /* Configuration whichever one is the console */

  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
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
  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
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
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint8_t ier;

  up_disableuartint(priv, &ier);
  up_waittxready(priv);
  up_serialout(priv, UART_THR_OFFSET, (uint8_t) ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, UART_THR_OFFSET, '\r');
    }

  up_waittxready(priv);
  up_restoreuartint(priv, ier);
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
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  return ch;
}
#endif /* USE_SERIALDRIVER */
