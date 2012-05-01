/****************************************************************************
 * arch/arm/src/calypso/calypso_serial.c
 *
 *   Copyright (C) 2011 Stefan Richter. All rights reserved.
 *   Author: Stefan Richter <ichgeh@l--putt.de>
 *
 * based on c5471/c5471_serial.c
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BASE_BAUD     115200

#if defined(CONFIG_UART_IRDA_HWFLOWCONTROL) || defined(CONFIG_UART_MODEM_HWFLOWCONTROL)
# define CONFIG_UART_HWFLOWCONTROL
#endif

#if UART_FCR_OFFS == UART_EFR_OFFS
# define UART_MULTIPLEX_REGS
// HW flow control not supported yet
# undef  CONFIG_UART_HWFLOWCONTROL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uart_regs_s
{
  uint32_t  ier;
  uint32_t  lcr;
  uint32_t  fcr;
#ifdef CONFIG_UART_HWFLOWCONTROL
  uint32_t  efr;
  uint32_t  tcr;
#endif
};

struct up_dev_s
{
  unsigned int         uartbase;	/* Base address of UART registers */
  unsigned int         baud_base;	/* Base baud for conversions */
  unsigned int         baud;		/* Configured baud */
  uint8_t              xmit_fifo_size;	/* Size of transmit FIFO */
  uint8_t              irq;			/* IRQ associated with this UART */
  uint8_t              parity;		/* 0=none, 1=odd, 2=even */
  uint8_t              bits;		/* Number of bits (7 or 8) */
#ifdef CONFIG_UART_HWFLOWCONTROL
  bool                 flowcontrol;	/* true: Hardware flow control
									 * is enabled. */
#endif
  bool                 stopbits2;	/* true: Configure with 2
									 * stop bits instead of 1 */
  struct uart_regs_s   regs;		/* Shadow copy of readonly regs */

#ifdef CONFIG_SERCOMM_CONSOLE
  bool                 sercomm;		/* Call sercomm in interrupt if true */
#endif
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
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
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

static char g_irdarxbuffer[CONFIG_UART_IRDA_RXBUFSIZE];
static char g_irdatxbuffer[CONFIG_UART_IRDA_TXBUFSIZE];
static char g_modemrxbuffer[CONFIG_UART_MODEM_RXBUFSIZE];
static char g_modemtxbuffer[CONFIG_UART_MODEM_TXBUFSIZE];

/* This describes the state of the C5471 serial IRDA port. */

static struct up_dev_s g_irdapriv =
{
  .xmit_fifo_size = UART_IRDA_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .uartbase       = UART_IRDA_BASE,
  .baud           = CONFIG_UART_IRDA_BAUD,
  .irq            = UART_IRQ_IRDA,
  .parity         = CONFIG_UART_IRDA_PARITY,
  .bits           = CONFIG_UART_IRDA_BITS,
#ifdef CONFIG_UART_IRDA_HWFLOWCONTROL
  .flowcontrol    = true,
#endif
  .stopbits2      = CONFIG_UART_IRDA_2STOP,

#ifdef CONFIG_SERCOMM_CONSOLE
  .sercomm        = false,
#endif
};

static uart_dev_t g_irdaport =
{
  .recv     =
  {
    .size   = CONFIG_UART_IRDA_RXBUFSIZE,
    .buffer = g_irdarxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_IRDA_TXBUFSIZE,
    .buffer = g_irdatxbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_irdapriv,
};

/* This describes the state of the C5471 serial Modem port. */

static struct up_dev_s g_modempriv =
{
  .xmit_fifo_size = UART_XMIT_FIFO_SIZE,
  .baud_base      = BASE_BAUD,
  .uartbase       = UART_MODEM_BASE,
  .baud           = CONFIG_UART_MODEM_BAUD,
  .irq            = UART_IRQ_MODEM,
  .parity         = CONFIG_UART_MODEM_PARITY,
  .bits           = CONFIG_UART_MODEM_BITS,
#ifdef CONFIG_UART_MODEM_HWFLOWCONTROL
  .flowcontrol    = true,
#endif
  .stopbits2      = CONFIG_UART_MODEM_2STOP,

#ifdef CONFIG_SERCOMM_CONSOLE
  .sercomm        = false,
#endif
};

static uart_dev_t g_modemport =
{
  .recv     =
  {
    .size   = CONFIG_UART_MODEM_RXBUFSIZE,
    .buffer = g_modemrxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_MODEM_TXBUFSIZE,
    .buffer = g_modemtxbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_modempriv,
};

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_SERIAL_IRDA_CONSOLE
# define CONSOLE_DEV     g_irdaport
# define TTYS0_DEV       g_irdaport
# define TTYS1_DEV       g_modemport
#else
# define CONSOLE_DEV     g_modemport
# define TTYS0_DEV       g_modemport
# define TTYS1_DEV       g_irdaport
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_inserial
 ****************************************************************************/

static inline uint32_t up_inserial(struct up_dev_s *priv, uint32_t offset)
{
#if UART_REGISTER_BITS == 8
  return getreg8(priv->uartbase + offset);
#elif UART_REGISTER_BITS == 32
  return getreg32(priv->uartbase + offset);
#else
#error Unsupported number of bits set in UART_REGISTER_BITS
#endif
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, uint32_t offset, uint32_t value)
{
#if UART_REGISTER_BITS == 8
  putreg8(value & 0xff, priv->uartbase + offset);
#elif UART_REGISTER_BITS == 32
  putreg32(value, priv->uartbase + offset);
#endif
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint16_t *ier)
{
  if (ier)
    {
      *ier = priv->regs.ier & UART_IER_INTMASK;
    }
  priv->regs.ier &= ~UART_IER_INTMASK;
  up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint16_t ier)
{
  priv->regs.ier |= ier & (UART_IER_RECVINT|UART_IER_XMITINT);
  up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if ((up_inserial(priv, UART_SSR_OFFS) & UART_SSR_TXFULL) == 0)
        {
          break;
        }
    }
}
/****************************************************************************
 * Name: up_disablebreaks
 ****************************************************************************/

static inline void up_disablebreaks(struct up_dev_s *priv)
{
  priv->regs.lcr &= ~UART_LCR_BOC;
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv)
{
  priv->regs.lcr |= UART_LCR_BOC;
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);
}

/****************************************************************************
 * Name: up_setrate
 ****************************************************************************/

static inline void up_setrate(struct up_dev_s *priv, unsigned int rate)
{
  uint32_t div_bit_rate;

  switch (rate)
    {
    case 115200:
      div_bit_rate = BAUD_115200;
      break;
    case 57600:
      div_bit_rate = BAUD_57600;
      break;
    case 38400:
      div_bit_rate = BAUD_38400;
      break;
    case 19200:
      div_bit_rate = BAUD_19200;
      break;
    case 4800:
      div_bit_rate = BAUD_4800;
      break;
    case 2400:
      div_bit_rate = BAUD_2400;
      break;
    case 1200:
      div_bit_rate = BAUD_1200;
      break;
    case 9600:
    default:
      div_bit_rate = BAUD_9600;
      break;
    }

#if UART_DIV_BIT_RATE_OFFS
  up_serialout(priv, UART_DIV_BIT_RATE_OFFS, div_bit_rate);
#else
  up_serialout(priv, UART_DIV_LOW_OFFS, div_bit_rate);
  up_serialout(priv, UART_DIV_HIGH_OFFS, div_bit_rate >> 8);
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
#include <stdio.h>
static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = dev->priv;
  unsigned int cval;

  if (priv->bits == 7)
    {
      cval = UART_LCR_7BITS;
    }
  else
    {
      cval = UART_LCR_8BITS;
    }

  if (priv->stopbits2)
    {
      cval |= UART_LCR_2STOP;
    }

  if (priv->parity == 1)   /* Odd parity */
    {
      cval |= (UART_LCR_PAREN|UART_LCR_PARODD);
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      cval |= (UART_LCR_PAREN|UART_LCR_PAREVEN);
    }

  /* Both the IrDA and MODEM UARTs support RESET and UART mode. */

  up_serialout(priv, UART_MDR_OFFS, MDR_RESET_MODE);
  up_serialout(priv, UART_LCR_OFFS, 0xbf);
  up_serialout(priv, UART_XON1_OFFS, 0x00);
  up_serialout(priv, UART_XON2_OFFS, 0x00);
  up_serialout(priv, UART_XOFF1_OFFS, 0x00);
  up_serialout(priv, UART_XOFF2_OFFS, 0x00);
  up_serialout(priv, UART_EFR_OFFS, 0x00);
  up_serialout(priv, UART_LCR_OFFS, 0x00);
  up_mdelay(5);

  up_serialout(priv, UART_MDR_OFFS, MDR_UART_MODE);
  up_mdelay(5);

  priv->regs.ier = up_inserial(priv, UART_IER_OFFS);
  priv->regs.lcr = up_inserial(priv, UART_LCR_OFFS);
#ifdef CONFIG_UART_HWFLOWCONTROL
  if (priv->flowcontrol)
    {
      priv->regs.efr = up_inserial(priv, UART_EFR_OFFS);
      priv->regs.tcr = up_inserial(priv, UART_TCR_OFFS);
    }
#endif

  up_disableuartint(priv, NULL);

#ifdef UART_MULTIPLEX_REGS
  up_serialout(priv, UART_LCR_OFFS, 0x00bf);
#endif

  up_serialout(priv, UART_EFR_OFFS,  0x0010);           /* Unprotect enhanced control */

#ifdef UART_MULTIPLEX_REGS
  priv->regs.lcr = 0x80;
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);
  //up_serialout(priv, UART_MCR_OFFS, 1<<4);              /* loopback */
#endif

  up_serialout(priv, UART_TFCR_OFFS, 0);                /* Reset to 0 */
  up_serialout(priv, UART_RFCR_OFFS, UART_FCR_RX_CLR);  /* Clear RX fifo */
  up_serialout(priv, UART_TFCR_OFFS, UART_FCR_TX_CLR);  /* Clear TX fifo */
  priv->regs.fcr = UART_FCR_FIFO_EN;
  up_serialout(priv, UART_TFCR_OFFS, priv->regs.fcr); /* Enable RX/TX fifos */

  up_disablebreaks(priv);

  /* Set the RX and TX trigger levels to the minimum */

  priv->regs.fcr = (priv->regs.fcr & 0xffffff0f) | UART_FCR_FTL;
  up_serialout(priv, UART_RFCR_OFFS, priv->regs.fcr);

  up_setrate(priv, priv->baud);

#ifdef UART_MULTIPLEX_REGS
  up_serialout(priv, UART_SCR_OFFS, 1);  /* Disable DMA */
  priv->regs.lcr = (uint32_t)cval;       /* Configure mode, return to THR/RHR */
#else
  priv->regs.lcr &= 0xffffffe0;       /* clear original field, and... */
  priv->regs.lcr |= (uint32_t)cval;   /* Set new bits in that field. */
#endif
  up_serialout(priv, UART_LCR_OFFS, priv->regs.lcr);

#ifdef CONFIG_UART_HWFLOWCONTROL
  if (priv->flowcontrol)
    {
      /* Set the FIFO level triggers for flow control
       * Halt = 48 bytes, resume = 12 bytes
       */

      priv->regs.tcr = (priv->regs.tcr & 0xffffff00) | 0x0000003c;
      up_serialout(priv, UART_TCR_OFFS, priv->regs.tcr);

      /* Enable RTS/CTS flow control */

      priv->regs.efr |= 0x000000c0;
      up_serialout(priv, UART_EFR_OFFS, priv->regs.efr);
    }
  else
    {
      /* Disable RTS/CTS flow control */

      priv->regs.efr &= 0xffffff3f;
      up_serialout(priv, UART_EFR_OFFS, priv->regs.efr);
    }
#endif
#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
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
  volatile uint32_t  cause;

  if (g_irdapriv.irq == irq)
    {
      dev = &g_irdaport;
    }
  else if (g_modempriv.irq == irq)
    {
      dev = &g_modemport;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  cause = up_inserial(priv, UART_ISR_OFFS) & 0x0000003f;

  if ((cause & 0x0000000c) == 0x0000000c)
    {
      uint32_t ier_val = 0;

      /* Is this an interrupt from the IrDA UART? */

      if (irq == UART_IRQ_IRDA)
         {
           /* Save the currently enabled IrDA UART interrupts
            * so that we can restore the IrDA interrupt state
            * below.
            */

           ier_val = up_inserial(priv, UART_IER_OFFS);

           /* Then disable all IrDA UART interrupts */

           up_serialout(priv, UART_IER_OFFS, 0);
         }

      /* Receive characters from the RX fifo */

#ifdef CONFIG_SERCOMM_CONSOLE
      if (priv->sercomm)
         {
           sercomm_recvchars(dev);
         }
      else
#endif
         {
           uart_recvchars(dev);
         }

      /* read UART_RHR to clear int condition
       * toss = up_inserialchar(priv,&status);
       */

      /* Is this an interrupt from the IrDA UART? */

      if (irq == UART_IRQ_IRDA)
         {
           /* Restore the IrDA UART interrupt enables */

           up_serialout(priv, UART_IER_OFFS, ier_val);
         }
    }
  else if ((cause & 0x0000000c) == 0x00000004)
    {
#ifdef CONFIG_SERCOMM_CONSOLE
      if (priv->sercomm)
         {
           sercomm_recvchars(dev);
         }
      else
#endif
         {
           uart_recvchars(dev);
         }
    }

  if ((cause & 0x00000002) != 0)
    {
#ifdef CONFIG_SERCOMM_CONSOLE
      if (priv->sercomm)
         {
           sercomm_xmitchars(dev);
         }
      else
#endif
         {
           uart_xmitchars(dev);
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
  struct up_dev_s   *priv  = (struct up_dev_s*)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             *get_errno_ptr() = EINVAL;
             ret = ERROR;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(priv);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_disablebreaks(priv);
        irqrestore(flags);
      }
      break;

    default:
      *get_errno_ptr() = ENOTTY;
      ret = ERROR;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 * Called (usually) from the interrupt level to receive one character from
 * the UART.  Error bits associated with the receipt are provided in the
 * the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t rhr;
  uint32_t lsr;

  /* Construct a 16bit status word that uses the high byte to
   * hold the status bits associated with framing,parity,break
   * and a low byte that holds error bits of LSR for
   * conditions such as overflow, etc.
   */

  rhr = up_inserial(priv, UART_RHR_OFFS);
  lsr = up_inserial(priv, UART_LSR_OFFS);

  *status = (unsigned int)((rhr & 0x0000ff00) | (lsr & 0x000000ff));

  return rhr & 0x000000ff;
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
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->regs.ier |= UART_IER_RECVINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
#endif
    }
  else
    {
      priv->regs.ier &= ~UART_IER_RECVINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
    }
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
  return up_inserial(priv, UART_LSR_OFFS) & UART_RX_FIFO_NOEMPTY;
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
  up_serialout(priv, UART_THR_OFFS, (uint8_t)ch);
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
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->regs.ier |= UART_IER_XMITINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
#endif
    }
  else
    {
      priv->regs.ier &= ~UART_IER_XMITINT;
      up_serialout(priv, UART_IER_OFFS, priv->regs.ier);
    }
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
  return (up_inserial(priv, UART_SSR_OFFS) & UART_SSR_TXFULL) == 0;
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
  return (up_inserial(priv, UART_LSR_OFFS) & UART_LSR_TREF) != 0;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  up_disableuartint(TTYS0_DEV.priv, NULL);
  up_disableuartint(TTYS1_DEV.priv, NULL);

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
#ifdef CONFIG_SERCOMM_CONSOLE
  ((struct up_dev_s*)TTYS0_DEV.priv)->sercomm = true;
  (void)sercomm_register("/dev/console", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS1_DEV);
#else
  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
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
  uint16_t  ier;

  up_disableuartint(priv, &ier);
  up_waittxready(priv);
  up_serialout(priv, UART_THR_OFFS, (uint8_t)ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, UART_THR_OFFS, '\r');
    }

  up_waittxready(priv);
  up_restoreuartint(priv, ier);
  return ch;
}

