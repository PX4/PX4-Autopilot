/****************************************************************************
 * arch/sh/src/sh1/sh1_serial.c
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
#include <nuttx/serial.h>
#include <arch/serial.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Are there any SCIs? */

#if !defined(CONFIG_SH1_SCI0) && !defined(CONFIG_SH1_SCI1)
#  ifdef USE_SERIALDRIVER
#    error "Serial driver selected, but SCIs not enabled"
#    undef USE_SERIALDRIVER
#  endif
#endif

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_SH1_SCI0)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_SH1_SCI1)
#  define HAVE_CONSOLE 1
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#else
#  if defined(CONFIG_SCI0_SERIAL_CONSOLE) || defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    error "Serial console selected, but corresponding SCI not enabled"
#  endif
#  undef HAVE_CONSOLE
#  undef CONFIG_SCI0_SERIAL_CONSOLE
#  undef CONFIG_SCI1_SERIAL_CONSOLE
#endif

#ifdef USE_SERIALDRIVER

/* Which SCI with be tty0/console and which tty1? */

/* CONFIG_SCI0_SERIAL_CONSOLE (implies CONFIG_SH1_SCI0 also defined) */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_sci0port     /* SCI0 is console */
#  define TTYS0_DEV       g_sci0port     /* SCI0 is tty0 */
#  ifdef CONFIG_SH1_SCI1
#    define TTYS1_DEV     g_sci1port     /* SCI1 is tty1 */
#  else
#    undef TTYS1_DEV
#  endif

/* CONFIG_SCI1_SERIAL_CONSOLE (implies CONFIG_SH1_SCI1 also defined) */

#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_sci1port     /* SCI1 is console */
#  define TTYS0_DEV       g_sci1port     /* SCI1 is tty0 */
#  ifdef CONFIG_SH1_SCI0
#    define TTYS1_DEV     g_sci0port     /* SCI0 is tty1 */
#  else
#    undef TTYS1_DEV                     /* No tty1 */
#  endif

/* No console, at least one of CONFIG_SH1_SCI0 and CONFIG_SH1_SCI1 defined */

#elif defined(CONFIG_SH1_SCI0)
#  undef  CONSOLE_DEV                    /* No console */
#  define TTYS0_DEV       g_sci0port     /* SCI0 is tty0 */
#  ifdef CONFIG_SH1_SCI1
#    define TTYS1_DEV     g_sci1port     /* SCI1 is tty1 */
#  else
#    undef TTYS1_DEV                     /* No tty1 */
#  endif

/* Otherwise, there is no console and only CONFIG_SH1_SCI1 is defined */

#else
#  undef  CONSOLE_DEV                    /* No console */
#  define TTYS0_DEV       g_sci1port     /* SCI1 is tty0 */
#  undef TTYS1_DEV                       /* No tty1 */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
          uint32_t scibase;   /* Base address of SCI registers */
          uint32_t baud;      /* Configured baud */
 volatile  uint8_t scr;       /* Saved SCR value */
 volatile  uint8_t ssr;       /* Saved SR value (only used during interrupt processing) */
           uint8_t irq;       /* Base IRQ associated with this SCI */
           uint8_t parity;    /* 0=none, 1=odd, 2=even */
           uint8_t bits;      /* Number of bits (7 or 8) */
              bool stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_sci_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};

/* I/O buffers */

#ifdef CONFIG_SH1_SCI0
static char g_sci0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_sci0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif
#ifdef CONFIG_SH1_SCI1
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

/* This describes the state of the SH1 SCI0 port. */

#ifdef CONFIG_SH1_SCI0
static struct up_dev_s g_sci0priv =
{
  .scibase        = SH1_SCI0_BASE,
  .baud           = CONFIG_SCI0_BAUD,
  .irq            = SH1_SCI0_IRQBASE,
  .parity         = CONFIG_SCI0_PARITY,
  .bits           = CONFIG_SCI0_BITS,
  .stopbits2      = CONFIG_SCI0_2STOP,
};

static uart_dev_t g_sci0port =
{
  .recv     =
  {
    .size   = CONFIG_SCI0_RXBUFSIZE,
    .buffer = g_sci0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_SCI0_TXBUFSIZE,
    .buffer = g_sci0txbuffer,
  },
  .ops      = &g_sci_ops,
  .priv     = &g_sci0priv,
};
#endif

/* This describes the state of the SH1 SCI1 port. */

#ifdef CONFIG_SH1_SCI1
static struct up_dev_s g_sci1priv =
{
  .scibase        = SH1_SCI1_BASE,
  .baud           = CONFIG_SCI1_BAUD,
  .irq            = SH1_SCI1_IRQBASE,
  .parity         = CONFIG_SCI1_PARITY,
  .bits           = CONFIG_SCI1_BITS,
  .stopbits2      = CONFIG_SCI1_2STOP,
};

static uart_dev_t g_sci1port =
{
  .recv     =
  {
    .size   = CONFIG_SCI1_RXBUFSIZE,
    .buffer = g_sci1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_SCI1_TXBUFSIZE,
    .buffer = g_sci1txbuffer,
   },
  .ops      = &g_sci_ops,
  .priv     = &g_sci1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
  putreg8(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_disablesciint
 ****************************************************************************/

static inline void up_disablesciint(struct up_dev_s *priv, uint8_t *scr)
{
  /* Return a copy of the current scr settings */

  if (scr)
    {
      *scr = priv->scr;
    }

  /* The disable all interrupts */

  priv->scr &= ~SH1_SCISCR_ALLINTS;
  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/

static inline void up_restoresciint(struct up_dev_s *priv, uint8_t scr)
{
  /* Set the interrupt bits in the scr value */

  priv->scr  &= ~SH1_SCISCR_ALLINTS;
  priv->scr |= (scr & SH1_SCISCR_ALLINTS);
  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TDR empty condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check if the TDR is empty.  The TDR becomes empty when:  (1) the
       * the chip is reset or enters standby mode, (2) the TE bit in the SCR
       * is cleared, or (3) the current TDR contents are loaded in the TSR so
       * that new data can be written in the TDR.
       */

      if ((up_serialin(priv, SH1_SCI_SSR_OFFSET) & SH1_SCISSR_TDRE) != 0)
        {
          /* The TDR is empty... return */
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: up_setbrr
 *
 * Description:
 *   Calculate the correct value for the BRR given the configured frequency
 *   and the desired BAUD settings.
 *
 ****************************************************************************/

static inline void up_setbrr(struct up_dev_s *priv, unsigned int baud)
{
  /* The calculation of the BRR to achieve the desired BAUD is given by the
   * following formula:
   *
   *   brr = (f/(64*2**(2n-1)*b))-1
   *
   * Where:
   *
   *   b = bit rate
   *   f = frequency (Hz)
   *   n = divider setting (0, 1, 2, 3)
   *
   * For n == 0 and with rounding this becomes:
   *
   *   brr = ((((f+16)/32) +(b/2)) / b) - 1
   *
   * For example, if the processor is clocked at 10 MHz and 9600 is the
   * desired BAUD:
   *
   *   brr = ((10,000,016/32) + 4800) / 9600 -1
   *       = 32
   */

  uint32_t brr = ((((SH1_CLOCK + 16) / 32) + (baud >> 1)) / baud) - 1;
  up_serialout(priv, SH1_SCI_BRR_OFFSET, (uint16_t)brr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t smr;

  /* Disable the transmitter and receiver */

  priv->scr  = up_serialin(priv, SH1_SCI_SCR_OFFSET);
  priv->scr &= ~(SH1_SCISCR_TE | SH1_SCISCR_RE);
  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);

  /* Set communication to be asynchronous with the configured number of data
   * bits, parity, and stop bits.  Use the internal clock (undivided)
   */

  smr = 0;
  if (priv->bits == 7)
    {
      smr |= SH1_SCISMR_CHR;
    }

  if (priv->parity == 1)
    {
      smr |= (SH1_SCISMR_PE|SH1_SCISMR_OE);
    }
  else if (priv->parity == 2)
    {
      smr |= SH1_SCISMR_PE;
    }

  if (priv->stopbits2)
    {
      smr |= SH1_SCISMR_STOP;
    }

  up_serialout(priv, SH1_SCI_SMR_OFFSET, smr);

  /* Set the baud based on the configured console baud and configured
   * system clock.
   */

  up_setbrr(priv, priv->baud);

  /* Select the internal clock source as input */

  priv->scr &= ~SH1_SCISCR_CKEMASK;
  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);

  /* Wait a bit for the clocking to settle */

  up_udelay(100);

  /* Then enable the transmitter and reciever */

  priv->scr |= (SH1_SCISCR_TE | SH1_SCISCR_RE);
  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);
#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disablesciint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method is
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

  /* Attach the RDR full IRQ (RXI) that is enabled by the RIE SCR bit */

  ret = irq_attach(priv->irq + SH1_RXI_IRQ_OFFSET, up_interrupt);
  if (ret == OK)
    {
      /* The RIE interrupt enable also enables the receive error interrupt (ERI) */

      ret = irq_attach(priv->irq + SH1_ERI_IRQ_OFFSET, up_interrupt);
      if (ret == OK)
        {
          /* Attach the TDR empty IRQ (TXI) enabled by the TIE SCR bit */

          ret = irq_attach(priv->irq + SH1_TXI_IRQ_OFFSET, up_interrupt);
          if (ret == OK)
            {
              /* All SCI0 interrupts share the same prioritization */

              up_prioritize_irq(priv->irq, 7);  /* Set SCI priority midway */

              /* Return OK on success */

              return OK;
            }

          /* Detach the ERI interrupt on failure */

          (void)irq_detach(priv->irq + SH1_ERI_IRQ_OFFSET);
        }

      /* Detach the RXI interrupt on failure */

      (void)irq_detach(priv->irq + SH1_RXI_IRQ_OFFSET);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Disable all SCI interrupts */

  up_disablesciint(priv, NULL);

  /* Detach the SCI interrupts */

  (void)irq_detach(priv->irq + SH1_RXI_IRQ_OFFSET);
  (void)irq_detach(priv->irq + SH1_ERI_IRQ_OFFSET);
  (void)irq_detach(priv->irq + SH1_RXI_IRQ_OFFSET);

  /* Set the interrupt priority to zero (masking all SCI interrupts).  NOTE
   * that all SCI0 interrupts share the same prioritization.
   */

  up_prioritize_irq(priv->irq, 0);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the SCI interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   up_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;

#ifdef CONFIG_SH1_SCI0
  if ((irq >= g_sci0priv.irq) && 
      (irq <= g_sci0priv.irq +  SH1_SCI_NIRQS))
    {
      dev = &g_sci0port;
    }
  else
#endif
#ifdef CONFIG_SH1_SCI1
  if ((irq >= g_sci1priv.irq) && 
      (irq <= g_sci1priv.irq +  SH1_SCI_NIRQS))
    {
      dev = &g_sci1port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Get the current SCI status  */

  priv->ssr = up_serialin(priv, SH1_SCI_SSR_OFFSET);

  /* Handle receive-related events with RIE is enabled.  RIE is enabled at
   * times that driver is open EXCEPT when the driver is actively copying
   * data from the circular buffer.  In that case, the read events must
   * pend until RIE is set
   */

  if ((priv->scr & SH1_SCISCR_RIE) != 0)
    {
      /* Handle incoming, receive bytes (RDRF: Receive Data Register Full) */

      if ((priv->ssr & SH1_SCISSR_RDRF) != 0)
        {
           /* Rx data register not empty ... process incoming bytes */

           uart_recvchars(dev);
        }

      /* Clear all read related events (probably already done in up_receive)) */

      priv->ssr &= ~(SH1_SCISSR_RDRF|SH1_SCISSR_ORER|SH1_SCISSR_FER|SH1_SCISSR_PER);
    }

  /* Handle outgoing, transmit bytes (TDRE: Transmit Data Register Empty)
   * when TIE is enabled.  TIE is only enabled when the driver is waiting with
   * buffered data.  Since TDRE is usually true, 
   */

  if ((priv->ssr & SH1_SCISSR_TDRE) != 0 && (priv->scr & SH1_SCISCR_TIE) != 0)
    {
       /* Tx data register empty ... process outgoing bytes */

       uart_xmitchars(dev);

       /* Clear the TDR empty flag (Possibly done in up_send, will have not
        * effect if the TDR is still empty)
        */

      priv->ssr &= ~SH1_SCISSR_TDRE;
    }

  /* Clear all (clear-able) status flags.  Note that that SH-1 requires
   * that you read the bit in the "1" then write "0" to the bit in order
   * to clear it.  Any bits in the SSR that transitioned from 0->1 after
   * we read the SR will not be effected by the following:
   */

  up_serialout(priv, SH1_SCI_SSR_OFFSET, priv->ssr);
  return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t rdr;
  uint8_t ssr;

  /* Read the character from the RDR port */

  rdr  = up_serialin(priv, SH1_SCI_RDR_OFFSET);

  /* Clear all read related status in  real ssr (so that when when rxavailable
   * is called again, it will return false.
   */

  ssr = up_serialin(priv, SH1_SCI_SSR_OFFSET);
  ssr &= ~(SH1_SCISSR_RDRF|SH1_SCISSR_ORER|SH1_SCISSR_FER|SH1_SCISSR_PER);
  up_serialout(priv, SH1_SCI_SSR_OFFSET, ssr);

  /* For status, return the SSR at the time that the interrupt was received */

  *status = (uint32_t)priv->ssr << 8 | rdr;

  /* Return the received character */

  return (int)rdr;
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
  irqstate_t flags;

  /* Disable interrupts to prevent asynchronous accesses */

  flags = irqsave();

  /* Are we enabling or disabling? */

  if (enable)
    {
      /* Enable the RDR full interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->scr |= SH1_SCISCR_RIE;
#endif
    }
  else
    {
      /* Disable the RDR full interrupt */

      priv->scr &= ~SH1_SCISCR_RIE;
    }

  /* Write the modified SCR value to hardware */

  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the RDR is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  /* Return true if the RDR full bit is set in the SSR */

  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, SH1_SCI_SSR_OFFSET) & SH1_SCISSR_RDRF) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t ssr;

  /* Write the data to the TDR */

  up_serialout(priv, SH1_SCI_TDR_OFFSET, (uint8_t)ch);

  /* Clear the TDRE bit in the SSR */

  ssr  = up_serialin(priv, SH1_SCI_SSR_OFFSET);
  ssr &= ~SH1_SCISSR_TDRE;
  up_serialout(priv, SH1_SCI_SSR_OFFSET, ssr);
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

  /* Disable interrupts to prevent asynchronous accesses */

  flags = irqsave();

  /* Are we enabling or disabling? */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TDR empty interrupt */

      priv->scr |= SH1_SCISCR_TIE;

      /* If the TDR is already empty, then don't wait for the interrupt */

#if 1
      if (up_txready(dev))
        {
          /* Tx data register empty ... process outgoing bytes.  Note:
           * this could call up_txint to be called recursively.  However,
           * in this event, priv->scr should hold the correct value upon
           * return from uuart_xmitchars().
           */

          uart_xmitchars(dev);
        }
#endif
#endif
    }
  else
    {
      /* Disable the TDR empty interrupt */

      priv->scr &= ~SH1_SCISCR_TIE;
    }

  /* Write the modified SCR value to hardware */

  up_serialout(priv, SH1_SCI_SCR_OFFSET, priv->scr);
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the TDR is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return (up_serialin(priv, SH1_SCI_SSR_OFFSET) & SH1_SCISSR_TDRE) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level SCI initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_consoleinit.
 *
 ****************************************************************************/

void up_earlyconsoleinit(void)
{
  /* NOTE:  All GPIO configuration for the SCIs was performed in
   * up_lowsetup
   */

  /* Disable all SCIs */

#ifdef TTYS0_DEV
  up_disablesciint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disablesciint(TTYS1_DEV.priv, NULL);
#endif
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/

void up_consoleinit(void)
{
  /* Register the console */

#ifdef HAVE_CONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all SCIs */

#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
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
  uint8_t  scr;

  up_disablesciint(priv, &scr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout(priv, SH1_SCI_TDR_OFFSET, '\r');
    }

  up_waittxready(priv);
  up_serialout(priv, SH1_SCI_TDR_OFFSET, (uint8_t)ch);

  up_waittxready(priv);
  up_restoresciint(priv, scr);
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
