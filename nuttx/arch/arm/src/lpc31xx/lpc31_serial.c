/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_serial.c
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

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "lpc31_cgudrvr.h"
#include "lpc31_uart.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t  ier;       /* Saved IER value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier);
static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier);
static inline void up_enablebreaks(void);
static inline void up_disablebreaks(void);
static inline void up_configbaud(void);

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

static char g_rxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_txbuffer[CONFIG_UART_TXBUFSIZE];

/* This describes the state of the single LPC313XX uart port. */

static struct up_dev_s g_uartpriv;
static uart_dev_t g_uartport =
{
  .recv     =
  {
    .size   = CONFIG_UART_RXBUFSIZE,
    .buffer = g_rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_TXBUFSIZE,
    .buffer = g_txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uartpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLINTS;
    }

  priv->ier &= ~UART_IER_ALLINTS;
  putreg32((uint32_t)priv->ier, LPC31_UART_IER);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier)
{
  priv->ier |= ier & UART_IER_ALLINTS;
  putreg32((uint32_t)priv->ier, LPC31_UART_IER);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(void)
{
  uint32_t lcr = getreg32(LPC31_UART_LCR);
  lcr |= UART_LCR_BRKCTRL;
  putreg32(lcr, LPC31_UART_LCR);
}

/****************************************************************************
 * Name: up_disablebreaks
 ****************************************************************************/

static inline void up_disablebreaks(void)
{
  uint32_t lcr = getreg32(LPC31_UART_LCR);
  lcr &= ~UART_LCR_BRKCTRL;
  putreg32(lcr, LPC31_UART_LCR);
}

/****************************************************************************
 * Name: up_configbaud
 ****************************************************************************/

static inline void up_configbaud(void)
{
  /* In a buckled-up, embedded system, there is no reason to constantly
   * calculate the following.  The calculation can be skipped if the
   * MULVAL, DIVADDVAL, and DIVISOR values are provided in the configuration
   * file.
   */

#ifndef CONFIG_LPC31_UART_MULVAL
  uint32_t qtrclk;
  uint32_t regval;

  /* Test values calculated for every multiplier/divisor combination */

  uint32_t tdiv;
  uint32_t terr;
  int      tmulval;
  int      tdivaddval;

  /* Optimal multiplier/divider values */

  uint32_t div       = 0;
  uint32_t err       = 100000;
  int      mulval    = 1;
  int      divaddval = 0;

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
  
  qtrclk = lpc31_clkfreq(CLKID_UARTUCLK, DOMAINID_UART) >> 4;

  /* Try every valid multiplier, tmulval (or until a perfect
   * match is found).
   */

  for (tmulval = 1 ; tmulval <= 15 && err > 0; tmulval++)
    {
      /* Try every valid pre-scale div, tdivaddval (or until a perfect
       * match is found).
       */

      for (tdivaddval = 0 ; tdivaddval <= 15 && err > 0; tdivaddval++)
        {
          /* Calculate the divisor with these fractional divider settings */

          uint32_t tmp = (tmulval * qtrclk) / ((tmulval + tdivaddval));
          tdiv         = (tmp + (CONFIG_UART_BAUD>>1)) / CONFIG_UART_BAUD;

          /* Check if this candidate divisor is within a valid range */

          if (tdiv > 2 && tdiv < 0x10000)
            {
              /* Calculate the actual baud and the error */

              uint32_t actualbaud = tmp / tdiv;

              if (actualbaud <= CONFIG_UART_BAUD)
                {
                  terr = CONFIG_UART_BAUD - actualbaud;
                }
              else
                {
                  terr = actualbaud - CONFIG_UART_BAUD;
                }

              /* Is this the smallest error we have encountered? */

              if (terr < err)
                {
                  /* Yes, save these settings as the new, candidate optimal settings */

                  mulval     = tmulval ;
                  divaddval = tdivaddval;
                  div       = tdiv;
                  err       = terr;
                }
            }
        }
    }

    /* Set the Divisor Latch Access Bit (DLAB) to enable DLL/DLM access */

    regval  = getreg32(LPC31_UART_LCR);
    regval |= UART_LCR_DLAB;
    putreg32(regval, LPC31_UART_LCR);

    /* Configure the MS and LS DLAB registers */

    putreg32(div & UART_DLL_MASK, LPC31_UART_DLL);
    putreg32((div >> 8) & UART_DLL_MASK, LPC31_UART_DLM);

    regval &= ~UART_LCR_DLAB;
    putreg32(regval, LPC31_UART_LCR);

    /* Configure the Fractional Divider Register (FDR) */

    putreg32((mulval    << UART_FDR_MULVAL_SHIFT) |
             (divaddval << UART_FDR_DIVADDVAL_SHIFT),
             LPC31_UART_FDR);
#else
    /* Set the Divisor Latch Access Bit (DLAB) to enable DLL/DLM access */

    regval  = getreg32(LPC31_UART_LCR);
    regval |= UART_LCR_DLAB;
    putreg32(regval, LPC31_UART_LCR);

    /* Configure the MS and LS DLAB registers */

    putreg32(CONFIG_LPC31_UART_DIVISOR & UART_DLL_MASK, LPC31_UART_DLL);
    putreg32((CONFIG_LPC31_UART_DIVISOR >> 8) & UART_DLL_MASK, LPC31_UART_DLM);

    regval &= ~UART_LCR_DLAB;
    putreg32(regval, LPC31_UART_LCR);

    /* Configure the Fractional Divider Register (FDR) */

    putreg32((CONFIG_LPC31_UART_MULVAL    << UART_FDR_MULVAL_SHIFT) |
             (CONFIG_LPC31_UART_DIVADDVAL << UART_FDR_DIVADDVAL_SHIFT),
             LPC31_UART_FDR);
#endif
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_LPC31_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t regval;

  /* Clear fifos */

  putreg32((UART_FCR_RXFIFORST|UART_FCR_TXFIFORST), LPC31_UART_FCR);

  /* Set trigger */

  putreg32((UART_FCR_FIFOENABLE|UART_FCR_RXTRIGLEVEL_16), LPC31_UART_FCR);

  /* Set up the IER */

  priv->ier = getreg32(LPC31_UART_IER);

  /* Set up the LCR */

  regval = 0;

#if CONFIG_UART_BITS == 5
  regval |= UART_LCR_WDLENSEL_5BITS;
#elif CONFIG_UART_BITS == 6
  regval |= UART_LCR_WDLENSEL_6BITS;
#elif CONFIG_UART_BITS == 7
  regval |= UART_LCR_WDLENSEL_7BITS;
#else
  regval |= UART_LCR_WDLENSEL_8BITS;
#endif

#if CONFIG_UART_2STOP > 0
  regval |= UART_LCR_NSTOPBITS;
#endif

#if CONFIG_UART_PARITY == 1
  regval |= UART_LCR_PAREN;
#elif CONFIG_UART_PARITY == 2
  regval |= (UART_LCR_PAREVEN|UART_LCR_PAREN);
#endif
  putreg32(regval, LPC31_UART_LCR);

  /* Set the BAUD divisor */

  up_configbaud();

  /* Configure the FIFOs */

  putreg32((UART_FCR_RXTRIGLEVEL_16|UART_FCR_TXFIFORST|
            UART_FCR_RXFIFORST|UART_FCR_FIFOENABLE),
           LPC31_UART_FCR);

  /* The NuttX serial driver waits for the first THRE interrrupt before
   * sending serial data... However, it appears that the lpc313x hardware
   * does not generate that interrupt until a transition from not-empty
   * to empty.  So, the current kludge here is to send one NULL at
   * startup to kick things off.
   */

  putreg32('\0', LPC31_UART_THR);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   he the setup() method is called, however, the serial console may operate in
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(LPC31_IRQ_UART, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       up_enable_irq(LPC31_IRQ_UART);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  up_disable_irq(LPC31_IRQ_UART);
  irq_detach(LPC31_IRQ_UART);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the UART irq.  It should call  uart_transmitchars
 *   or uart_receivechar to perform the appropriate data transfers.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev   = &g_uartport;
  uint8_t            status;
  int                passes;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = getreg32(LPC31_UART_IIR);

      /* The NO INTERRUPT should be zero if there are pending
       * interrupts
       */

      if ((status & UART_IIR_NOINT) != 0)
        {
          /* Break out of the loop when there is no longer a pending
           * interrupt
           */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_INTID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_INTID_RDA: /* Received Data Available */
          case UART_IIR_INTID_TIMEOUT:  /* Character time-out */
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_INTID_THRE: /* Transmitter Holding Register empty */
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts */

          case UART_IIR_INTID_MS: /* Modem status */
            {
              /* Read the modem status register (MSR) to clear */

              status = getreg32(LPC31_UART_MSR);
              fvdbg("MSR: %02x\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS: /* Receiver Line Status */
            {
              /* Read the line status register (LSR) to clear */

              status = getreg32(LPC31_UART_LSR);
              fvdbg("LSR: %02x\n", status);
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

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks();
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_disablebreaks();
        irqrestore(flags);
      }
      break;

    default:
      errno = ENOTTY;
      ret = ERROR;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  uint32_t rbr;

  *status = getreg32(LPC31_UART_LSR);
  rbr     = getreg32(LPC31_UART_RBR);
  return rbr & 0xff;
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
      priv->ier |= UART_IER_RDAINTEN;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_RDAINTEN;
    }
  putreg32(priv->ier, LPC31_UART_IER);
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
  return ((getreg32(LPC31_UART_LSR) & UART_LSR_RDR) != 0);
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
  putreg32((uint32_t)ch, LPC31_UART_THR);
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
      priv->ier |= UART_IER_THREINTEN;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_THREINTEN;
    }
  putreg32(priv->ier, LPC31_UART_IER);
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
  return ((getreg32(LPC31_UART_LSR) & UART_LSR_THRE) != 0);
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
  return ((getreg32(LPC31_UART_LSR) & UART_LSR_TEMT) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup (via up_putc).  This must
 *   be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Enable UART system clock */

  lpc31_enableclock(CLKID_UARTAPBCLK);
  lpc31_enableclock(CLKID_UARTUCLK);

  /* Disable UART interrupts */

  up_disableuartint(g_uartport.priv, NULL);

  /* Configuration the serial console */

#if defined(CONFIG_UART_SERIAL_CONSOLE)
  g_uartport.isconsole = true;
  up_setup(&g_uartport);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#if defined(CONFIG_UART_SERIAL_CONSOLE)
  (void)uart_register("/dev/console", &g_uartport);
#endif
  (void)uart_register("/dev/ttyS0", &g_uartport);
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
  struct up_dev_s *priv = &g_uartpriv;
  uint8_t ier;

  /* Keep UART interrupts disabled so that we do not interfere with the
   * serial driver.
   */

  up_disableuartint(priv, &ier);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  /* Output the character */

  up_lowputc(ch);
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

  /* Output the character */

  up_lowputc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
