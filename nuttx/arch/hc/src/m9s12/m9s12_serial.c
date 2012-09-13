/****************************************************************************
 * arch/hc/src/m9s12/m9s12_serial.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include "up_internal.h"
#include "m9s12_serial.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Is there a serial console?  Need to have (1) at least SCI enabled, (2)
 * a serial console defined on an enabled SCI, and (3) serial console and
 * file descriptors selected in the configuration.  This driver is also
 * disabled if we are using ROM-resident serial I/O
 */

#if !defined(HAVE_SERIAL_CONSOLE) || !defined(USE_SERIALDRIVER) || \
    defined(CONFIG_HCS12_SERIALMON)
#  undef USE_SERIALDRIVER
#  undef USE_EARLYSERIALINIT
#endif

#ifdef USE_SERIALDRIVER

/* Yes, which is ttyS0 and which is ttyS1 */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_sci0port      /* SCI0 is console */
#  define TTYS0_DEV       g_sci0port      /* SCI0 is ttyS0 */
#  ifndef CONFIG_SCI1_DISABLE
#    define TTYS1_DEV     g_sci1port      /* SCI1 is ttyS1 */
#  else
#    undef TTYS1_DEV                      /* No ttyS1 */
#  endif
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_sci1port      /* SCI1 is console */
#  define TTYS0_DEV       g_sci1port      /* SCI1 is ttyS0 */
#  ifndef CONFIG_SCI0_DISABLE
#    define TTYS1_DEV     g_sci0port      /* SCI0 is ttyS1 */
#  else
#    undef TTYS1_DEV                      /* No ttyS1 */
#  endif
#elif !defined(CONFIG_SCI0_DISABLE)
#  undef  CONSOLE_DEV                     /* No console device */
#  define TTYS0_DEV       g_sci1port       /* SCI1 is ttyS0 */
#  ifndef CONFIG_SCI1_DISABLE
#    define TTYS1_DEV     g_sci1port      /* SCI1 is ttyS1 */
#  else
#    undef TTYS1_DEV                      /* No ttyS1 */
#  endif
#elif !defined(CONFIG_SCI1_DISABLE)
#  undef  CONSOLE_DEV                     /* No console device */
#  define TTYS0_DEV       g_sci1port      /* SCI1 is ttyS0 */
#  undef TTYS1_DEV                        /* No ttyS1 */
#else
#  error "No valid TTY devices"
#  undef  CONSOLE_DEV                     /* No console device */
#  undef  TTYS0_DEV                       /* No ttyS0 */
#  undef  TTYS1_DEV                       /* No ttyS1 */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t baud;     /* Configured baud */
  uint16_t scibase;  /* Base address of SCI registers */
  uint8_t  im;       /* Saved CR1 interrupt enables */
  uint8_t  sr1;      /* Saved error status flags */
  uint8_t  irq;      /* IRQ associated with this SCI */
  uint8_t  parity;   /* 0=none, 1=odd, 2=even */
  uint8_t  bits;     /* Number of bits (8 or 9) */
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

struct uart_ops_s g_sci_ops =
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

#ifndef CONFIG_SCI0_DISABLE
static char g_sci0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_sci0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif
#ifndef CONFIG_SCI1_DISABLE
static char g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

/* This describes the state of the sci0 port. */

#ifndef CONFIG_SCI0_DISABLE
static struct up_dev_s g_sci0priv =
{
  .baud           = CONFIG_SCI0_BAUD,
  .uartbase       = HCS12_SCI0_BASE,
  .irq            = HCS12_IRQ_VSCI0,
  .parity         = CONFIG_SCI0_PARITY,
  .bits           = CONFIG_SCI0_BITS,
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

/* This describes the state of the sci1 port. */

#ifndef CONFIG_SCI1_DISABLE
static struct up_dev_s g_sci1priv =
{
  .baud           = CONFIG_SCI1_BAUD,
  .uartbase       = HCS12_SCI1_BASE,
  .irq            = HCS12_IRQ_VSCI1,
  .parity         = CONFIG_SCI1_PARITY,
  .bits           = CONFIG_SCI1_BITS,
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
 * Name: up_setsciint
 ****************************************************************************/

static inline void up_setsciint(struct up_dev_s *priv)
{
  uint8_t regval;
 
  regval   = up_serialin(priv, HCS12_SCI_CR2_OFFSET);
  regval  &= ~SCI_CR2_ALLINTS;
  regval  |= priv->im;
  up_serialout(priv, HCS12_SCI_CR2_OFFSET, regval);
}

/****************************************************************************
 * Name: up_disablesciint
 ****************************************************************************/

static inline void up_disablesciint(struct up_dev_s *priv, uint8_t *im)
{
  uint8_t regval;
 
  /* Return the current interrupt mask value */

  if (im)
    {
      *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;
  up_setsciint(priv);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/

static inline void up_restoresciint(struct up_dev_s *priv, uint32_t im)
{
  priv->im = im & SCI_CR2_ALLINTS;
  up_setsciint(priv);
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
      /* Check Tx data register is empty */

      if ((up_serialin(priv, HCS12_SCI_SR1_OFFSET) & SCI_SR1_TDRE) != 0)
        {
          /* The Tx is empty... return */

          break;
        }
    }

  /* If we get here, then the wait has timed out and the Tx data register
   * remains full.
   */
}
#endif

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, fifos, etc. This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  uint8_t cr1;
#endif

#ifndef CONFIG_SUPPRESS_SCI_CONFIG
  /* Calculate the BAUD divisor */

  tmp = SCIBR_VALUE(priv->baud);
  DEBUGASSERT(tmp < 0xff);

  /* Disable the SCI */

  up_serialout(priv, HCS12_SCI_CR1_OFFSET, 0);
  up_serialout(priv, HCS12_SCI_CR2_OFFSET, 0);

  /* Set the BAUD divisor */

  up_serialout(priv, HCS12_SCI_BDH_OFFSET, (uint8_t)(tmp >> 8));
  up_serialout(priv, HCS12_SCI_BDL_OFFSET, (uint8_t)(tmp & 0xff));
  
  /* Set up the SCICR1 register */

  cr1 = 0;
  if (priv->bits == 9)
    {
      cr1 |= SCI_CR1_M;
    }

  switch (priv->parity)
    {
      case 0:
      default:
        break;
      case 1:
        cr1 |= SCI_CR1_PE|SCI_CR1_PT;
        break;
      case 2:
        cr1 |= SCI_CR1_PE;
        break;
    }

  up_serialout(priv, HCS12_SCI_CR1_OFFSET, cr1);
#endif

  /* Enable Rx and Tx, keeping all interrupts disabled. We don't want
   * interrupts until the interrupt vector is attached.
   */

  priv->im = 0;
  up_serialout(priv, HCS12_SCI_CR2_OFFSET, (SCI_CR2_TE|SCI_CR2_RE));
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial port is closed.
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

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the Rx interrupt (the TX interrupt is still disabled
        * until we have something to send). 
        */

       priv->im = SCI_CR2_RIE;
       up_setsciint(priv);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disablesciint(priv, NULL);
  irq_detach(priv->irq);
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
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  int                passes;
  bool               handled;

#ifndef CONFIG_SCI0_DISABLE
  if (g_sci0priv.irq == irq)
    {
      dev = &g_sci0port;
    }
  else
#endif
#ifndef CONFIG_SCI1_DISABLE
  if (g_sci1priv.irq == irq)
    {
      dev = &g_sci1port;
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

      /* Get the masked SCI status and clear the pending interrupts. */

       priv->sr1 = up_serialin(priv, HCS12_SCI_SR1_OFFSET);

      /* Handle incoming, receive bytes */

      if ((mis & SCI_SR1_RDRF) != 0)
        {
           /* Rx buffer not empty ... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((mis & SCI_SR1_TDRE) != 0)
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
  int                ret   = OK;

  switch (cmd)
    {
    /* Add IOCTL command support here */

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
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int rxd;

  /* Return the error indications */

  *status = (uint32_t)(priv->sr1 & ~SCI_CR2_ALLINTS);
  
  /* Get the Rx data */

  rxd = (int)up_serialin(priv, HCS12_SCI_DRL_OFFSET);
  if (priv->bits == 9)
    {
      if ((up_serialin(priv, HCS12_SCI_DRH_OFFSET) & SCI_DRH_R8) != 0)
        {
          rxd |= 0x0100;
        }
    }

  return rxd;
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
      priv->im |= SCI_CR2_RIE;
#endif
    }
  else
    {
      priv->im &= ~SCI_CR2_RIE;
    }

  up_setsciint(priv);
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
  return ((up_serialin(priv, HCS12_SCI_SR1_OFFSET) & SCI_SR1_RDRF) != 0);
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
  uint8_t regval;

  if (priv->bits == 9)
    {
      regval = up_serialin(priv, HCS12_SCI_DRH_OFFSET);
      if ((ch & 0x0100) == 0)
        {
          regval &= ~SCI_DRH_T8;
        }
      else
        {
          regval |= SCI_DRH_T8;
        }
      up_serialout(priv, HCS12_SCI_DRH_OFFSET, regval);
    }

  up_serialout(priv, HCS12_SCI_DRL_OFFSET, (uint8_t)(ch & 0xff));
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
      priv->im |= SCI_CR2_TIE;
      up_setsciint(priv);

      /* Fake a TX interrupt */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~SCI_CR2_TIE;
      up_setsciint(priv);
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
  return ((up_serialin(priv, HCS12_SCI_SR1_OFFSET) & SCI_SR1_TDRE) != 0);
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
  return ((up_serialin(priv, HCS12_SCI_SR1_OFFSET) & SCI_SR1_TC) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level SCI initialization early in  debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
  /* Disable all UARTS */

  up_disablesciint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disablesciint(TTYS1_DEV.priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

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

  up_disablesciint(priv, &im);
  up_waittxnotfull(priv);
  up_send(CONSOLE_DEV, ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxnotfull(priv);
      up_send(CONSOLE_DEV, '\r');
    }

  up_waittxnotfull(priv);
  up_restoresciint(priv, im);
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
#ifdef CONFIG_ARCH_LOWPUTC
  up_lowputc(ch);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
