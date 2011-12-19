/****************************************************************************
 * arch/arm/src/stm32/stm32_serial.c
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_uart.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

/* Is there a USART enabled? */

#if !defined(CONFIG_STM32_USART1) && !defined(CONFIG_STM32_USART2) && !defined(CONFIG_STM32_USART3)
#  error "No USARTs enabled"
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART1)
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART2)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE) && defined(CONFIG_STM32_USART3)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  warning "No valid CONFIG_USARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef CONFIG_USE_SERIALDRIVER

/* Which USART with be tty0/console and which tty1? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart1port     /* USART1 is console */
#  define TTYS0_DEV       g_usart1port     /* USART1 is ttyS0 */
#  ifdef CONFIG_STM32_USART2
#    define TTYS1_DEV     g_usart2port     /* USART2 is ttyS1 */
#    ifdef CONFIG_STM32_USART3
#      define TTYS2_DEV   g_usart3port     /* USART3 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_STM32_USART3
#      define TTYS1_DEV   g_usart3port     /* USART3 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart2port     /* USART2 is console */
#  define TTYS0_DEV       g_usart2port     /* USART2 is ttyS0 */
#  ifdef CONFIG_STM32_USART1
#    define TTYS1_DEV     g_usart1port     /* USART1 is ttyS1 */
#    ifdef CONFIG_STM32_USART3
#      define TTYS2_DEV   g_usart3port     /* USART3 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_STM32_USART3
#      define TTYS1_DEV   g_usart3port     /* USART3 is ttyS1 */
#    else
#      undef TTYS1_DEV                     /* No ttyS1 */
#    endif
#    undef TTYS2_DEV                       /* No ttyS2 */
#  endif
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_usart3port     /* USART3 is console */
#  define TTYS0_DEV       g_usart3port     /* USART3 is ttyS0 */
#  ifdef CONFIG_STM32_USART1
#    define TTYS1_DEV     g_usart1port     /* USART1 is ttyS1 */
#    ifdef CONFIG_STM32_USART2
#      define TTYS2_DEV   g_usart2port     /* USART2 is ttyS2 */
#    else
#      undef TTYS2_DEV                     /* No ttyS2 */
#    endif
#  else
#    ifdef CONFIG_STM32_USART2
#      define TTYS1_DEV   g_usart2port     /* USART2 is ttyS1 */
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
  uint32_t usartbase; /* Base address of USART registers */
  uint32_t apbclock;  /* PCLK 1 or 2 frequency */
  uint32_t baud;      /* Configured baud */
  uint16_t ie;        /* Saved interrupt mask bits value */
  uint16_t sr;        /* Saved status bits */
  uint8_t  irq;       /* IRQ associated with this USART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
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

#ifdef CONFIG_STM32_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif
#ifdef CONFIG_STM32_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_STM32_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the STM32 USART1 ports. */

#ifdef CONFIG_STM32_USART1
static struct up_dev_s g_usart1priv =
{
  .usartbase      = STM32_USART1_BASE,
  .apbclock       = STM32_PCLK2_FREQUENCY,
  .baud           = CONFIG_USART1_BAUD,
  .irq            = STM32_IRQ_USART1,
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

/* This describes the state of the STM32 USART2 port. */

#ifdef CONFIG_STM32_USART2
static struct up_dev_s g_usart2priv =
{
  .usartbase      = STM32_USART2_BASE,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = STM32_IRQ_USART2,
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

/* This describes the state of the STM32 USART3 port. */

#ifdef CONFIG_STM32_USART3
static struct up_dev_s g_usart3priv =
{
  .usartbase      = STM32_USART3_BASE,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = STM32_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
};

static uart_dev_t g_usart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_usart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_usart3txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_usart3priv,
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

static void up_restoreusartint(struct up_dev_s *priv, uint16_t ie)
{
  uint32_t cr;

  /* Save the interrupt mask */

  priv->ie = ie;

  /* And restore the interrupt state (see the interrupt enable/usage table above) */

  cr = up_serialin(priv, STM32_USART_CR1_OFFSET);
  cr &= ~(USART_CR1_RXNEIE|USART_CR1_TXEIE|USART_CR1_PEIE);
  cr |= (ie & (USART_CR1_RXNEIE|USART_CR1_TXEIE|USART_CR1_PEIE));
  up_serialout(priv, STM32_USART_CR1_OFFSET, cr);

  cr = up_serialin(priv, STM32_USART_CR3_OFFSET);
  cr &= ~USART_CR3_EIE;
  cr |= (ie & USART_CR3_EIE);
  up_serialout(priv, STM32_USART_CR3_OFFSET, cr);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static inline void up_disableusartint(struct up_dev_s *priv, uint16_t *ie)
{
  if (ie)
    {
      uint32_t cr1;
      uint32_t cr3;

      /* USART interrupts:
       *
       * Enable             Bit Status          Meaning                        Usage
       * ------------------ --- --------------- ------------------------------ ----------
       * USART_CR1_IDLEIE    4  USART_SR_IDLE   Idle Line Detected             (not used)
       * USART_CR1_RXNEIE    5  USART_SR_RXNE   Received Data Ready to be Read
       * "              "       USART_SR_ORE    Overrun Error Detected
       * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete          (not used)
       * USART_CR1_TXEIE     7  USART_SR_TXE    Transmit Data Register Empty
       * USART_CR1_PEIE      8  USART_SR_PE     Parity Error
       *
       * USART_CR2_LBDIE     6  USART_SR_LBD    Break Flag                     (not used)
       * USART_CR3_EIE       0  USART_SR_FE     Framing Error
       * "           "          USART_SR_NE     Noise Error
       * "           "          USART_SR_ORE    Overrun Error Detected
       * USART_CR3_CTSIE    10  USART_SR_CTS    CTS flag                       (not used)
       */

      cr1 = up_serialin(priv, STM32_USART_CR1_OFFSET);
      cr3 = up_serialin(priv, STM32_USART_CR3_OFFSET);

      /* Return the current interrupt mask value for the used interrupts.  Notice
       * that this depends on the fact that none of the used interrupt enable bits
       * overlap.  This logic would fail if we needed the break interrupt!
       */

      *ie = (cr1 & (USART_CR1_RXNEIE|USART_CR1_TXEIE|USART_CR1_PEIE)) | (cr3 & USART_CR3_EIE);
    }

  /* Disable all interrupts */

  up_restoreusartint(priv, 0);
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
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint32_t usartdiv32;
  uint32_t mantissa;
  uint32_t fraction;
  uint32_t brr;
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled and the pins were configured in stm32_lowsetup().
   */

  /* Configure CR2 */
  /* Clear STOP, CLKEN, CPOL, CPHA, LBCL, and interrupt enable bits */

  regval = up_serialin(priv, STM32_USART_CR2_OFFSET);
  regval &= ~(USART_CR2_STOP_MASK|USART_CR2_CLKEN|USART_CR2_CPOL|
              USART_CR2_CPHA|USART_CR2_LBCL|USART_CR2_LBDIE);

  /* Configure STOP bits */

  if (priv->stopbits2)
    {
      regval |= USART_CR2_STOP2;
    }
  up_serialout(priv, STM32_USART_CR2_OFFSET, regval);

  /* Configure CR1 */
  /* Clear M, PCE, PS, TE, REm and all interrupt enable bits */

  regval  = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_M|USART_CR1_PCE|USART_CR1_PS|USART_CR1_TE|
              USART_CR1_RE|USART_CR1_ALLINTS);

  /* Configure word length and parity mode */

  if (priv->bits == 9)				/* Default: 1 start, 8 data, n stop */
    {
      regval |= USART_CR1_M;			/* 1 start, 9 data, n stop */
    }

  if (priv->parity == 1)			/* Odd parity */
    {
      regval |= (USART_CR1_PCE|USART_CR1_PS);
    }
  else if (priv->parity == 2)			/* Even parity */
    {
      regval |= USART_CR1_PCE;
    }
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);

  /* Configure CR3 */
  /* Clear CTSE, RTSE, and all interrupt enable bits */

  regval  = up_serialin(priv, STM32_USART_CR3_OFFSET);
  regval &= ~(USART_CR3_CTSIE|USART_CR3_CTSE|USART_CR3_RTSE|USART_CR3_EIE);

  /* Configure hardware flow control -- Not yet supported */

  up_serialout(priv, STM32_USART_CR3_OFFSET, regval);

  /* Configure the USART Baud Rate.  The baud rate for the receiver and
   * transmitter (Rx and Tx) are both set to the same value as programmed
   * in the Mantissa and Fraction values of USARTDIV.
   *
   *   baud     = fCK / (16 * usartdiv)
   *   usartdiv = fCK / (16 * baud)
   *
   * Where fCK is the input clock to the peripheral (PCLK1 for USART2, 3, 4, 5
   * or PCLK2 for USART1)
   *
   * First calculate (NOTE: all stand baud values are even so dividing by two
   * does not lose precision):
   *
   *   usartdiv32 = 32 * usartdiv = fCK / (baud/2)
   */

   usartdiv32 = priv->apbclock / (priv->baud >> 1);

   /* The mantissa part is then */

   mantissa   = usartdiv32 >> 5;
   brr        = mantissa << USART_BRR_MANT_SHIFT;

   /* The fractional remainder (with rounding) */

   fraction   = (usartdiv32 - (mantissa << 5) + 1) >> 1;
   brr       |= fraction << USART_BRR_FRAC_SHIFT;
   up_serialout(priv, STM32_USART_BRR_OFFSET, brr);

  /* Enable Rx, Tx, and the USART */

  regval      = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval     |= (USART_CR1_UE|USART_CR1_TE|USART_CR1_RE);
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);
#endif

  /* Set up the cached interrupt enables value */

  priv->ie    = 0;
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
  uint32_t regval;

  /* Disable all interrupts */

  up_disableusartint(priv, NULL);

  /* Disable Rx, Tx, and the UART */

  regval      = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval     &= ~(USART_CR1_UE|USART_CR1_TE|USART_CR1_RE);
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);
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
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the USART
        */

       up_enable_irq(priv->irq);
    }
  return ret;
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
  up_disable_irq(priv->irq);
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
  int                passes;
  bool               handled;

#ifdef CONFIG_STM32_USART1
  if (g_usart1priv.irq == irq)
    {
      dev = &g_usart1port;
    }
  else
#endif
#ifdef CONFIG_STM32_USART2
  if (g_usart2priv.irq == irq)
    {
      dev = &g_usart2port;
    }
  else
#endif
#ifdef CONFIG_STM32_USART3
  if (g_usart3priv.irq == irq)
    {
      dev = &g_usart3port;
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

      /* Get the masked USART status and clear the pending interrupts. */

      priv->sr = up_serialin(priv, STM32_USART_SR_OFFSET);

      /* USART interrupts:
       *
       * Enable             Bit Status          Meaning                         Usage
       * ------------------ --- --------------- ------------------------------- ----------
       * USART_CR1_IDLEIE    4  USART_SR_IDLE   Idle Line Detected              (not used)
       * USART_CR1_RXNEIE    5  USART_SR_RXNE   Received Data Ready to be Read
       * "              "       USART_SR_ORE    Overrun Error Detected
       * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete           (not used)
       * USART_CR1_TXEIE     7  USART_SR_TXE    Transmit Data Register Empty
       * USART_CR1_PEIE      8  USART_SR_PE     Parity Error
       *
       * USART_CR2_LBDIE     6  USART_SR_LBD    Break Flag                      (not used)
       * USART_CR3_EIE       0  USART_SR_FE     Framing Error
       * "           "          USART_SR_NE     Noise Error
       * "           "          USART_SR_ORE    Overrun Error Detected
       * USART_CR3_CTSIE    10  USART_SR_CTS    CTS flag                        (not used)
       *
       * NOTE: Some of these status bits must be cleared by explicity writing zero
       * to the SR register: USART_SR_CTS, USART_SR_LBD. Note of those are currently
       * being used.
       */

      /* Handle incoming, receive bytes (with or without timeout) */

      if ((priv->sr & USART_SR_RXNE) != 0 && (priv->ie & USART_CR1_RXNEIE) != 0)
        {
           /* Received data ready... process incoming bytes */

           uart_recvchars(dev);
           handled = true;
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & USART_SR_TXE) != 0 && (priv->ie & USART_CR1_TXEIE) != 0)
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
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#ifdef CONFIG_USART_BREAKS
  struct up_dev_s   *priv  = (struct up_dev_s*)dev->priv;
#endif
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

#ifdef CONFIG_USART_BREAKS
    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        uint32_t cr2 = up_serialin(priv, STM32_USART_CR2_OFFSET);
        up_serialout(priv, STM32_USART_CR2_OFFSET, cr2 | USART_CR2_LINEN);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        uint32_t cr1 = up_serialin(priv, STM32_USART_CR2_OFFSET);
        up_serialout(priv, STM32_USART_CR2_OFFSET, cr2 & ~USART_CR2_LINEN);
        irqrestore(flags);
      }
      break;
#endif

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
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t dr;

  /* Get the Rx byte */

  dr       = up_serialin(priv, STM32_USART_DR_OFFSET);

  /* Get the Rx byte plux error information.  Return those in status */

  *status  = priv->sr << 16 | dr;
  priv->sr = 0;

  /* Then return the actual received byte */

  return dr & 0xff;
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
  uint16_t ie;

      /* USART receive interrupts:
       *
       * Enable             Bit Status          Meaning                         Usage
       * ------------------ --- --------------- ------------------------------- ----------
       * USART_CR1_IDLEIE    4  USART_SR_IDLE   Idle Line Detected              (not used)
       * USART_CR1_RXNEIE    5  USART_SR_RXNE   Received Data Ready to be Read
       * "              "       USART_SR_ORE    Overrun Error Detected
       * USART_CR1_PEIE      8  USART_SR_PE     Parity Error
       *
       * USART_CR2_LBDIE     6  USART_SR_LBD    Break Flag                      (not used)
       * USART_CR3_EIE       0  USART_SR_FE     Framing Error
       * "           "          USART_SR_NE     Noise Error
       * "           "          USART_SR_ORE    Overrun Error Detected
       */

  ie = priv->ie;
  if (enable)
    {
      /* Receive an interrupt when their is anything in the Rx data register (or an Rx
       * timeout occurs).
       */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
#ifdef CONFIG_USART_ERRINTS
      ie |= (USART_CR1_RXNEIE|USART_CR1_PEIE|USART_CR3_EIE);
#else
      ie |= USART_CR1_RXNEIE;
#endif
#endif
    }
  else
    {
      ie &= ~(USART_CR1_RXNEIE|USART_CR1_PEIE|USART_CR3_EIE);
    }

  /* Then set the new interrupt state */

  up_restoreusartint(priv, ie);
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
  return ((up_serialin(priv, STM32_USART_SR_OFFSET) & USART_SR_RXNE) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the USART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, STM32_USART_DR_OFFSET, (uint32_t)ch);
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

  /* USART transmit interrupts:
   *
   * Enable             Bit Status          Meaning                      Usage
   * ------------------ --- --------------- ---------------------------- ----------
   * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete        (not used)
   * USART_CR1_TXEIE     7  USART_SR_TXE    Transmit Data Register Empty
   * USART_CR3_CTSIE    10  USART_SR_CTS    CTS flag                     (not used)
   */
 
  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_restoreusartint(priv, priv->ie | USART_CR1_TXEIE);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      up_restoreusartint(priv, priv->ie & ~USART_CR1_TXEIE);
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
  return ((up_serialin(priv, STM32_USART_SR_OFFSET) & USART_SR_TXE) != 0);
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
 *   before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* NOTE:  All GPIO configuration for the USARTs was performed in
   * stm32_lowsetup
   */

  /* Disable all USARTS */

  up_disableusartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableusartint(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableusartint(TTYS2_DEV.priv, NULL);
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
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint16_t ie;

  up_disableusartint(priv, &ie);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
  up_restoreusartint(priv, ie);
#endif
  return ch;
}

#else /* CONFIG_USE_SERIALDRIVER */

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

#endif /* CONFIG_USE_SERIALDRIVER */
