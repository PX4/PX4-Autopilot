/****************************************************************************
 * arch/arm/src/stm32/stm32_serial.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_uart.h"
#include "stm32_dma.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Some sanity checks *******************************************************/
/* DMA configuration */

/* If DMA is enabled on any USART, then very that other pre-requisites
 * have also been selected.
 */

#if SERIAL_HAVE_DMA

#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
/* Verify that DMA has been enabled and the DMA channel has been defined.
 */

#    if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART6_RXDMA)
#      ifndef CONFIG_STM32_DMA2
#        error STM32 USART1/6 receive DMA requires CONFIG_STM32_DMA2
#      endif
#    endif

#    if defined(CONFIG_USART2_RXDMA) || defined(CONFIG_USART3_RXDMA) || \
      defined(CONFIG_UART4_RXDMA) || defined(CONFIG_UART5_RXDMA)
#      ifndef CONFIG_STM32_DMA1
#        error STM32 USART2/3/4/5 receive DMA requires CONFIG_STM32_DMA1
#      endif
#    endif

/* Currently RS-485 support cannot be enabled when RXDMA is in use due to lack
 * of testing - RS-485 support was developed on STM32F1x
 */

#  if (defined(CONFIG_USART1_RXDMA) && defined(CONFIG_USART1_RS485)) || \
      (defined(CONFIG_USART2_RXDMA) && defined(CONFIG_USART2_RS485)) || \
      (defined(CONFIG_USART3_RXDMA) && defined(CONFIG_USART3_RS485)) || \
      (defined(CONFIG_UART4_RXDMA) && defined(CONFIG_UART4_RS485)) || \
      (defined(CONFIG_UART5_RXDMA) && defined(CONFIG_UART5_RS485)) || \
      (defined(CONFIG_USART6_RXDMA) && defined(CONFIG_USART6_RS485))
#    error "RXDMA and RS-485 cannot be enabled at the same time for the same U[S]ART"
#  endif

/* For the F4, there are alternate DMA channels for USART1 and 6.
 * Logic in the board.h file make the DMA channel selection by defining
 * the following in the board.h file.
 */

#    if defined(CONFIG_USART1_RXDMA) && !defined(DMAMAP_USART1_RX)
#      error "USART1 DMA channel not defined (DMAMAP_USART1_RX)"
#    endif

#    if defined(CONFIG_USART2_RXDMA) && !defined(DMAMAP_USART2_RX)
#      error "USART2 DMA channel not defined (DMAMAP_USART2_RX)"
#    endif

#    if defined(CONFIG_USART3_RXDMA) && !defined(DMAMAP_USART3_RX)
#      error "USART3 DMA channel not defined (DMAMAP_USART3_RX)"
#    endif

#    if defined(CONFIG_UART4_RXDMA) && !defined(DMAMAP_UART4_RX)
#      error "UART4 DMA channel not defined (DMAMAP_UART4_RX)"
#    endif

#    if defined(CONFIG_UART5_RXDMA) && !defined(DMAMAP_UART5_RX)
#      error "UART5 DMA channel not defined (DMAMAP_UART5_RX)"
#    endif

#    if defined(CONFIG_USART6_RXDMA) && !defined(DMAMAP_USART6_RX)
#      error "USART6 DMA channel not defined (DMAMAP_USART6_RX)"
#    endif

#  elif defined(CONFIG_STM32_STM32F10XX)

#    if defined(CONFIG_USART1_RXDMA) || defined(CONFIG_USART2_RXDMA) || \
      defined(CONFIG_USART3_RXDMA)
#      ifndef CONFIG_STM32_DMA1
#        error STM32 USART1/2/3 receive DMA requires CONFIG_STM32_DMA1
#      endif
#    endif

#    if defined(CONFIG_UART4_RXDMA)
#      ifndef CONFIG_STM32_DMA2
#        error STM32 USART4 receive DMA requires CONFIG_STM32_DMA2
#      endif
#    endif

/* There are no optional DMA channel assignments for the F1 */

#    define DMAMAP_USART1_RX  DMACHAN_USART1_RX
#    define DMAMAP_USART2_RX  DMACHAN_USART2_RX
#    define DMAMAP_USART3_RX  DMACHAN_USART3_RX
#    define DMAMAP_UART4_RX   DMACHAN_USART4_RX

#  endif

/* The DMA buffer size when using RX DMA to emulate a FIFO.
 *
 * When streaming data, the generic serial layer will be called
 * everytime the FIFO receives half this number of bytes.
 */

#  define RXDMA_BUFFER_SIZE   32

/* DMA priority */

#  ifndef CONFIG_USART_DMAPRIO
#    if defined(CONFIG_STM32_STM32F10XX)
#      define CONFIG_USART_DMAPRIO  DMA_CCR_PRIMED
#    elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#      define CONFIG_USART_DMAPRIO  DMA_SCR_PRIMED
#    else
#      error "Unknown STM32 DMA"
#    endif
#  endif
#  if defined(CONFIG_STM32_STM32F10XX)
#    if (CONFIG_USART_DMAPRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_USART_DMAPRIO"
#    endif
#  elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    if (CONFIG_USART_DMAPRIO & ~DMA_SCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_USART_DMAPRIO"
#    endif
#  else
#    error "Unknown STM32 DMA"
#  endif

/* DMA control word */

#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#    define SERIAL_DMA_CONTROL_WORD      \
                (DMA_SCR_DIR_P2M       | \
                 DMA_SCR_CIRC          | \
                 DMA_SCR_MINC          | \
                 DMA_SCR_PSIZE_8BITS   | \
                 DMA_SCR_MSIZE_8BITS   | \
                 CONFIG_USART_DMAPRIO  | \
                 DMA_SCR_PBURST_SINGLE | \
                 DMA_SCR_MBURST_SINGLE)
#  else
#    define SERIAL_DMA_CONTROL_WORD      \
                (DMA_CCR_CIRC          | \
                 DMA_CCR_MINC          | \
                 DMA_CCR_PSIZE_8BITS   | \
                 DMA_CCR_MSIZE_8BITS   | \
                 CONFIG_USART_DMAPRIO)
# endif

#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_PM_SERIAL_ACTIVITY)
#  define CONFIG_PM_SERIAL_ACTIVITY 10
#endif

#ifdef USE_SERIALDRIVER
#ifdef HAVE_UART

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  struct uart_dev_s dev;       /* Generic UART device */
  uint16_t          ie;        /* Saved interrupt mask bits value */
  uint16_t          sr;        /* Saved status bits */

  /* If termios are supported, then the following fields may vary at
   * runtime.
   */

#ifdef CONFIG_SERIAL_TERMIOS
  uint8_t           parity;    /* 0=none, 1=odd, 2=even */
  uint8_t           bits;      /* Number of bits (7 or 8) */
  bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
  uint32_t          baud;      /* Configured baud */
#else
  const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
  const uint8_t     bits;      /* Number of bits (7 or 8) */
  const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
  const uint32_t    baud;      /* Configured baud */
#endif

  const uint8_t     irq;       /* IRQ associated with this USART */
  const uint32_t    apbclock;  /* PCLK 1 or 2 frequency */
  const uint32_t    usartbase; /* Base address of USART registers */
  const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
  const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
  const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
  const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */

#ifdef SERIAL_HAVE_DMA
  const unsigned int rxdma_channel; /* DMA channel assigned */
#endif

  int (* const vector)(int irq, void *context); /* Interrupt handler */

  /* RX DMA state */

#ifdef SERIAL_HAVE_DMA
  DMA_HANDLE        rxdma;     /* currently-open receive DMA stream */
  bool              rxenable;  /* DMA-based reception en/disable */
  uint32_t          rxdmanext; /* Next byte in the DMA buffer to be read */
  char       *const rxfifo;    /* Receive DMA buffer */
#endif

#ifdef HAVE_RS485
  const uint32_t    rs485_dir_gpio; /* U[S]ART RS-485 DIR GPIO pin configuration */
  const bool        rs485_dir_polarity; /* U[S]ART RS-485 DIR pin state for TX enabled */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_setspeed(struct uart_dev_s *dev);
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt_common(struct up_dev_s *dev);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifndef SERIAL_HAVE_ONLY_DMA
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_DMA
static int  up_dma_setup(struct uart_dev_s *dev);
static void up_dma_shutdown(struct uart_dev_s *dev);
static int  up_dma_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool up_dma_rxavailable(struct uart_dev_s *dev);

static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate);
#endif

#ifdef CONFIG_STM32_USART1
static int up_interrupt_usart1(int irq, void *context);
#endif
#ifdef CONFIG_STM32_USART2
static int up_interrupt_usart2(int irq, void *context);
#endif
#ifdef CONFIG_STM32_USART3
static int up_interrupt_usart3(int irq, void *context);
#endif
#ifdef CONFIG_STM32_UART4
static int up_interrupt_uart4(int irq, void *context);
#endif
#ifdef CONFIG_STM32_UART5
static int up_interrupt_uart5(int irq, void *context);
#endif
#ifdef CONFIG_STM32_USART6
static int up_interrupt_usart6(int irq, void *context);
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static const struct uart_ops_s g_uart_ops =
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
#endif

#ifdef SERIAL_HAVE_DMA
static const struct uart_ops_s g_uart_dma_ops =
{
  .setup          = up_dma_setup,
  .shutdown       = up_dma_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_dma_receive,
  .rxint          = up_dma_rxint,
  .rxavailable    = up_dma_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};
#endif

/* I/O buffers */

#ifdef CONFIG_STM32_USART1
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
# ifdef CONFIG_USART1_RXDMA
static char g_usart1rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32_USART2
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
# ifdef CONFIG_USART2_RXDMA
static char g_usart2rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32_USART3
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
# ifdef CONFIG_USART3_RXDMA
static char g_usart3rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32_UART4
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
# ifdef CONFIG_UART4_RXDMA
static char g_uart4rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32_UART5
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
# ifdef CONFIG_UART5_RXDMA
static char g_uart5rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

#ifdef CONFIG_STM32_USART6
static char g_usart6rxbuffer[CONFIG_USART6_RXBUFSIZE];
static char g_usart6txbuffer[CONFIG_USART6_TXBUFSIZE];
# ifdef CONFIG_USART6_RXDMA
static char g_usart6rxfifo[RXDMA_BUFFER_SIZE];
# endif
#endif

/* This describes the state of the STM32 USART1 ports. */

#ifdef CONFIG_STM32_USART1
static struct up_dev_s g_usart1priv =
{
  .dev =
    {
#if CONSOLE_UART == 1
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART1_RXBUFSIZE,
        .buffer  = g_usart1rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART1_TXBUFSIZE,
        .buffer  = g_usart1txbuffer,
      },
#ifdef CONFIG_USART1_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart1priv,
    },

  .irq           = STM32_IRQ_USART1,
  .parity        = CONFIG_USART1_PARITY,
  .bits          = CONFIG_USART1_BITS,
  .stopbits2     = CONFIG_USART1_2STOP,
  .baud          = CONFIG_USART1_BAUD,
  .apbclock      = STM32_PCLK2_FREQUENCY,
  .usartbase     = STM32_USART1_BASE,
  .tx_gpio       = GPIO_USART1_TX,
  .rx_gpio       = GPIO_USART1_RX,
#ifdef GPIO_USART1_CTS
  .cts_gpio      = GPIO_USART1_CTS,
#endif
#ifdef GPIO_USART1_RTS
  .rts_gpio      = GPIO_USART1_RTS,
#endif
#ifdef CONFIG_USART1_RXDMA
  .rxdma_channel = DMAMAP_USART1_RX,
  .rxfifo        = g_usart1rxfifo,
#endif
  .vector        = up_interrupt_usart1,

#ifdef CONFIG_USART1_RS485
  .rs485_dir_gpio = GPIO_USART1_RS485_DIR,
#  if (CONFIG_USART1_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART2 port. */

#ifdef CONFIG_STM32_USART2
static struct up_dev_s g_usart2priv =
{
  .dev =
    {
#if CONSOLE_UART == 2
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART2_RXBUFSIZE,
        .buffer  = g_usart2rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART2_TXBUFSIZE,
        .buffer  = g_usart2txbuffer,
      },
#ifdef CONFIG_USART2_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart2priv,
    },

  .irq           = STM32_IRQ_USART2,
  .parity        = CONFIG_USART2_PARITY,
  .bits          = CONFIG_USART2_BITS,
  .stopbits2     = CONFIG_USART2_2STOP,
  .baud          = CONFIG_USART2_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_USART2_BASE,
  .tx_gpio       = GPIO_USART2_TX,
  .rx_gpio       = GPIO_USART2_RX,
#ifdef GPIO_USART2_CTS
  .cts_gpio      = GPIO_USART2_CTS,
#endif
#ifdef GPIO_USART2_RTS
  .rts_gpio      = GPIO_USART2_RTS,
#endif
#ifdef CONFIG_USART2_RXDMA
  .rxdma_channel = DMAMAP_USART2_RX,
  .rxfifo        = g_usart2rxfifo,
#endif
  .vector        = up_interrupt_usart2,

#ifdef CONFIG_USART2_RS485
  .rs485_dir_gpio = GPIO_USART2_RS485_DIR,
#  if (CONFIG_USART2_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART3 port. */

#ifdef CONFIG_STM32_USART3
static struct up_dev_s g_usart3priv =
{
  .dev =
    {
#if CONSOLE_UART == 3
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_USART3_RXBUFSIZE,
        .buffer  = g_usart3rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_USART3_TXBUFSIZE,
        .buffer  = g_usart3txbuffer,
      },
#ifdef CONFIG_USART3_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_usart3priv,
    },

  .irq           = STM32_IRQ_USART3,
  .parity        = CONFIG_USART3_PARITY,
  .bits          = CONFIG_USART3_BITS,
  .stopbits2     = CONFIG_USART3_2STOP,
  .baud          = CONFIG_USART3_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_USART3_BASE,
  .tx_gpio       = GPIO_USART3_TX,
  .rx_gpio       = GPIO_USART3_RX,
#ifdef GPIO_USART3_CTS
  .cts_gpio      = GPIO_USART3_CTS,
#endif
#ifdef GPIO_USART3_RTS
  .rts_gpio      = GPIO_USART3_RTS,
#endif
#ifdef CONFIG_USART3_RXDMA
  .rxdma_channel = DMAMAP_USART3_RX,
  .rxfifo        = g_usart3rxfifo,
#endif
  .vector        = up_interrupt_usart3,

#ifdef CONFIG_USART3_RS485
  .rs485_dir_gpio = GPIO_USART3_RS485_DIR,
#  if (CONFIG_USART3_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART4 port. */

#ifdef CONFIG_STM32_UART4
static struct up_dev_s g_uart4priv =
{
  .dev =
    {
#if CONSOLE_UART == 4
      .isconsole = true,
#endif
      .recv      =
      {
        .size    = CONFIG_UART4_RXBUFSIZE,
        .buffer  = g_uart4rxbuffer,
      },
      .xmit      =
      {
        .size    = CONFIG_UART4_TXBUFSIZE,
        .buffer  = g_uart4txbuffer,
      },
#ifdef CONFIG_UART4_RXDMA
      .ops       = &g_uart_dma_ops,
#else
      .ops       = &g_uart_ops,
#endif
      .priv      = &g_uart4priv,
    },

  .irq           = STM32_IRQ_UART4,
  .parity        = CONFIG_UART4_PARITY,
  .bits          = CONFIG_UART4_BITS,
  .stopbits2     = CONFIG_UART4_2STOP,
  .baud          = CONFIG_UART4_BAUD,
  .apbclock      = STM32_PCLK1_FREQUENCY,
  .usartbase     = STM32_UART4_BASE,
  .tx_gpio       = GPIO_UART4_TX,
  .rx_gpio       = GPIO_UART4_RX,
#ifdef GPIO_UART4_CTS
  .cts_gpio      = GPIO_UART4_CTS,
#endif
#ifdef GPIO_UART4_RTS
  .rts_gpio      = GPIO_UART4_RTS,
#endif
#ifdef CONFIG_UART4_RXDMA
  .rxdma_channel = DMAMAP_UART4_RX,
  .rxfifo        = g_uart4rxfifo,
#endif
  .vector        = up_interrupt_uart4,

#ifdef CONFIG_UART4_RS485
  .rs485_dir_gpio = GPIO_UART4_RS485_DIR,
#  if (CONFIG_UART4_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 UART5 port. */

#ifdef CONFIG_STM32_UART5
static struct up_dev_s g_uart5priv =
{
  .dev =
    {
#if CONSOLE_UART == 5
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_UART5_RXBUFSIZE,
        .buffer = g_uart5rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_UART5_TXBUFSIZE,
        .buffer = g_uart5txbuffer,
      },
#ifdef CONFIG_UART5_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_uart5priv,
    },

  .irq            = STM32_IRQ_UART5,
  .parity         = CONFIG_UART5_PARITY,
  .bits           = CONFIG_UART5_BITS,
  .stopbits2      = CONFIG_UART5_2STOP,
  .baud           = CONFIG_UART5_BAUD,
  .apbclock       = STM32_PCLK1_FREQUENCY,
  .usartbase      = STM32_UART5_BASE,
  .tx_gpio        = GPIO_UART5_TX,
  .rx_gpio        = GPIO_UART5_RX,
#ifdef GPIO_UART5_CTS
  .cts_gpio       = GPIO_UART5_CTS,
#endif
#ifdef GPIO_UART5_RTS
  .rts_gpio       = GPIO_UART5_RTS,
#endif
#ifdef CONFIG_UART5_RXDMA
  .rxdma_channel = DMAMAP_UART5_RX,
  .rxfifo        = g_uart5rxfifo,
#endif
  .vector         = up_interrupt_uart5,

#ifdef CONFIG_UART5_RS485
  .rs485_dir_gpio = GPIO_UART5_RS485_DIR,
#  if (CONFIG_UART5_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This describes the state of the STM32 USART6 port. */

#ifdef CONFIG_STM32_USART6
static struct up_dev_s g_usart6priv =
{
  .dev =
    {
#if CONSOLE_UART == 6
      .isconsole = true,
#endif
      .recv     =
      {
        .size   = CONFIG_USART6_RXBUFSIZE,
        .buffer = g_usart6rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_USART6_TXBUFSIZE,
        .buffer = g_usart6txbuffer,
      },
#ifdef CONFIG_USART6_RXDMA
      .ops      = &g_uart_dma_ops,
#else
      .ops      = &g_uart_ops,
#endif
      .priv     = &g_usart6priv,
    },

  .irq            = STM32_IRQ_USART6,
  .parity         = CONFIG_USART6_PARITY,
  .bits           = CONFIG_USART6_BITS,
  .stopbits2      = CONFIG_USART6_2STOP,
  .baud           = CONFIG_USART6_BAUD,
  .apbclock       = STM32_PCLK2_FREQUENCY,
  .usartbase      = STM32_USART6_BASE,
  .tx_gpio        = GPIO_USART6_TX,
  .rx_gpio        = GPIO_USART6_RX,
#ifdef GPIO_USART6_CTS
  .cts_gpio       = GPIO_USART6_CTS,
#endif
#ifdef GPIO_USART6_RTS
  .rts_gpio       = GPIO_USART6_RTS,
#endif
#ifdef CONFIG_USART6_RXDMA
  .rxdma_channel = DMAMAP_USART6_RX,
  .rxfifo        = g_usart6rxfifo,
#endif
  .vector         = up_interrupt_usart6,

#ifdef CONFIG_USART6_RS485
  .rs485_dir_gpio = GPIO_USART6_RS485_DIR,
#  if (CONFIG_USART6_RS485_DIR_POLARITY == 0)
  .rs485_dir_polarity = false,
#  else
  .rs485_dir_polarity = true,
#  endif
#endif
};
#endif

/* This table lets us iterate over the configured USARTs */

static struct up_dev_s *uart_devs[STM32_NUSART] =
{
#ifdef CONFIG_STM32_USART1
  [0] = &g_usart1priv,
#endif
#ifdef CONFIG_STM32_USART2
  [1] = &g_usart2priv,
#endif
#ifdef CONFIG_STM32_USART3
  [2] = &g_usart3priv,
#endif
#ifdef CONFIG_STM32_UART4
  [3] = &g_uart4priv,
#endif
#ifdef CONFIG_STM32_UART5
  [4] = &g_uart5priv,
#endif
#ifdef CONFIG_STM32_USART6
  [5] = &g_usart6priv,
#endif
};

#ifdef CONFIG_PM
static  struct pm_callback_s g_serialcb =
{
  .notify  = up_pm_notify,
  .prepare = up_pm_prepare,
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
  cr &= ~(USART_CR1_USED_INTS);
  cr |= (ie & (USART_CR1_USED_INTS));
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
       * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete          (used only for RS-485)
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

      *ie = (cr1 & (USART_CR1_USED_INTS)) | (cr3 & USART_CR3_EIE);
    }

  /* Disable all interrupts */

  up_restoreusartint(priv, 0);
}

/****************************************************************************
 * Name: up_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_nextrx(struct up_dev_s *priv)
{
  size_t dmaresidual;

  dmaresidual = stm32_dmaresidual(priv->rxdma);

  return (RXDMA_BUFFER_SIZE - (int)dmaresidual);
}
#endif

/****************************************************************************
 * Name: up_setspeed
 *
 * Description:
 *   Set the serial line speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_setspeed(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t usartdiv32;
  uint32_t mantissa;
  uint32_t fraction;
  uint32_t brr;

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
}
#endif

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
  uint32_t regval;

  /* Note: The logic here depends on the fact that that the USART module
   * was enabled in stm32_lowsetup().
   */

  /* Configure pins for USART use */

  stm32_configgpio(priv->tx_gpio);
  stm32_configgpio(priv->rx_gpio);

  if (priv->cts_gpio != 0)
    {
      stm32_configgpio(priv->cts_gpio);
    }

  if (priv->rts_gpio != 0)
    {
      stm32_configgpio(priv->rts_gpio);
    }

#if HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    {
      stm32_configgpio(priv->rs485_dir_gpio);
      stm32_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
    }
#endif

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

  if (priv->bits == 9)         /* Default: 1 start, 8 data, n stop */
    {
      regval |= USART_CR1_M;   /* 1 start, 9 data, n stop */
    }

  if (priv->parity == 1)       /* Odd parity */
    {
      regval |= (USART_CR1_PCE|USART_CR1_PS);
    }
  else if (priv->parity == 2)  /* Even parity */
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

  /* Configure the USART Baud Rate. */

  up_setspeed(dev);

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
 * Name: up_dma_setup
 *
 * Description:
 *   Configure the USART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int result;
  uint32_t regval;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {    
      result = up_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

  /* Acquire the DMA channel.  This should always succeed. */

  priv->rxdma = stm32_dmachannel(priv->rxdma_channel);

  /* Configure for circular DMA reception into the RX fifo */

  stm32_dmasetup(priv->rxdma,
                 priv->usartbase + STM32_USART_DR_OFFSET,
                 (uint32_t)priv->rxfifo,
                 RXDMA_BUFFER_SIZE,
                 SERIAL_DMA_CONTROL_WORD);

  /* Reset our DMA shadow pointer to match the address just
   * programmed above.
   */

  priv->rxdmanext = 0;

  /* Enable receive DMA for the UART */

  regval  = up_serialin(priv, STM32_USART_CR3_OFFSET);
  regval |= USART_CR3_DMAR;
  up_serialout(priv, STM32_USART_CR3_OFFSET, regval);

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  stm32_dmastart(priv->rxdma, up_dma_rxcallback, (void *)priv, true);

  return OK;
}
#endif

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

  regval  = up_serialin(priv, STM32_USART_CR1_OFFSET);
  regval &= ~(USART_CR1_UE|USART_CR1_TE|USART_CR1_RE);
  up_serialout(priv, STM32_USART_CR1_OFFSET, regval);
}

/****************************************************************************
 * Name: up_dma_shutdown
 *
 * Description:
 *   Disable the USART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Perform the normal UART shutdown */

  up_shutdown(dev);

  /* Stop the DMA channel */

  stm32_dmastop(priv->rxdma);

  /* Release the DMA channel */

  stm32_dmafree(priv->rxdma);
  priv->rxdma = NULL;
}
#endif

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

  ret = irq_attach(priv->irq, priv->vector);
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
 * Name: up_interrupt_common
 *
 * Description:
 *   This is the USART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   approprite uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt_common(struct up_dev_s *priv)
{
  int  passes;
  bool handled;

  /* Report serial activity to the power management logic */

#if defined(CONFIG_PM) && CONFIG_PM_SERIAL_ACTIVITY > 0
  pm_activity(CONFIG_PM_SERIAL_ACTIVITY);
#endif

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  handled = true;
  for (passes = 0; passes < 256 && handled; passes++)
    {
      handled = false;

      /* Get the masked USART status word. */

      priv->sr = up_serialin(priv, STM32_USART_SR_OFFSET);

      /* USART interrupts:
       *
       * Enable             Bit Status          Meaning                         Usage
       * ------------------ --- --------------- ------------------------------- ----------
       * USART_CR1_IDLEIE    4  USART_SR_IDLE   Idle Line Detected              (not used)
       * USART_CR1_RXNEIE    5  USART_SR_RXNE   Received Data Ready to be Read
       * "              "       USART_SR_ORE    Overrun Error Detected
       * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete           (used only for RS-485)
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

#ifdef HAVE_RS485
      /* Transmission of whole buffer is over - TC is set, TXEIE is cleared.
       * Note - this should be first, to have the most recent TC bit value from
       * SR register - sending data affects TC, but without refresh we will not
       * know that...
       */

      if ((priv->sr & USART_SR_TC) != 0 && (priv->ie & USART_CR1_TCIE) != 0 &&
          (priv->ie & USART_CR1_TXEIE) == 0)
        {
          stm32_gpiowrite(priv->rs485_dir_gpio, !priv->rs485_dir_polarity);
          up_restoreusartint(priv, priv->ie & ~USART_CR1_TCIE);
        }
#endif

      /* Handle incoming, receive bytes. */

      if ((priv->sr & USART_SR_RXNE) != 0 && (priv->ie & USART_CR1_RXNEIE) != 0)
        {
           /* Received data ready... process incoming bytes.  NOTE the check for
            * RXNEIE:  We cannot call uart_recvchards of RX interrupts are disabled.
            */

           uart_recvchars(&priv->dev);
           handled = true;
        }

       /* We may still have to read from the DR register to clear any pending
        * error conditions.
        */

      else if ((priv->sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0)
        {
          /* If an error occurs, read from DR to clear the error (data has
           * been lost).  If ORE is set along with RXNE then it tells you
           * that the byte *after* the one in the data register has been
           * lost, but the data register value is correct.  That case will
           * be handled above if interrupts are enabled. Otherwise, that
           * good byte will be lost.
           */

          (void)up_serialin(priv, STM32_USART_DR_OFFSET);
        }

      /* Handle outgoing, transmit bytes */

      if ((priv->sr & USART_SR_TXE) != 0 && (priv->ie & USART_CR1_TXEIE) != 0)
        {
           /* Transmit data register empty ... process outgoing bytes */

           uart_xmitchars(&priv->dev);
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
#ifdef CONFIG_SERIAL_TERMIOS
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

#ifdef CONFIG_STM32_USART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        /* Change the TX port to be open-drain/push-pull and enable/disable
         * half-duplex mode.
         */

        uint32_t cr = up_serialin(priv, STM32_USART_CR3_OFFSET);

        if (arg == SER_SINGLEWIRE_ENABLED)
          {
            stm32_configgpio(priv->tx_gpio | GPIO_OPENDRAIN);
            cr |= USART_CR3_HDSEL;
          }
        else
          {
            stm32_configgpio(priv->tx_gpio | GPIO_PUSHPULL);
            cr &= ~USART_CR3_HDSEL;
          }

        up_serialout(priv, STM32_USART_CR3_OFFSET, cr);
      }
     break;    
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios*)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Other termios fields are not yet returned.
         * Note that only cfsetospeed is not necessary because we have
         * knowledge that only one speed is supported.
         */

        cfsetispeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios*)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* TODO:  Handle other termios settings.
         * Note that only cfgetispeed is used besued we have knowledge
         * that only one speed is supported.
         */

        priv->baud = cfgetispeed(termiosp);
        up_setspeed(dev);
      }
      break;
#endif

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

#ifndef SERIAL_HAVE_ONLY_DMA
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
#endif

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;
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

  flags = irqsave();
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
  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_DMA
static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, STM32_USART_SR_OFFSET) & USART_SR_RXNE) != 0);
}
#endif

/****************************************************************************
 * Name: up_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the USART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static int up_dma_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int c = 0;

  if (up_dma_nextrx(priv) != priv->rxdmanext)
    {
      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;
      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  return c;
}
#endif

/****************************************************************************
 * Name: up_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* En/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;
}
#endif

/****************************************************************************
 * Name: up_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static bool up_dma_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (up_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

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
#ifdef HAVE_RS485
  if (priv->rs485_dir_gpio != 0)
    stm32_gpiowrite(priv->rs485_dir_gpio, priv->rs485_dir_polarity);
#endif
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
   * USART_CR1_TCIE      6  USART_SR_TC     Transmission Complete        (used only for RS-485)
   * USART_CR1_TXEIE     7  USART_SR_TXE    Transmit Data Register Empty
   * USART_CR3_CTSIE    10  USART_SR_CTS    CTS flag                     (not used)
   */

  flags = irqsave();
  if (enable)
    {
      /* Set to receive an interrupt when the TX data register is empty */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      uint16_t ie = priv->ie | USART_CR1_TXEIE;

      /* If RS-485 is supported on this U[S]ART, then also enable the
       * transmission complete interrupt.
       */

#  ifdef HAVE_RS485
      if (priv->rs485_dir_gpio != 0)
        {
          ie |= USART_CR1_TCIE;
        }
#  endif

      up_restoreusartint(priv, ie);

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
 * Name: up_interrupt_u[s]art[n]
 *
 * Description:
 *   Interrupt handlers for U[S]ART[n] where n=1,..,6.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_USART1
static int up_interrupt_usart1(int irq, void *context)
{
  return up_interrupt_common(&g_usart1priv);
}
#endif

#ifdef CONFIG_STM32_USART2
static int up_interrupt_usart2(int irq, void *context)
{
  return up_interrupt_common(&g_usart2priv);
}
#endif

#ifdef CONFIG_STM32_USART3
static int up_interrupt_usart3(int irq, void *context)
{
  return up_interrupt_common(&g_usart3priv);
}
#endif

#ifdef CONFIG_STM32_UART4
static int up_interrupt_uart4(int irq, void *context)
{
  return up_interrupt_common(&g_uart4priv);
}
#endif

#ifdef CONFIG_STM32_UART5
static int up_interrupt_uart5(int irq, void *context)
{
  return up_interrupt_common(&g_uart5priv);
}
#endif

#ifdef CONFIG_STM32_USART6
static int up_interrupt_usart6(int irq, void *context)
{
  return up_interrupt_common(&g_usart6priv);
}
#endif

/****************************************************************************
 * Name: up_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
static void up_dma_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct up_dev_s *priv = (struct up_dev_s*)arg;

  if (priv->rxenable && up_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);
    }
}
#endif

#endif /* HAVE UART */

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         strucure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */

        }
        break;

      case(PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */

        }
        break;

      case(PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */

        }
        break;

      case(PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */

        }
        break;

      default:
        /* Should not get here */
        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         strucure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

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
#ifdef HAVE_UART
  unsigned i;

  /* Disable all USART interrupts */

  for (i = 0; i < STM32_NUSART; i++)
    {
      if (uart_devs[i])
        {
          up_disableusartint(uart_devs[i], NULL);
        }
    }

  /* Configure whichever one is the console */

#if CONSOLE_UART > 0
  up_setup(&uart_devs[CONSOLE_UART - 1]->dev);
#endif
#endif /* HAVE UART */
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
#ifdef HAVE_UART
  char devname[16];
  unsigned i, j;
#ifdef CONFIG_PM
  int ret;
#endif

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_serialcb);
  DEBUGASSERT(ret == OK);
#endif

  /* Register the console */

#if CONSOLE_UART > 0
  (void)uart_register("/dev/console", &uart_devs[CONSOLE_UART - 1]->dev);
  (void)uart_register("/dev/ttyS0",   &uart_devs[CONSOLE_UART - 1]->dev);

  /* If we need to re-initialise the console to enable DMA do that here. */

# ifdef SERIAL_HAVE_CONSOLE_DMA
  up_dma_setup(&uart_devs[CONSOLE_UART - 1]->dev);
# endif

#endif /* CONSOLE_UART > 0 */

  /* Register all remaining USARTs */

  strcpy(devname, "/dev/ttySx");

  for (i = 0, j = 1; i < STM32_NUSART; i++)
    {

      /* don't create a device for the console - we did that above */

      if ((uart_devs[i] == 0) || (uart_devs[i]->dev.isconsole))
        {
          continue;
        }

      /* register USARTs as devices in increasing order */

      devname[9] = '0' + j++;
      (void)uart_register(devname, &uart_devs[i]->dev);
    }
#endif /* HAVE UART */
}

/****************************************************************************
 * Name: stm32_serial_dma_poll
 *
 * Description:
 *   Checks receive DMA buffers for received bytes that have not accumulated
 *   to the point where the DMA half/full interrupt has triggered.
 *
 *   This function should be called from a timer or other periodic context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_DMA
void stm32_serial_dma_poll(void)
{
    irqstate_t flags;

    flags = irqsave();

#ifdef CONFIG_USART1_RXDMA
  if (g_usart1priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart1priv.rxdma, 0, &g_usart1priv);
    }
#endif

#ifdef CONFIG_USART2_RXDMA
  if (g_usart2priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart2priv.rxdma, 0, &g_usart2priv);
    }
#endif

#ifdef CONFIG_USART3_RXDMA
  if (g_usart3priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart3priv.rxdma, 0, &g_usart3priv);
    }
#endif

#ifdef CONFIG_UART4_RXDMA
  if (g_uart4priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart4priv.rxdma, 0, &g_uart4priv);
    }
#endif

#ifdef CONFIG_UART5_RXDMA
  if (g_uart5priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_uart5priv.rxdma, 0, &g_uart5priv);
    }
#endif

#ifdef CONFIG_USART6_RXDMA
  if (g_usart6priv.rxdma != NULL)
    {
      up_dma_rxcallback(g_usart6priv.rxdma, 0, &g_usart6priv);
    }
#endif

  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct up_dev_s *priv = uart_devs[CONSOLE_UART - 1];
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
#if CONSOLE_UART > 0
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
