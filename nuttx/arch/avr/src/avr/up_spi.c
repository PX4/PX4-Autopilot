/****************************************************************************
 * arch/arm/src/avr/up_spi.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <avr/io.h>
#include <avr/power.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "avr_internal.h"

#ifdef CONFIG_AVR_SPI

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG     /* Define to enable debug */
#undef SPI_VERBOSE   /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct avr_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = avr_spiselect,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = avr_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = avr_spicmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

static struct avr_spidev_s g_spidev =
{
  .spidev            = { &g_spiops },
}; 

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  uint32_t actual;
#ifndef CONFIG_SPI_OWNBUS
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;

  /* Has the request frequency changed? */

  if (frequency != priv->frequency)
    {
#endif
      /* Read the SPI status and control registers, clearing all divider bits */

      uint8_t spcr = SPCR & ~((1 << SPR0) | (1 << SPR1));
      uint8_t spsr = SPSR & ~(1 << SPI2X);

      /* Select the best divider bits */

      if (frequency >= BOARD_CPU_CLOCK / 2)
        {
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 2;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 4)
        {
          actual = BOARD_CPU_CLOCK / 4;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 8)
        {
          spcr  |= (1 << SPR0);
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 8;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 16)
        {
          spcr  |= (1 << SPR0);
          actual = BOARD_CPU_CLOCK / 16;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 32)
        {
          spcr  |= (1 << SPR1);
          spsr  |= (1 << SPI2X);
          actual = BOARD_CPU_CLOCK / 32;
        }
      else if (frequency >= BOARD_CPU_CLOCK / 64)
        {
          spcr  |= (1 << SPR1);
          actual = BOARD_CPU_CLOCK / 64;
        }
      else /* if (frequency >= BOARD_CPU_CLOCK / 128) */
        {
          spcr  |= (1 << SPR0)|(1 << SPR1);
          actual = BOARD_CPU_CLOCK / 128;
        }

      /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
      priv->frequency = frequency;
      priv->actual    = actual;
    }
  else
    {
      actual          = priv->actual;
    }
#endif

  spidbg("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
#ifndef CONFIG_SPI_OWNBUS
  FAR struct avr_spidev_s *priv = (FAR struct avr_spidev_s *)dev;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
#endif
      uint8_t regval;

      /* Yes... Set SPI CR appropriately */

      regval = SPCR;
      regval &= ~((1 << CPOL) | (1 << CPHA));

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;
 
        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= (1 << CPHA);
          break;
 
        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= (1 << CPOL);
          break;
 
        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= ((1 << CPOL) | (1 << CPHA));
          break;
 
        default:
          DEBUGASSERT(FALSE);
          return;
        }

      SPSR = regval;

      /* Save the mode so that subsequent re-configuratins will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->mode = mode;
    }
#endif
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests (only nbits == 8 is supported)
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  DEBUGASSERT(dev && nbits == 8);
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  /* Write the data to transmitted to the SPI Data Register */

  SPDR = (uint8_t)wd;

  /* Wait for transmission to complete */

  while (!(SPSR & (1<<SPIF)));

  /* Then return the received value */

  return (uint16_t)SPDR;
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;

  spidbg("nwords: %d\n", nwords);
  while (nwords-- > 0)
  {
    (void)spi_send(dev, (uint16_t)*ptr++);
  }
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-wordselected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;

  spidbg("nwords: %d\n", nwords);
  while (nwords-- > 0)
  {
    *ptr++ = spi_send(dev, (uint16_t)0xff);
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct avr_spidev_s *priv = &g_spidev;
  irqstate_t flags;
  uint8_t regval;

  /* Make sure that clocks are provided to the SPI module */

  flags = irqsave();
  power_spi_enable();

  /* Set MOSI and SCK as outputs, all others are inputs (default on reset):
   *
   * PB3: PDO/MISO/PCINT3
   * PB2: PDI/MOSI/PCINT2
   * PB1: SCK/PCINT1
   * PB0: SS/PCINT0
   */

  DDRB |= (1 << 2) | (1 << 1);

  /* - Enable SPI
   * - Set Master
   * - Set clock rate oscillator/4
   * - Set CPOL for SPI clock mode 0
   */

  SPCR = (1 << SPE) | (1 << MSTR);

  /* Set clock rate to f(osc)/8 */
  /* SPSR |= (1 << 0); */

  /* Clear status flags by reading them */

  regval = SPSR;
  regval = SPDR;

  /* Set the initial SPI configuration */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif
  return &priv->spidev;
}
#endif /* CONFIG_AVR_SPI */

