/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_ssp.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "lpc43_syscon.h"
#include "lpc43_pinconn.h"
#include "lpc43_ssp.h"

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_SSP_DEBUG - Define to enable basic SSP debug
 * CONFIG_SSP_VERBOSE - Define to enable verbose SSP debug
 */

#ifdef CONFIG_SSP_DEBUG
#  define sspdbg  lldbg
#  ifdef CONFIG_SSP_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef CONFIG_SSP_VERBOSE
#  define sspdbg(x...)
#  define spivdbg(x...)
#endif

/* SSP Clocking.
 *
 * The CPU clock by 1, 2, 4, or 8 to get the SSP peripheral clock (SSP_CLOCK).
 * SSP_CLOCK may be further divided by 2-254 to get the SSP clock.  If we
 * want a usable range of 4KHz to 25MHz for the SSP, then:
 *
 * 1. SSPCLK must be greater than (2*25MHz) = 50MHz, and
 * 2. SSPCLK must be less than (254*40Khz) = 101.6MHz.
 *
 * If we assume that CCLK less than or equal to 100MHz, we can just
 * use the CCLK undivided to get the SSP_CLOCK.
 */

#if LPC43_CCLK > 100000000
#  error "CCLK <= 100,000,000 assumed"
#endif

#define SSP_PCLKSET_DIV    SYSCON_PCLKSEL_CCLK
#define SSP_CLOCK          LPC43_CCLK

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes the state of the SSP driver */

struct lpc43_sspdev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         sspbase;    /* SPIn base address */
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  uint8_t          sspirq;     /* SPI IRQ number */
#endif
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (4 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t ssp_getreg(FAR struct lpc43_sspdev_s *priv, uint8_t offset);
static inline void ssp_putreg(FAR struct lpc43_sspdev_s *priv, uint8_t offset,
                                 uint32_t value);

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      ssp_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t ssp_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void     ssp_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     ssp_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t ssp_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void     ssp_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     ssp_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);

/* Initialization */

#ifdef CONFIG_LPC43_SSP0
static inline FAR struct lpc43_sspdev_s *lpc43_ssp0initialize(void);
#endif
#ifdef CONFIG_LPC43_SSP1
static inline FAR struct lpc43_sspdev_s *lpc43_ssp1initialize(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LPC43_SSP0
static const struct spi_ops_s g_spi0ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = ssp_lock,
#endif
  .select            = lpc43_ssp0select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
  .status            = lpc43_ssp0status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc43_ssp0cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc43_ssp0register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc43_sspdev_s g_ssp0dev =
{
  .spidev            = { &g_spi0ops },
  .sspbase           = LPC43_SSP0_BASE,
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  .sspirq            = LPC43_IRQ_SSP0,
#endif
}; 
#endif /* CONFIG_LPC43_SSP0 */

#ifdef CONFIG_LPC43_SSP1
static const struct spi_ops_s g_spi1ops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = ssp_lock,
#endif
  .select            = lpc43_ssp1select,   /* Provided externally */
  .setfrequency      = ssp_setfrequency,
  .setmode           = ssp_setmode,
  .setbits           = ssp_setbits,
  .status            = lpc43_ssp1status,   /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc43_ssp1cmddata,  /* Provided externally */
#endif
  .send              = ssp_send,
  .sndblock          = ssp_sndblock,
  .recvblock         = ssp_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc43_ssp1register, /* Provided externally */
#else
  .registercallback  = 0,                  /* Not implemented */
#endif
};

static struct lpc43_sspdev_s g_ssp1dev =
{
  .spidev            = { &g_spi1ops },
  .sspbase           = LPC43_SSP1_BASE,
#ifdef CONFIG_LPC43_SSP_INTERRUPTS
  .sspirq            = LPC43_IRQ_SSP1,
#endif
}; 
#endif /* CONFIG_LPC43_SSP1 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssp_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t ssp_getreg(FAR struct lpc43_sspdev_s *priv, uint8_t offset)
{
  return getreg32(priv->sspbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: ssp_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

static inline void ssp_putreg(FAR struct lpc43_sspdev_s *priv, uint8_t offset, uint32_t value)
{
  putreg32(value, priv->sspbase + (uint32_t)offset);
}

/****************************************************************************
 * Name: ssp_lock
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
static int ssp_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;

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
 * Name: ssp_setfrequency
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

static uint32_t ssp_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  uint32_t divisor;
  uint32_t actual;

  /* Check if the requested frequence is the same as the frequency selection */

  DEBUGASSERT(priv && frequency <= SSP_CLOCK / 2);
#ifndef CONFIG_SPI_OWNBUS
  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }
#endif

  /* frequency = SSP_CLOCK / divisor, or divisor = SSP_CLOCK / frequency */

  divisor = SSP_CLOCK / frequency;

   /* "In master mode, CPSDVSRmin = 2 or larger (even numbers only)" */

  if (divisor < 2)
    {
      divisor = 2;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;

  /* Save the new divisor value */
  
  ssp_putreg(priv, LPC43_SSP_CPSR_OFFSET, divisor);

  /* Calculate the new actual */

  actual = SSP_CLOCK / divisor;

  /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = frequency;
  priv->actual    = actual;
#endif

  sspdbg("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: ssp_setmode
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

static void ssp_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->mode)
    {
#endif
      /* Yes... Set CR0 appropriately */

      regval = ssp_getreg(priv, LPC43_SSP_CR0_OFFSET);
      regval &= ~(SSP_CR0_CPOL|SSP_CR0_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;
 
        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SSP_CR0_CPHA;
          break;
 
        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SSP_CR0_CPOL;
          break;
 
        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SSP_CR0_CPOL|SSP_CR0_CPHA);
          break;
 
        default:
          sspdbg("Bad mode: %d\n", mode);
          DEBUGASSERT(FALSE);
          return;
        }

      ssp_putreg(priv, LPC43_SSP_CR0_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->mode = mode;
    }
#endif
}

/****************************************************************************
 * Name: ssp_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ssp_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 3 && nbits < 17);
#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->nbits)
    {
#endif
      /* Yes... Set CR1 appropriately */

      regval = ssp_getreg(priv, LPC43_SSP_CR0_OFFSET);
      regval &= ~SSP_CR0_DSS_MASK;
      regval |= ((nbits - 1) << SSP_CR0_DSS_SHIFT);
      regval = ssp_getreg(priv, LPC43_SSP_CR0_OFFSET);

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->nbits = nbits;
    }
#endif
}

/****************************************************************************
 * Name: ssp_send
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

static uint16_t ssp_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  register uint32_t regval;

  /* Wait while the TX FIFO is full */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF));

  /* Write the byte to the TX FIFO */

  ssp_putreg(priv, LPC43_SSP_DR_OFFSET, (uint32_t)wd);

  /* Wait for the RX FIFO not empty */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
  sspdbg("%04x->%04x\n", wd, regval);
  return (uint16_t)regval;
}

/*************************************************************************
 * Name: ssp_sndblock
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
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssp_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  union
  {
    FAR const uint8_t  *p8;
    FAR const uint16_t *p16;
    FAR const void     *pv;
  } u;
  uint32_t data;
  uint32_t sr;

  /* Loop while thre are bytes remaining to be sent */

  sspdbg("nwords: %d\n", nwords);
  u.pv = buffer;
  while (nwords > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF) && nwords)
        {
          /* Fetch the data to send */

          if (priv->nbits > 8)
            {
              data = (uint32_t)*u.p16++;
            }
          else
            {
              data = (uint32_t)*u.p8++;
            }

          /* Send the data */

          ssp_putreg(priv, LPC43_SSP_DR_OFFSET, data);
          nwords--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  sspdbg("discarding\n");
  do
    {
      /* Is there anything in the RX fifo? */

      sr = ssp_getreg(priv, LPC43_SSP_SR_OFFSET);
      if ((sr & SSP_SR_RNE) != 0)
        {
          /* Yes.. Read and discard */

          (void)ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
        }

      /* There is a race condition where TFE may go true just before
       * RNE goes true and this loop terminates prematurely.  The nasty little
       * delay in the following solves that (it could probably be tuned
       * to improve performance).
       */

      else if ((sr & SSP_SR_TFE) != 0)
        {
          up_udelay(100);
          sr = ssp_getreg(priv, LPC43_SSP_SR_OFFSET);
        }
    }
  while ((sr & SSP_SR_RNE) != 0 || (sr & SSP_SR_TFE) == 0);
}

/****************************************************************************
 * Name: ssp_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssp_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;
  union
  {
    FAR uint8_t  *p8;
    FAR uint16_t *p16;
    FAR void     *pv;
  } u;
  uint32_t data;
  uint32_t rxpending = 0;

  /* While there is remaining to be sent (and no synchronization error has occurred) */

  sspdbg("nwords: %d\n", nwords);
  u.pv = buffer;
  while (nwords || rxpending)
    {
      /* Fill the transmit FIFO with 0xffff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spivdbg("TX: rxpending: %d nwords: %d\n", rxpending, nwords);
      while ((ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF) &&
             (rxpending < LPC43_SSP_FIFOSZ) && nwords)
        {
          ssp_putreg(priv, LPC43_SSP_DR_OFFSET, 0xffff);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not empty */

      spivdbg("RX: rxpending: %d\n", rxpending);
      while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE)
        {
          data = (uint8_t)ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
          if (priv->nbits > 8)
            {
              *u.p16++ = (uint16_t)data;
            }
          else
            {
              *u.p8++  = (uint8_t)data;
            }
          rxpending--;
        }
    }
}

/****************************************************************************
 * Name: lpc43_ssp0initialize
 *
 * Description:
 *   Initialize the SSP0
 *
 * Input Parameter:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SSP0
static inline FAR struct lpc43_sspdev_s *lpc43_ssp0initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SSP0 pins and
   * one SSP1 pin (SCK) have multiple, alternative pin selection.
   * Definitions in the board.h file must be provided to resolve the
   * board-specific pin configuration like:
   *
   * #define GPIO_SSP0_SCK GPIO_SSP0_SCK_1
   */

  flags = irqsave();
  lpc43_configgpio(GPIO_SSP0_SCK);
  lpc43_configgpio(GPIO_SSP0_MISO);
  lpc43_configgpio(GPIO_SSP0_MOSI);

  /* Configure clocking */

  regval  = getreg32(LPC43_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_SSP0_MASK;
  regval |= (SSP_PCLKSET_DIV << SYSCON_PCLKSEL1_SSP0_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL1);

  /* Enable peripheral clocking to SSP0 */

  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSSP0;
  putreg32(regval, LPC43_SYSCON_PCONP);
  irqrestore(flags);

  return &g_ssp0dev;
}
#endif

/****************************************************************************
 * Name: lpc43_ssp1initialize
 *
 * Description:
 *   Initialize the SSP1
 *
 * Input Parameter:
 *   None
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SSP1
static inline FAR struct lpc43_sspdev_s *lpc43_ssp1initialize(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SSP0 pins and
   * one SSP1 pin (SCK) have multiple, alternative pin selection.
   * Definitions in the board.h file must be provided to resolve the
   * board-specific pin configuration like:
   *
   * #define GPIO_SSP0_SCK GPIO_SSP0_SCK_1
   */

  flags = irqsave();
  lpc43_configgpio(GPIO_SSP1_SCK);
  lpc43_configgpio(GPIO_SSP1_MISO);
  lpc43_configgpio(GPIO_SSP1_MOSI);

  /* Configure clocking */

  regval  = getreg32(LPC43_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_SSP1_MASK;
  regval |= (SSP_PCLKSET_DIV << SYSCON_PCLKSEL0_SSP1_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL0);

  /* Enable peripheral clocking to SSP0 and SSP1 */

  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCSSP1;
  putreg32(regval, LPC43_SYSCON_PCONP);
  irqrestore(flags);

  return &g_ssp1dev;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_sspinitialize
 *
 * Description:
 *   Initialize the selected SSP port (0=SSP0, 1=SSP1)
 *
 * Input Parameter:
 *   port - Port number (0=SSP0, 1=SSP1)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lpc43_sspinitialize(int port)
{
  FAR struct lpc43_sspdev_s *priv;
  uint32_t regval;
  int i;

  /* Only the SSP0 and SSP1 interfaces are supported */

  switch (port)
    {
#ifdef CONFIG_LPC43_SSP0
    case 0:
      priv = lpc43_ssp0initialize();
      break;
#endif
#ifdef CONFIG_LPC43_SSP1
    case 1:
      priv = lpc43_ssp1initialize();
      break;
#endif
    default:
      return NULL;
    }

  /* Configure 8-bit SPI mode */

  ssp_putreg(priv, LPC43_SSP_CR0_OFFSET, SSP_CR0_DSS_8BIT|SSP_CR0_FRF_SPI);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  ssp_putreg(priv, LPC43_SSP_CR1_OFFSET, 0);
  ssp_putreg(priv, LPC43_SSP_IMSC_OFFSET, 0);

  /* Set the initial SSP configuration */

#ifndef CONFIG_SPI_OWNBUS
  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
#endif

  /* Select a default frequency of approx. 400KHz */

  ssp_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif

  /* Enable the SPI */

  regval = ssp_getreg(priv, LPC43_SSP_CR1_OFFSET);
  ssp_putreg(priv, LPC43_SSP_CR1_OFFSET, regval | SSP_CR1_SSE);
  for (i = 0; i < LPC43_SSP_FIFOSZ; i++)
    {
      (void)ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
    }

  return &priv->spidev;
}

/****************************************************************************
 * Name: ssp_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be done
 *   after a device is deselected if you worry about such things.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ssp_flush(FAR struct spi_dev_s *dev)
{
  FAR struct lpc43_sspdev_s *priv = (FAR struct lpc43_sspdev_s *)dev;

  /* Wait for the TX FIFO not full indication */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_TNF));
  ssp_putreg(priv, LPC43_SSP_DR_OFFSET, 0xff);

  /* Wait until TX FIFO and TX shift buffer are empty */

  while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_BSY);

  /* Wait until RX FIFO is not empty */

  while (!(ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE));

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      (void)ssp_getreg(priv, LPC43_SSP_DR_OFFSET);
    }
  while (ssp_getreg(priv, LPC43_SSP_SR_OFFSET) & SSP_SR_RNE);
}

#endif /* CONFIG_LPC43_SSP0/1 */

