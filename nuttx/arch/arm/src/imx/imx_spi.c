/****************************************************************************
 * arch/arm/src/imx/imx_spi.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "imx_gpio.h"
#include "imx_cspi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The i.MX1/L supports 2 SPI interfaces.  Which have been enabled? */

#ifndef CONFIG_SPI1_DISABLE
#  define SPI1_NDX 0           /* Index to SPI1 in g_spidev[] */
#  ifndef CONFIG_SPI2_DISABLE
#   define SPI2_NDX 1          /* Index to SPI2 in g_spidev[] */
#   define NSPIS 2             /* Two SPI interfaces: SPI1 & SPI2 */
#  else
#   define NSPIS 1             /* One SPI interface: SPI1 */
#  endif
#else
#  ifndef CONFIG_SPI2_DISABLE
#   define SPI2_NDX 0          /* Index to SPI2 in g_spidev[] */
#   define NSPIS 1             /* One SPI interface: SPI2 */
#  else
#   define NSPIS 0             /* No SPI interfaces */
#  endif
#endif

/* Compile the rest of the file only if at least one SPI interface has been
 * enabled.
 */

#if NSPIS > 0

/* The number of words that will fit in the Tx FIFO */

#define IMX_TXFIFO_WORDS 8

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct imx_spidev_s
{
  const struct spi_ops_s *ops;  /* Common SPI operations */
#ifndef CONFIG_SPI_POLLWAIT
  sem_t  sem;                   /* Wait for transfer to complete */
#endif

  /* These following are the source and destination buffers of the transfer.
   * they are retained in this structure so that they will be accessible
   * from an interrupt handler.  The actual type of the buffer is uint8_t is
   * nbits <=8 and uint16_t is nbits >8.
   */

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  /* These are functions pointers that are configured to perform the
   * appropriate transfer for the particular kind of exchange that is
   * occurring.  Differnt functions may be selected depending on (1)
   * if the tx or txbuffer is NULL and depending on the number of bits
   * per word.
   */

  void  (*txword)(struct imx_spidev_s *priv);
  void  (*rxword)(struct imx_spidev_s *priv);

  uint32_t base;                /* SPI register base address */
  uint32_t frequency;           /* Current desired SCLK frequency */
  uint32_t actual;              /* Current actual SCLK frequency */

  int    ntxwords;              /* Number of words left to transfer on the Tx FIFO */
  int    nrxwords;              /* Number of words received on the Rx FIFO */
  int    nwords;                /* Number of words to be exchanged */

  uint8_t  mode;                /* Current mode */
  uint8_t  nbits;               /* Current number of bits per word */
#ifndef CONFIG_SPI_POLLWAIT
  uint8_t  irq;                 /* SPI IRQ number */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* SPI register access */
 
static inline uint32_t spi_getreg(struct imx_spidev_s *priv, unsigned int offset);
static inline void spi_putreg(struct imx_spidev_s *priv, unsigned int offset, uint32_t value);

/* SPI data transfer */

static void   spi_txnull(struct imx_spidev_s *priv);
static void   spi_txuint16(struct imx_spidev_s *priv);
static void   spi_txuint8(struct imx_spidev_s *priv);
static void   spi_rxnull(struct imx_spidev_s *priv);
static void  spi_rxuint16(struct imx_spidev_s *priv);
static void   spi_rxuint8(struct imx_spidev_s *priv);
static int    spi_performtx(struct imx_spidev_s *priv);
static inline void spi_performrx(struct imx_spidev_s *priv);
static int    spi_transfer(struct imx_spidev_s *priv, const void *txbuffer,
                           void *rxbuffer, unsigned int nwords);

/* Interrupt handling */

#ifndef CONFIG_SPI_POLLWAIT
static inline struct imx_spidev_s *spi_mapirq(int irq);
static int    spi_interrupt(int irq, void *context);
#endif

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int    spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void   spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void   spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void   spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                           FAR void *rxbuffer, size_t nwords);
#else
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common SPI operations */

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock         = spi_lock,
#endif
  .select       = imx_spiselect,    /* Provided externally by board logic */
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = imx_spistatus,    /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = imx_spicmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
};

/* This supports is up to two SPI busses/ports */

static struct imx_spidev_s g_spidev[] =
{
#ifndef CONFIG_SPI1_DISABLE
  {
    .ops  = &g_spiops,
    .base = IMX_CSPI1_VBASE,
#ifndef CONFIG_SPI_POLLWAIT
    .irq  = IMX_IRQ_CSPI1,
#endif
  },
#endif
#ifndef CONFIG_SPI2_DISABLE
  {
    .ops  = &g_spiops,
    .base = IMX_CSPI2_VBASE,
#ifndef CONFIG_SPI_POLLWAIT
    .irq  = IMX_IRQ_CSPI2,
#endif
  },
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Read the SPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct imx_spidev_s *priv, unsigned int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write the value to the SPI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SPI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(struct imx_spidev_s *priv, unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: spi_txnull, spi_txuint16, and spi_txuint8
 *
 * Description:
 *   Transfer all ones, a uint8_t, or uint16_t to Tx FIFO and update the txbuffer
 *   pointer appropriately.  The selected function dependes on (1) if there
 *   is a source txbuffer provided, and (2) if the number of bits per
 *   word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_txnull(struct imx_spidev_s *priv)
{
  spi_putreg(priv, CSPI_TXD_OFFSET, 0xffff);
}

static void spi_txuint16(struct imx_spidev_s *priv)
{
  uint16_t *ptr = (uint16_t*)priv->txbuffer;
  spi_putreg(priv, CSPI_TXD_OFFSET, *ptr++);
  priv->txbuffer = (void*)ptr;
}

static void spi_txuint8(struct imx_spidev_s *priv)
{
  uint8_t *ptr = (uint8_t*)priv->txbuffer;
  spi_putreg(priv, CSPI_TXD_OFFSET, *ptr++);
  priv->txbuffer = (void*)ptr;
}

/****************************************************************************
 * Name: spi_rxnull,spi_rxuint16, and spi_rxuint8
 *
 * Description:
 *   Discard input, save a uint8_t, or or save a uint16_t from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a desination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_rxnull(struct imx_spidev_s *priv)
{
  (void)spi_getreg(priv, CSPI_RXD_OFFSET);
}

static void spi_rxuint16(struct imx_spidev_s *priv)
{
  uint16_t *ptr = (uint16_t*)priv->rxbuffer;
  *ptr++ = (uint16_t)spi_getreg(priv, CSPI_TXD_OFFSET);
  priv->rxbuffer = (void*)ptr;
}

static void spi_rxuint8(struct imx_spidev_s *priv)
{
  uint8_t *ptr = (uint8_t*)priv->rxbuffer;
  *ptr++ = (uint8_t)spi_getreg(priv, CSPI_TXD_OFFSET);
  priv->rxbuffer = (void*)ptr;
}

/****************************************************************************
 * Name: spi_performtx
 *
 * Description:
 *   If the Tx FIFO is empty, then transfer as many words as we can to
 *   the FIFO.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   The number of words written to the Tx FIFO (a value from 0 to 8,
 *   inclusive).
 *
 ****************************************************************************/

static int spi_performtx(struct imx_spidev_s *priv)
{
  uint32_t regval;
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is empty */

  if ((spi_getreg(priv, CSPI_INTCS_OFFSET) & CSPI_INTCS_TE) != 0)
    {
      /* Check if all of the Tx words have been sent */

      if (priv->ntxwords > 0)
        {
          /* No.. Transfer more words until either the TxFIFO is full or
           * until all of the user provided data has been sent.
           */

          for (; ntxd < priv->ntxwords && ntxd < IMX_TXFIFO_WORDS; ntxd++)
            {
               priv->txword(priv);
            }

          /* Update the count of words to to transferred */

          priv->ntxwords -= ntxd;
        }
      else
        {
          /* Yes.. The transfer is complete, disable Tx FIFO empty interrupt */

          regval = spi_getreg(priv, CSPI_INTCS_OFFSET);
          regval &= ~CSPI_INTCS_TEEN;
          spi_putreg(priv, CSPI_INTCS_OFFSET, regval);
        }
    }
  return ntxd;
}

/****************************************************************************
 * Name: spi_performrx
 *
 * Description:
 *   Transfer as many bytes as possible from the Rx FIFO to the user Rx
 *   buffer (if one was provided).
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_performrx(struct imx_spidev_s *priv)
{
  /* Loop while data is available in the Rx FIFO */

  while ((spi_getreg(priv, CSPI_INTCS_OFFSET) & CSPI_INTCS_RR) != 0)
    {
      /* Have all of the requested words been transferred from the Rx FIFO? */

      if (priv->nrxwords < priv->nwords)
        {
          /* No.. Read more data from Rx FIFO */

          priv->rxword(priv);
          priv->nrxwords++;
        }
    }
}

/****************************************************************************
 * Name: spi_startxfr
 *
 * Description:
 *   If data was added to the Tx FIFO, then start the exchange
 *
 * Input Parameters:
 *   priv  - Device-specific state data
 *   ntxd  - The number of bytes added to the Tx FIFO by spi_performtx.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_startxfr(struct imx_spidev_s *priv, int ntxd)
{
  uint32_t regval;

  /* The XCH bit initiates an exchange in master mode.  It remains set
   * remains set while the exchange is in progress but is automatically
   * clear when all data in the Tx FIFO and shift register are shifted out.
   * So if we have added data to the Tx FIFO on this interrupt, we must
   * set the XCH bit to resume the exchange.
   */

  if (ntxd > 0)
   {
      regval = spi_getreg(priv, CSPI_CTRL_OFFSET);
      regval |= CSPI_CTRL_XCH;
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);
   }
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of uint8_t's;
 *              if the interface uses >8 bits per word, then this is the
 *              number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

static int spi_transfer(struct imx_spidev_s *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
#ifndef CONFIG_SPI_POLLWAIT
  irqstate_t flags;
#endif
  uint32_t regval;
  int ntxd;
  int ret;

  /* Set up to perform the transfer */

  priv->txbuffer     = (uint8_t*)txbuffer; /* Source buffer */
  priv->rxbuffer     = (uint8_t*)rxbuffer; /* Destination buffer */
  priv->ntxwords     = nwords;           /* Number of words left to send */
  priv->nrxwords     = 0;                /* Number of words received */
  priv->nwords       = nwords;           /* Total number of exchanges */

  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 8)
    {
      priv->txword = spi_txuint16;
      priv->rxword =spi_rxuint16;
    }
  else
    {
      priv->txword = spi_txuint8;
      priv->rxword = spi_rxuint8;
    }

  if (!txbuffer)
    {
      priv->txword = spi_txnull;
    }

  if (!rxbuffer)
    {
      priv->rxword = spi_rxnull;
    }

  /* Prime the Tx FIFO to start the sequence (saves one interrupt) */

#ifndef CONFIG_SPI_POLLWAIT
  flags = irqsave();
  ntxd  = spi_performtx(priv);
  spi_startxfr(priv, ntxd);

  /* Enable transmit empty interrupt */

  regval = spi_getreg(priv, CSPI_INTCS_OFFSET);
  regval |= CSPI_INTCS_TEEN;
  spi_putreg(priv, CSPI_INTCS_OFFSET, regval);
  irqrestore(flags);

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  do
    {
      /* Wait to be signaled from the interrupt handler */

      ret = sem_wait(&priv->sem);
    }
  while (ret < 0 && errno == EINTR);
#else
  /* Perform the transfer using polling logic.  This will totally
   * dominate the CPU until the transfer is complete.  Only recommended
   * if (1) your SPI is very fast, and (2) if you only use very short
   * transfers.
   */

  do
    {
      /* Handle outgoing Tx FIFO transfers */

      ntxd = spi_performtx(priv);

      /* Handle incoming Rx FIFO transfers */

      spi_performrx(priv);

      /* Resume the transfer */

      spi_startxfr(priv, ntxd);

      /* If there are other threads at this same priority level,
       * the following may help:
       */

      sched_yield();
    }
  while (priv->nrxwords < priv->nwords);
#endif
  return OK;
}

/****************************************************************************
 * Name: spi_mapirq
 *
 * Description:
 *   Map an IRQ number into the appropriate SPI device
 *
 * Input Parameters:
 *   irq   - The IRQ number to be mapped
 *
 * Returned Value:
 *   On success, a reference to the private data structgure for this IRQ.
 *   NULL on failrue.
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static inline struct imx_spidev_s *spi_mapirq(int irq)
{
  switch (irq)
    {
#ifndef CONFIG_SPI1_DISABLE
      case IMX_IRQ_CSPI1:
        return &g_spidev[SPI1_NDX];
#endif
#ifndef CONFIG_SPI2_DISABLE
      case IMX_IRQ_CSPI2:
        return &g_spidev[SPI2_NDX];
#endif
      default:
        return NULL;
    }
}
#endif

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of uint8_t's;
 *              if the interface uses >8 bits per word, then this is the
 *              number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_POLLWAIT
static int spi_interrupt(int irq, void *context)
{
  struct imx_spidev_s *priv = spi_mapirq(irq);
  int ntxd;

  DEBUGASSERT(priv != NULL);

  /* Handle outgoing Tx FIFO transfers */

  ntxd = spi_performtx(priv);

  /* Handle incoming Rx FIFO transfers */

  spi_performrx(priv);

  /* Resume the transfer */

  spi_startxfr(priv, ntxd);

  /* Check if the transfer is complete */

  if (priv->nrxwords >= priv->nwords)
    {
      /* Yes, wake up the waiting thread */

      sem_post(&priv->sem);
    }
  return OK;
}
#endif

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
  /* Not implemented */

  return -ENOSYS;
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
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  uint32_t actual = priv->actual;

  if (priv && frequency != priv->frequency)
    {
      uint32_t freqbits;
      uint32_t regval;

      if (frequency >= IMX_PERCLK2_FREQ / 4)
        {
          freqbits = CSPI_CTRL_DIV4;
          actual   = IMX_PERCLK2_FREQ / 4;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 8)
        {
          freqbits = CSPI_CTRL_DIV8;
          actual   = IMX_PERCLK2_FREQ / 8;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 16)
        {
          freqbits = CSPI_CTRL_DIV16;
          actual   = IMX_PERCLK2_FREQ / 16;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 32)
        {
          freqbits = CSPI_CTRL_DIV32;
          actual   = IMX_PERCLK2_FREQ / 32;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 64)
        {
          freqbits = CSPI_CTRL_DIV64;
          actual   = IMX_PERCLK2_FREQ / 64;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 128)
        {
          freqbits = CSPI_CTRL_DIV128;
          actual   = IMX_PERCLK2_FREQ / 128;
        }
      else if (frequency >= IMX_PERCLK2_FREQ / 256)
        {
          freqbits = CSPI_CTRL_DIV256;
          actual   = IMX_PERCLK2_FREQ / 256;
        }
      else /*if (frequency >= IMX_PERCLK2_FREQ / 512) */
        {
          freqbits = CSPI_CTRL_DIV512;
          actual   = IMX_PERCLK2_FREQ / 512;
        }

      /* Then set the selected frequency */

      regval = spi_getreg(priv, CSPI_CTRL_OFFSET);
      regval &= ~(CSPI_CTRL_DATARATE_MASK);
      regval |= freqbits;
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);

      priv->frequency = frequency;
      priv->actual    = actual;
    }

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
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  if (priv && mode != priv->mode)
    {
      uint32_t modebits;
      uint32_t regval;

      /* Select the CTL register bits based on the selected mode */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0 CHPHA=0 */
          modebits = 0;
          break;

        case SPIDEV_MODE1: /* CPOL=0 CHPHA=1 */
          modebits = CSPI_CTRL_PHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1 CHPHA=0 */
          modebits = CSPI_CTRL_POL;
         break;

        case SPIDEV_MODE3: /* CPOL=1 CHPHA=1 */
          modebits = CSPI_CTRL_PHA|CSPI_CTRL_POL;
          break;

        default:
          return;
        }

      /* Then set the selected mode */

      regval = spi_getreg(priv, CSPI_CTRL_OFFSET);
      regval &= ~(CSPI_CTRL_PHA|CSPI_CTRL_POL);
      regval |= modebits;
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  if (priv && nbits != priv->nbits && nbits > 0 && nbits <= 16)
    {
      uint32_t regval = spi_getreg(priv, CSPI_CTRL_OFFSET);
      regval       &= ~CSPI_CTRL_BITCOUNT_MASK;
      regval       |= ((nbits - 1) << CSPI_CTRL_BITCOUNT_SHIFT);
      spi_putreg(priv, CSPI_CTRL_OFFSET, regval);
      priv->nbits   = nbits;
    }
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
  struct imx_spidev_s *priv = (struct imx_spidev_s*)dev;
  uint16_t response = 0;

  (void)spi_transfer(priv, &wd, &response, 1);
  return response;
}

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   buffer   - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  (void)spi_transfer(priv, txbuffer, rxbuffer, nwords);
}
#endif

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
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  (void)spi_transfer(priv, buffer, NULL, nwords);
}
#endif

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
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  struct imx_spidev_s *priv = (struct imx_spidev_s *)dev;
  (void)spi_transfer(priv, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize common parts the selected SPI port.  Initialization of
 *   chip select GPIOs must have been performed by board specific logic
 *   prior to calling this function.  Specifically:  GPIOs should have
 *   been configured for output, and all chip selects disabled.
 *
 *   One GPIO, SS (PB2 on the eZ8F091) is reserved as a chip select.  However,
 *   If multiple devices on on the bus, then multiple chip selects will be
 *   required.  Theregore, all GPIO chip management is deferred to board-
 *   specific logic.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  struct imx_spidev_s *priv;
  uint8_t regval;

  /* Only the SPI1 interface is supported */

  switch (port)
    {
#ifndef CONFIG_SPI1_DISABLE
    case 1:
      /* Select SPI1 */

      priv = &g_spidev[SPI1_NDX];

      /* Configure SPI1 GPIOs (NOTE that SS is not initialized here, the
       * logic in this file makes no assumptions about chip select)
       */

      imxgpio_configpfinput(GPIOC, 13);  /* Port C, pin 13: RDY */
      imxgpio_configpfoutput(GPIOC, 14); /* Port C, pin 14: SCLK */
      imxgpio_configpfinput(GPIOC, 16);  /* Port C, pin 16: MISO */
      imxgpio_configpfoutput(GPIOC, 17); /* Port C, pin 17: MOSI */
      break;
#endif /* CONFIG_SPI1_DISABLE */

#ifndef CONFIG_SPI2_DISABLE
    case 2:
      /* Select SPI2 */

      priv = &g_spidev[SPI2_NDX];

      /* Configure SPI2 GPIOs */
      /* SCLK: AIN of Port A, pin 0 -OR- AIN of Port D, pin 7 */

#if 1
      imxgpio_configoutput(GPIOA, 0); /* Set GIUS=1 OCR=0 DIR=OUT */
#else
      imxgpio_configoutput(GPIOD, 7); /* Set GIUS=1 OCR=0 DIR=OUT */
#endif

      /* SS: AIN of Port A, pin 17 -OR- AIN of Port D, pin 8.(NOTE that SS
       * is not initialized here, the logic in this file makes no assumptions
       * about chip select)
       */

      /* RXD: AOUT of Port A, pin 1 -OR- AOUT of Port D, pin 9 */

#if 1
      imxgpio_configinput(GPIOA, 1); /* Set GIUS=1 OCR=0 DIR=IN */

      /* Select input from SPI2_RXD_0 pin (AOUT Port A, pin 1) */

      regval = getreg32(IMX_SC_FMCR);
      regval &= ~FMCR_SPI2_RXDSEL;
      putreg32(regval, IMX_SC_FMCR);
#else
      imxgpio_configinput(GPIOD, 9); /* Set GIUS=1 OCR=0 DIR=IN */

      /* Select input from SPI2_RXD_1 pin (AOUT Port D, pin 9) */

      regval = getreg32(IMX_SC_FMCR);
      regval |= FMCR_SPI2_RXDSEL;
      putreg32(regval, IMX_SC_FMCR);
#endif

      /* TXD: BIN of Port D, pin 31 -OR- AIN of Port D, pin 10 */

#if 1
      imxgpio_configinput(GPIOD, 31);
      imxgpio_ocrbin(GPIOD, 31);
      imxgpio_dirout(GPIOD, 31);
#else 
      imxgpio_configoutput(GPIOD, 10);
#endif
      break;
#endif /* CONFIG_SPI2_DISABLE */

    default:
      return NULL;
    }

  /* Initialize the state structure */

#ifndef CONFIG_SPI_POLLWAIT
   sem_init(&priv->sem, 0, 0);
#endif

  /* Initialize control register: min frequency, ignore ready, master mode, mode=0, 8-bit */

  spi_putreg(priv, CSPI_CTRL_OFFSET, 
             CSPI_CTRL_DIV512 |                /* Lowest frequency */
             CSPI_CTRL_DRCTL_IGNRDY |          /* Ignore ready */
             CSPI_CTRL_MODE |                  /* Master mode */
             (7 << CSPI_CTRL_BITCOUNT_SHIFT)); /* 8-bit data */

  /* Make sure state agrees with data */

  priv->mode  = SPIDEV_MODE0;
  priv->nbits = 8;

  /* Set the initial clock frequency for identification mode < 400kHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Enable interrupts on data ready (and certain error conditions */

#ifndef CONFIG_SPI_POLLWAIT
  spi_putreg(priv, CSPI_INTCS_OFFSET,
             CSPI_INTCS_RREN |                 /* RXFIFO Data Ready Interrupt Enable */
             CSPI_INTCS_ROEN |                 /* RXFIFO Overflow Interrupt Enable */
             CSPI_INTCS_BOEN);                 /* Bit Count Overflow Interrupt Enable */
#else
  spi_putreg(priv, CSPI_INTCS_OFFSET, 0);      /* No interrupts */
#endif

  /* Set the clock source=bit clock and number of clocks inserted between
   * transactions = 2.
   */

  spi_putreg(priv, CSPI_SPCR_OFFSET, 2);

  /* No DMA */

  spi_putreg(priv, CSPI_DMA_OFFSET, 0);

  /* Attach the interrupt */

#ifndef CONFIG_SPI_POLLWAIT
  irq_attach(priv->irq, (xcpt_t)spi_interrupt);
#endif

  /* Enable SPI */

  regval = spi_getreg(priv, CSPI_CTRL_OFFSET);
  regval |= CSPI_CTRL_SPIEN;
  spi_putreg(priv, CSPI_CTRL_OFFSET, regval);

  /* Enable SPI interrupts */

#ifndef CONFIG_SPI_POLLWAIT
  up_enable_irq(priv->irq);
#endif
  return (FAR struct spi_dev_s *)priv;
}

#endif /* NSPIS > 0 */
