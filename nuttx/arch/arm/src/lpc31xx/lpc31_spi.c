/************************************************************************************
 * arm/arm/src/lpc31xx/lpc31_spi.c
 *
 *   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
 *   Author: David Hewson, deriving in part from other SPI drivers originally by
 *           Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <arch/board/board.h>

#include "lpc31_spi.h"
#include "lpc31_ioconfig.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Debug ****************************************************************************/
/* CONFIG_LPC31_SPI_REGDEBUG enabled very low, register-level debug output.
 * CONFIG_DEBUG must also be defined
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_LPC31_SPI_REGDEBUG
#endif

/* FIFOs ****************************************************************************/

#define SPI_FIFO_DEPTH  64    /* 64 words deep (8- or 16-bit words) */

/* Timing ***************************************************************************/

#define SPI_MAX_DIVIDER 65024 /* = 254 * (255 + 1) */
#define SPI_MIN_DIVIDER 2

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct lpc31_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of work in bits (8 or 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
    
  uint32_t         slv1;
  uint32_t         slv2;
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

#ifdef CONFIG_LPC31_SPI_REGDEBUG
static bool        spi_checkreg(bool wr, uint32_t value, uint32_t address);
static void        spi_putreg(uint32_t value, uint32_t address);
static uint32_t    spi_getreg(uint32_t address);
#else
#  define spi_putreg(v,a) putreg32(v,a)
#  define spi_getreg(a)   getreg32(a)
#endif

static inline void spi_drive_cs(FAR struct lpc31_spidev_s *priv, uint8_t slave, uint8_t val);
static inline void spi_select_slave(FAR struct lpc31_spidev_s *priv, uint8_t slave);
static inline uint16_t spi_readword(FAR struct lpc31_spidev_s *priv);
static inline void spi_writeword(FAR struct lpc31_spidev_s *priv, uint16_t word);

static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void        spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint8_t     spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
static uint16_t    spi_send(FAR struct spi_dev_s *dev, uint16_t word);
static void        spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                                 size_t nwords);
#endif


/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc31_spicmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

static struct lpc31_spidev_s g_spidev = 
{
  .spidev            = { &g_spiops },
};

#ifdef CONFIG_LPC31_SPI_REGDEBUG
static bool     g_wrlast;
static uint32_t g_addresslast;
static uint32_t g_valuelast;
static int      g_ntimes;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC31_SPI_REGDEBUG
static bool spi_checkreg(bool wr, uint32_t value, uint32_t address)
{
  if (wr == g_wrlast && value == g_valuelast && address == g_addresslast)
    {
      g_ntimes++;
      return false;
    }
  else
    {
      if (g_ntimes > 0)
        {
          lldbg("...[Repeats %d times]...\n", g_ntimes);
        }

      g_wrlast      = wr;
      g_valuelast   = value;
      g_addresslast = address;
      g_ntimes      = 0;
    }
  return true;
}
#endif

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to an SPI register
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC31_SPI_REGDEBUG
static void spi_putreg(uint32_t value, uint32_t address)
{
  if (spi_checkreg(true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
    }
  putreg32(value, address);
}
#endif

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Read a 32-bit value from an SPI register
 *
 * Input Parameters:
 *   address - The address of the register to read from
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC31_SPI_REGDEBUG
static uint32_t spi_getreg(uint32_t address)
{
  uint32_t value = getreg32(address);
  if (spi_checkreg(false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
    }
  return value;
}
#endif

/****************************************************************************
 * Name: spi_drive_cs
 *
 * Description:
 *   Drive the chip select signal for this slave
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   slave - slave id
 *   value - value (0 for assert)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_drive_cs(FAR struct lpc31_spidev_s *priv, uint8_t slave, uint8_t val)
{
  switch (slave)
    {
    case 0:
      if (val == 0)
        {
          spi_putreg(IOCONFIG_SPI_CSOUT0, LPC31_IOCONFIG_SPI_MODE0RESET);
        }
      else
        {
          spi_putreg(IOCONFIG_SPI_CSOUT0, LPC31_IOCONFIG_SPI_MODE0SET);
        }
      spi_putreg(IOCONFIG_SPI_CSOUT0, LPC31_IOCONFIG_SPI_MODE1SET);
      break;
      
    case 1:
      if (val == 0)
        {
          spi_putreg(IOCONFIG_EBII2STX0_MUARTCTSN, LPC31_IOCONFIG_EBII2STX0_MODE0RESET);
        }
      else
        {
          spi_putreg(IOCONFIG_EBII2STX0_MUARTCTSN, LPC31_IOCONFIG_EBII2STX0_MODE0SET);
        }
      spi_putreg(IOCONFIG_EBII2STX0_MUARTCTSN, LPC31_IOCONFIG_EBII2STX0_MODE1SET);
      break;

    case 2:
      if (val == 0)
        {
          spi_putreg(IOCONFIG_EBII2STX0_MUARTRTSN, LPC31_IOCONFIG_EBII2STX0_MODE0RESET);
        }
      else
        {
          spi_putreg(IOCONFIG_EBII2STX0_MUARTRTSN, LPC31_IOCONFIG_EBII2STX0_MODE0SET);
        }
      spi_putreg(IOCONFIG_EBII2STX0_MUARTRTSN, LPC31_IOCONFIG_EBII2STX0_MODE1SET);
      break;
    }
}

/****************************************************************************
 * Name: spi_select_slave
 *
 * Description:
 *   Select the slave device for the next transfer
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   slave - slave id
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_select_slave(FAR struct lpc31_spidev_s *priv, uint8_t slave)
{
  switch (slave)
    {
    case 0:
      spi_putreg(priv->slv1, LPC31_SPI_SLV0_1);
      spi_putreg(priv->slv2, LPC31_SPI_SLV0_2);
      spi_putreg(SPI_SLVENABLE1_ENABLED, LPC31_SPI_SLVENABLE);
      break;
      
    case 1:
      spi_putreg(priv->slv1, LPC31_SPI_SLV1_1);
      spi_putreg(priv->slv2, LPC31_SPI_SLV1_2);
      spi_putreg(SPI_SLVENABLE2_ENABLED, LPC31_SPI_SLVENABLE);
      break;

    case 2:
      spi_putreg(priv->slv1, LPC31_SPI_SLV2_1);
      spi_putreg(priv->slv2, LPC31_SPI_SLV2_2);
      spi_putreg(SPI_SLVENABLE3_ENABLED, LPC31_SPI_SLVENABLE);
      break;
    }
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct lpc31_spidev_s *priv)
{
  /* Wait until the RX FIFO is not empty */

  while ((spi_getreg(LPC31_SPI_STATUS) & SPI_STATUS_RXFIFOEMPTY) != 0);

  /* Then return the received word */

  return (uint16_t)spi_getreg(LPC31_SPI_FIFODATA);
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one word to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   word - Word to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct lpc31_spidev_s *priv, uint16_t word)
{
  /* Wait until the TX FIFO is not full */

  while ((spi_getreg(LPC31_SPI_STATUS) & SPI_STATUS_TXFIFOFULL) != 0);

  /* Then send the word */

  spi_putreg(word, LPC31_SPI_FIFODATA);
}

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

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;

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

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselecte.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  struct lpc31_spidev_s *priv = (struct lpc31_spidev_s *) dev;
  uint8_t slave = 0;
  
  /* FIXME: map the devid to the SPI slave - this should really
   * be in board specific code..... */
  switch (devid)
    {
    case SPIDEV_FLASH:
      slave = 0;
      break;

    case SPIDEV_MMCSD:
      slave = 1;
      break;

    case SPIDEV_ETHERNET:
      slave = 2;
      break;

    default:
      return;
    }
      
  /*
   * Since we don't use sequential multi-slave mode, but rather 
   * perform the transfer piecemeal by consecutive calls to 
   * SPI_SEND, then we must manually assert the chip select 
   * across the whole transfer 
   */

  if (selected)
  {
      spi_drive_cs(priv, slave, 0);
      spi_select_slave(priv, slave);
      
      /* Enable SPI as master and notify of slave enables change */

      spi_putreg((1 << SPI_CONFIG_INTERSLVDELAY_SHIFT) | SPI_CONFIG_UPDENABLE | SPI_CONFIG_SPIENABLE, LPC31_SPI_CONFIG);
  }
  else
  {
      spi_drive_cs(priv, slave, 1);
      
      /* Disable all slaves */

      spi_putreg(0, LPC31_SPI_SLVENABLE);
      
      /* Disable SPI as master */

      spi_putreg(SPI_CONFIG_UPDENABLE, LPC31_SPI_CONFIG);
  }
}

/************************************************************************************
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
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;
  uint32_t spi_clk, div, div1, div2;
  
  if (priv->frequency != frequency)
  {
      /* The SPI clock is derived from the (main system oscillator / 2),
       * so compute the best divider from that clock */
      
      spi_clk = lpc31_clkfreq(CLKID_SPICLK, DOMAINID_SPI);
      
      /* Find closest divider to get at or under the target frequency */
      
      div = (spi_clk + frequency / 2) / frequency;
      
      if (div > SPI_MAX_DIVIDER)
        {
          div = SPI_MAX_DIVIDER;
        }
      else if (div < SPI_MIN_DIVIDER)
        {
          div = SPI_MIN_DIVIDER;
        }
      
      div2 = (((div-1) / 512) + 2) * 2;
      div1 = ((((div + div2 / 2) / div2) - 1));

      priv->slv1 = (priv->slv1 & ~(SPI_SLV_1_CLKDIV2_MASK | SPI_SLV_1_CLKDIV1_MASK)) | (div2 << SPI_SLV_1_CLKDIV2_SHIFT) | (div1 << SPI_SLV_1_CLKDIV1_SHIFT);

      priv->frequency = frequency;
      priv->actual    = frequency;                // FIXME
  }

  return priv->actual;
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

/* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* SPO=0; SPH=0 */
          setbits = 0;
          clrbits = SPI_SLV_2_SPO|SPI_SLV_2_SPH;
          break;
 
        case SPIDEV_MODE1: /* SPO=0; SPH=1 */
          setbits = SPI_SLV_2_SPH;
          clrbits = SPI_SLV_2_SPO;
          break;
 
        case SPIDEV_MODE2: /* SPO=1; SPH=0 */
          setbits = SPI_SLV_2_SPO;
          clrbits = SPI_SLV_2_SPH;
          break;
 
        case SPIDEV_MODE3: /* SPO=1; SPH=1 */
          setbits = SPI_SLV_2_SPO|SPI_SLV_2_SPH;
          clrbits = 0;
          break;
 
        default:
          return;
        }

      priv->slv2 = (priv->slv2 & ~clrbits) | setbits;
      priv->mode = mode;
    }
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      priv->slv2  = (priv->slv2 & ~SPI_SLV_2_WDSIZE_MASK) | ((nbits-1) << SPI_SLV_2_WDSIZE_SHIFT);
      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
    /* FIXME: is there anyway to determine this 
    *         it should probably be board dependant anyway */

    return SPI_STATUS_PRESENT;
}

/************************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   word - The word to send.  the size of the data is determined by the
 *          number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t word)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;
  DEBUGASSERT(priv);

  spi_writeword(priv, word);
  return spi_readword(priv);
}

/*************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct lpc31_spidev_s *priv = (FAR struct lpc31_spidev_s *)dev;
  unsigned int maxtx;
  unsigned int ntx;

  DEBUGASSERT(priv);

  /* 8- or 16-bit mode? */

  if (priv->nbits == 16)
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      while (nwords > 0)
        {
          /* Fill up the TX FIFO */

          maxtx = nwords > SPI_FIFO_DEPTH ? SPI_FIFO_DEPTH : nwords;
          for (ntx = 0; ntx < maxtx; ntx++)
            {
              /* Get the next word to write.  Is there a source buffer? */

              word = src ? *src++ : 0xffff;

              /* Then send the word */

              spi_writeword(priv, word);
            }
          nwords -= maxtx;

          /* Then empty the RX FIFO */

          while (ntx-- > 0)
            {
              word = spi_readword(priv);

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        } 
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      while (nwords > 0)
        {
          /* Fill up the TX FIFO */

          maxtx = nwords > SPI_FIFO_DEPTH ? SPI_FIFO_DEPTH : nwords;
          for (ntx = 0; ntx < maxtx; ntx++)
            {
              /* Get the next word to write.  Is there a source buffer? */

              word = src ? *src++ : 0xff;

              /* Then send the word */

              spi_writeword(priv, (uint16_t)word);
            }
          nwords -= maxtx;

          /* Then empty the RX FIFO */

          while (ntx-- > 0)
            {
              word = (uint8_t)spi_readword(priv);

              /* Is there a buffer to receive the return value? */

              if (dest)
                {
                  *dest++ = word;
                }
            }
        }
    }
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 ************************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct lpc31_spidev_s *priv = &g_spidev;

  /* Only the SPI0 interface is supported */

  if (port != 0)
    {
      return NULL;
    }

  /* Configure SPI pins.  Nothing needs to be done here because the SPI pins
   * default to "driven-by-IP" on reset.
   */

#ifdef CONFIG_LPC31_SPI_REGDEBUG
  lldbg("PINS: %08x MODE0: %08x MODE1: %08x\n",
        spi_getreg(LPC31_IOCONFIG_SPI_PINS),
        spi_getreg(LPC31_IOCONFIG_SPI_MODE0),
        spi_getreg(LPC31_IOCONFIG_SPI_MODE1));
#endif

  /* Enable SPI clocks */

  lpc31_enableclock(CLKID_SPIPCLK);
  lpc31_enableclock(CLKID_SPIPCLKGATED);
  lpc31_enableclock(CLKID_SPICLK);
  lpc31_enableclock(CLKID_SPICLKGATED);

  /* Soft Reset the module */

  lpc31_softreset(RESETID_SPIRSTAPB);
  lpc31_softreset(RESETID_SPIRSTIP);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  sem_init(&priv->exclsem, 0, 1);

  /* Reset the SPI block */

  spi_putreg(SPI_CONFIG_SOFTRST, LPC31_SPI_CONFIG);

  /* Initialise Slave 0 settings registers */

  priv->slv1 = 0;
  priv->slv2 = 0;

  /* Configure initial default mode */

  priv->mode = SPIDEV_MODE1;
  spi_setmode(&priv->spidev, SPIDEV_MODE0);

  /* Configure word width */

  priv->nbits = 0;
  spi_setbits(&priv->spidev, 8);

  /* Select a default frequency of approx. 400KHz */

  priv->frequency = 0;
  spi_setfrequency(&priv->spidev, 400000);

  return (FAR struct spi_dev_s *)priv;
}
