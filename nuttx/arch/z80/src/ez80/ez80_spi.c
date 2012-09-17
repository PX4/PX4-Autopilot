/****************************************************************************
 * arch/z80/src/ez80/ez80_spi.c
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

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <arch/io.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "ez80f91_spi.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_EZ80F91
# define GPIOB_SPI_PINSET  0x38  /* MISO+MSOI+SCK. Excludes SS */
#else
#  error "Check GPIO initialization for this chip"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int    spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                uint32_t frequency);
static void   spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void   spi_sndblock(FAR struct spi_dev_s *dev,
                FAR const uint8_t *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR uint8_t *buffer,
                size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  spi_lock,
#endif
  ez80_spiselect,    /* Provided externally by board logic */
  spi_setfrequency,
  spi_setmode,
  NULL,              /* Variable number of bits not implemented */
  ez80_spistatus,    /* Provided externally by board logic */
#ifdef CONFIG_SPI_CMDDATA
  ez80_spicmddata,
#endif
  spi_send,
  spi_sndblock,
  spi_recvblock,
  0                  /* registercallback not yet implemented */
};

/* This supports is only a single SPI bus/port.  If you port this to an
 * architecture with multiple SPI busses/ports, then the following must
 * become an array with one 'struct spi_dev_s' instance per bus.
 */

static struct spi_dev_s g_spidev = { &g_spiops };

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
  /* We want select divisor to provide the highest frequency (SPIR) that does NOT
   * exceed the requested frequency.:
   *
   *   SPIR <= System Clock Frequency / (2 * BRG)
   *
   * So
   *
   *   BRG >= System Clock Frequency / (2 * SPIR)
   */
 
  uint32_t brg = ((EZ80_SYS_CLK_FREQ+1)/2 + frequency - 1) / frequency;

  /* "When configured as a Master, the 16-bit divisor value must be between
   * 0003h and FFFFh, inclusive. When configured as a Slave, the 16-bit
   * divisor value must be between 0004h and FFFFh, inclusive."
   */

  if (brg < 3)
    {
      brg = 3;
    }
  else if (brg > 0xffff)
    {
      brg = 0xfff;
    }

  outp(EZ80_SPI_BRG_L, brg & 0xff);
  outp(EZ80_SPI_BRG_L, (brg >> 8) & 0xff);

  return ((EZ80_SYS_CLK_FREQ+1)/2 + brg - 1) / brg;
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
  uint8_t modebits;
  uint8_t regval;

  /* Select the CTL register bits based on the selected mode */

  switch (mode)
    {
    case SPIDEV_MODE0: /* CPOL=0 CHPHA=0 */
      modebits = 0;
      break;

    case SPIDEV_MODE1: /* CPOL=0 CHPHA=1 */
      modebits = SPI_CTL_CPHA;
      break;

    case SPIDEV_MODE2: /* CPOL=1 CHPHA=0 */
      modebits = SPI_CTL_CPOL;
      break;

    case SPIDEV_MODE3: /* CPOL=1 CHPHA=1 */
      modebits = (SPI_CTL_CPOL|SPI_CTL_CPHA);
      break;

    default:
      return;
    }

    /* Then set those bits in the CTL register */

    regval = inp(EZ80_SPI_CTL);
    regval &= ~(SPI_CTL_CPOL|SPI_CTL_CPHA);
    regval |= modebits;
    outp(EZ80_SPI_CTL, regval);
}

/****************************************************************************
 * Name: spi_waitspif
 *
 * Description:
 *   Wait for the SPIF bit to be set in the status register signifying the
 *   the data transfer was finished.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Status register mode bits
 *
 ****************************************************************************/

static uint8_t spi_waitspif(void)
{
  uint8_t status;

  /* Wait for the device to be ready to accept another byte (or for an error
   * to be reported
   */

  do
    {
      status = inp(EZ80_SPI_SR) & (SPI_SR_SPIF|SPI_SR_WCOL|SPI_SR_MODF);
    }
  while (status == 0);
  return status;
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Send one byte on SPI, return th response
 *
 * Input Parameters:
 *   ch - the byte to send
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint8_t spi_transfer(uint8_t ch)
{
   uint8_t status;

  /* Send the byte, repeating if some error occurs */

  for(;;)
    {
      outp(EZ80_SPI_TSR, ch);

      /* Wait for the device to be ready to accept another byte */

      status = spi_waitspif();
      if ((status & SPI_SR_SPIF) != 0)
        {
          return inp(EZ80_SPI_RBR);
        }
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
  return spi_transfer((uint8_t)wd);
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
 *   buflen - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t buflen)
{
  FAR const uint8_t *ptr = (FAR const uint8_t*)buffer;
  uint8_t response;

  /* Loop while thre are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      response = spi_transfer(*ptr++);
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
 *   buflen - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t buflen)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint8_t response;

  /* Loop while thre are bytes remaining to be sent */

  while (buflen-- > 0)
    {
      *ptr++ = spi_transfer(0xff);
    }
}

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
  uint8_t regval;

  /* Only the SPI1 interface is supported */

#ifdef CONFIG_DEBUG
  if (port != 1)
    {
      return NULL;
    }
#endif

  /* Disable SPI */

  outp(EZ80_SPI_CTL, 0);

  /* Configure GPIOs.  For the eZ80F91, the pin mapping for the four SPI pins
   * is:
   *
   *  GPIO ALT   MASTER  SLAVE   COMMENT
   *  ---- ----- ------- ------- ---------------------------------
   *   PB2 SS    INPUT   INPUT   Managed by board specific logic
   *   PB3 SCLK  OUTPUT  INPUT
   *   PB4 MISO  INPUT   OUTPUT
   *   PB5 MOSI  OUTPUT  INPUT
   *
   * Select the alternate function for PB2-5:
   */

#ifdef CONFIG_ARCH_CHIP_EZ80F91
  regval  = inp(EZ80_PB_DDR);
  regval |= GPIOB_SPI_PINSET;
  outp(EZ80_PB_DDR, regval);

  regval  = inp(EZ80_PB_ALT1);
  regval &= ~GPIOB_SPI_PINSET;
  outp(EZ80_PB_ALT1, regval);

  regval  = inp(EZ80_PB_ALT2);
  regval |= GPIOB_SPI_PINSET;
  outp(EZ80_PB_ALT2, regval);
#else
#  error "Check GPIO initialization for this chip"
#endif

  /* Set the initial clock frequency for indentification mode < 400kHz */

  spi_setfrequency(NULL, 400000);

  /* Enable the SPI.
   * NOTE 1: Interrupts are not used in this driver version.
   * NOTE 2: Initial mode is mode=0.
   */

  outp(EZ80_SPI_CTL, SPI_CTL_SPIEN|SPI_CTL_MASTEREN);

  return &g_spidev;
}
