/****************************************************************************
 * config/mcu123-lpc214x/src/up_spi1.c
 * arch/arm/src/board/up_spi1.c
 *
 *   Copyright (C) 2008-2010, 2012 Gregory Nutt. All rights reserved.
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
 * One the mcu123.com lpc214x board, the MMC slot is connect via SPI with the
 * following SPI mode pinout:
 *
 *   1 CS: Chip select (low) - SSEL1      5 SCLK: Clock         - SCK1
 *   2 DI: Data input        - MOSI1      6 Vss2: Supply Voltage- GRND
 *   3 Vss: Supply Voltage   - GRND       7 DO: Data Output     - MISO1
 *   4 Vdd: Power Supply     - Vcc        8 - N/C
 *
 * The LPC214x supports one SPI port (SPI0) and one SSP port (SPI1).  SPI1
 * is used to interface with the MMC connect
 *
 *   SCK1  - pin 47, P0.17/CAP1.2/SCK1/MAT1.2
 *   MISO1 - pin 53, P0.18/CAP1.3/MISO1/MAT1.3
 *   MOSI1 - pin 54, P0.19/MAT1.2/MOSI1/CAP1.2
 *   SSEL1 - pin 55, P0.20/MAT1.3/SSEL1/EINT3
 *
 * SPI0 is available on the mcu123.com board (pins 27, 29, 30, and 31).
 * Pin 27 is dedicated to a chip select, pins 30 and 31 connect to keys, nd
 * pin 29 is unconnected.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "lpc214x_power.h"
#include "lpc214x_pinsel.h"
#include "lpc214x_spi.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file */

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/* Clocking */

#define LPC214X_CCLKFREQ  (LPC214X_FOSC*LPC214X_PLL_M)
#define LPC214X_PCLKFREQ  (LPC214X_CCLKFREQ/LPC214X_APB_DIV)

/* Use either FIO or legacy GPIO */

#ifdef CONFIG_LPC214x_FIO
#  define CS_SET_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_SET_OFFSET)
#  define CS_CLR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_CLR_OFFSET)
#  define CS_DIR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_DIR_OFFSET)
#else
#  define CS_SET_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET)
#  define CS_CLR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_CLR_OFFSET)
#  define CS_DIR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static uint8_t spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

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
  uint32_t bit = 1 << 20;

  if (selected)
    {
      /* Enable slave select (low enables) */

      spidbg("CS asserted\n");
      putreg32(bit, CS_CLR_REGISTER);
    }
  else
    {
      /* Disable slave select (low enables) */

      spidbg("CS de-asserted\n");
      putreg32(bit, CS_SET_REGISTER);

      /* Wait for the TX FIFO not full indication */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));
      putreg16(0xff, LPC214X_SPI1_DR);

      /* Wait until TX FIFO and TX shift buffer are empty */

      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_BSY);

      /* Wait until RX FIFO is not empty */

      while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

      /* Then read and discard bytes until the RX FIFO is empty */

      do
        {
          (void)getreg16(LPC214X_SPI1_DR);
        }
      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE);
    }
}

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
  uint32_t divisor = LPC214X_PCLKFREQ / frequency;

  if (divisor < 2)
    {
      divisor = 2;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;
  putreg8(divisor, LPC214X_SPI1_CPSR);

  spidbg("Frequency %d->%d\n", frequency, LPC214X_PCLKFREQ / divisor);
  return LPC214X_PCLKFREQ / divisor;
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
  /* I don't think there is anyway to determine these things on the mcu123.com
   * board.
   */

  spidbg("Return SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

 #ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#  error "spi_cmddata not implemented"
  return -ENOSYS;
}
#endif

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
  register uint16_t regval;

  /* Wait while the TX FIFO is full */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF));

  /* Write the byte to the TX FIFO */

  putreg16((uint8_t)wd, LPC214X_SPI1_DR);

  /* Wait for the RX FIFO not empty */

  while (!(getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE));

  /* Get the value from the RX FIFO and return it */

  regval = getreg16(LPC214X_SPI1_DR);
  spidbg("%04x->%04x\n", wd, regval);
  return regval;
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
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
  FAR const uint8_t *ptr = (FAR const uint8_t *)buffer;
  uint8_t sr;

  /* Loop while thre are bytes remaining to be sent */

  spidbg("nwords: %d\n", nwords);
  while (nwords > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) && nwords)
        {
          /* Send the data */

          putreg16((uint16_t)*ptr, LPC214X_SPI1_DR);
          ptr++;
          nwords--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  spidbg("discarding\n");
  do
    {
      /* Is there anything in the RX fifo? */

      sr = getreg8(LPC214X_SPI1_SR);
      if ((sr & LPC214X_SPI1SR_RNE) != 0)
        {
          /* Yes.. Read and discard */

          (void)getreg16(LPC214X_SPI1_DR);
        }

      /* There is a race condition where TFE may go true just before
       * RNE goes true and this loop terminates prematurely.  The nasty little
       * delay in the following solves that (it could probably be tuned
       * to improve performance).
       */

      else if ((sr & LPC214X_SPI1SR_TFE) != 0)
        {
          up_udelay(100);
          sr = getreg8(LPC214X_SPI1_SR);
        }
    }
  while ((sr & LPC214X_SPI1SR_RNE) != 0 || (sr & LPC214X_SPI1SR_TFE) == 0);
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
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint32_t rxpending = 0;

  /* While there is remaining to be sent (and no synchronization error has occurred) */

  spidbg("nwords: %d\n", nwords);
  while (nwords || rxpending)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      spivdbg("TX: rxpending: %d nwords: %d\n", rxpending, nwords);
      while ((getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_TNF) &&
             (rxpending < LPC214X_SPI1_FIFOSZ) && nwords)
        {
          putreg16(0xff, LPC214X_SPI1_DR);
          nwords--;
          rxpending++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not empty */

      spivdbg("RX: rxpending: %d\n", rxpending);
      while (getreg8(LPC214X_SPI1_SR) & LPC214X_SPI1SR_RNE)
        {
          *ptr++ = (uint8_t)getreg16(LPC214X_SPI1_DR);
          rxpending--;
        }
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
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  uint32_t regval32;
  uint8_t regval8;
  int i;

  /* Only the SPI1 interface is supported */

#ifdef CONFIG_DEBUG
  if (port != 1)
    {
      return NULL;
    }
#endif

  /* Configure multiplexed pins as connected on the mcu123.com board:
   *
   *   PINSEL1 P0.17/CAP1.2/SCK1/MAT1.2  Bits 2-3=10 for SCK1
   *   PINSEL1 P0.18/CAP1.3/MISO1/MAT1.3 Bits 4-5=10 for MISO1
   *   PINSEL1 P0.19/MAT1.2/MOSI1/CAP1.2 Bits 6-7=10 for MOSI1
   *   PINSEL1 P0.20/MAT1.3/SSEL1/EINT3  Bits 8-9=10 for P0.20 (we'll control it via GPIO or FIO)
   */

  regval32  = getreg32(LPC214X_PINSEL1);
  regval32 &= ~(LPC214X_PINSEL1_P017_MASK|LPC214X_PINSEL1_P018_MASK|
                LPC214X_PINSEL1_P019_MASK|LPC214X_PINSEL1_P020_MASK);
  regval32 |= (LPC214X_PINSEL1_P017_SCK1|LPC214X_PINSEL1_P018_MISO1|
                LPC214X_PINSEL1_P019_MOSI1|LPC214X_PINSEL1_P020_GPIO);
  putreg32(regval32, LPC214X_PINSEL1);

  /* Disable chip select using P0.20 (SSEL1)  (low enables) */

  regval32 = 1 << 20;
  putreg32(regval32, CS_SET_REGISTER);
  regval32 |= getreg32(CS_DIR_REGISTER);
  putreg32(regval32, CS_DIR_REGISTER);

  /* Enable peripheral clocking to SPI1 */

  regval32  = getreg32(LPC214X_PCON_PCONP);
  regval32 |= LPC214X_PCONP_PCSPI1;
  putreg32(regval32, LPC214X_PCON_PCONP);

  /* Configure 8-bit SPI mode */

  putreg16(LPC214X_SPI1CR0_DSS8BIT|LPC214X_SPI1CR0_FRFSPI, LPC214X_SPI1_CR0);

  /* Disable the SSP and all interrupts (we'll poll for all data) */

  putreg8(0, LPC214X_SPI1_CR1);
  putreg8(0, LPC214X_SPI1_IMSC);

  /* Set the initial clock frequency for indentification mode < 400kHz */

  spi_setfrequency(NULL, 400000);

  /* Enable the SPI */

  regval8 = getreg8(LPC214X_SPI1_CR1);
  putreg8(regval8 | LPC214X_SPI1CR1_SSE, LPC214X_SPI1_CR1);

  for (i = 0; i < 8; i++)
    {
      (void)getreg16(LPC214X_SPI1_DR);
    }

  return &g_spidev;
}
