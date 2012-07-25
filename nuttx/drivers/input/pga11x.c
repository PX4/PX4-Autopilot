/****************************************************************************
 * drivers/input/pga11x.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "PGA112, PGA113, PGA116, PGA117: Zerø-Drift PROGRAMMABLE GAIN AMPLIFIER
 *   with MUX", SBOS424B, March 2008, Revised September 2008, Texas
 *   Instruments Incorporated"
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/input/pga11x.h>

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_PGA11X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The PGA112/PGA113 have a three-wire SPI digital interface; the
 * PGA116/PGA117 have a four-wire SPI digital interface. The PGA116/117 also
 * have daisy-chain capability (The PGA112/PGA113 can be used as the last device
 * in a daisy-chain as shown if write-only communication is acceptable).
 */

/* PGA11x commands (PGA112/PGA113) */

#define PGA11X_CMD_READ          0x6a00
#define PGA11X_CMD_WRITE         0x2a00
#define PGA11X_CMA_NOOP          0x0000
#define PGA11X_CMA_SDN_DIS       0xe100
#define PGA11X_CMA_SDN_EN        0xe1f1

/* SPI Daisy-Chain Commands (PGA116/PGA117) */

#define PGA11X_DCCMD_SELECTOR    0x8000
#define PGA11X_DCCMD_NOOP        (PGA11X_DCCMD_SELECTOR | PGA11X_CMA_NOOP)
#define PGA11X_DCCMA_SDN_DIS     (PGA11X_DCCMD_SELECTOR | PGA11X_CMA_SDN_DIS)
#define PGA11X_DCCMA_SDN_EN      (PGA11X_DCCMD_SELECTOR | PGA11X_CMA_SDN_EN)
#define PGA11X_DCCMD_READ        (PGA11X_DCCMD_SELECTOR | PGA11X_CMD_READ)
#define PGA11X_DCCMD_WRITE       (PGA11X_DCCMD_SELECTOR | PGA11X_CMD_WRITE)

/* Write command Gain Selection Bits (PGA112/PGA113)
 *
 * the PGA112 and PGA116 provide binary gain selections (1, 2, 4, 8, 16, 32,
 * 64, 128); the PGA113 and PGA117 provide scope gain selections (1, 2, 5, 10,
 * 20, 50, 100, 200).
 */

#define PGA11X_GAIN_SHIFT        (4)      /* Bits 4-7: Gain Selection Bits */
#define PGA11X_GAIN_MASK         (15 << PGA11X_GAIN_SHIFT)

/* Write command Mux Channel Selection Bits
 *
 * The PGA112/PGA113 have a two-channel input MUX; the PGA116/PGA117 have a
 * 10-channel input MUX.
 */

#define PGA11X_CHAN_SHIFT        (0)      /* Bits 0-3: Channel Selection Bits */
#define PGA11X_CHAN_MASK         (15 << PGA11X_CHAN_SHIFT)

/* Other definitions ********************************************************/

#define SPI_DUMMY 0xff

/* Debug ****************************************************************************/
/* Check if (non-standard) SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pga11x_configure
 *
 * Description:
 *   Configure the SPI bus as needed for the PGA11x device.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pga11x_configure(FAR struct spi_dev_s *spi)
{
  spivdbg("MODE: %d BITS: 8 Frequency: %d\n",
          CONFIG_PGA11X_SPIMODE, CONFIG_PGA11X_SPIFREQUENCY);

  /* Call the setfrequency, setbits, and setmode methods to make sure that
   * the SPI is properly configured for the device.
   */

  SPI_SETMODE(spi, CONFIG_PGA11X_SPIMODE);
  SPI_SETBITS(spi, 8);
  (void)SPI_SETFREQUENCY(spi, CONFIG_PGA11X_SPIFREQUENCY);
}

/****************************************************************************
 * Name: pga11x_lock
 *
 * Description:
 *   Lock the SPI bus and configure it as needed for the PGA11x device.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static void pga11x_lock(FAR struct spi_dev_s *spi)
{
  spivdbg("Locking\n");

  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the SPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  SPI_LOCK(spi, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the SPI is properly configured for the device.
   * If the SPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  pga11x_configure(spi);
}
#else
#  define pga11x_lock(spi)
#endif

/****************************************************************************
 * Name: pga11x_unlock
 *
 * Description:
 *   Lock the SPI bus and configure it as needed for the PGA11x device.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void pga11x_unlock(FAR struct spi_dev_s *spi)
{
  spivdbg("Unlocking\n");

  SPI_LOCK(spi, false);
}
#else
#  define pga11x_unlock(spi)
#endif

/****************************************************************************
 * Name: pga11x_send16
 *
 * Description:
 *   Send 16 bits of data, ignoring any returned data
 *
 * Input Parameters:
 *   spi - PGA11X driver instance
 *   word - The data to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The bus is locked and the device is selected
 *
 ****************************************************************************/

static void pga11x_send16(FAR struct spi_dev_s *spi, uint16_t word)
{
  spivdbg("Send %04x\n", word);

  /* The logical interface is 16-bits wide.  However, this driver uses a
   * 8-bit configuration for greaer portability.
   *
   * Send the MS byte first.  Then the LS byte.
   */
 
  SPI_SEND(spi, word >> 8);
  SPI_SEND(spi, word & 0xff);
}

/****************************************************************************
 * Name: pga11x_recv16
 *
 * Description:
 *   Receive 16 bits of data.
 *
 * Input Parameters:
 *   spi -  PGA11X driver instance
 *
 * Returned Value:
 *   The received 16-bit value
 *
 * Assumptions:
 *   The bus is locked and the device is selected
 *
 ****************************************************************************/

static uint16_t pga11x_recv16(FAR struct spi_dev_s *spi)
{
  uint8_t msb;
  uint8_t lsb;

  /* The logical interface is 16-bits wide.  However, this driver uses a
   * 8-bit configuration for greaer portability.
   *
   * Send a dummy byte and receive MS byte first.  Then the LS byte.
   */
 
  msb = SPI_SEND(spi, SPI_DUMMY);
  lsb = SPI_SEND(spi, SPI_DUMMY);
  spivdbg("Received %02x %02x\n", msb, lsb);

  return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

/****************************************************************************
 * Name: pga11x_write
 *
 * Description:
 *   Send a 16-bit command.
 *
 * Input Parameters:
 *   spi -  PGA11X driver instance
 *
 * Returned Value:
 *   The received 16-bit value
 *
 * Assumptions:
 *   The device is NOT selected and the NOT bus is locked.
 *
 ****************************************************************************/

#ifndef CONFIG_PGA11X_DAISYCHAIN
static void pga11x_write(FAR struct spi_dev_s *spi, uint16_t cmd)
{
  spivdbg("cmd %04x\n", cmd);

  /* Lock, select, send the 16-bit command, de-select, and un-lock. */

  pga11x_lock(spi);
  SPI_SELECT(spi, SPIDEV_MUX, true);
  pga11x_send16(spi, cmd);
  SPI_SELECT(spi, SPIDEV_MUX, false);
  pga11x_unlock(spi);
}
#else
{
  spivdbg("cmd %04x\n", cmd);

  /* Lock, select, send the 16-bit command, the 16-bit daisy-chain command,
   * de-select, and un-lock.
   */

  pga11x_lock(spi);
  SPI_SELECT(spi, SPIDEV_MUX, true);
  pga11x_send16(spi, cmd);
  pga11x_send16(spi, cmd | PGA11X_DCCMD_SELECTOR);
  SPI_SELECT(spi, SPIDEV_MUX, false);
  pga11x_unlock(spi);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pga11x_initialize
 *
 * Description:
 *   Initialize the PGA117 amplifier/multiplexer.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   On success, a non-NULL opaque handle is returned; a NULL is returned
 *   on any failure.  This handle may be used with the other PGA117 interface
 *   functions to control the multiplexer
 *
 ****************************************************************************/

PGA11X_HANDLE pga11x_initialize(FAR struct spi_dev_s *spi)
{
  spivdbg("Entry\n");

  /* Configure the SPI us for the device.  Do this now only if the PGA11X is
   * the only device on the bus.
   */

#ifdef CONFIG_SPI_OWNBUS
  pga11x_configure(spi);
#endif

  /* No other special state is required, just return the SPI driver instance
   * as the handle.  This gives us a place to extend functionality in the
   * future if neccessary.
   */
 
  return (PGA11X_HANDLE)spi;
}

/****************************************************************************
 * Name: pga11x_select
 *
 * Description:
 *   Select an input channel and gain.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *   channel - See the PGA11X_CHAN_* definitions above
 *   gain    - See the PGA11X_GAIN_* definitions above
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_select(PGA11X_HANDLE handle, uint8_t channel, uint8_t gain)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t cmd;

  spivdbg("channel: %d gain: %d\n");
  DEBUGASSERT(handle);

  /* Format the command */

  cmd = PGA11X_CMD_WRITE |
        (channel << PGA11X_CHAN_SHIFT) |
        (gain << PGA11X_GAIN_SHIFT);

  /* Lock the bus and send the command */

  pga11x_write(spi, cmd);
  return OK;
}

/****************************************************************************
 * Name: pga11x_read
 *
 * Description:
 *   Read from the PGA117 amplifier/multiplexer.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   16-bit value read from the device (32-bits in daisy chain mode)
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
uint32_t pga11x_read(PGA11X_HANDLE handle)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t msvalue;
  uint16_t lsvalue;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Lock the bus and read the configuration */

  pga11x_lock(spi);

  /* Select, send the 16-bit command, the 16-bit daisy-chain command, and
   * then de-select the part. I do not know if de-selection between word
   * transfers is required.  However, it is shown in the timing diagrams
   * for the part.
   */

  SPI_SELECT(spi, SPIDEV_MUX, true);
  pga11x_send16(spi, PGA11X_CMD_READ);
  pga11x_send16(spi, PGA11X_DCCMD_READ);
  SPI_SELECT(spi, SPIDEV_MUX, false);

  /* Re-select, get the returned values, de-select, and unlock */

  SPI_SELECT(spi, SPIDEV_MUX, true);
  msvalue = pga11x_recv16(spi);
  mssvalue = pga11x_recv16(spi);
  SPI_SELECT(spi, SPIDEV_MUX, false);
  pga11x_unlock(spi);

  spivdbg("Returning %04x %04x\n", msvalue, lsvalue);
  return (uint32_t)msvalue << 16 | (uint32_t)lsvalue;
}
#else
uint16_t pga11x_read(PGA11X_HANDLE handle)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t value;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Lock the bus and read the configuration */

  pga11x_lock(spi);

  /* Select, send the 16-bit PGA11X_CMD_READ command, and de-select.  I do
   * not know if de-selection between word transfers is required.  However,
   * it is shown in the timing diagrams for the part.
   */

  SPI_SELECT(spi, SPIDEV_MUX, true);
  pga11x_send16(spi, PGA11X_CMD_READ);
  SPI_SELECT(spi, SPIDEV_MUX, false);

  /* Re-select, get the returned value, de-select, and unlock */

  SPI_SELECT(spi, SPIDEV_MUX, true);
  value = pga11x_recv16(spi);
  SPI_SELECT(spi, SPIDEV_MUX, false);
  pga11x_unlock(spi);

  spivdbg("Returning: %04x\n", value);
  return value;
}
#endif

/****************************************************************************
 * Name: pga11x_noop
 *
 * Description:
 *   Perform PGA11x no-operation.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_noop(PGA11X_HANDLE handle)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Lock the bus and send the NOOP command */

  pga11x_write(spi, PGA11X_CMA_NOOP);
  return OK;
}

/****************************************************************************
 * Name: pga11x_shutdown
 *
 * Description:
 *   Put the PGA11x in shutdown down mode.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_shutdown(PGA11X_HANDLE handle)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Lock the bus and enter shutdown mode by issuing an SDN_EN command */

  pga11x_write(spi, PGA11X_CMA_SDN_EN);
  return OK;
}

/****************************************************************************
 * Name: pga11x_enable
 *
 * Description:
 *   Take the PGA11x out of shutdown down mode.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_enable(PGA11X_HANDLE handle)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Lock the bus and send the shutdown disable command.  Shutdown mode is
   * cleared (returned to the last valid write configuration) by the SDN_DIS
   * command or by any valid Write command
   */

  pga11x_write(spi, PGA11X_CMA_SDN_DIS);
  return OK;
}

#endif /* CONFIG_INPUT && CONFIG_INPUT_PGA11X */

