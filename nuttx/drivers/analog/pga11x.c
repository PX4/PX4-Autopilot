/****************************************************************************
 * drivers/analog/pga11x.c
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

#include <nuttx/analog/pga11x.h>

#if defined(CONFIG_ADC) && defined(CONFIG_ADC_PGA11X)

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
#define PGA11X_CMD_NOOP          0x0000
#define PGA11X_CMD_SDN_DIS       0xe100
#define PGA11X_CMD_SDN_EN        0xe1f1

/* SPI Daisy-Chain Commands (PGA116/PGA117) */

#define PGA11X_DCCMD_SELECTOR    0x8000
#define PGA11X_DCCMD_NOOP        (PGA11X_DCCMD_SELECTOR | PGA11X_CMD_NOOP)
#define PGA11X_DCCMD_SDN_DIS     (PGA11X_DCCMD_SELECTOR | PGA11X_CMD_SDN_DIS)
#define PGA11X_DCCMD_SDN_EN      (PGA11X_DCCMD_SELECTOR | PGA11X_CMD_SDN_EN)
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

/* Debug ********************************************************************/
/* Check if (non-standard) SPI debug is enabled */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg dbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg dbg
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
 *   cmd -  PGA11X command (non-daisy chained)
 *   u1cmd - PGA11X U1 command (daisy chained)
 *   u2cmd - PGA11X U2 command (daisy chained)
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
static void pga11x_write(FAR struct spi_dev_s *spi, uint16_t u1cmd, uint16_t u2cmd)
{
  spivdbg("U1 cmd: %04x U2 cmd: %04x\n", u1cmd, u2cmd);

  /* Lock, select, send the U2 16-bit command, the U1 16-bit command, de-select,
   * and un-lock.
   */

  pga11x_lock(spi);
  SPI_SELECT(spi, SPIDEV_MUX, true);
  pga11x_send16(spi, u2cmd);
  pga11x_send16(spi, u1cmd);
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
 *   Initialize the PGA117 amplifier/multiplexer(s).
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
 *   Select an input channel and gain for all PGA11xs.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_select() configures
 *   both chips in the daisy-chain.  pga11x_uselect() is provided to support
 *   configuring the parts in the daisychain independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   settings - New channel and gain settings
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_select(PGA11X_HANDLE handle,
                  FAR const struct pga11x_settings_s *settings)
{
#ifndef CONFIG_PGA11X_DAISYCHAIN
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t cmd;

  DEBUGASSERT(handle && settings);
  spivdbg("channel: %d gain: %d\n", settings->channel, settings->gain);

  /* Format the command */

  cmd = PGA11X_CMD_WRITE |
        ((uint16_t)settings->channel << PGA11X_CHAN_SHIFT) |
        ((uint16_t)settings->gain << PGA11X_GAIN_SHIFT);

  /* Send the command */

  pga11x_write(spi, cmd);
  return OK;
#else
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t u1cmd;
  uint16_t u2cmd;

  DEBUGASSERT(handle && settings);
  spivdbg("U1 channel: %d gain: %d\n", settings->u1.channel, settings->u1.gain);
  spivdbg("U1 channel: %d gain: %d\n", settings->u1.channel, settings->u1.gain);

  /* Format the commands */

  u1cmd = PGA11X_CMD_WRITE |
          ((uint16_t)settings->u1.channel << PGA11X_CHAN_SHIFT) |
          ((uint16_t)settings->u1.gain << PGA11X_GAIN_SHIFT);

  u2cmd = PGA11X_DCCMD_WRITE |
          ((uint16_t)settings->u2.channel << PGA11X_CHAN_SHIFT) |
          ((uint16_t)settings->u2.gain << PGA11X_GAIN_SHIFT);

  /* Send the command */

  pga11x_write(spi, u1cmd, u2cmd);
  return OK;
#endif
}

/****************************************************************************
 * Name: pga11x_uselect
 *
 * Description:
 *   Select an input channel and gain for one PGA11x.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_uselect() configures
 *   one chips in the daisy-chain.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *   settings - New channel and gain settings
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uselect(PGA11X_HANDLE handle, int pos,
                   FAR const struct pga11x_usettings_s *settings)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t u1cmd;
  uint16_t u2cmd;

  spivdbg("channel: %d gain: %d\n", settings->channel, settings->gain);
  DEBUGASSERT(handle);

  /* Format the commands */

  if (pos == 0)
    {
      u1cmd = PGA11X_CMD_WRITE |
              ((uint16_t)settings->channel << PGA11X_CHAN_SHIFT) |
              ((uint16_t)settings->gain << PGA11X_GAIN_SHIFT);
      u2cmd = PGA11X_DCCMD_NOOP;
    }
  else /* if (pos == 1) */
    {
      u1cmd = PGA11X_CMD_NOOP;
      u2cmd = PGA11X_DCCMD_WRITE |
              ((uint16_t)settings->channel << PGA11X_CHAN_SHIFT) |
              ((uint16_t)settings->gain << PGA11X_GAIN_SHIFT);
    }

  /* Send the command */

  pga11x_write(spi, u1cmd, u2cmd);
  return OK;
}
#endif

/****************************************************************************
 * Name: pga11x_read
 *
 * Description:
 *   Read from all PGA117 amplifier/multiplexers.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_read() reads from
 *   both chips in the daisy-chain.  pga11x_uread() is provided to support
 *   accessing the parts independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   settings - Returned channel and gain settings
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

int pga11x_read(PGA11X_HANDLE handle, FAR struct pga11x_settings_s *settings)
{
#ifdef CONFIG_PGA11X_DAISYCHAIN
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;
  uint16_t u1value;
  uint16_t u2value;

  spivdbg("Entry\n");
  DEBUGASSERT(handle && settings);

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
  u2value = pga11x_recv16(spi);
  u1value = pga11x_recv16(spi);
  SPI_SELECT(spi, SPIDEV_MUX, false);
  pga11x_unlock(spi);

  /* Decode the returned value */

  spivdbg("Returning %04x %04x\n", u2value, u1value);
  settings->u1.channel = (uint8_t)((u1value & PGA11X_CHAN_MASK) >> PGA11X_CHAN_SHIFT);
  settings->u1.gain    = (uint8_t)((u1value & PGA11X_GAIN_MASK) >> PGA11X_GAIN_SHIFT);
  settings->u2.channel = (uint8_t)((u2value & PGA11X_CHAN_MASK) >> PGA11X_CHAN_SHIFT);
  settings->u2.gain    = (uint8_t)((u2value & PGA11X_GAIN_MASK) >> PGA11X_GAIN_SHIFT);
  return OK;
#else
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

  /* Decode the returned value */

  spivdbg("Returning: %04x\n", value);
  settings->channel = (uint8_t)((value & PGA11X_CHAN_MASK) >> PGA11X_CHAN_SHIFT);
  settings->gain    = (uint8_t)((value & PGA11X_GAIN_MASK) >> PGA11X_GAIN_SHIFT);
  return OK;
#endif
}

/****************************************************************************
 * Name: pga11x_uread
 *
 * Description:
 *   Read from one PGA117 amplifier/multiplexer.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_read() reads
 *   the parts independently.
 *
 * Input Parameters:
 *   spi      - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *   settings - Returned channel and gain settings
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uread(PGA11X_HANDLE handle, int pos,
                 FAR struct pga11x_usettings_s *settings)
{
  struct pga11x_settings_s both;
  int ret = pga11x_read(handle, &both);
  if (ret == OK)
    {
      if (pos == 0)
        {
          settings->channel = both.u1.channel;
          settings->gain    = both.u1.gain;
        }
      else /* if (pos == 1) */
        {
          settings->channel = both.u2.channel;
          settings->gain    = both.u2.gain;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: pga11x_shutdown
 *
 * Description:
 *   Put all PGA11x's in shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_shutdown() controls
 *   both chips in the daisy-chain.  pga11x_ushutdown() is provided to
 *   control the parts independently.
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

  /* Enter shutdown mode by issuing an SDN_EN command */

#ifdef CONFIG_PGA11X_DAISYCHAIN
  pga11x_write(spi, PGA11X_CMD_SDN_EN, PGA11X_DCCMD_SDN_EN);
#else
  pga11x_write(spi, PGA11X_CMD_SDN_EN);
#endif
  return OK;
}

/****************************************************************************
 * Name: pga11x_ushutdown
 *
 * Description:
 *   Put one PGA11x in shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_ushutdown() is
 *   provided to shutdown the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_ushutdown(PGA11X_HANDLE handle, int pos)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Enter shutdown mode by issuing an SDN_EN command */

  if (pos == 0)
    {
      pga11x_write(spi, PGA11X_CMD_SDN_EN, PGA11X_DCCMD_NOOP);
    }
  else
    {
      pga11x_write(spi, PGA11X_CMD_NOOP, PGA11X_DCCMD_SDN_EN);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: pga11x_enable
 *
 * Description:
 *   Take all PGA11x's out of shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_enable() controls
 *   both chips in the daisy-chain.  pga11x_uenable() is provided to
 *   control the parts independently.
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

#ifdef CONFIG_PGA11X_DAISYCHAIN
  pga11x_write(spi, PGA11X_CMD_SDN_DIS, PGA11X_DCCMD_SDN_DIS);
#else
  pga11x_write(spi, PGA11X_CMD_SDN_DIS);
#endif
  return OK;
}

/****************************************************************************
 * Name: pga11x_uenable
 *
 * Description:
 *   Take one PGA11x out of shutdown down mode.
 *
 *   If CONFIG_PGA11X_DAISYCHAIN is defined, then pga11x_uenable() is
 *   provided to enable the parts independently.
 *
 * Input Parameters:
 *   spi - An SPI "bottom half" device driver instance
 *   pos      - Position of the chip in the daisy chain (0 or 1)
 *
 * Returned Value:
 *   Zero on sucess; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PGA11X_DAISYCHAIN
int pga11x_uenable(PGA11X_HANDLE handle, int pos)
{
  FAR struct spi_dev_s *spi = (FAR struct spi_dev_s *)handle;

  spivdbg("Entry\n");
  DEBUGASSERT(handle);

  /* Enter shutdown mode by issuing an SDN_EN command */

  if (pos == 0)
    {
      pga11x_write(spi, PGA11X_CMD_SDN_DIS, PGA11X_DCCMD_NOOP);
    }
  else
    {
      pga11x_write(spi, PGA11X_CMD_NOOP, PGA11X_DCCMD_SDN_DIS);
    }

  return OK;
}
#endif

#endif /* CONFIG_ADC && CONFIG_ADC_PGA11X */

