/****************************************************************************
 * include/nuttx/spi.h
 *
 *   Copyright(C) 2008-2011 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NUTTX_SPI_H
#define __NUTTX_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SPI_OWNBUS - Set if there is only one active device on the SPI bus.
 *   No locking or SPI configuration will be performed. It is not necessary
 *   for clients to lock, re-configure, etc..
 * CONFIG_SPI_EXCHANGE - Driver supports a single exchange method
 *   (vs a recvblock() and sndblock ()methods).
 * CONFIG_SPI_CMDDATA - Devices on the SPI bus require out-of-band support
 *   to distinguish command transfers from data transfers.  Such devices
 *   will often support either 9-bit SPI (yech) or 8-bit SPI and a GPIO
 *   output that selects between command and data.
 */

/* Access macros ************************************************************/

/****************************************************************************
 * Name: SPI_LOCK
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
#  define SPI_LOCK(d,l) (d)->ops->lock(d,l)
#else
#  define SPI_LOCK(d,l)
#endif

/****************************************************************************
 * Name: SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
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

#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))

/****************************************************************************
 * Name: SPI_SETFREQUENCY
 *
 * Description:
 *   Set the SPI frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define SPI_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: SPI_SETMODE
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

#define SPI_SETMODE(d,m) \
  do { if ((d)->ops->setmode) (d)->ops->setmode(d,m); } while (0)

/****************************************************************************
 * Name: SPI_SETBITS
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests.
 *           If value is greater > 0 then it implies MSB first
 *           If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SETBITS(d,b) \
  do { if ((d)->ops->setbits) (d)->ops->setbits(d,b); } while (0)

/****************************************************************************
 * Name: SPI_STATUS
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

#define SPI_STATUS(d,id) \
  ((d)->ops->status ? (d)->ops->status(d, id) : SPI_STATUS_PRESENT)

/* SPI status bits -- Some dedicated for SPI MMC/SD support and may have no
 * relationship to SPI other than needed by the SPI MMC/SD interface
 */

#define SPI_STATUS_PRESENT     0x01 /* Bit 0=1: MMC/SD card present */
#define SPI_STATUS_WRPROTECTED 0x02 /* Bit 1=1: MMC/SD card write protected */

/****************************************************************************
 * Name: SPI_CMDDATA
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
#  define SPI_CMDDATA(d,id,cmd) ((d)->ops->cmddata(d,id,cmd))
#endif

/****************************************************************************
 * Name: SPI_SEND
 *
 * Description:
 *   Exchange one word on SPI. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

#define SPI_SEND(d,wd) ((d)->ops->send(d,(uint16_t)wd))

/****************************************************************************
 * Name: SPI_SNDBLOCK
 *
 * Description:
 *   Send a block of data on SPI. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
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

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->exchange(d,b,0,l))
#else
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->sndblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_RECVBLOCK
 *
 * Description:
 *   Receive a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->exchange(d,0,b,l))
#else
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->recvblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_EXCHANGE(d,t,r,l) ((d)->ops->exchange(d,t,r,l))
#endif

/****************************************************************************
 * Name: SPI_REGISTERCALLBACK
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change (i.e, anything that would be reported differently by SPI_STATUS).
 *   Optional
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#define SPI_REGISTERCALLBACK(d,c,a) \
  ((d)->ops->registercallback ? (d)->ops->registercallback(d,c,a) : -ENOSYS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The type of the media change callback function */

typedef void (*spi_mediachange_t)(FAR void *arg);

/* If the board supports multiple SPI devices, this enumeration identifies
 * which is selected or de-seleted.
 */

enum spi_dev_e
{
  SPIDEV_NONE = 0,    /* Not a valid value */
  SPIDEV_MMCSD,       /* Select SPI MMC/SD device */
  SPIDEV_FLASH,       /* Select SPI FLASH device */
  SPIDEV_ETHERNET,    /* Select SPI ethernet device */
  SPIDEV_DISPLAY,     /* Select SPI LCD/OLED display device */
  SPIDEV_WIRELESS,    /* Select SPI Wireless device */
  SPIDEV_TOUCHSCREEN  /* Select SPI touchscreen device */
};

/* Certain SPI devices may required differnt clocking modes */

enum spi_mode_e
{
  SPIDEV_MODE0 = 0,   /* CPOL=0 CHPHA=0 */
  SPIDEV_MODE1,       /* CPOL=0 CHPHA=1 */
  SPIDEV_MODE2,       /* CPOL=1 CHPHA=0 */
  SPIDEV_MODE3        /* CPOL=1 CHPHA=1 */
};

/* The SPI vtable */

struct spi_dev_s;
struct spi_ops_s
{
#ifndef CONFIG_SPI_OWNBUS
  int      (*lock)(FAR struct spi_dev_s *dev, bool lock);
#endif
  void     (*select)(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                     bool selected);
  uint32_t (*setfrequency)(FAR struct spi_dev_s *dev, uint32_t frequency);
  void     (*setmode)(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
  void     (*setbits)(FAR struct spi_dev_s *dev, int nbits);
  uint8_t  (*status)(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
  int      (*cmddata)(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
  uint16_t (*send)(FAR struct spi_dev_s *dev, uint16_t wd);
#ifdef CONFIG_SPI_EXCHANGE
  void     (*exchange)(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                       FAR void *rxbuffer, size_t nwords);
#else
  void     (*sndblock)(FAR struct spi_dev_s *dev, FAR const void *buffer,
                       size_t nwords);
  void     (*recvblock)(FAR struct spi_dev_s *dev, FAR void *buffer,
                        size_t nwords);
#endif
  int     (*registercallback)(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                              void *arg);
};

/* SPI private data.  This structure only defines the initial fields of the
 * structure visible to the SPI client.  The specific implementation may 
 * add additional, device specific fields
 */

struct spi_dev_s
{
  const struct spi_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR struct spi_dev_s *up_spiinitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __NUTTX_SPI_H */
