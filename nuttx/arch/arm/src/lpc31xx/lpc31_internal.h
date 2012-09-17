/************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_internal.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_INTERNAL_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "up_arch.h"
#include "chip.h"
#include "lpc31_ioconfig.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* NVIC priority levels *************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/* Configure a pin as an input */

static inline void gpio_configinput(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0RESET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1RESET_OFFSET;
  putreg32(bit, regaddr);
}

/* Return the current state of an input GPIO pin */

static inline bool lpc31_gpioread(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr = ioconfig + LPC31_IOCONFIG_PINS_OFFSET;
  return (getreg32(regaddr) & bit) != 0;
}

/* Configure the pin so that it is driven by the device */

static inline void gpio_configdev(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1RESET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0SET_OFFSET;
  putreg32(bit, regaddr);
}

/* Configure a pin as a low output */

static inline void gpio_outputlow(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1SET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0RESET_OFFSET;
  putreg32(bit, regaddr);
}

/* Configure a pin as a high output */

static inline void gpio_outputhigh(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1SET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0SET_OFFSET;
  putreg32(bit, regaddr);
}

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: lpc31_lowsetup
 *
 * Description:
 *   Called early in up_boot.  Performs chip-common low level initialization.
 *
 ************************************************************************************/

EXTERN void lpc31_lowsetup(void);

/************************************************************************************
 * Name: lpc31_clockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 ************************************************************************************/

EXTERN void lpc31_clockconfig(void);

/************************************************************************************
 * Name:  lpc31_spiselect and lpc31_spistatus
 *
 * Description:
 *   The external functions, lpc31_spiselect, lpc31_spistatus, and
 *   lpc31_spicmddata must be provided by board-specific logic.  These are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods
 *   (including up_spiinitialize()) are provided by common LPC31XX logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in lpc31_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide lpc31_spiselect() and lpc31_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is selected in your NuttX configuration, provide
 *      the lpc31_spicmddata() function in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in the
 *      way your board is configured.
 *   4. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   5. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;
EXTERN void  lpc31_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc31_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc31_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif

/************************************************************************************
 * Name:  lpc31_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB software
 *   connect and disconnect), then the board software must provide lpc31_pullup.
 *   See include/nuttx/usb/usbdev.h for additional description of this method.
 *   Alternatively, if no pull-up GPIO the following EXTERN can be redefined to be
 *   NULL.
 *
 ************************************************************************************/

struct usbdev_s;
EXTERN int lpc31_usbpullup(FAR struct usbdev_s *dev,  bool enable);

/************************************************************************************
 * Name:  lpc31_usbsuspend
 *
 * Description:
 *   Board logic must provide the lpc31_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

struct usbdev_s;
EXTERN void lpc31_usbsuspend(FAR struct usbdev_s *dev, bool resume);

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See include/nuttx/sdio.h */
EXTERN FAR struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a 
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_INTERNAL_H */
