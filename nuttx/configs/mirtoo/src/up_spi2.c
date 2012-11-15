/************************************************************************************
 * configs/mirtoo/src/up_spi2.c
 * arch/arm/src/board/up_spi2.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "pic32mx-internal.h"
#include "pic32mx-pps.h"
#include "mirtoo-internal.h"

#ifdef CONFIG_PIC32MX_SPI2

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* The Mirtoo module as two on-board SPI devices:
 *
 * SST25VF032B - 32-Mbit SPI Serial FLASH
 *
 * PGA117 - Zero drift programmable gain amplifier with MUX. The PGA117 offers 10
 * analog inputs, a four-pin SPI interface with daisy-chain capability, and hardware
 * and software shutdown in a TSSOP-20 package. Only 8 of the analog inputs (PORT0-7)
 * are used on the Mirtoo module.
 *
 * Chip selects:
 *
 * ------ -------- ------------------------- --------------------------------
 *  PIN   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *  RPA1   SI     PGA117 and SST25VF032B     SPI2 data OUT (SDO2)
 *  RPA2   SO     PGA117 and SST25VF032B     R1, SPI2 data IN (SDI2)
 *  RPA3   SO     PGA117 and SST25VF032B     R0, SPI2 data IN (SDI2)
 *  SCK2   SCK    PGA117 and SST25VF032B     SPI2 clock
 *
 *  RB7   ~CSAI   PGA117                     PGA117 chip select (active low)
 *  RB13  ~CSM    SST25VF032B                SST25VF032B chip select (active low)
 */

#define GPIO_SI             (GPIO_OUTPUT|GPIO_PORTA|GPIO_PIN1)
#ifdef CONFIG_MIRTOO_RELEASE == 1
#  define GPIO_SO           (GPIO_INPUT|GPIO_PORTA|GPIO_PIN2)
#else
#  define GPIO_SO           (GPIO_INPUT|GPIO_PORTA|GPIO_PIN3)
#endif
#define GPIO_SCK            (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN15)

#define GPIO_PGA117_CS      (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN7)
#define GPIO_SST25VF032B_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN13)

/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_DEBUG_SPI - Define to enable basic SPI debug
 */

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  define spivdbg llvdbg
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_spi2initialize
 *
 * Description:
 *   Called to configure SPI2 chip select GPIO pins for the Mirtoo module.
 *
 ************************************************************************************/

void weak_function pic32mx_spi2initialize(void)
{
  /* Make sure that TRIS pins are set correctly.  Configure the SPI pins as digital
   * inputs and outputs first.
   */

  pic32mx_configgpio(GPIO_SI);
  pic32mx_configgpio(GPIO_SO);
  pic32mx_configgpio(GPIO_SCK);

  /* Configure SPI2 data in and data out to use RPA2 and 1, respectively */

  putreg32(PPS_INSEL_RPA2,  PIC32MX_PPS_SDI2R);
  putreg32(PPS_OUTSEL_SDO2, PIC32MX_PPS_RPA1R);

  /* Configure the SPI chip select GPIOs */

  pic32mx_configgpio(GPIO_PGA117_CS);
  pic32mx_configgpio(GPIO_SST25VF032B_CS);
}

/************************************************************************************
 * Name:  pic32mx_spi2select, pic32mx_spi2status, and pic32mx_spi2cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common PIC32MX logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide pic32mx_spiNselect() and pic32mx_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mx_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;

void  pic32mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_FLASH)
    {
      pic32mx_gpiowrite(GPIO_SST25VF032B_CS, !selected);
    }
  else if (devid == SPIDEV_MUX)
    {
      pic32mx_gpiowrite(GPIO_PGA117_CS, !selected);
    }
}

uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
  return 0;
}
#endif

#endif /* CONFIG_PIC32MX_SPI2 */
