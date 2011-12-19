/************************************************************************************
 * configs/vsn/src/spi.c
 * arch/arm/src/board/spi.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Gregory Nutt <spudmonkey@racsa.co.cr>
 *            Uros Platise <uros.platise@isotel.eu>
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

/** \file
 *  \author Gregory Nutt, Uros Platise
 *  \brief SPI Slave Selects
 */


#include <nuttx/config.h>
#include <nuttx/spi.h>

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)

#include <arch/board/board.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"
#include "stm32_waste.h"
#include "vsn.h"


/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif


/************************************************************************************
 * Public Functions
 ************************************************************************************/


/** Called to configure SPI chip select GPIO pins for the VSN board.
 */
 
void weak_function stm32_spiinitialize(void)
{
  /* NOTE: Clocking for SPI1 and/or SPI2 and SPI3 was already provided in stm32_rcc.c.
   *       Configurations of SPI pins is performed in stm32_spi.c.
   *       Here, we only initialize chip select pins unique to the board architecture.
   */
   
#ifdef CONFIG_STM32_SPI2
    stm32_configgpio(GPIO_CC1101_CS);
#endif

#ifdef CONFIG_STM32_SPI3
    stm32_configgpio(GPIO_FRAM_CS);
#endif
}

/** Selects: stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 **/

#ifdef CONFIG_STM32_SPI1

void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}

#endif

#ifdef CONFIG_STM32_SPI2

void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
    spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
    
    if (devid == SPIDEV_WIRELESS) {
    
        stm32_gpiowrite(GPIO_CC1101_CS, !selected);

        /* Wait for MISO to go low, indicates that Quart has stabilized */
        if (selected) {
            while( stm32_gpioread(GPIO_SPI2_MISO) ) up_waste();
        }
        
    }
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}

#endif

#ifdef CONFIG_STM32_SPI3

void stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  if (devid == SPIDEV_FLASH)
  {
    /* Set the GPIO low to select and high to de-select */
    stm32_gpiowrite(GPIO_FRAM_CS, !selected);
  }
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return SPI_STATUS_PRESENT;
}

#endif

#endif /* CONFIG_STM32_SPI1 || CONFIG_STM32_SPI2 */
