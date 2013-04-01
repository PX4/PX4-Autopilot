/************************************************************************************
 * arch/arm/src/stm32/stm32_spi.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_STC_STM32_STM32_SPI_H
#define __ARCH_ARM_STC_STM32_STM32_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/stm32_spi.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

struct  spi_dev_s;
enum    spi_dev_e;

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  stm32_spi1/2/...select and stm32_spi1/2/...status
 *
 * Description:
 *   The external functions, stm32_spi1/2/...select, stm32_spi1/2/...status, and
 *   stm32_spi1/2/...cmddata must be provided by board-specific logic.  These are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods
 *   (including up_spiinitialize()) are provided by common STM32 logic.  To use this
 *   common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/...select() and stm32_spi1/2/...status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file, then
 *      provide stm32_spi1/2/...cmddata() functions in your board-specific logic. 
 *      These functions will perform cmd/data selection operations using GPIOs in the
 *      way your board is configured.
 *   4. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   5. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

EXTERN void  stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

EXTERN void  stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

EXTERN void  stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi3cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

EXTERN void  stm32_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi4cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

EXTERN void  stm32_spi5select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi5cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

EXTERN void  stm32_spi6select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t stm32_spi6status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int stm32_spi6cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_STC_STM32_STM32_SPI_H */

