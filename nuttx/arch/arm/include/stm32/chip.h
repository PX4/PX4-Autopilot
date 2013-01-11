/************************************************************************************
 * arch/arm/include/stm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_STM32_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip and provide alternate function pin-mapping
 *
 * NOTE: Each GPIO pin may serve either for general purpose I/O or for a special
 * alternate function (such as USART, CAN, USB, SDIO, etc.).  That particular
 * pin-mapping will depend on the package and STM32 family.  If you are incorporating
 * a new STM32 chip into NuttX, you will need to add the pin-mapping to a header file
 * and to include that header file below. The chip-specific pin-mapping is defined in
 * the chip datasheet.
 */

/* STM32 F100 Value Line ************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32F100C8) || defined(CONFIG_ARCH_CHIP_STM32F100CB) \
 || defined(CONFIG_ARCH_CHIP_STM32F100R8) || defined(CONFIG_ARCH_CHIP_STM32F100RB)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2,3,4 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 3 additional timers (15-17) that don't fit any existing category
#  define STM32_NDMA                     1   /* DMA1 */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    64  /* GPIOA-D */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100V8) || defined(CONFIG_ARCH_CHIP_STM32F100VB)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  define CONFIG_STM32_MEDIUMDENSITY     1   /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    3   /* 16-bit general timers TIM2,3,4 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 3 additional timers (15-17) that don't fit any existing category
#  define STM32_NDMA                     1   /* DMA1 */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F100 High-density value Line ************************************************************/

#elif defined(CONFIG_ARCH_CHIP_STM32F100RC) || defined(CONFIG_ARCH_CHIP_STM32F100RD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100RE)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    0   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 6 additional timers (12-17) that don't fit any existing category
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    64  /* GPIOA-D */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F100VC) || defined(CONFIG_ARCH_CHIP_STM32F100VD) \
 || defined(CONFIG_ARCH_CHIP_STM32F100VE)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  define CONFIG_STM32_VALUELINE         1   /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* 2 basic timers: TIM6, TIM7 */
// TODO: there are also 6 additional timers (12-17) that don't fit any existing category
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     0   /* No CAN */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     2   /* DAC 1-2 */
#  define STM32_NCRC                     1   /* CRC1 */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F103 High Density Family ***************************************************/
/* STM32F103RC, STM32F103RD, and STM32F103RE are all provided in 64 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103RET6)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef CONFIG_STM32_LOWDENSITY             /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* 16-bit generall timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* CAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    51  /* GPIOA-D */
#  define STM32_NADC                     2   /* ADC1-2 */
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103VC, STM32F103VD, and STM32F103VE are all provided in 100 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103VCT6) || defined(CONFIG_ARCH_CHIP_STM32F103VET6)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and TIM8 */
#  define STM32_NGTIM                    4   /* General timers TIM2,3,4,5 */
#  define STM32_NBTIM                    2   /* Two basic timers TIM6 and TIM7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   5   /* USART1-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* bxCAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     3   /* ADC1-3 */
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NTHERNET                 0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32F103ZC, STM32F103ZD, and STM32F103ZE are all provided in 144 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#elif defined(CONFIG_ARCH_CHIP_STM32F103ZET6) 
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  define CONFIG_STM32_HIGHDENSITY       1   /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx families */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timer TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit generall timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    0   /* No basic timers */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     2   /* SPI1-2 */
#  define STM32_NI2S                     0   /* No I2S (?) */
#  define STM32_NUSART                   3   /* USART1-3 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     1   /* CAN1 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    112 /* GPIOA-G */
#  define STM32_NADC                     1   /* ADC1 */
#  define STM32_NDAC                     0   /* No DAC */
#  define STM32_NCRC                     0   /* No CRC */
#  define STM32_NETHERNET                0   /* No ethernet */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F105/F107 Connectivity Line *******************************************************/
#elif defined(CONFIG_ARCH_CHIP_STM32F105VBT7)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_CONNECTIVITYLINE  1   /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit generall timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3, UART 4-5 */
#  define STM32_NI2C                     2   /* I2C1-2 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2*/
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  define CONFIG_STM32_STM32F10XX        1   /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  define CONFIG_STM32_CONNECTIVITYLINE  1   /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    1   /* One advanced timers TIM1 */
#  define STM32_NGTIM                    4   /* 16-bit generall timers TIM2,3,4,5 with DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   5   /* USART1-3, UART 4-5 */
#  define STM32_NI2C                     1   /* I2C1 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    0   /* No SDIO */
#  define STM32_NUSBOTG                  0   /* No USB OTG FS/HS */
#  define STM32_NGPIO                    80  /* GPIOA-E */
#  define STM32_NADC                     2   /* ADC1-2*/
#  define STM32_NDAC                     2   /* DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     0   /* No random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

/* STM32 F2 Family ******************************************************************/
#elif defined(CONFIG_ARCH_CHIP_STM32F207IG)  /* UFBGA-176 1024Kb FLASH 128Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  define CONFIG_STM32_STM32F20XX        1   /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F40XX            /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    140 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

/* STM23 F4 Family ******************************************************************/
#elif defined(CONFIG_ARCH_CHIP_STM32F405RG)  /* LQFP 64 10x10x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    0   /* No FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405VG)  /* LQFP 100 14x14x1.4  1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F405ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                0   /* No Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    0   /* No digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VE)  /* LQFP-100 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407VG)  /* LQFP-100 14x14x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 16 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZE)  /* LQFP-144 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407ZG)  /* LQFP 144 20x20x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IE)  /* LQFP 176 24x24x1.4 512Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 (?) */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#elif defined(CONFIG_ARCH_CHIP_STM32F407IG)  /* BGA 176; LQFP 176 24x24x1.4 1024Kb FLASH 192Kb SRAM */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx and STM32407xx */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     3   /* SPI1-3 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   6   /* USART1-3 and 6, UART 4-5 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

#else
#  error "Unsupported STM32 chip"
#endif

#endif /* __ARCH_ARM_INCLUDE_STM32_CHIP_H */

