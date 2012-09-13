/************************************************************************************
 * arch/avr/src/at32uc3/chip.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_CHIP_H
#define __ARCH_AVR_SRC_AT32UC3_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

/* UC3 A0/A1 Series */
/* UC3 A2/A3 Series */

/* UC3 B0 (64-pin) / B1 (48-pin, no USB host) Series */

#if CONFIG_ARCH_CHIP_AT32UC3B064
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (64*1024)    /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (16*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B0128
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (128*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B0256
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (256*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B0512
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B0 1            /* UC3 B0 (64-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (512*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (96*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  define AVR32_USB_HOST             1            /* USB host support (OTG) */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               3            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 1            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                44           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 2            /* Number of crystal oscillators */
#  define AVR32_NADC10               8            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B164
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (64*1024)    /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (16*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B1128
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (128*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B1256
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (256*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (32*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#elif CONFIG_ARCH_CHIP_AT32UC3B1512
#  define CONFIG_ARCH_CHIP_AT32UC3B  1            /* UC3 B series */
#  define CONFIG_ARCH_CHIP_AT32UC3B1 1            /* UC3 B0 (48-pin) series */
#  define AVR32_ONCHIP_FLASH_SIZE    (512*1024)   /* Size of on-chip FLASH (bytes) */
#  define AVR32_ONCHIP_SRAM_SIZE     (96*1024)    /* Size of on-chip SRAM (bytes) */
#  define AVR32_USB_FULLSPEED        1            /* USB full-speed support */
#  undef  AVR32_USB_HOST                          /* No USB host support */
#  define AVR32_USB_DEVICE           1            /* USB device support */
#  define AVR32_NUSART               2            /* Number of USARTs */
#  define AVR32_NSPI                 2            /* Number of SPI */
#  define AVR32_NTWI                 1            /* Number of TWI (I2C) */
#  define AVR32_NSSC                 0            /* Number of SSC (I2S audio) */
#  define AVR32_NGPIO                28           /* Number of GPIO pins */
#  define AVR32_NTIMER               3            /* Number of Timers */
#  define AVR32_NPWM                 13           /* Number of PWM channels */
#  define AVR32_NOSC                 1            /* Number of crystal oscillators */
#  define AVR32_NADC10               6            /* Number of 10-bit A/D channels */
#  define AVR32_NDMAC                7            /* Number of DMA channels */
#else
#  error "Unsupported AVR32 chip"
#endif

/* Include only the memory map.  Other chip hardware files should then include this
 * file for the proper setup
 */

#include "at32uc3_memorymap.h"

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_CHIP_H */

