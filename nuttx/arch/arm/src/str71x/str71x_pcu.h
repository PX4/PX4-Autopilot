/************************************************************************************
 * arch/arm/src/str71x/str71x_pcu.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_PCU_H
#define __ARCH_ARM_SRC_STR71X_STR71X_PCU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Power Control Unit (PCU) register offsets ****************************************/

#define STR71X_PCU_MDIVR_OFFSET     (0x0000)  /* 16-bits wide */
#define STR71X_PCU_PDIVR_OFFSET     (0x0004)  /* 16-bits wide */
#define STR71X_PCU_RSTR_OFFSET      (0x0008)  /* 16-bits wide */
#define STR71X_PCU_PLL2CR_OFFSET    (0x000c)  /* 16-bits wide */
#define STR71X_PCU_BOOTCR_OFFSET    (0x0010)  /* 16-bits wide */
#define STR71X_PCU_PWRCR_OFFSET     (0x0014)  /* 16-bits wide */

/* Power Control Unit (PCU) register addresses **************************************/

#define STR71X_PCU_MDIVR            (STR71X_PCU_BASE + STR71X_PCU_MDIVR_OFFSET)
#define STR71X_PCU_PDIVR            (STR71X_PCU_BASE + STR71X_PCU_PDIVR_OFFSET)
#define STR71X_PCU_RSTR             (STR71X_PCU_BASE + STR71X_PCU_RSTR_OFFSET)
#define STR71X_PCU_PLL2CR           (STR71X_PCU_BASE + STR71X_PCU_PLL2CR_OFFSET)
#define STR71X_PCU_BOOTCR           (STR71X_PCU_BASE + STR71X_PCU_BOOTCR_OFFSET)
#define STR71X_PCU_PWRCR            (STR71X_PCU_BASE + STR71X_PCU_PWRCR_OFFSET)

/* Register bit settings ************************************************************/

/* PCU MDIVR register bit definitions */

#define STR71X_PCUMDIVR_FACTMASK    (0x0003) /* Bits 0-1: Division factor for main system clock */
#define STR71X_PCUMDIVR_DIV1        (0x0000) /*   MCLK = RCLK */
#define STR71X_PCUMDIVR_DIV2        (0x0001) /*   MCLK = RCLK / 2 */
#define STR71X_PCUMDIVR_DIV4        (0x0002) /*   MCLK = RCLK / 4 */
#define STR71X_PCUMDIVR_DIV8        (0x0003) /*   MCLK = RCLK / 8 */

/* PCU PDIVR register bit definitions */

#define STR71X_PCUPDIVR_FACT1MASK   (0x0003) /* Bits 0-1: Division factor for APB1 peripherals */
#define STR71X_PCUPDIVR_APB1DIV1    (0x0000) /*   PCLK1 = RCLK */
#define STR71X_PCUPDIVR_APB1DIV2    (0x0001) /*   PCLK1 = RCLK / 2 */
#define STR71X_PCUPDIVR_APB1DIV4    (0x0002) /*   PCLK1 = RCLK / 4 */
#define STR71X_PCUPDIVR_APB1DIV8    (0x0003) /*   PCLK1 = RCLK / 8 */
#define STR71X_PCUPDIVR_FACT2MASK   (0x0300) /* Bits 8-9: Division factor for APB2 peripherals */
#define STR71X_PCUPDIVR_APB2DIV1    (0x0000) /*   PCLK2 = RCLK */
#define STR71X_PCUPDIVR_APB2DIV2    (0x0100) /*   PCLK2 = RCLK / 2 */
#define STR71X_PCUPDIVR_APB2DIV4    (0x0200) /*   PCLK2 = RCLK / 4 */
#define STR71X_PCUPDIVR_APB2DIV8    (0x0300) /*   PCLK2 = RCLK / 8 */

/* PCU RSTR register bit definitions */

#define STR71X_PCURSTR_EMIRESET     (0x0004) /* Bit 2: EMI reset */

/* PCU PLL2CR register bit definitions */

#define STR71X_PCUPPL2CR_DXMASK     (0x0007) /* Bits 0-2: PLL2 output clock divider */
#define STR71X_PCUPPL2CR_DIV1       (0x0000) /*   PLL2 / 1 */
#define STR71X_PCUPPL2CR_DIV2       (0x0001) /*   PLL2 / 2 */
#define STR71X_PCUPPL2CR_DIV3       (0x0002) /*   PLL2 / 3 */
#define STR71X_PCUPPL2CR_DIV4       (0x0003) /*   PLL2 / 4 */
#define STR71X_PCUPPL2CR_DIV5       (0x0004) /*   PLL2 / 5 */
#define STR71X_PCUPPL2CR_DIV6       (0x0005) /*   PLL2 / 6 */
#define STR71X_PCUPPL2CR_DIV7       (0x0006) /*   PLL2 / 7 */
#define STR71X_PCUPPL2CR_OFF        (0x0007) /*   PLL2 OFF */
#define STR71X_PCUPPL2CR_MXMASK     (0x0030) /* Bits 4-5: PLL2 multiplier */
#define STR71X_PCUPPL2CR_MUL20      (0x0000) /*   CLK2 * 20 */
#define STR71X_PCUPPL2CR_MUL12      (0x0010) /*   CLK2 * 12 */
#define STR71X_PCUPPL2CR_MUL28      (0x0020) /*   CLK2 * 28 */
#define STR71X_PCUPPL2CR_MUL16      (0x0030) /*   CLK2 * 16 */
#define STR71X_PCUPPL2CR_FRQRNG     (0x0040) /* Bit 6: PLL2 frequency range selection */
#define STR71X_PCUPPL2CR_PLLEN      (0x0080) /* Bit 7: PLL2 enable */
#define STR71X_PCUPPL2CR_USBEN      (0x0100) /* Bit 8: Enable PLL clock to USB */
#define STR71X_PCUPPL2CR_IRQMASK    (0x0200) /* Bit 9: Enable interrupt request CPU on lock transition */
#define STR71X_PCUPPL2CR_IRQPEND    (0x0400) /* Bit 10: Interrtup request to CPU on lock transition pending */
#define STR71X_PCUPPL2CR_LOCK       (0x8000) /* Bit 15: PLL2 locked */

/* PCU BOOTCR register bit definitions */

#define STR71X_PCUBOOTCR_BOOTMASK   (0x0003) /* Bits 0-1: Boot mode */
#define STR71X_PCUBOOTCR_BMFLASH    (0x0000) /*   FLASH */
#define STR71X_PCUBOOTCR_BMRAM      (0x0002) /*   RAM */
#define STR71X_PCUBOOTCR_BMEXTMEM   (0x0003) /*   FLASH */
#define STR71X_PCUBOOTCR_BSPIOEN    (0x0004) /* Bit 2: Enable BSPI0 */
#define STR71X_PCUBOOTCR_USBFILTEN  (0x0008) /* Bit 3: Enable USB standby filtering */
#define STR71X_PCUBOOTCR_LPOWDBGEN  (0x0010) /* Bit 4: Enable reserved features for STOP mode */
#define STR71X_PCUBOOTCR_ACDEN      (0x0020) /* Bit 5: Enable ADC */
#define STR71X_PCUBOOTCR_CANACTIVE  (0x0040) /* Bit 6: CAN active */
#define STR71X_PCUBOOTCR_HDLCACTIVE (0x0080) /* Bit 7: HDLC active */
#define STR71X_PCUBOOTCR_PKG64      (0x0200) /* Bit 9: Die is hosted in 64-pin package */

/* PCU PWRCR register bit definitions */

#define STR71X_PCUPWRCR_VRBYP       (0x0008) /* Bit 3: Main regulator bypass */
#define STR71X_PCUPWRCR_LPRWFI      (0x0010) /* Bit 4: Low power regulator in wait-for-interrupt mode */
#define STR71X_PCUPWRCR_LPRBYP      (0x0020) /* Bit 5: Low power regulator bypass */
#define STR71X_PCUPWRCR_PWRDWN      (0x0040) /* Bit 6: Activate standby mode */
#define STR71X_PCUPWRCR_OSCBYP      (0x0080) /* Bit 7: 32KHz oscillator bypass */
#define STR71X_PCUPWRCR_LVDDIS      (0x0100) /* Bit 8: Low voltage detector disable */
#define STR71X_PCUPWRCR_FLASHLP     (0x0200) /* Bit 9: FLASH low speed (low power) select */
#define STR71X_PCUPWRCR_VROK        (0x1000) /* Bit 12: Voltage regulator OK */
#define STR71X_PCUPWRCR_WKUPALRM    (0x2000) /* Bit 13: Wakeup or alarm active */
#define STR71X_PCUPWRCR_BUSY        (0x4000) /* Bit 14: PCU register backup logic busy */
#define STR71X_PCUPWRCR_WREN        (0x8000) /* Bit 15: PCU register write enable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_PCU_H */
