/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_gpio.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPIO_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPIO_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

 /* Register Offsets *********************************************************************************/

/* Pin interrupt registers (relative to LPC43_GPIOINT_BASE) */

#define LPC43_GPIOINT_ISEL_OFFSET   0x0000 /* Pin Interrupt Mode register */
#define LPC43_GPIOINT_IENR_OFFSET   0x0004 /* Pin interrupt level (rising edge) interrupt enable register */
#define LPC43_GPIOINT_SIENR_OFFSET  0x0008 /* Pin interrupt level (rising edge) interrupt set register */
#define LPC43_GPIOINT_CIENR_OFFSET  0x000c /* Pin interrupt level (rising edge interrupt) clear register */
#define LPC43_GPIOINT_IENF_OFFSET   0x0010 /* Pin interrupt active level (falling edge) interrupt enable register */
#define LPC43_GPIOINT_SIENF_OFFSET  0x0014 /* Pin interrupt active level (falling edge) interrupt set register */
#define LPC43_GPIOINT_CIENF_OFFSET  0x0018 /* Pin interrupt active level (falling edge) interrupt clear register */
#define LPC43_GPIOINT_RISE_OFFSET   0x001c /* Pin interrupt rising edge register */
#define LPC43_GPIOINT_FALL_OFFSET   0x0020 /* Pin interrupt falling edge register */
#define LPC43_GPIOINT_IST_OFFSET    0x0024 /* Pin interrupt status register */

/* GPIO GROUP interrupt registers (relative to either LPC43_GRP0INT_BASE or LPC43_GRP1INT_BASE) */

#define LPC43_GRPINT_CTRL_OFFSET    0x0000 /* GPIO grouped interrupt control register */

#define LPC43_GRPINT_POL_OFFSET(p)  (0x0020 + ((p) << 2 ))
#define LPC43_GRPINT_POL0_OFFSET    0x0020 /* GPIO grouped interrupt port 0 polarity register */
#define LPC43_GRPINT_POL1_OFFSET    0x0024 /* GPIO grouped interrupt port 1 polarity register */
#define LPC43_GRPINT_POL2_OFFSET    0x0028 /* GPIO grouped interrupt port 2 polarity register */
#define LPC43_GRPINT_POL3_OFFSET    0x002c /* GPIO grouped interrupt port 3 polarity register */
#define LPC43_GRPINT_POL4_OFFSET    0x0030 /* GPIO grouped interrupt port 4 polarity register */
#define LPC43_GRPINT_POL5_OFFSET    0x0034 /* GPIO grouped interrupt port 5 polarity register */
#define LPC43_GRPINT_POL6_OFFSET    0x0038 /* GPIO grouped interrupt port 6 polarity register */
#define LPC43_GRPINT_POL7_OFFSET    0x003c /* GPIO grouped interrupt port 7 polarity register */

#define LPC43_GRPINT_ENA_OFFSET(p)  (0x0040 + ((p) << 2 ))
#define LPC43_GRPINT_ENA0_OFFSET    0x0040 /* GPIO grouped interrupt port 0 enable register */
#define LPC43_GRPINT_ENA1_OFFSET    0x0044 /* GPIO grouped interrupt port 1 enable register */
#define LPC43_GRPINT_ENA2_OFFSET    0x0048 /* GPIO grouped interrupt port 2 enable register */
#define LPC43_GRPINT_ENA3_OFFSET    0x004c /* GPIO grouped interrupt port 3 enable register */
#define LPC43_GRPINT_ENA4_OFFSET    0x0050 /* GPIO grouped interrupt port 4 enable register */
#define LPC43_GRPINT_ENA5_OFFSET    0x0054 /* GPIO grouped interrupt port 5 enable register */
#define LPC43_GRPINT_ENA6_OFFSET    0x0058 /* GPIO grouped interrupt port 5 enable register */
#define LPC43_GRPINT_ENA7_OFFSET    0x005c /* GPIO grouped interrupt port 5 enable register */

/* GPIO Port Registers (relative to LPC43_GPIO_BASE) */

#define LPC43_GPIO_B_OFFSET(p,n)    (((p) << 5) + (n))
#define LPC43_GPIO_B0_OFFSET(n)     (0x0000 + (n)) /* PIO0_0 to PIO0_31 byte pin registers */
#define LPC43_GPIO_B1_OFFSET(n)     (0x0020 + (n)) /* PIO1_0 to PIO1_31 byte pin registers */
#define LPC43_GPIO_B2_OFFSET(n)     (0x0040 + (n)) /* PIO2_0 to PIO2_31 byte pin registers */
#define LPC43_GPIO_B3_OFFSET(n)     (0x0060 + (n)) /* PIO3_0 to PIO3_31 byte pin registers */
#define LPC43_GPIO_B4_OFFSET(n)     (0x0080 + (n)) /* PIO4_0 to PIO4_31 byte pin registers */
#define LPC43_GPIO_B5_OFFSET(n)     (0x00a0 + (n)) /* PIO5_0 to PIO5_31 byte pin registers */
#define LPC43_GPIO_B6_OFFSET(n)     (0x00c0 + (n)) /* PIO6_0 to PIO6_31 byte pin registers */
#define LPC43_GPIO_B7_OFFSET(n)     (0x00e0 + (n)) /* PIO7_0 to PIO7_31 byte pin registers */

#define LPC43_GPIO_W_OFFSET(p,n)    (0x1000 + ((p) << 7) + ((n) << 2))
#define LPC43_GPIO_W0_OFFSET(n)     (0x1000 + ((n) << 2)) /* PIO0_0 to PIO0_31 word pin registers */
#define LPC43_GPIO_W1_OFFSET(n)     (0x1080 + ((n) << 2)) /* PIO1_0 to PIO1_31 word pin registers */
#define LPC43_GPIO_W2_OFFSET(n)     (0x1100 + ((n) << 2)) /* PIO2_0 to PIO2_31 word pin registers */
#define LPC43_GPIO_W3_OFFSET(n)     (0x1180 + ((n) << 2)) /* PIO3_0 to PIO3_31 word pin registers */
#define LPC43_GPIO_W4_OFFSET(n)     (0x1200 + ((n) << 2)) /* PIO4_0 to PIO4_31 word pin registers */
#define LPC43_GPIO_W5_OFFSET(n)     (0x1280 + ((n) << 2)) /* PIO5_0 to PIO5_31 word pin registers */
#define LPC43_GPIO_W6_OFFSET(n)     (0x1300 + ((n) << 2)) /* PIO6_0 to PIO6_31 word pin registers */
#define LPC43_GPIO_W7_OFFSET(n)     (0x1380 + ((n) << 2)) /* PIO7_0 to PIO7_31 word pin registers */

#define LPC43_GPIO_DIR_OFFSET(p)    (0x2000 + ((p) << 2))
#define LPC43_GPIO_DIR0_OFFSET      0x2000 /* Direction registers port 0 */
#define LPC43_GPIO_DIR1_OFFSET      0x2004 /* Direction registers port 1 */
#define LPC43_GPIO_DIR2_OFFSET      0x2008 /* Direction registers port 2 */
#define LPC43_GPIO_DIR3_OFFSET      0x200c /* Direction registers port 3 */
#define LPC43_GPIO_DIR4_OFFSET      0x2010 /* Direction registers port 4 */
#define LPC43_GPIO_DIR5_OFFSET      0x2014 /* Direction registers port 5 */
#define LPC43_GPIO_DIR6_OFFSET      0x2018 /* Direction registers port 6 */
#define LPC43_GPIO_DIR7_OFFSET      0x201c /* Direction registers port 7 */

#define LPC43_GPIO_MASK_OFFSET(p)   (0x2080 + ((p) << 2))
#define LPC43_GPIO_MASK0_OFFSET     0x2080 /* Mask register port 0 */
#define LPC43_GPIO_MASK1_OFFSET     0x2084 /* Mask register port 1 */
#define LPC43_GPIO_MASK2_OFFSET     0x2088 /* Mask register port 2 */
#define LPC43_GPIO_MASK3_OFFSET     0x208c /* Mask register port 3 */
#define LPC43_GPIO_MASK4_OFFSET     0x2090 /* Mask register port 4 */
#define LPC43_GPIO_MASK5_OFFSET     0x2094 /* Mask register port 5 */
#define LPC43_GPIO_MASK6_OFFSET     0x2098 /* Mask register port 6 */
#define LPC43_GPIO_MASK7_OFFSET     0x209c /* Mask register port 7 */

#define LPC43_GPIO_PIN_OFFSET(p)    (0x2100 + ((p) << 2))
#define LPC43_GPIO_PIN0_OFFSET      0x2100 /* Port pin register port 0 */
#define LPC43_GPIO_PIN1_OFFSET      0x2104 /* Port pin register port 1 */
#define LPC43_GPIO_PIN2_OFFSET      0x2108 /* Port pin register port 2 */
#define LPC43_GPIO_PIN3_OFFSET      0x210c /* Port pin register port 3 */
#define LPC43_GPIO_PIN4_OFFSET      0x2110 /* Port pin register port 4 */
#define LPC43_GPIO_PIN5_OFFSET      0x2114 /* Port pin register port 5 */
#define LPC43_GPIO_PIN6_OFFSET      0x2118 /* Port pin register port 6 */
#define LPC43_GPIO_PIN7_OFFSET      0x211c /* Port pin register port 7 */

#define LPC43_GPIO_MPIN_OFFSET(p)   (0x2100 + ((p) << 2))
#define LPC43_GPIO_MPIN0_OFFSET     0x2180 /* Masked port register port 0 */
#define LPC43_GPIO_MPIN1_OFFSET     0x2184 /* Masked port register port 1 */
#define LPC43_GPIO_MPIN2_OFFSET     0x2188 /* Masked port register port 2 */
#define LPC43_GPIO_MPIN3_OFFSET     0x218c /* Masked port register port 3 */
#define LPC43_GPIO_MPIN4_OFFSET     0x2190 /* Masked port register port 4 */
#define LPC43_GPIO_MPIN5_OFFSET     0x2194 /* Masked port register port 5 */
#define LPC43_GPIO_MPIN6_OFFSET     0x2198 /* Masked port register port 6 */
#define LPC43_GPIO_MPIN7_OFFSET     0x219c /* Masked port register port 7 */

#define LPC43_GPIO_SET_OFFSET(p)    (0x2200 + ((p) << 2))
#define LPC43_GPIO_SET0_OFFSET      0x2200 /* Write: Set register for port 0 */
#define LPC43_GPIO_SET1_OFFSET      0x2204 /* Write: Set register for port 1 */
#define LPC43_GPIO_SET2_OFFSET      0x2208 /* Write: Set register for port 2 */
#define LPC43_GPIO_SET3_OFFSET      0x220c /* Write: Set register for port 3 */
#define LPC43_GPIO_SET4_OFFSET      0x2210 /* Write: Set register for port 4 */
#define LPC43_GPIO_SET5_OFFSET      0x2214 /* Write: Set register for port 5 */
#define LPC43_GPIO_SET6_OFFSET      0x2218 /* Write: Set register for port 6 */
#define LPC43_GPIO_SET7_OFFSET      0x221c /* Write: Set register for port 7 */

#define LPC43_GPIO_CLR_OFFSET(p)    (0x2280 + ((p) << 2))
#define LPC43_GPIO_CLR0_OFFSET      0x2280 /* Clear port 0 */
#define LPC43_GPIO_CLR1_OFFSET      0x2284 /* Clear port 1 */
#define LPC43_GPIO_CLR2_OFFSET      0x2288 /* Clear port 2 */
#define LPC43_GPIO_CLR3_OFFSET      0x228c /* Clear port 3 */
#define LPC43_GPIO_CLR4_OFFSET      0x2290 /* Clear port 4 */
#define LPC43_GPIO_CLR5_OFFSET      0x2294 /* Clear port 5 */
#define LPC43_GPIO_CLR6_OFFSET      0x2298 /* Clear port 6 */
#define LPC43_GPIO_CLR7_OFFSET      0x229c /* Clear port 7 */

#define LPC43_GPIO_NOT_OFFSET(p)    (0x2300 + ((p) << 2))
#define LPC43_GPIO_NOT0_OFFSET      0x2300 /* Toggle port 0 */
#define LPC43_GPIO_NOT1_OFFSET      0x2304 /* Toggle port 1 */
#define LPC43_GPIO_NOT2_OFFSET      0x2308 /* Toggle port 2 */
#define LPC43_GPIO_NOT3_OFFSET      0x230c /* Toggle port 3 */
#define LPC43_GPIO_NOT4_OFFSET      0x2310 /* Toggle port 4 */
#define LPC43_GPIO_NOT5_OFFSET      0x2314 /* Toggle port 5 */
#define LPC43_GPIO_NOT6_OFFSET      0x2318 /* Toggle port 6 */
#define LPC43_GPIO_NOT7_OFFSET      0x231c /* Toggle port 7 */

/* Register Addresses *******************************************************************************/

/* Pin interrupt registers (relative to LPC43_GPIOINT_BASE) */

#define LPC43_GPIOINT_ISEL          (LPC43_GPIOINT_BASE+LPC43_GPIOINT_ISEL_OFFSET)
#define LPC43_GPIOINT_IENR          (LPC43_GPIOINT_BASE+LPC43_GPIOINT_IENR_OFFSET)
#define LPC43_GPIOINT_SIENR         (LPC43_GPIOINT_BASE+LPC43_GPIOINT_SIENR_OFFSET)
#define LPC43_GPIOINT_CIENR         (LPC43_GPIOINT_BASE+LPC43_GPIOINT_CIENR_OFFSET)
#define LPC43_GPIOINT_IENF          (LPC43_GPIOINT_BASE+LPC43_GPIOINT_IENF_OFFSET)
#define LPC43_GPIOINT_SIENF         (LPC43_GPIOINT_BASE+LPC43_GPIOINT_SIENF_OFFSET)
#define LPC43_GPIOINT_CIENF         (LPC43_GPIOINT_BASE+LPC43_GPIOINT_CIENF_OFFSET)
#define LPC43_GPIOINT_RISE          (LPC43_GPIOINT_BASE+LPC43_GPIOINT_RISE_OFFSET)
#define LPC43_GPIOINT_FALL          (LPC43_GPIOINT_BASE+LPC43_GPIOINT_FALL_OFFSET)
#define LPC43_GPIOINT_IST           (LPC43_GPIOINT_BASE+LPC43_GPIOINT_IST_OFFSET)

/* GPIO GROUP0 interrupt registers (relative to LPC43_GRP0INT_BASE) */

#define LPC43_GRP0INT_CTRL          (LPC43_GRP0INT_BASE+LPC43_GRPINT_CTRL_OFFSET)

#define LPC43_GRP0INT_POL(p)        (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL_OFFSET(p))
#define LPC43_GRP0INT_POL0          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL0_OFFSET)
#define LPC43_GRP0INT_POL1          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL1_OFFSET)
#define LPC43_GRP0INT_POL2          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL2_OFFSET)
#define LPC43_GRP0INT_POL3          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL3_OFFSET)
#define LPC43_GRP0INT_POL4          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL4_OFFSET)
#define LPC43_GRP0INT_POL5          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL5_OFFSET)
#define LPC43_GRP0INT_POL6          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL6_OFFSET)
#define LPC43_GRP0INT_POL7          (LPC43_GRP0INT_BASE+LPC43_GRPINT_POL7_OFFSET)

#define LPC43_GRP0INT_ENA(p)        (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA_OFFSET(p))
#define LPC43_GRP0INT_ENA0          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA0_OFFSET)
#define LPC43_GRP0INT_ENA1          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA1_OFFSET)
#define LPC43_GRP0INT_ENA2          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA2_OFFSET)
#define LPC43_GRP0INT_ENA3          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA3_OFFSET)
#define LPC43_GRP0INT_ENA4          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA4_OFFSET)
#define LPC43_GRP0INT_ENA5          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA5_OFFSET)
#define LPC43_GRP0INT_ENA6          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA6_OFFSET)
#define LPC43_GRP0INT_ENA7          (LPC43_GRP0INT_BASE+LPC43_GRPINT_ENA7_OFFSET)

/* GPIO GROUP1 interrupt registers (relative to LPC43_GRP1INT_BASE) */

#define LPC43_GRP1INT_CTRL          (LPC43_GRP1INT_BASE+LPC43_GRPINT_CTRL_OFFSET)

#define LPC43_GRP1INT_POL(p)        (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL_OFFSET(p))
#define LPC43_GRP1INT_POL0          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL0_OFFSET)
#define LPC43_GRP1INT_POL1          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL1_OFFSET)
#define LPC43_GRP1INT_POL2          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL2_OFFSET)
#define LPC43_GRP1INT_POL3          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL3_OFFSET)
#define LPC43_GRP1INT_POL4          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL4_OFFSET)
#define LPC43_GRP1INT_POL5          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL5_OFFSET)
#define LPC43_GRP1INT_POL6          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL6_OFFSET)
#define LPC43_GRP1INT_POL7          (LPC43_GRP1INT_BASE+LPC43_GRPINT_POL7_OFFSET)

#define LPC43_GRP1INT_ENA(p)        (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA_OFFSET(p))
#define LPC43_GRP1INT_ENA0          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA0_OFFSET)
#define LPC43_GRP1INT_ENA1          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA1_OFFSET)
#define LPC43_GRP1INT_ENA2          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA2_OFFSET)
#define LPC43_GRP1INT_ENA3          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA3_OFFSET)
#define LPC43_GRP1INT_ENA4          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA4_OFFSET)
#define LPC43_GRP1INT_ENA5          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA5_OFFSET)
#define LPC43_GRP1INT_ENA6          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA6_OFFSET)
#define LPC43_GRP1INT_ENA7          (LPC43_GRP1INT_BASE+LPC43_GRPINT_ENA7_OFFSET)

/* GPIO Port Registers (relative to LPC43_GPIO_BASE) */

#define LPC43_GPIO_B(p,n)           (LPC43_GPIO_BASE+LPC43_GPIO_B_OFFSET(p,n))
#define LPC43_GPIO_B0(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B0_OFFSET(n))
#define LPC43_GPIO_B1(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B1_OFFSET(n))
#define LPC43_GPIO_B2(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B2_OFFSET(n))
#define LPC43_GPIO_B3(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B3_OFFSET(n))
#define LPC43_GPIO_B4(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B4_OFFSET(n))
#define LPC43_GPIO_B5(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B5_OFFSET(n))
#define LPC43_GPIO_B6(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B6_OFFSET(n))
#define LPC43_GPIO_B7(n)            (LPC43_GPIO_BASE+LPC43_GPIO_B7_OFFSET(n))

#define LPC43_GPIO_W(p,n)           (LPC43_GPIO_BASE+LPC43_GPIO_W_OFFSET(p,n))
#define LPC43_GPIO_W0(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W0_OFFSET(n))
#define LPC43_GPIO_W1(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W1_OFFSET(n))
#define LPC43_GPIO_W2(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W2_OFFSET(n))
#define LPC43_GPIO_W3(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W3_OFFSET(n))
#define LPC43_GPIO_W4(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W4_OFFSET(n))
#define LPC43_GPIO_W5(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W5_OFFSET(n))
#define LPC43_GPIO_W6(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W6_OFFSET(n))
#define LPC43_GPIO_W7(n)            (LPC43_GPIO_BASE+LPC43_GPIO_W7_OFFSET(n))

#define LPC43_GPIO_DIR(p)           (LPC43_GPIO_BASE+LPC43_GPIO_DIR_OFFSET(p))
#define LPC43_GPIO_DIR0             (LPC43_GPIO_BASE+LPC43_GPIO_DIR0_OFFSET)
#define LPC43_GPIO_DIR1             (LPC43_GPIO_BASE+LPC43_GPIO_DIR1_OFFSET)
#define LPC43_GPIO_DIR2             (LPC43_GPIO_BASE+LPC43_GPIO_DIR2_OFFSET)
#define LPC43_GPIO_DIR3             (LPC43_GPIO_BASE+LPC43_GPIO_DIR3_OFFSET)
#define LPC43_GPIO_DIR4             (LPC43_GPIO_BASE+LPC43_GPIO_DIR4_OFFSET)
#define LPC43_GPIO_DIR5             (LPC43_GPIO_BASE+LPC43_GPIO_DIR5_OFFSET)
#define LPC43_GPIO_DIR6             (LPC43_GPIO_BASE+LPC43_GPIO_DIR6_OFFSET)
#define LPC43_GPIO_DIR7             (LPC43_GPIO_BASE+LPC43_GPIO_DIR7_OFFSET)

#define LPC43_GPIO_MASK(p)          (LPC43_GPIO_BASE+LPC43_GPIO_MASK_OFFSET(p))
#define LPC43_GPIO_MASK0            (LPC43_GPIO_BASE+LPC43_GPIO_MASK0_OFFSET)
#define LPC43_GPIO_MASK1            (LPC43_GPIO_BASE+LPC43_GPIO_MASK1_OFFSET)
#define LPC43_GPIO_MASK2            (LPC43_GPIO_BASE+LPC43_GPIO_MASK2_OFFSET)
#define LPC43_GPIO_MASK3            (LPC43_GPIO_BASE+LPC43_GPIO_MASK3_OFFSET)
#define LPC43_GPIO_MASK4            (LPC43_GPIO_BASE+LPC43_GPIO_MASK4_OFFSET)
#define LPC43_GPIO_MASK5            (LPC43_GPIO_BASE+LPC43_GPIO_MASK5_OFFSET)
#define LPC43_GPIO_MASK6            (LPC43_GPIO_BASE+LPC43_GPIO_MASK6_OFFSET)
#define LPC43_GPIO_MASK7            (LPC43_GPIO_BASE+LPC43_GPIO_MASK7_OFFSET)

#define LPC43_GPIO_PIN(p)           (LPC43_GPIO_BASE+LPC43_GPIO_PIN_OFFSET(p))
#define LPC43_GPIO_PIN0             (LPC43_GPIO_BASE+LPC43_GPIO_PIN0_OFFSET)
#define LPC43_GPIO_PIN1             (LPC43_GPIO_BASE+LPC43_GPIO_PIN1_OFFSET)
#define LPC43_GPIO_PIN2             (LPC43_GPIO_BASE+LPC43_GPIO_PIN2_OFFSET)
#define LPC43_GPIO_PIN3             (LPC43_GPIO_BASE+LPC43_GPIO_PIN3_OFFSET)
#define LPC43_GPIO_PIN4             (LPC43_GPIO_BASE+LPC43_GPIO_PIN4_OFFSET)
#define LPC43_GPIO_PIN5             (LPC43_GPIO_BASE+LPC43_GPIO_PIN5_OFFSET)
#define LPC43_GPIO_PIN6             (LPC43_GPIO_BASE+LPC43_GPIO_PIN6_OFFSET)
#define LPC43_GPIO_PIN7             (LPC43_GPIO_BASE+LPC43_GPIO_PIN7_OFFSET)

#define LPC43_GPIO_MPIN(p)          (LPC43_GPIO_BASE+LPC43_GPIO_MPIN_OFFSET(p))
#define LPC43_GPIO_MPIN0            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN0_OFFSET)
#define LPC43_GPIO_MPIN1            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN1_OFFSET)
#define LPC43_GPIO_MPIN2            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN2_OFFSET)
#define LPC43_GPIO_MPIN3            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN3_OFFSET)
#define LPC43_GPIO_MPIN4            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN4_OFFSET)
#define LPC43_GPIO_MPIN5            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN5_OFFSET)
#define LPC43_GPIO_MPIN6            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN6_OFFSET)
#define LPC43_GPIO_MPIN7            (LPC43_GPIO_BASE+LPC43_GPIO_MPIN7_OFFSET)

#define LPC43_GPIO_SET(p)           (LPC43_GPIO_BASE+LPC43_GPIO_SET_OFFSET(p))
#define LPC43_GPIO_SET0             (LPC43_GPIO_BASE+LPC43_GPIO_SET0_OFFSET)
#define LPC43_GPIO_SET1             (LPC43_GPIO_BASE+LPC43_GPIO_SET1_OFFSET)
#define LPC43_GPIO_SET2             (LPC43_GPIO_BASE+LPC43_GPIO_SET2_OFFSET)
#define LPC43_GPIO_SET3             (LPC43_GPIO_BASE+LPC43_GPIO_SET3_OFFSET)
#define LPC43_GPIO_SET4             (LPC43_GPIO_BASE+LPC43_GPIO_SET4_OFFSET)
#define LPC43_GPIO_SET5             (LPC43_GPIO_BASE+LPC43_GPIO_SET5_OFFSET)
#define LPC43_GPIO_SET6             (LPC43_GPIO_BASE+LPC43_GPIO_SET6_OFFSET)
#define LPC43_GPIO_SET7             (LPC43_GPIO_BASE+LPC43_GPIO_SET7_OFFSET)

#define LPC43_GPIO_CLR(p)           (LPC43_GPIO_BASE+LPC43_GPIO_CLR_OFFSET(p))
#define LPC43_GPIO_CLR0             (LPC43_GPIO_BASE+LPC43_GPIO_CLR0_OFFSET)
#define LPC43_GPIO_CLR1             (LPC43_GPIO_BASE+LPC43_GPIO_CLR1_OFFSET)
#define LPC43_GPIO_CLR2             (LPC43_GPIO_BASE+LPC43_GPIO_CLR2_OFFSET)
#define LPC43_GPIO_CLR3             (LPC43_GPIO_BASE+LPC43_GPIO_CLR3_OFFSET)
#define LPC43_GPIO_CLR4             (LPC43_GPIO_BASE+LPC43_GPIO_CLR4_OFFSET)
#define LPC43_GPIO_CLR5             (LPC43_GPIO_BASE+LPC43_GPIO_CLR5_OFFSET)
#define LPC43_GPIO_CLR6             (LPC43_GPIO_BASE+LPC43_GPIO_CLR6_OFFSET)
#define LPC43_GPIO_CLR7             (LPC43_GPIO_BASE+LPC43_GPIO_CLR7_OFFSET)

#define LPC43_GPIO_NOT(p)           (LPC43_GPIO_BASE+LPC43_GPIO_NOT_OFFSET(p))
#define LPC43_GPIO_NOT0             (LPC43_GPIO_BASE+LPC43_GPIO_NOT0_OFFSET)
#define LPC43_GPIO_NOT1             (LPC43_GPIO_BASE+LPC43_GPIO_NOT1_OFFSET)
#define LPC43_GPIO_NOT2             (LPC43_GPIO_BASE+LPC43_GPIO_NOT2_OFFSET)
#define LPC43_GPIO_NOT3             (LPC43_GPIO_BASE+LPC43_GPIO_NOT3_OFFSET)
#define LPC43_GPIO_NOT4             (LPC43_GPIO_BASE+LPC43_GPIO_NOT4_OFFSET)
#define LPC43_GPIO_NOT5             (LPC43_GPIO_BASE+LPC43_GPIO_NOT5_OFFSET)
#define LPC43_GPIO_NOT6             (LPC43_GPIO_BASE+LPC43_GPIO_NOT6_OFFSET)
#define LPC43_GPIO_NOT7             (LPC43_GPIO_BASE+LPC43_GPIO_NOT7_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Pin Interrupt Mode register */

#define GPIOINT_ISEL(i)             (1 << (i)) /* Bits 0-7: Selects the interrupt mode */

/* Pin interrupt level (rising edge) interrupt enable register */

#define GPIOINT_IENR(i)             (1 << (i)) /* Bits 0-7: Enables the rising edge or level interrupt */

/* Pin interrupt level (rising edge) interrupt set register */

#define GPIOINT_SIENR(i)            (1 << (i)) /* Bits 0-7: Set bits in the IENR, enabling interrupts */

/* Pin interrupt level (rising edge interrupt) clear register */

#define GPIOINT_CIENR(i)            (1 << (i)) /* Bits 0-7: Clears bits in the IENR, disabling interrupts */

/* Pin interrupt active level (falling edge) interrupt enable register */

#define GPIOINT_IENF(i)             (1 << (i)) /* Bits 0-7: Enables the falling edge or configures the active level interrupt */

/* Pin interrupt active level (falling edge) interrupt set register */

#define GPIOINT_SIENF(i)            (1 << (i)) /* Bits 0-7: Set bits in the IENF, enabling interrupts */

/* Pin interrupt active level (falling edge) interrupt clear register */

#define GPIOINT_CIENF(i)            (1 << (i)) /* Bits 0-7: Clears bits in the IENF, disabling interrupts */

/* Pin interrupt rising edge register */

#define GPIOINT_RISE(i)             (1 << (i)) /* Bits 0-7: Rising edge detect */

/* Pin interrupt falling edge register */

#define GPIOINT_FALL(i)             (1 << (i)) /* Bits 0-7: Falling edge detect */

/* Pin interrupt status register */

#define GPIOINT_IST(i)              (1 << (i)) /* Bits 0-7: Pin interrupt status */

/* GPIO grouped interrupt control registers */

#define GRPINT_CTRL_INT             (1 << 0)  /* Bit 0:  Group interrupt status */
#define GRPINT_CTRL_COMB            (1 << 1)  /* Bit 1:  Combine enabled inputs for group interrupt */
#define GRPINT_CTRL_TRIG            (1 << 2)  /* Bit 2:  Group interrupt trigger */
                                              /* Bits 3-31:  Reserved */
/* GPIO grouped interrupt polarity registers */

#define GRPINT_POL(p)               (1 << (p)) /* Bits 0-31: Configure polarity of port pins */

/* GPIO grouped interrupt enable registers */

#define GRPINT_ENA(p)               (1 << (p)) /* Bits 0-31: Enable pin for group interrupt */

/* Byte pin registers */

#define GPIO_B                      (1 << 0)  /* Bit 0:  State of GPIO pin */
                                              /* Bits 1-7: Reserved */
/* Byte word registers.  On Read:  0x00000000 or 0xffffffff.  On write 0x0000000 or any
 * non-zero value
 */

/* Direction registers */

#define GPIO_DIR(p)                 (1 << (p)) /* Bits 0-31: Selects pin direction for pin */

/* Mask registers */

#define GPIO_MASK(p)                (1 << (p)) /* Bits 0-31: Controls which bits are active */

/* Port pin registers */

#define GPIO_PIN(p)                 (1 << (p)) /* Bits 0-31: Read/write pin state */

/* Masked port registers */

#define GPIO_MPIN(p)                (1 << (p)) /* Bits 0-31: Read/write masked pin state */

/* Write: Set registers */

#define GPIO_SET(p)                 (1 << (p)) /* Bits 0-31: Read or set output bits */

/* Write: Clear registers */

#define GPIO_CLR(p)                 (1 << (p)) /* Bits 0-31: Clear output bits */

/* Toggle registers */

#define GPIO_NOT(p)                 (1 << (p)) /* Bits 0-31: Toggle output bits */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GPIO_H */
