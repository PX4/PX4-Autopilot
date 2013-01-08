/************************************************************************************
 * arch/arm/src/lm3s/chip/lm3s_syscontrol.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM3S_CHIP_LM3S_SYSCONTROL_H
#define __ARCH_ARM_SRC_LM3S_CHIP_LM3S_SYSCONTROL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* System Control Register Offsets **************************************************/

#define LM3S_SYSCON_DID0_OFFSET       0x000 /* Device Identification 0 */
#define LM3S_SYSCON_DID1_OFFSET       0x004 /* Device Identification 1 */
#define LM3S_SYSCON_DC0_OFFSET        0x008 /* Device Capabilities 0 */
#define LM3S_SYSCON_DC1_OFFSET        0x010 /* Device Capabilities 1 */
#define LM3S_SYSCON_DC2_OFFSET        0x014 /* Device Capabilities 2 */
#define LM3S_SYSCON_DC3_OFFSET        0x018 /* Device Capabilities 3 */
#define LM3S_SYSCON_DC4_OFFSET        0x01c /* Device Capabilities 4 */
#define LM3S_SYSCON_PBORCTL_OFFSET    0x030 /* Brown-Out Reset Control */
#define LM3S_SYSCON_LDOPCTL_OFFSET    0x034 /* LDO Power Control */
#define LM3S_SYSCON_SRCR0_OFFSET      0x040 /* Software Reset Control 0 */
#define LM3S_SYSCON_SRCR1_OFFSET      0x044 /* Software Reset Control 1 */
#define LM3S_SYSCON_SRCR2_OFFSET      0x048 /* Software Reset Control 2*/
#define LM3S_SYSCON_RIS_OFFSET        0x050 /* Raw Interrupt Status */
#define LM3S_SYSCON_IMC_OFFSET        0x054 /* Interrupt Mask Control */
#define LM3S_SYSCON_MISC_OFFSET       0x058 /* Masked Interrupt Status and Clear */
#define LM3S_SYSCON_RESC_OFFSET       0x05c /* Reset Cause */
#define LM3S_SYSCON_RCC_OFFSET        0x060 /* Run-Mode Clock Configuration */
#define LM3S_SYSCON_PLLCFG_OFFSET     0x064 /* XTAL to PLL Translation */
#define LM3S_SYSCON_RCC2_OFFSET       0x070 /* Run-Mode Clock Configuration 2 */
#define LM3S_SYSCON_RCGC0_OFFSET      0x100 /* Run Mode Clock Gating Control Register 0 */
#define LM3S_SYSCON_RCGC1_OFFSET      0x104 /* Run Mode Clock Gating Control Register 1 */
#define LM3S_SYSCON_RCGC2_OFFSET      0x108 /* Run Mode Clock Gating Control Register 2 */
#define LM3S_SYSCON_SCGC0_OFFSET      0x110 /* Sleep Mode Clock Gating Control Register 0 */
#define LM3S_SYSCON_SCGC1_OFFSET      0x114 /* Sleep Mode Clock Gating Control Register 1 */
#define LM3S_SYSCON_SCGC2_OFFSET      0x118 /* Sleep Mode Clock Gating Control Register 2 */
#define LM3S_SYSCON_DCGC0_OFFSET      0x120 /* Deep Sleep Mode Clock Gating Control Register 0 */
#define LM3S_SYSCON_DCGC1_OFFSET      0x124 /* Deep Sleep Mode Clock Gating Control Register 1 */
#define LM3S_SYSCON_DCGC2_OFFSET      0x128 /* Deep Sleep Mode Clock Gating Control Register 2 */
#define LM3S_SYSCON_DSLPCLKCFG_OFFSET 0x144 /* Deep Sleep Clock Configuration*/

/* System Control Register Addresses ************************************************/

#define LM3S_SYSCON_DID0              (LM3S_SYSCON_BASE + LM3S_SYSCON_DID0_OFFSET)
#define LM3S_SYSCON_DID1              (LM3S_SYSCON_BASE + LM3S_SYSCON_DID1_OFFSET)
#define LM3S_SYSCON_DC0               (LM3S_SYSCON_BASE + LM3S_SYSCON_DC0_OFFSET)
#define LM3S_SYSCON_DC1               (LM3S_SYSCON_BASE + LM3S_SYSCON_DC1_OFFSET)
#define LM3S_SYSCON_DC2               (LM3S_SYSCON_BASE + LM3S_SYSCON_DC2_OFFSET)
#define LM3S_SYSCON_DC3               (LM3S_SYSCON_BASE + LM3S_SYSCON_DC3_OFFSET)
#define LM3S_SYSCON_DC4               (LM3S_SYSCON_BASE + LM3S_SYSCON_DC4_OFFSET)
#define LM3S_SYSCON_PBORCTL           (LM3S_SYSCON_BASE + LM3S_SYSCON_PBORCTL_OFFSET)
#define LM3S_SYSCON_LDOPCTL           (LM3S_SYSCON_BASE + LM3S_SYSCON_LDOPCTL_OFFSET)
#define LM3S_SYSCON_SRCR0             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR0_OFFSET)
#define LM3S_SYSCON_SRCR1             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR1_OFFSET)
#define LM3S_SYSCON_SRCR2             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR2_OFFSET)
#define LM3S_SYSCON_RIS               (LM3S_SYSCON_BASE + LM3S_SYSCON_RIS_OFFSET)
#define LM3S_SYSCON_IMC               (LM3S_SYSCON_BASE + LM3S_SYSCON_IMC_OFFSET)
#define LM3S_SYSCON_MISC              (LM3S_SYSCON_BASE + LM3S_SYSCON_MISC_OFFSET)
#define LM3S_SYSCON_RESC              (LM3S_SYSCON_BASE + LM3S_SYSCON_RESC_OFFSET)
#define LM3S_SYSCON_RCC               (LM3S_SYSCON_BASE + LM3S_SYSCON_RCC_OFFSET)
#define LM3S_SYSCON_PLLCFG            (LM3S_SYSCON_BASE + LM3S_SYSCON_PLLCFG_OFFSET)
#define LM3S_SYSCON_RCC2              (LM3S_SYSCON_BASE + LM3S_SYSCON_RCC2_OFFSET)
#define LM3S_SYSCON_RCGC0             (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGC0_OFFSET)
#define LM3S_SYSCON_RCGC1             (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGC1_OFFSET)
#define LM3S_SYSCON_RCGC2             (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGC2_OFFSET)
#define LM3S_SYSCON_SCGC0             (LM3S_SYSCON_BASE + LM3S_SYSCON_SCGC0_OFFSET)
#define LM3S_SYSCON_SCGC1             (LM3S_SYSCON_BASE + LM3S_SYSCON_SCGC1_OFFSET)
#define LM3S_SYSCON_SCGC2             (LM3S_SYSCON_BASE + LM3S_SYSCON_SCGC2_OFFSET)
#define LM3S_SYSCON_DCGC0             (LM3S_SYSCON_BASE + LM3S_SYSCON_DCGC0_OFFSET)
#define LM3S_SYSCON_DCGC1             (LM3S_SYSCON_BASE + LM3S_SYSCON_DCGC1_OFFSET)
#define LM3S_SYSCON_DCGC2             (LM3S_SYSCON_BASE + LM3S_SYSCON_DCGC2_OFFSET)
#define LM3S_SYSCON_DSLPCLKCFG        (LM3S_SYSCON_BASE + LM3S_SYSCON_DSLPCLKCFG_OFFSET)

/* System Control Register Bit Definitions ******************************************/

/* Device Identification 0 (DID0), offset 0x000 */

#define SYSCON_DID0_MINOR_SHIFT       0         /* Bits 7-0: Minor Revision of the device */
#define SYSCON_DID0_MINOR_MASK        (0xff << SYSCON_DID0_MINOR_SHIFT)
#define SYSCON_DID0_MAJOR_SHIFT       8         /* Bits 15-8: Major Revision of the device */
#define SYSCON_DID0_MAJOR_MASK        (0xff << SYSCON_DID0_MAJOR_SHIFT)
#define SYSCON_DID0_CLASS_SHIFT       16        /* Bits 23-16: Device Class */
#define SYSCON_DID0_CLASS_MASK        (0xff << SYSCON_DID0_CLASS_SHIFT)
#define SYSCON_DID0_VER_SHIFT         28        /* Bits 30-28: DID0 Version */
#define SYSCON_DID0_VER_MASK          (7 << SYSCON_DID0_VER_SHIFT)

/* Device Identification 1 (DID1), offset 0x004 */

#define SYSCON_DID1_QUAL_SHIFT        0         /* Bits 1-0: Qualification Status */
#define SYSCON_DID1_QUAL_MASK         (0x03 << SYSCON_DID1_QUAL_SHIFT)
#define SYSCON_DID1_ROHS              (1 << 2)  /* Bit 2: RoHS-Compliance */
#define SYSCON_DID1_PKG_SHIFT         3 /* Bits 4-3: Package Type */
#define SYSCON_DID1_PKG_MASK          (0x03 << SYSCON_DID1_PKG_SHIFT)
#define SYSCON_DID1_TEMP_SHIFT        5         /* Bits 7-5: Temperature Range */
#define SYSCON_DID1_TEMP_MASK         (0x07 << SYSCON_DID1_TEMP_SHIFT)
#define SYSCON_DID1_PINCOUNT_SHIFT    13        /* Bits 15-13: Package Pin Count */
#define SYSCON_DID1_PINCOUNT_MASK     (0x07 << SYSCON_DID1_PINCOUNT_SHIFT)
#define SYSCON_DID1_PARTNO_SHIFT      16        /* Bits 23-16: Part Number */
#define SYSCON_DID1_PARTNO_MASK       (0xff << SYSCON_DID1_PARTNO_SHIFT)
#define SYSCON_DID1_FAM_SHIFT         24        /* Bits 27-24: Family */
#define SYSCON_DID1_FAM_MASK          (0x0f << SYSCON_DID1_FAM_SHIFT)
#define SYSCON_DID1_VER_SHIFT         28        /* Bits 31-28:  DID1 Version */
#define SYSCON_DID1_VER_MASK          (0x0f << SYSCON_DID1_VER_SHIFT)

/* Device Capabilities 0 (DC0), offset 0x008 */

#define SYSCON_DC0_FLASHSZ_SHIFT      0         /* Bits 15-0: FLASH Size */
#define SYSCON_DC0_FLASHSZ_MASK       (0xffff << SYSCON_DC0_FLASHSZ_SHIFT)
#define SYSCON_DC0_SRAMSZ_SHIFT       16        /* Bits 31-16: SRAM Size */
#define SYSCON_DC0_SRAMSZ_MASK        (0xffff << SYSCON_DC0_SRAMSZ_SHIFT)

/* Device Capabilities 1 (DC1), offset 0x010 */

#define SYSCON_DC1_JTAG               (1 << 0)  /* Bit 0: JTAG Present */
#define SYSCON_DC1_SWD                (1 << 1)  /* Bit 1: SWD Present */
#define SYSCON_DC1_SWO                (1 << 2)  /* Bit 2: SWO Trace Port Present */
#define SYSCON_DC1_WDT                (1 << 3)  /* Bit 3: Watchdog Timer Present */
#define SYSCON_DC1_PLL                (1 << 4)  /* Bit 4: PLL Present */
#define SYSCON_DC1_TEMPSNS            (1 << 5)  /* Bit 5: Temp Sensor Present */
#define SYSCON_DC1_HIB                (1 << 6)  /* Bit 6: Hibernation Module Present */
#define SYSCON_DC1_MPU                (1 << 7)  /* Bit 7: MPU Present */
#define SYSCON_DC1_MAXADCSPD_SHIFT    8         /* Bits 9-8: Max ADC Speed */
#define SYSCON_DC1_MAXADCSPD_MASK     (0x03 << SYSCON_DC1_MAXADCSPD_SHIFT)
#define SYSCON_DC1_ADC                (1 << 16) /* Bit 16: ADC Module Present */
#define SYSCON_DC1_MINSYSDIV_SHIFT    12        /* Bits 15-12: System Clock Divider Minimum */
#define SYSCON_DC1_MINSYSDIV_MASK     (0x0f << SYSCON_DC1_MINSYSDIV_SHIFT)

/* Device Capabilities 2 (DC2), offset 0x014 */

#define SYSCON_DC2_UART0              (1 << 0)  /* Bit 0: UART0 Present */
#define SYSCON_DC2_UART1              (1 << 1)  /* Bit 1: UART1 Present */
#define SYSCON_DC2_SSI0               (1 << 4)  /* Bit 4: SSI0 Present */
#define SYSCON_DC2_SSI1               (1 << 5)  /* Bit 5: SSI1 Present */
#define SYSCON_DC2_I2C0               (1 << 12) /* Bit 12: I2C Module 0 Present */
#define SYSCON_DC2_I2C1               (1 << 14) /* Bit 14: I2C Module 1 Present */
#define SYSCON_DC2_TIMER0             (1 << 16) /* Bit 16: Timer 0 Present */
#define SYSCON_DC2_TIMER1             (1 << 17) /* Bit 17: Timer 1 Present */
#define SYSCON_DC2_TIMER2             (1 << 18) /* Bit 18: Timer 2 Present */
#define SYSCON_DC2_TIMER3             (1 << 19) /* Bit 19: Timer 3 Present */
#define SYSCON_DC2_COMP0              (1 << 24) /* Bit 24: Analog Comparator 0 Present */
#define SYSCON_DC2_COMP1              (1 << 25) /* Bit 25: Analog Comparator 1 Present */

/* Device Capabilities 3 (DC3), offset 0x018 */

#define SYSCON_DC3_C0MINUS            (1 << 6)  /* Bit 6: C0- Pin Present */
#define SYSCON_DC3_C0PLUS             (1 << 7)  /* Bit 7: C0+ Pin Present */
#define SYSCON_DC3_C0O                (1 << 8)  /* Bit 8: C0o Pin Present */
#define SYSCON_DC3_C1MINUS            (1 << 9)  /* Bit 9: C1- Pin Present */
#define SYSCON_DC3_C1PLUS             (1 << 10) /* Bit 10: C1+ Pin Present */
#define SYSCON_DC3_ADC0               (1 << 16) /* Bit 16: ADC0 Pin Present */
#define SYSCON_DC3_ADC1               (1 << 17) /* Bit 17: ADC1 Pin Present */
#define SYSCON_DC3_ADC2               (1 << 18) /* Bit 18: ADC2 Pin Present */
#define SYSCON_DC3_ADC3               (1 << 19) /* Bit 19: ADC3 Pin Present */
#define SYSCON_DC3_ADC4               (1 << 20) /* Bit 20: ADC4 Pin Present */
#define SYSCON_DC3_ADC5               (1 << 21) /* Bit 21: ADC5 Pin Present */
#define SYSCON_DC3_ADC6               (1 << 22) /* Bit 22: ADC6 Pin Present */
#define SYSCON_DC3_ADC7               (1 << 23) /* Bit 23: ADC7 Pin Present */
#define SYSCON_DC3_CCP0               (1 << 24) /* Bit 24: CCP0 Pin Present */
#define SYSCON_DC3_CCP1               (1 << 25) /* Bit 25: CCP1 Pin Present */
#define SYSCON_DC3_CCP2               (1 << 26) /* Bit 26: CCP2 Pin Present */
#define SYSCON_DC3_CCP3               (1 << 27) /* Bit 27: CCP3 Pin Present */
#define SYSCON_DC3_CCP4               (1 << 28) /* Bit 28: CCP4 Pin Present */
#define SYSCON_DC3_CCP5               (1 << 29) /* Bit 29: CCP5 Pin Present */
#define SYSCON_DC3_32KHZ              (1 << 31) /* Bit 31: 32KHz Input Clock Available */

/* Device Capabilities 4 (DC4), offset 0x01c */

#define SYSCON_DC4_GPIO(n)            (1 << (n))
#define SYSCON_DC4_GPIOA              (1 << 0)  /* Bit 0: GPIO Port A Present */
#define SYSCON_DC4_GPIOB              (1 << 1)  /* Bit 1: GPIO Port B Present */
#define SYSCON_DC4_GPIOC              (1 << 2)  /* Bit 2: GPIO Port C Present */
#define SYSCON_DC4_GPIOD              (1 << 3)  /* Bit 3: GPIO Port D Present */
#define SYSCON_DC4_GPIOE              (1 << 4)  /* Bit 4: GPIO Port E Present */
#define SYSCON_DC4_GPIOF              (1 << 5)  /* Bit 5: GPIO Port F Present */
#define SYSCON_DC4_GPIOG              (1 << 6)  /* Bit 6: GPIO Port G Present */
#define SYSCON_DC4_GPIOH              (1 << 7)  /* Bit 7: GPIO Port H Present */
#define SYSCON_DC4_EMAC0              (1 << 28) /* Bit 28: Ethernet MAC0 Present */
#define SYSCON_DC4_EPHY0              (1 << 30) /* Bit 30: Ethernet PHY0 Present */

/* Brown-Out Reset Control (PBORCTL), offset 0x030 */

#define SYSCON_PBORCTL_BORIOR         (1 << 1)  /* Bit 1: BOR Interrupt or Reset */

/* LDO Power Control (LDOPCTL), offset 0x034 */

#define SYSCON_LDOPCTL_VADJ_SHIFT     0         /* Bits 5-0: LDO Output Voltage */
#define SYSCON_LDOPCTL_VADJ_MASK      (0x3f << SYSCON_LDOPCTL_VADJ_SHIFT)
#  define SYSCON_LPDOPCTL_2500MV      (0x00 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.5V (reset)*/
#  define SYSCON_LPDOPCTL_2450MV      (0x01 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.45V */
#  define SYSCON_LPDOPCTL_2400MV      (0x02 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.4V */
#  define SYSCON_LPDOPCTL_2350MV      (0x03 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.35V */
#  define SYSCON_LPDOPCTL_2300MV      (0x04 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.3V */
#  define SYSCON_LPDOPCTL_2250MV      (0x05 << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.25V */
#  define SYSCON_LPDOPCTL_2750MV      (0x1b << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.75V */
#  define SYSCON_LPDOPCTL_2700MV      (0x1c << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.7V */
#  define SYSCON_LPDOPCTL_2650MV      (0x1d << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.65V */
#  define SYSCON_LPDOPCTL_2600MV      (0x1e << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.6V */
#  define SYSCON_LPDOPCTL_2550MV      (0x1f << SYSCON_LDOPCTL_VADJ_SHIFT) /* 2.55V */

/* Software Reset Control 0 (SRCR0), offset 0x040 */

#define SYSCON_SRCR0_WDT              (1 << 3)  /* Bit 3: WDT Reset Control */
#define SYSCON_SRCR0_HIB              (1 << 6)  /* Bit 6: HIB Reset Control */
#define SYSCON_SRCR0_ADC              (1 << 16) /* Bit 16: ADC0 Reset Control */

/* Software Reset Control 1 (SRCR1), offset 0x044 */

#define SYSCON_SRCR1_UART0            (1 << 0)  /* Bit 0: UART0 Reset Control */
#define SYSCON_SRCR1_UART1            (1 << 1)  /* Bit 1: UART1 Reset Control */
#define SYSCON_SRCR1_SSI0             (1 << 4)  /* Bit 4: SSI0 Reset Control1 */
#define SYSCON_SRCR1_SSI1             (1 << 5)  /* Bit 5: SSI1 Reset Control */
#define SYSCON_SRCR1_I2C0             (1 << 12) /* Bit 12: I2C0 Reset Control */
#define SYSCON_SRCR1_I2C1             (1 << 14) /* Bit 14: I2C1 Reset Control */
#define SYSCON_SRCR1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Reset Control */
#define SYSCON_SRCR1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Reset Control */
#define SYSCON_SRCR1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Reset Control */
#define SYSCON_SRCR1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Reset Control */
#define SYSCON_SRCR1_COMP0            (1 << 24) /* Bit 24: Analog Comp 0 Reset Control */
#define SYSCON_SRCR1_COMP1            (1 << 25) /* Bit 25: Analog Comp 1 Reset Control */

/* Software Reset Control 2 (SRCR2), offset 0x048 */

#define SYSCON_SRCR2_GPIO(n)          (1 << (n))
#define SYSCON_SRCR2_GPIOA            (1 << 0)  /* Bit 0: Port A Reset Control */
#define SYSCON_SRCR2_GPIOB            (1 << 1)  /* Bit 1: Port B Reset Control */
#define SYSCON_SRCR2_GPIOC            (1 << 2)  /* Bit 2: Port C Reset Control */
#define SYSCON_SRCR2_GPIOD            (1 << 3)  /* Bit 3: Port D Reset Control */
#define SYSCON_SRCR2_GPIOE            (1 << 4)  /* Bit 4: Port E Reset Control */
#define SYSCON_SRCR2_GPIOF            (1 << 5)  /* Bit 5: Port F Reset Control */
#define SYSCON_SRCR2_GPIOG            (1 << 6)  /* Bit 6: Port G Reset Control */
#define SYSCON_SRCR2_GPIOH            (1 << 7)  /* Bit 7: Port H Reset Control */
#define SYSCON_SRCR2_EMAC0            (1 << 28) /* Bit 28: MAC0 Reset Control */
#define SYSCON_SRCR2_EPHY0            (1 << 30) /* Bit 30: PHY0 Reset Control */

/* Raw Interrupt Status (RIS), offset 0x050 */

#define SYSCON_RIS_BORRIS             (1 << 1)  /* Bit 1: Brown-Out Reset Raw Interrupt Status */
#define SYSCON_RIS_PLLLRIS            (1 << 6)  /* Bit 6: PLL Lock Raw Interrupt Status */

/* Interrupt Mask Control (IMC), offset 0x054 */

#define SYSCON_IMC_BORIM              (1 << 1)  /* Bit 1: Brown-Out Reset Interrupt Mask */
#define SYSCON_IMC_PLLLIM             (1 << 6)  /* Bit 6: PLL Lock Interrupt Mask */

/* Masked Interrupt Status and Clear (MISC), offset 0x058 */

#define SYSCON_MISC_BORMIS            (1 << 1)  /* Bit 1: BOR Masked Interrupt Status */
#define SYSCON_MISC_PLLLMIS           (1 << 6)  /* Bit 6: PLL Lock Masked Interrupt Status */

/* Reset Cause (RESC), offset 0x05C */

#define SYSCON_RESC_EXT               (1 << 0)  /* Bit 0: External Reset */
#define SYSCON_RESC_POR               (1 << 1)  /* Bit 1: Power-On Reset */
#define SYSCON_RESC_BOR               (1 << 2)  /* Bit 2: Brown-Out Reset */
#define SYSCON_RESC_WDT               (1 << 3)  /* Bit 3: Watchdog Timer Reset */
#define SYSCON_RESC_SW                (1 << 4)  /* Bit 4: Software Reset */

/* Run-Mode Clock Configuration (RCC), offset 0x060 */

#define SYSCON_RCC_MOSCDIS            (1 << 0)  /* Bit 0: Main Oscillator Disable */
#define SYSCON_RCC_IOSCDIS            (1 << 1)  /* Bit 1: Internal Oscillator Disable */
#define SYSCON_RCC_OSCSRC_SHIFT       4         /* Bits 5-4: Oscillator Source */
#define SYSCON_RCC_OSCSRC_MASK        (0x03 << SYSCON_RCC_OSCSRC_SHIFT)
#  define SYSCON_RCC_OSCSRC_MOSC      (0 << SYSCON_RCC_OSCSRC_SHIFT) /* Main oscillator */
#  define SYSCON_RCC_OSCSRC_IOSC      (1 << SYSCON_RCC_OSCSRC_SHIFT) /* Internal oscillator (reset) */
#  define SYSCON_RCC_OSCSRC_IOSC4     (2 << SYSCON_RCC_OSCSRC_SHIFT) /* Internal oscillator / 4 */
#  define SYSCON_RCC_OSCSRC_30KHZ     (3 << SYSCON_RCC_OSCSRC_SHIFT) /* 30KHz internal oscillator */
#define SYSCON_RCC_XTAL_SHIFT         6         /* Bits 10-6: Crystal Value */
#define SYSCON_RCC_XTAL_MASK          (0x1f << SYSCON_RCC_XTAL_SHIFT)
#  define SYSCON_RCC_XTAL1000KHZ      ( 0 << SYSCON_RCC_XTAL_SHIFT)  /* 1.0000MHz (NO PLL) */
#  define SYSCON_RCC_XTAL1843KHZ      ( 1 << SYSCON_RCC_XTAL_SHIFT)  /* 1.8432MHz (NO PLL) */
#  define SYSCON_RCC_XTAL2000KHZ      ( 2 << SYSCON_RCC_XTAL_SHIFT)  /* 2.0000MHz (NO PLL) */
#  define SYSCON_RCC_XTAL2580KHZ      ( 3 << SYSCON_RCC_XTAL_SHIFT)  /* 2.4576MHz (NO PLL) */
#  define SYSCON_RCC_XTAL3580KHZ      ( 4 << SYSCON_RCC_XTAL_SHIFT)  /* 3.5795MHz */
#  define SYSCON_RCC_XTAL3686KHZ      ( 5 << SYSCON_RCC_XTAL_SHIFT)  /* 3.6864MHz */
#  define SYSCON_RCC_XTAL4000KHZ      ( 6 << SYSCON_RCC_XTAL_SHIFT)  /* 4.0000MHz */
#  define SYSCON_RCC_XTAL4096KHZ      ( 7 << SYSCON_RCC_XTAL_SHIFT)  /* 4.0960MHz */
#  define SYSCON_RCC_XTAL4915KHZ      ( 8 << SYSCON_RCC_XTAL_SHIFT)  /* 4.9152MHz */
#  define SYSCON_RCC_XTAL5000KHZ      ( 9 << SYSCON_RCC_XTAL_SHIFT)  /* 5.0000MHz */
#  define SYSCON_RCC_XTAL5120KHZ      (10 << SYSCON_RCC_XTAL_SHIFT)  /* 5.1200MHz */
#  define SYSCON_RCC_XTAL6000KHZ      (11 << SYSCON_RCC_XTAL_SHIFT)  /* 6.0000MHz (reset value) */
#  define SYSCON_RCC_XTAL6144KHZ      (12 << SYSCON_RCC_XTAL_SHIFT)  /* 6.1440MHz */
#  define SYSCON_RCC_XTAL7373KHZ      (13 << SYSCON_RCC_XTAL_SHIFT)  /* 7.3728MHz */
#  define SYSCON_RCC_XTAL8000KHZ      (14 << SYSCON_RCC_XTAL_SHIFT)  /* 8.0000MHz */
#  define SYSCON_RCC_XTAL8192KHZ      (15 << SYSCON_RCC_XTAL_SHIFT)  /* 8.1920MHz */
#ifdef CONFIG_ARCH_CHIP_LM3S9B96
#  define SYSCON_RCC_XTAL10000KHZ     (16 << SYSCON_RCC_XTAL_SHIFT)  /* 10.0 MHz (USB) */
#  define SYSCON_RCC_XTAL12000KHZ     (17 << SYSCON_RCC_XTAL_SHIFT)  /* 12.0 MHz (USB) */
#  define SYSCON_RCC_XTAL12888KHZ     (18 << SYSCON_RCC_XTAL_SHIFT)  /* 12.288 MHz */
#  define SYSCON_RCC_XTAL13560KHZ     (19 << SYSCON_RCC_XTAL_SHIFT)  /* 13.56 MHz */
#  define SYSCON_RCC_XTAL14318KHZ     (20 << SYSCON_RCC_XTAL_SHIFT)  /* 14.31818 MHz */
#  define SYSCON_RCC_XTAL16000KHZ     (21 << SYSCON_RCC_XTAL_SHIFT)  /* 16.0 MHz (USB) */
#  define SYSCON_RCC_XTAL16384KHZ     (22 << SYSCON_RCC_XTAL_SHIFT)  /* 16.384 MHz */
#endif
#define SYSCON_RCC_BYPASS             (1 << 11) /* Bit 11: PLL Bypass */
#define SYSCON_RCC_PWRDN              (1 << 13) /* Bit 13: PLL Power Down */
#define SYSCON_RCC_USESYSDIV          (1 << 22) /* Bit 22: Enable System Clock Divider */
#define SYSCON_RCC_SYSDIV_SHIFT       23        /* Bits 26-23: System Clock Divisor */
#define SYSCON_RCC_SYSDIV_MASK        (0x0f << SYSCON_RCC_SYSDIV_SHIFT)
#  define SYSCON_RCC_SYSDIV(n)        (((n)-1) << SYSCON_RCC_SYSDIV_SHIFT)
#define SYSCON_RCC_ACG                (1 << 27) /* Bit 27: Auto Clock Gating */

/* XTAL to PLL Translation (PLLCFG), offset 0x064 */

#define SYSCON_PLLCFG_F_SHIFT         5         /* Bits 13-5: PLL F Value */
#define SYSCON_PLLCFG_F_MASK          (0x1ff << SYSCON_PLLCFG_F_SHIFT)
#define SYSCON_PLLCFG_R_SHIFT         0         /* Bits 4-0: PLL R Value  */
#define SYSCON_PLLCFG_R_MASK          (0x1f << SYSCON_PLLCFG_R_SHIFT)

/* Run-Mode Clock Configuration 2 (RCC2), offset 0x070 */

#define SYSCON_RCC2_OSCSRC2_SHIFT     4         /* Bits 6-4: Oscillator Source */
#define SYSCON_RCC2_OSCSRC2_MASK      (0x07 << SYSCON_RCC2_OSCSRC2_SHIFT)
#  define SYSCON_RCC2_OSCSRC2_MOSC    (0 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Main oscillator */
#  define SYSCON_RCC2_OSCSRC2_IOSC    (1 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Internal oscillator (reset) */
#  define SYSCON_RCC2_OSCSRC2_IOSC4   (2 << SYSCON_RCC2_OSCSRC2_SHIFT) /* Internal oscillator / 4 */
#  define SYSCON_RCC2_OSCSRC2_30KHZ   (3 << SYSCON_RCC2_OSCSRC2_SHIFT) /* 30KHz internal oscillator */
#  define SYSCON_RCC2_OSCSRC2_32KHZ   (7 << SYSCON_RCC2_OSCSRC2_SHIFT) /* 32.768KHz external oscillator */
#define SYSCON_RCC2_BYPASS2           (1 << 11) /* Bit 11: Bypass PLL */
#define SYSCON_RCC2_PWRDN2            (1 << 13) /* Bit 13: Power-Down PLL */
#define SYSCON_RCC2_SYSDIV2_SHIFT     23        /* Bits 28-23: System Clock Divisor */
#define SYSCON_RCC2_SYSDIV2_MASK      (0x3f << SYSCON_RCC2_SYSDIV2_SHIFT)
#  define SYSCON_RCC2_SYSDIV(n)       ((n-1) << SYSCON_RCC2_SYSDIV2_SHIFT)
#define SYSCON_RCC2_USERCC2           (1 << 31) /* Bit 31: Use RCC2 When set */

/* Run Mode Clock Gating Control Register 0 (RCGC0), offset 0x100 */

#define SYSCON_RCGC0_WDT              (1 << 3)  /* Bit 3: WDT Clock Gating Control */
#define SYSCON_RCGC0_HIB              (1 << 6)  /* Bit 6: HIB Clock Gating Control */
#define SYSCON_RCGC0_MAXADCSPD_SHIFT  8         /* Bits 9-8: ADC Sample Speed */
#define SYSCON_RCGC0_MAXADCSPD_MASK   (0x03 << SYSCON_RCGC0_MAXADCSPD_SHIFT)
#define SYSCON_RCGC0_ADC              (1 << 16) /* Bit 16: ADC0 Clock Gating Control */

/* Run Mode Clock Gating Control Register 1 (RCGC1), offset 0x104 */

#define SYSCON_RCGC1_UART0            (1 << 0)  /* Bit 0: UART0 Clock Gating Control */
#define SYSCON_RCGC1_UART1            (1 << 1)  /* Bit 1: UART1 Clock Gating Control */
#define SYSCON_RCGC1_SSI0             (1 << 4)  /* Bit 4: SSI0 Clock Gating Control */
#define SYSCON_RCGC1_SSI1             (1 << 5)  /* Bit 5: SSI1 Clock Gating Control */
#define SYSCON_RCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_RCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_RCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_RCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_RCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_RCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_RCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_RCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */

/* Run Mode Clock Gating Control Register 2 (RCGC2), offset 0x108 */

#define SYSCON_RCGC2_GPIO(n)          (1 << (n))
#define SYSCON_RCGC2_GPIOA            (1 << 0)  /* Bit 0: Port A Clock Gating Control */
#define SYSCON_RCGC2_GPIOB            (1 << 1)  /* Bit 1: Port B Clock Gating Control */
#define SYSCON_RCGC2_GPIOC            (1 << 2)  /* Bit 2: Port C Clock Gating Control */
#define SYSCON_RCGC2_GPIOD            (1 << 3)  /* Bit 3: Port D Clock Gating Control */
#define SYSCON_RCGC2_GPIOE            (1 << 4)  /* Bit 4: Port E Clock Gating Control */
#define SYSCON_RCGC2_GPIOF            (1 << 5)  /* Bit 5: Port F Clock Gating Control */
#define SYSCON_RCGC2_GPIOG            (1 << 6)  /* Bit 6: Port G Clock Gating Control */
#define SYSCON_RCGC2_GPIOH            (1 << 7)  /* Bit 7: Port H Clock Gating Control */
#define SYSCON_RCGC2_EMAC0            (1 << 28) /* Bit 28: MAC0 Clock Gating Control */
#define SYSCON_RCGC2_EPHY0            (1 << 30) /* Bit 30: PHY0 Clock Gating Control */

/* Sleep Mode Clock Gating Control Register 0 (SCGC0), offset 0x110 */

#define SYSCON_SCGC0_WDT              (1 << 3)  /* Bit 3: WDT Clock Gating Control */
#define SYSCON_SCGC0_HIB              (1 << 6)  /* Bit 6: HIB Clock Gating Control */
#define SYSCON_SCGC0_MAXADCSPD_SHIFT  8         /* Bits 9-8: ADC Sample Speed */
#define SYSCON_SCGC0_MAXADCSPD_MASK   (0x03 << SYSCON_SCGC0_MAXADCSPD_SHIFT)
#define SYSCON_SCGC0_ADC              (1 << 16) /* Bit 16: ADC0 Clock Gating Control */

/* Sleep Mode Clock Gating Control Register 1 (SCGC1), offset 0x114 */

#define SYSCON_SCGC1_UART0            (1 << 0)  /* Bit 0: UART0 Clock Gating Control */
#define SYSCON_SCGC1_UART1            (1 << 1)  /* Bit 1: UART1 Clock Gating Control */
#define SYSCON_SCGC1_SSI0             (1 << 4)  /* Bit 4: SSI0 Clock Gating Control */
#define SYSCON_SCGC1_SSI1             (1 << 5)  /* Bit 5: SSI1 Clock Gating Control */
#define SYSCON_SCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_SCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_SCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_SCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_SCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_SCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_SCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_SCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */

/* Sleep Mode Clock Gating Control Register 2 (SCGC2), offset 0x118 */

#define SYSCON_SCGC2_GPIO(n)          (1 << (n))
#define SYSCON_SCGC2_GPIOA            (1 << 0)  /* Bit 0: Port A Clock Gating Control */
#define SYSCON_SCGC2_GPIOB            (1 << 1)  /* Bit 1: Port B Clock Gating Control */
#define SYSCON_SCGC2_GPIOC            (1 << 2)  /* Bit 2: Port C Clock Gating Control */
#define SYSCON_SCGC2_GPIOD            (1 << 3)  /* Bit 3: Port D Clock Gating Control */
#define SYSCON_SCGC2_GPIOE            (1 << 4)  /* Bit 4: Port E Clock Gating Control */
#define SYSCON_SCGC2_GPIOF            (1 << 5)  /* Bit 5: Port F Clock Gating Control */
#define SYSCON_SCGC2_GPIOG            (1 << 6)  /* Bit 6: Port G Clock Gating Control */
#define SYSCON_SCGC2_GPIOH            (1 << 7)  /* Bit 7: Port H Clock Gating Control */
#define SYSCON_SCGC2_EMAC0            (1 << 28) /* Bit 28: MAC0 Clock Gating Control */
#define SYSCON_SCGC2_EPHY0            (1 << 30) /* Bit 30: PHY0 Clock Gating Control */

/* Deep Sleep Mode Clock Gating Control Register 0 (DCGC0), offset 0x120 */

#define SYSCON_DCGC0_WDT              (1 << 3)  /* Bit 3: WDT Clock Gating Control */
#define SYSCON_DCGC0_HIB              (1 << 6)  /* Bit 6: HIB Clock Gating Control */
#define SYSCON_DCGC0_MAXADCSPD_SHIFT  8         /* Bits 9-8: ADC Sample Speed */
#define SYSCON_DCGC0_MAXADCSPD_MASK   (0x03 << SYSCON_DCGC0_MAXADCSPD_SHIFT)
#define SYSCON_DCGC0_ADC              (1 << 16) /* Bit 16: ADC0 Clock Gating Control */

/* Deep Sleep Mode Clock Gating Control Register 1 (DCGC1), offset 0x124 */

#define SYSCON_DCGC1_UART0            (1 << 0)  /* Bit 0: UART0 Clock Gating Control */
#define SYSCON_DCGC1_UART1            (1 << 1)  /* Bit 1: UART1 Clock Gating Control */
#define SYSCON_DCGC1_SSI0             (1 << 4)  /* Bit 4: SSI0 Clock Gating Control */
#define SYSCON_DCGC1_SSI1             (1 << 5)  /* Bit 5: SSI1 Clock Gating Control */
#define SYSCON_DCGC1_I2C0             (1 << 12) /* Bit 12: I2C0 Clock Gating Control */
#define SYSCON_DCGC1_I2C1             (1 << 14) /* Bit 14: I2C1 Clock Gating Control */
#define SYSCON_DCGC1_TIMER0           (1 << 16) /* Bit 16: Timer 0 Clock Gating Control */
#define SYSCON_DCGC1_TIMER1           (1 << 17) /* Bit 17: Timer 1 Clock Gating Control */
#define SYSCON_DCGC1_TIMER2           (1 << 18) /* Bit 18: Timer 2 Clock Gating Control */
#define SYSCON_DCGC1_TIMER3           (1 << 19) /* Bit 19: Timer 3 Clock Gating Control */
#define SYSCON_DCGC1_COMP0            (1 << 24) /* Bit 24: Analog Comparator 0 Clock Gating */
#define SYSCON_DCGC1_COMP1            (1 << 25) /* Bit 25: Analog Comparator 1 Clock Gating */

/* Deep Sleep Mode Clock Gating Control Register 2 (DCGC2), offset 0x128 */

#define SYSCON_DCGC2_GPIO(n)          (1 << (n))
#define SYSCON_DCGC2_GPIOA            (1 << 0)  /* Bit 0: Port A Clock Gating Control */
#define SYSCON_DCGC2_GPIOB            (1 << 1)  /* Bit 1: Port B Clock Gating Control */
#define SYSCON_DCGC2_GPIOC            (1 << 2)  /* Bit 2: Port C Clock Gating Control */
#define SYSCON_DCGC2_GPIOD            (1 << 3)  /* Bit 3: Port D Clock Gating Control */
#define SYSCON_DCGC2_GPIOE            (1 << 4)  /* Bit 4: Port E Clock Gating Control */
#define SYSCON_DCGC2_GPIOF            (1 << 5)  /* Bit 5: Port F Clock Gating Control */
#define SYSCON_DCGC2_GPIOG            (1 << 6)  /* Bit 6: Port G Clock Gating Control */
#define SYSCON_DCGC2_GPIOH            (1 << 7)  /* Bit 7: Port H Clock Gating Control */
#define SYSCON_DCGC2_EMAC0            (1 << 28) /* Bit 28: MAC0 Clock Gating Control */
#define SYSCON_DCGC2_EPHY0            (1 << 30) /* Bit 30: PHY0 Clock Gating Control */

/* Deep Sleep Clock Configuration (DSLPCLKCFG), offset 0x144 */

#define SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT 23 /* Bits 28-23: Divider Field Override */
#define SYSCON_DSLPCLKCFG_DSDIVORIDE_MASK  (0x3f << SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT)
#define SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT   4 /* Bits 6-4: Clock Source */
#define SYSCON_DSLPCLKCFG_DSOSCSRC_MASK    (0x07 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM3S_CHIP_LM3S_SYSCONTROL_H */
