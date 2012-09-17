/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_scu.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCU_SCU_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCU_SCU_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

/* Pin Groups */

#define SPSP0                          0
#define SPSP1                          1
#define SPSP2                          2
#define SPSP3                          3
#define SPSP4                          4
#define SPSP5                          5
#define SPSP6                          6
#define SPSP7                          7
#define SPSP8                          8
#define SPSP9                          9
#define SPSP10                        10
#define SPSP11                        11
#define SPSP12                        12
#define SPSP13                        13
#define SPSP14                        14
#define SPSP15                        15

#define LPC43_SCU_SFSP_OFFSET(p,n)    (((p) << 7) | ((n) << 2))
#define LPC43_SCU_SFSP0_OFFSET(n)     (0x0000 | ((n) << 2))
#define LPC43_SCU_SFSP1_OFFSET(n)     (0x0080 | ((n) << 2))
#define LPC43_SCU_SFSP2_OFFSET(n)     (0x0100 | ((n) << 2))
#define LPC43_SCU_SFSP3_OFFSET(n)     (0x0180 | ((n) << 2))
#define LPC43_SCU_SFSP4_OFFSET(n)     (0x0200 | ((n) << 2))
#define LPC43_SCU_SFSP5_OFFSET(n)     (0x0280 | ((n) << 2))
#define LPC43_SCU_SFSP6_OFFSET(n)     (0x0300 | ((n) << 2))
#define LPC43_SCU_SFSP7_OFFSET(n)     (0x0380 | ((n) << 2))
#define LPC43_SCU_SFSP8_OFFSET(n)     (0x0400 | ((n) << 2))
#define LPC43_SCU_SFSP9_OFFSET(n)     (0x0480 | ((n) << 2))
#define LPC43_SCU_SFSPA_OFFSET(n)     (0x0500 | ((n) << 2))
#define LPC43_SCU_SFSPB_OFFSET(n)     (0x0580 | ((n) << 2))
#define LPC43_SCU_SFSPC_OFFSET(n)     (0x0600 | ((n) << 2))
#define LPC43_SCU_SFSPD_OFFSET(n)     (0x0680 | ((n) << 2))
#define LPC43_SCU_SFSPE_OFFSET(n)     (0x0700 | ((n) << 2))
#define LPC43_SCU_SFSPF_OFFSET(n)     (0x0780 | ((n) << 2))

/* CLKn pins */

#define SFSCLK0                       0
#define SFSCLK1                       1
#define SFSCLK2                       2
#define SFSCLK3                       3

#define LPC43_SCU_SFSCLK_OFFSET(n)    (0x0c00 | ((n) << 2))
#define LPC43_SCU_SFSCLK0_OFFSET      0x0c00
#define LPC43_SCU_SFSCLK1_OFFSET      0x0c04
#define LPC43_SCU_SFSCLK2_OFFSET      0x0c08
#define LPC43_SCU_SFSCLK3_OFFSET      0x0c0c

/* USB1 USB1_DP/USB1_DM pins and I2C-bus open-drain pins */

#define LPC43_SCU_SFSUSB_OFFSET       0x0c80
#define LPC43_SCU_SFSI2C0_OFFSET      0x0c84

/* ADC pin select registers */

#define ENAIO0                        0
#define ENAIO1                        1
#define ENAIO2                        2

#define LPC43_SCU_ENAIO_OFFSET(n)     (0x0c88 | ((n) << 2))
#define LPC43_SCU_ENAIO0_OFFSET       0x0c88
#define LPC43_SCU_ENAIO1_OFFSET       0x0c8c
#define LPC43_SCU_ENAIO2_OFFSET       0x0c90

/* EMC delay register */

#define LPC43_SCU_EMCDELAYCLK_OFFSET  0x0d00

/* Pin interrupt select registers */

#define PINTSEL0                      0
#define PINTSEL1                      1

#define LPC43_SCU_PINTSEL_OFFSET(n)   (0x0e00 | ((n) << 2))
#define LPC43_SCU_PINTSEL0_OFFSET     0x0e00
#define LPC43_SCU_PINTSEL1_OFFSET     0x0e04

/* Register Addresses *******************************************************************************/

/* Pin Groups */

#define LPC43_SCU_SFSP(p,n)           (LPC43_SCU_BASE+LPC43_SCU_SFSP_OFFSET(p,n))
#define LPC43_SCU_SFSP0(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP0_OFFSET(n))
#define LPC43_SCU_SFSP1(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP1_OFFSET(n))
#define LPC43_SCU_SFSP2(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP2_OFFSET(n))
#define LPC43_SCU_SFSP3(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP3_OFFSET(n))
#define LPC43_SCU_SFSP4(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP4_OFFSET(n))
#define LPC43_SCU_SFSP5(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP5_OFFSET(n))
#define LPC43_SCU_SFSP6(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP6_OFFSET(n))
#define LPC43_SCU_SFSP7(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP7_OFFSET(n))
#define LPC43_SCU_SFSP8(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP8_OFFSET(n))
#define LPC43_SCU_SFSP9(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSP9_OFFSET(n))
#define LPC43_SCU_SFSPA(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPA_OFFSET(n))
#define LPC43_SCU_SFSPB(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPB_OFFSET(n))
#define LPC43_SCU_SFSPC(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPC_OFFSET(n))
#define LPC43_SCU_SFSPD(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPD_OFFSET(n))
#define LPC43_SCU_SFSPE(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPE_OFFSET(n))
#define LPC43_SCU_SFSPF(n)            (LPC43_SCU_BASE+LPC43_SCU_SFSPF_OFFSET(n))

/* CLKn pins */

#define LPC43_SCU_SFSCLK(n)           (LPC43_SCU_BASE+LPC43_SCU_SFSCLK_OFFSET(n))
#define LPC43_SCU_SFSCLK0             (LPC43_SCU_BASE+LPC43_SCU_SFSCLK0_OFFSET)
#define LPC43_SCU_SFSCLK1             (LPC43_SCU_BASE+LPC43_SCU_SFSCLK1_OFFSET)
#define LPC43_SCU_SFSCLK2             (LPC43_SCU_BASE+LPC43_SCU_SFSCLK2_OFFSET)
#define LPC43_SCU_SFSCLK3             (LPC43_SCU_BASE+LPC43_SCU_SFSCLK3_OFFSET)

/* USB1 USB1_DP/USB1_DM pins and I2C-bus open-drain pins */

#define LPC43_SCU_SFSUSB              (LPC43_SCU_BASE+LPC43_SCU_SFSUSB_OFFSET)
#define LPC43_SCU_SFSI2C0             (LPC43_SCU_BASE+LPC43_SCU_SFSI2C0_OFFSET)

/* ADC pin select registers */

#define LPC43_SCU_ENAIO(n)            (LPC43_SCU_BASE+LPC43_SCU_ENAIO_OFFSET(n))
#define LPC43_SCU_ENAIO0              (LPC43_SCU_BASE+LPC43_SCU_ENAIO0_OFFSET)
#define LPC43_SCU_ENAIO1              (LPC43_SCU_BASE+LPC43_SCU_ENAIO1_OFFSET)
#define LPC43_SCU_ENAIO2              (LPC43_SCU_BASE+LPC43_SCU_ENAIO2_OFFSET)

/* EMC delay register */

#define LPC43_SCU_EMCDELAYCLK         (LPC43_SCU_BASE+LPC43_SCU_EMCDELAYCLK_OFFSET)

/* Pin interrupt select registers */

#define LPC43_SCU_PINTSEL(n)          (LPC43_SCU_BASE+LPC43_SCU_PINTSEL_OFFSET(n))
#define LPC43_SCU_PINTSEL0            (LPC43_SCU_BASE+LPC43_SCU_PINTSEL0_OFFSET)
#define LPC43_SCU_PINTSEL1            (LPC43_SCU_BASE+LPC43_SCU_PINTSEL1_OFFSET)

/* Register Bit Definitions *************************************************************************/
/* Common Pin configuration register bit settings */

#define SCU_PIN_MODE_SHIFT            (0)       /* Bits 0-2: Select pin function */
#define SCU_PIN_MODE_MASK             (7 << SCU_PIN_MODE_SHIFT)
#  define SCU_PIN_MODE_FUNC(n)        ((n) << SCU_PIN_MODE_SHIFT)
#  define SCU_PIN_MODE_FUNC0          (0 << SCU_PIN_MODE_SHIFT) /* Function 0 (default) */
#  define SCU_PIN_MODE_FUNC1          (1 << SCU_PIN_MODE_SHIFT) /* Function 1 */
#  define SCU_PIN_MODE_FUNC2          (2 << SCU_PIN_MODE_SHIFT) /* Function 2 */
#  define SCU_PIN_MODE_FUNC3          (3 << SCU_PIN_MODE_SHIFT) /* Function 3 */
#  define SCU_PIN_MODE_FUNC4          (4 << SCU_PIN_MODE_SHIFT) /* Function 4 */
#  define SCU_PIN_MODE_FUNC5          (5 << SCU_PIN_MODE_SHIFT) /* Function 5 */
#  define SCU_PIN_MODE_FUNC6          (6 << SCU_PIN_MODE_SHIFT) /* Function 6 */
#  define SCU_PIN_MODE_FUNC7          (7 << SCU_PIN_MODE_SHIFT) /* Function 7 */
#define SCU_PIN_EPD                   (1 << 3)  /* Bit 3:  Enable pull-down resistor at pad */
#define SCU_PIN_EPUN                  (1 << 4)  /* Bit 4:  Disable pull-up resistor at pad */
                                                /* Bit 5:  Usage varies with pin type */
#define SCU_PIN_EZI                   (1 << 6)  /* Bit 6:  Input buffer enable */
#define SCU_PIN_ZIF                   (1 << 7)  /* Bit 7:  Input glitch filter */
                                                /* Bits 8-9: Usage varies with pin type */
                                                /* Bits 10-31: Reserved */
/* Pin configuration registers for normal-drive pins (only):
 *
 *   P0_0 and P0_1
 *   P1_0 to P1_16 and P1_18 to P1_20
 *   P2_0 to P2_2 and P2_6 to P2_13
 *   P3_0 to P3_2 and P3_4 to P3_8
 *   P4_0 to P4_10
 *   P5_0 to P5_7
 *   P6_0 to P6_12
 *   P7_0 to P7_7
 *   P8_3 to P8_8
 *   P9_0 to P9_6
 *   PA_0 and PA_4
 *   PB_0 to PB_6
 *   PC_0 to PC_14
 *   PE_0 to PE_15
 *   PF_0 to PF_11
 */
                                                /* Bits 0-4: Same as common bit definitions */
#define SCU_NDPIN_EHS                 (1 << 5)  /* Bit 5:  EHS Select Slew rate */
                                                /* Bits 6-31: Same as common bit definitions */
/* Pin configuration registers for high-drive pins
 *
 *   P1_17
 *   P2_3 to P2_5
 *   P8_0 to P8_2
 *   PA_1 to PA_3
 */
                                                /* Bits 0-7:  Same as common bit definitions */
#define SCU_HDPIN_EHD_SHIFT           (8)       /* Bits 8-9: Select drive strength */
#define SCU_HDPIN_EHD_MASK            (3 << SCU_HDPIN_EHD_SHIFT)
#  define SCU_HDPIN_EHD_NORMAL        (0 << SCU_HDPIN_EHD_SHIFT) /* Normal-drive: 4 mA drive strength */
#  define SCU_HDPIN_EHD_MEDIUM        (1 << SCU_HDPIN_EHD_SHIFT) /* Medium-drive: 8 mA drive strength */
#  define SCU_HDPIN_EHD_HIGH          (2 << SCU_HDPIN_EHD_SHIFT) /* High-drive: 14 mA drive strength */
#  define SCU_HDPIN_EHD_ULTRA         (3 << SCU_HDPIN_EHD_SHIFT) /* Ultra high-drive: 20 mA drive strength */
                                                /* Bits 10-31:  Reserved */
/* Pin configuration registers for high-speed pins
 *
 * P3_3 and pins CLK0 to CLK3
 */
                                                /* Bits 0-4: Same as common bit definitions */
#define SCU_HSPIN_EHS                 (1 << 5)  /* Bit 5:  EHS Select Slew rate */
                                                /* Bits 6-31: Same as common bit definitions */
/* Pin configuration register for USB1 pins USB1_DP/USB1_DM */

#define SCU_SFSUSB_AIM                (1 << 0)  /* Bit 0:  Differential data input AIP/AIM */
#define SCU_SFSUSB_ESEA               (1 << 1)  /* Bit 1:  Control signal for differential input or single input */
#define SCU_SFSUSB_EPD                (1 << 2)  /* Bit 2:  Enable pull-down connect */
                                                /* Bit 3:  Reserved */
#define SCU_SFSUSB_EPWR               (1 << 4)  /* Bit 4:  Power mode */
#define SCU_SFSUSB_VBUS               (1 << 5)  /* Bit 5:  Enable the vbus_valid signal */
                                                /* Bits 6-31:  Reserved */
/* Pin configuration register for open-drain I2C-bus pins */

#define SCU_SFSI2C0_SCL_EFP           (1 << 0)  /* Bit 0:  Select input glitch filter time constant for the SCL pin */
                                                /* Bit 1:  Reserved */
#define SCU_SFSI2C0_SCL_EHD           (1 << 2)  /* Bit 2:  Select I2C mode for the SCL pin */
#define SCU_SFSI2C0_SCL_EZI           (1 << 3)  /* Bit 3:  Enable the input receiver for the SCL pin */
                                                /* Bits 4-6: Reserved */
#define SCU_SFSI2C0_SCL_ZIF           (1 << 7)  /* Bit 7:  Enable or disable input glitch filter for the SCL pin */
#define SCU_SFSI2C0_SDA_EFP           (1 << 8)  /* Bit 8:  Select input glitch filter time constant for the SDA pin */
                                                /* Bit 9:  Reserved */
#define SCU_SFSI2C0_SDA_EHD           (1 << 10) /* Bit 10: Select I2C mode for the SDA pin */
#define SCU_SFSI2C0_SDA_EZI           (1 << 11) /* Bit 11: Enable the input receiver for the SDA pin */
                                                /* Bits 12-14: Reserved */
#define SCU_SFSI2C0_SDA_ZIF           (1 << 15) /* Bit 15: Enable or disable input glitch filter for the SDA pin */
                                                /* Bits 16-31: Reserved */
/* ADC0 function select register.  The following pins are controlled by the ENAIO0 register:
 *
 *   Pin   ADC function  ENAIO0 register bit
 *   P4_3  ADC0_0        0
 *   P4_1  ADC0_1        1
 *   PF_8  ADC0_2        2
 *   P7_5  ADC0_3        3
 *   P7_4  ADC0_4        4
 *   PF_10 ADC0_5        5
 *   PB_6  ADC0_6        6
 */

#define SCU_ENAI00_ADC0(n)            (1 << (n))
#define SCU_ENAI00_ADC0_0             (1 << 0)  /* Select ADC0_0 */
#define SCU_ENAI00_ADC0_1             (1 << 1)  /* Select ADC0_1 */
#define SCU_ENAI00_ADC0_2             (1 << 2)  /* Select ADC0_2 */
#define SCU_ENAI00_ADC0_3             (1 << 3)  /* Select ADC0_3 */
#define SCU_ENAI00_ADC0_4             (1 << 4)  /* Select ADC0_4 */
#define SCU_ENAI00_ADC0_5             (1 << 5)  /* Select ADC0_5 */
#define SCU_ENAI00_ADC0_6             (1 << 6)  /* Select ADC0_6 */

/* ADC1 function select register.  The following pins are controlled by the ENAIO1 register:
 *
 *   Pin   ADC function  ENAIO0 register bit
 *   PC_3  ADC1_0        0
 *   PC_0  ADC1_1        1
 *   PF_9  ADC1_2        2
 *   PF_6  ADC1_3        3
 *   PF_5  ADC1_4        4
 *   PF_11 ADC1_5        5
 *   P7_7  ADC1_6        6
 *   PF_7  ADC1_7        7
 */

#define SCU_ENAI01_ADC1(n)            (1 << (n))
#define SCU_ENAI01_ADC1_0             (1 << 0)  /* Select ADC1_0 */
#define SCU_ENAI01_ADC1_1             (1 << 1)  /* Select ADC1_1 */
#define SCU_ENAI01_ADC1_2             (1 << 2)  /* Select ADC1_2 */
#define SCU_ENAI01_ADC1_3             (1 << 3)  /* Select ADC1_3 */
#define SCU_ENAI01_ADC1_4             (1 << 4)  /* Select ADC1_4 */
#define SCU_ENAI01_ADC1_5             (1 << 5)  /* Select ADC1_5 */
#define SCU_ENAI01_ADC1_6             (1 << 6)  /* Select ADC1_6 */
#define SCU_ENAI01_ADC1_7             (1 << 7)  /* Select ADC1_7 */

/* Analog function select register.  The following pins are controlled by the ENAIO2 register:
 *
 *   Pin   ADC function         ENAIO0 register bit
 *   P4_4  DAC                  0
 *   PF_7  BG (band gap output) 4
 */

#define SCU_ENAI02_DAC                (1 << 0)  /* Select DAC */
#define SCU_ENAI02_BG                 (1 << 4)  /* Select band gap output */

/* EMC clock delay register.  The value 0x1111 corresponds to about 0.5 ns of delay */

#define SCU_EMCDELAYCLK_SHIFT         (0)       /* Bits 0-15: EMC_CLKn SDRAM clock output delay */
#define SCU_EMCDELAYCLK_MASK          (0xffff << SCU_EMCDELAYCLK_SHIFT)
#  define SCU_EMCDELAYCLK(n)          ((n) << SCU_EMCDELAYCLK_SHIFT) /* 0=no delay, N*0x1111 = N*0.5 ns delay */
                                                /* Bits 16-31:  Reserved */
/* Pin interrupt select register 0 */

#define SCU_GPIO_PORT0                0
#define SCU_GPIO_PORT1                1
#define SCU_GPIO_PORT2                2
#define SCU_GPIO_PORT3                3
#define SCU_GPIO_PORT4                4
#define SCU_GPIO_PORT5                5
#define SCU_GPIO_PORT6                6
#define SCU_GPIO_PORT7                7

#define SCU_GPIO_PIN0                 0
#define SCU_GPIO_PIN1                 1
#define SCU_GPIO_PIN2                 2
#define SCU_GPIO_PIN3                 3
#define SCU_GPIO_PIN4                 4
#define SCU_GPIO_PIN5                 5
#define SCU_GPIO_PIN6                 6
#define SCU_GPIO_PIN7                 7
#define SCU_GPIO_PIN8                 8
#define SCU_GPIO_PIN9                 9
#define SCU_GPIO_PIN10                10
#define SCU_GPIO_PIN11                11
#define SCU_GPIO_PIN12                12
#define SCU_GPIO_PIN13                13
#define SCU_GPIO_PIN14                14
#define SCU_GPIO_PIN15                15
#define SCU_GPIO_PIN16                16
#define SCU_GPIO_PIN17                17
#define SCU_GPIO_PIN18                18
#define SCU_GPIO_PIN19                19
#define SCU_GPIO_PIN20                20
#define SCU_GPIO_PIN21                21
#define SCU_GPIO_PIN22                22
#define SCU_GPIO_PIN23                23
#define SCU_GPIO_PIN24                24
#define SCU_GPIO_PIN25                25
#define SCU_GPIO_PIN26                26
#define SCU_GPIO_PIN27                27
#define SCU_GPIO_PIN28                28
#define SCU_GPIO_PIN29                29
#define SCU_GPIO_PIN30                30
#define SCU_GPIO_PIN31                31

#define SCU_PINTSEL0_SHIFT(n)         ((n) << 3)
#define SCU_PINTSEL0_MASK(n)          (0xff << SCU_PINTSEL0_SHIFT(n)))
#define SCU_PINTSEL0_INTPIN_SHIFT(n)  ((n) << 3)
#define SCU_PINTSEL0_INTPIN_MASK(n)   (31 << SCU_PINTSEL0_INTPIN_SHIFT(n))
#define SCU_PINTSEL0_PORTSEL_SHIFT(n) (((n) << 3) + 5)
#define SCU_PINTSEL0_PORTSEL_MASK(n)  (7 << SCU_PINTSEL0_PORTSEL_SHIFT(n))

#define SCU_PINTSEL0_INTPIN0_SHIFT    (0)       /* Bits 0-4: Pint interrupt 0 */
#define SCU_PINTSEL0_INTPIN0_MASK     (31 << SCU_PINTSEL0_INTPIN0_SHIFT)
#define SCU_PINTSEL0_PORTSEL0_SHIFT   (5)       /* Bits 5-7: Pin interrupt 0 */
#define SCU_PINTSEL0_PORTSEL0_MASK    (7 << SCU_PINTSEL0_PORTSEL0_SHIFT)
#define SCU_PINTSEL0_INTPIN1_SHIFT    (8)       /* Bits 8-12: Pint interrupt 1 */
#define SCU_PINTSEL0_INTPIN1_MASK     (31 << SCU_PINTSEL0_INTPIN1_SHIFT)
#define SCU_PINTSEL0_PORTSEL1_SHIFT   (13)      /* Bits 13-15: Pin interrupt 1 */
#define SCU_PINTSEL0_PORTSEL1_MASK    (7 << SCU_PINTSEL0_PORTSEL1_SHIFT)
#define SCU_PINTSEL0_INTPIN2_SHIFT    (16)      /* Bits 16-20: Pint interrupt 2 */
#define SCU_PINTSEL0_INTPIN2_MASK     (31 << SCU_PINTSEL0_INTPIN2_SHIFT)
#define SCU_PINTSEL0_PORTSEL2_SHIFT   (21)      /* Bits 21-23: Pin interrupt 2 */
#define SCU_PINTSEL0_PORTSEL2_MASK    (7 << SCU_PINTSEL0_PORTSEL2_SHIFT)
#define SCU_PINTSEL0_INTPIN3_SHIFT    (24)      /* Bits 24-28: Pint interrupt 3 */
#define SCU_PINTSEL0_INTPIN3_MASK     (31 << SCU_PINTSEL0_INTPIN3_SHIFT)
#define SCU_PINTSEL0_PORTSEL3_SHIFT   (29)      /* Bits 29-31: Pin interrupt 3 */
#define SCU_PINTSEL0_PORTSEL3_MASK    (7 << SCU_PINTSEL0_PORTSEL3_SHIFT)

/* Pin interrupt select register 1 */

#define SCU_PINTSEL1_SHIFT(n)         (((n) - 4) << 3)
#define SCU_PINTSEL1_MASK(n)          (0xff << SCU_PINTSEL1_SHIFT(n))
#define SCU_PINTSEL1_INTPIN_SHIFT(n)  (((n) - 4) << 3)
#define SCU_PINTSEL1_INTPIN_MASK(n)   (31 << SCU_PINTSEL0_INTPIN_SHIFT(n))
#define SCU_PINTSEL1_PORTSEL_SHIFT(n) ((((n) - 4) << 3) + 5)
#define SCU_PINTSEL1_PORTSEL_MASK(n)  (7 << SCU_PINTSEL0_PORTSEL_SHIFT(n))

#define SCU_PINTSEL1_INTPIN4_SHIFT    (0)       /* Bits 0-4: Pint interrupt 4 */
#define SCU_PINTSEL1_INTPIN4_MASK     (31 << SCU_PINTSEL1_INTPIN4_SHIFT)
#define SCU_PINTSEL1_PORTSEL4_SHIFT   (5)       /* Bits 5-7: Pin interrupt 4 */
#define SCU_PINTSEL1_PORTSEL4_MASK    (7 << SCU_PINTSEL1_PORTSEL4_SHIFT)
#define SCU_PINTSEL1_INTPIN5_SHIFT    (8)       /* Bits 8-12: Pint interrupt 5 */
#define SCU_PINTSEL1_INTPIN5_MASK     (31 << SCU_PINTSEL1_INTPIN5_SHIFT)
#define SCU_PINTSEL1_PORTSEL5_SHIFT   (13)       /* Bits 13-15: Pin interrupt 5 */
#define SCU_PINTSEL1_PORTSEL5_MASK    (7 << SCU_PINTSEL1_PORTSEL5_SHIFT)
#define SCU_PINTSEL1_INTPIN6_SHIFT    (16)       /* Bits 16-20: Pint interrupt 6 */
#define SCU_PINTSEL1_INTPIN6_MASK     (31 << SCU_PINTSEL1_INTPIN6_SHIFT)
#define SCU_PINTSEL1_PORTSEL6_SHIFT   (21)       /* Bits 21-23: Pin interrupt 6 */
#define SCU_PINTSEL1_PORTSEL6_MASK    (7 << SCU_PINTSEL1_PORTSEL6_SHIFT)
#define SCU_PINTSEL1_INTPIN7_SHIFT    (24)       /* Bits 24-28: Pint interrupt 7 */
#define SCU_PINTSEL1_INTPIN7_MASK     (31 << SCU_PINTSEL1_INTPIN7_SHIFT)
#define SCU_PINTSEL1_PORTSEL7_SHIFT   (29)       /* Bits 29-31: Pin interrupt 7 */
#define SCU_PINTSEL1_PORTSEL7_MASK    (7 << SCU_PINTSEL1_PORTSEL7_SHIFT)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCU_SCU_H */
