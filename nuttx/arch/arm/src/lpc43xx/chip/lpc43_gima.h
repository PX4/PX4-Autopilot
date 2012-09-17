/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_gima.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GIMA_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GIMA_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

/* Timer capture input multiplexor registers */

#define LPC43_GIMA_CAP_OFFSET(t,i)    (((t) << 4) | ((i) << 2))
#define LPC43_GIMA_CAP00_OFFSET       0x0000  /* Timer 0 CAP0_0 capture input multiplexer (GIMA output 0) */
#define LPC43_GIMA_CAP01_OFFSET       0x0004  /* Timer 0 CAP0_1 capture input multiplexer (GIMA output 1) */
#define LPC43_GIMA_CAP02_OFFSET       0x0008  /* Timer 0 CAP0_2 capture input multiplexer (GIMA output 2) */
#define LPC43_GIMA_CAP03_OFFSET       0x000c  /* Timer 0 CAP0_3 capture input multiplexer (GIMA output 3) */
#define LPC43_GIMA_CAP10_OFFSET       0x0010  /* Timer 1 CAP1_0 capture input multiplexer (GIMA output 4) */
#define LPC43_GIMA_CAP11_OFFSET       0x0014  /* Timer 1 CAP1_1 capture input multiplexer (GIMA output 5) */
#define LPC43_GIMA_CAP12_OFFSET       0x0018  /* Timer 1 CAP1_2 capture input multiplexer (GIMA output 6) */
#define LPC43_GIMA_CAP13_OFFSET       0x001c  /* Timer 1 CAP1_3 capture input multiplexer (GIMA output 7) */
#define LPC43_GIMA_CAP20_OFFSET       0x0020  /* Timer 2 CAP2_0 capture input multiplexer (GIMA output 8) */
#define LPC43_GIMA_CAP21_OFFSET       0x0024  /* Timer 2 CAP2_1 capture input multiplexer (GIMA output 9) */
#define LPC43_GIMA_CAP22_OFFSET       0x0028  /* Timer 2 CAP2_2 capture input multiplexer (GIMA output 10) */
#define LPC43_GIMA_CAP23_OFFSET       0x002c  /* Timer 2 CAP2_3 capture input multiplexer (GIMA output 11) */
#define LPC43_GIMA_CAP30_OFFSET       0x0030  /* Timer 3 CAP3_0 capture input multiplexer (GIMA output 12) */
#define LPC43_GIMA_CAP31_OFFSET       0x0034  /* Timer 3 CAP3_1 capture input multiplexer (GIMA output 13) */
#define LPC43_GIMA_CAP32_OFFSET       0x0038  /* Timer 3 CAP3_2 capture input multiplexer (GIMA output 14) */
#define LPC43_GIMA_CAP33_OFFSET       0x003c  /* Timer 3 CAP3_3 capture input multiplexer (GIMA output 15) */

#define LPC43_GIMA_CTIN_OFFSET(i)     (0x0040 + ((i) << 2))
#define LPC43_GIMA_CTIN0_OFFSET       0x0040  /* SCT CTIN_0 capture input multiplexer (GIMA output 16) */
#define LPC43_GIMA_CTIN1_OFFSET       0x0044  /* SCT CTIN_1 capture input multiplexer (GIMA output 17) */
#define LPC43_GIMA_CTIN2_OFFSET       0x0048  /* SCT CTIN_2 capture input multiplexer (GIMA output 18) */
#define LPC43_GIMA_CTIN3_OFFSET       0x004c  /* SCT CTIN_3 capture input multiplexer (GIMA output 19) */
#define LPC43_GIMA_CTIN4_OFFSET       0x0050  /* SCT CTIN_4 capture input multiplexer (GIMA output 20) */
#define LPC43_GIMA_CTIN5_OFFSET       0x0054  /* SCT CTIN_5 capture input multiplexer (GIMA output 21) */
#define LPC43_GIMA_CTIN6_OFFSET       0x0058  /* SCT CTIN_6 capture input multiplexer (GIMA output 22) */
#define LPC43_GIMA_CTIN7_OFFSET       0x005c  /* SCT CTIN_7 capture input multiplexer (GIMA output 23) */

#define LPC43_GIMA_VADCTRIG_OFFSET    0x0060  /* VADC trigger input multiplexer (GIMA output 24) */
#define LPC43_GIMA_EVNTRTR13_OFFSET   0x0064  /* Event router input 13 multiplexer (GIMA output 25) */
#define LPC43_GIMA_EVNTRTR14_OFFSET   0x0068  /* Event router input 14 multiplexer (GIMA output 26) */
#define LPC43_GIMA_EVNTRTR16_OFFSET   0x006c  /* Event router input 16 multiplexer (GIMA output 27) */
#define LPC43_GIMA_ADCSTART0_OFFSET   0x0070  /* ADC start0 input multiplexer (GIMA output 28) */
#define LPC43_GIMA_ADCSTART1_OFFSET   0x0074  /* ADC start1 input multiplexer (GIMA output 29) */

/* Register Addresses *******************************************************************************/

#define LPC43_GIMA_CAP(t,i)           (LPC43_GIMA_BASE+LPC43_GIMA_CAP_OFFSET(t,i))
#define LPC43_GIMA_CAP00              (LPC43_GIMA_BASE+LPC43_GIMA_CAP00_OFFSET)
#define LPC43_GIMA_CAP01              (LPC43_GIMA_BASE+LPC43_GIMA_CAP01_OFFSET)
#define LPC43_GIMA_CAP02              (LPC43_GIMA_BASE+LPC43_GIMA_CAP02_OFFSET)
#define LPC43_GIMA_CAP03              (LPC43_GIMA_BASE+LPC43_GIMA_CAP03_OFFSET)
#define LPC43_GIMA_CAP10              (LPC43_GIMA_BASE+LPC43_GIMA_CAP10_OFFSET)
#define LPC43_GIMA_CAP11              (LPC43_GIMA_BASE+LPC43_GIMA_CAP11_OFFSET)
#define LPC43_GIMA_CAP12              (LPC43_GIMA_BASE+LPC43_GIMA_CAP12_OFFSET)
#define LPC43_GIMA_CAP13              (LPC43_GIMA_BASE+LPC43_GIMA_CAP13_OFFSET)
#define LPC43_GIMA_CAP20              (LPC43_GIMA_BASE+LPC43_GIMA_CAP20_OFFSET)
#define LPC43_GIMA_CAP21              (LPC43_GIMA_BASE+LPC43_GIMA_CAP21_OFFSET)
#define LPC43_GIMA_CAP22              (LPC43_GIMA_BASE+LPC43_GIMA_CAP22_OFFSET)
#define LPC43_GIMA_CAP23              (LPC43_GIMA_BASE+LPC43_GIMA_CAP23_OFFSET)
#define LPC43_GIMA_CAP30              (LPC43_GIMA_BASE+LPC43_GIMA_CAP30_OFFSET)
#define LPC43_GIMA_CAP31              (LPC43_GIMA_BASE+LPC43_GIMA_CAP31_OFFSET)
#define LPC43_GIMA_CAP32              (LPC43_GIMA_BASE+LPC43_GIMA_CAP32_OFFSET)
#define LPC43_GIMA_CAP33              (LPC43_GIMA_BASE+LPC43_GIMA_CAP33_OFFSET)

#define LPC43_GIMA_CTIN(i)            (LPC43_GIMA_BASE+LPC43_GIMA_CTIN_OFFSET(i))
#define LPC43_GIMA_CTIN0              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN0_OFFSET)
#define LPC43_GIMA_CTIN1              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN1_OFFSET)
#define LPC43_GIMA_CTIN2              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN2_OFFSET)
#define LPC43_GIMA_CTIN3              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN3_OFFSET)
#define LPC43_GIMA_CTIN4              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN4_OFFSET)
#define LPC43_GIMA_CTIN5              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN5_OFFSET)
#define LPC43_GIMA_CTIN6              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN6_OFFSET)
#define LPC43_GIMA_CTIN7              (LPC43_GIMA_BASE+LPC43_GIMA_CTIN7_OFFSET)
#define LPC43_GIMA_VADCTRIG           (LPC43_GIMA_BASE+LPC43_GIMA_VADCTRIG_OFFSET)
#define LPC43_GIMA_EVNTRTR13          (LPC43_GIMA_BASE+LPC43_GIMA_EVNTRTR13_OFFSET)
#define LPC43_GIMA_EVNTRTR14          (LPC43_GIMA_BASE+LPC43_GIMA_EVNTRTR14_OFFSET)
#define LPC43_GIMA_EVNTRTR16          (LPC43_GIMA_BASE+LPC43_GIMA_EVNTRTR16_OFFSET)
#define LPC43_GIMA_ADCSTART0          (LPC43_GIMA_BASE+LPC43_GIMA_ADCSTART0_OFFSET)
#define LPC43_GIMA_ADCSTART1          (LPC43_GIMA_BASE+LPC43_GIMA_ADCSTART1_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Common register field definitions */

#define GIMA_INV                      (1 << 0)  /* Bit 0:  Invert input  */
#define GIMA_EDGE                     (1 << 1)  /* Bit 1:  Enable rising edge detection */
#define GIMA_SYNCH                    (1 << 2)  /* Bit 2:  Enable synchronization */
#define GIMA_PULSE                    (1 << 3)  /* Bit 3:  Enable single pulse generation */
#define GIMA_SELECT_SHIFT             (4)       /* Bits 4-7: Select input */
#define GIMA_SELECT_MASK              (15 << GIMA_SELECT_SHIFT)
                                                /* Bits 8-31:  Reserved */
/* Timer 0 CAP0_0 capture input multiplexer (GIMA output 0) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP00_SELECT_CTIN0     (0 << GIMA_SELECT_SHIFT) /* CTIN_0 */
#  define GIMA_CAP00_SELECT_SGPIO3    (1 << GIMA_SELECT_SHIFT) /* SGPIO3 */
#  define GIMA_CAP00_SELECT_TOCAP0    (2 << GIMA_SELECT_SHIFT) /* T0_CAP0 */
                                                /* Bits 8-31: Reserved */
/* Timer 0 CAP0_1 capture input multiplexer (GIMA output 1) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP01_SELECT_CTIN1     (0 << GIMA_SELECT_SHIFT) /* CTIN_1 */
#  define GIMA_CAP01_SELECT_U2TX      (1 << GIMA_SELECT_SHIFT) /* USART2 TX active */
#  define GIMA_CAP01_SELECT_TOCAP1    (2 << GIMA_SELECT_SHIFT) /* T0_CAP1 */
                                                /* Bits 8-31: Reserved */
/* Timer 0 CAP0_2 capture input multiplexer (GIMA output 2) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP02_SELECT_CTIN2     (0 << GIMA_SELECT_SHIFT) /* CTIN_2 */
#  define GIMA_CAP02_SELECT_SGPIO3D   (1 << GIMA_SELECT_SHIFT) /* SGPIO3_DIV */
#  define GIMA_CAP02_SELECT_T0CAP2    (2 << GIMA_SELECT_SHIFT) /* T0_CAP2 */
                                                /* Bits 8-31: Reserved */
/* Timer 0 CAP0_3 capture input multiplexer (GIMA output 3) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP03_SELECT_CTOUT15   (0 << GIMA_SELECT_SHIFT) /* CTOUT_15 or T3_MAT3 */
#  define GIMA_CAP03_SELECT_T0CAP3    (1 << GIMA_SELECT_SHIFT) /* T0_CAP3 */
#  define GIMA_CAP03_SELECT_T3MAT3    (2 << GIMA_SELECT_SHIFT) /* T3_MAT3 */
                                                /* Bits 8-31: Reserved */
/* Timer 1 CAP1_0 capture input multiplexer (GIMA output 4) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP10_SELECT_CTIN0     (0 << GIMA_SELECT_SHIFT) /* CTIN_0 */
#  define GIMA_CAP10_SELECT_SGPIO12   (1 << GIMA_SELECT_SHIFT) /* SGPIO12 */
#  define GIMA_CAP10_SELECT_T1CAP0    (2 << GIMA_SELECT_SHIFT) /* T1_CAP0 */
                                                /* Bits 8-31: Reserved */
/* Timer 1 CAP1_1 capture input multiplexer (GIMA output 5) */
                                                 /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP11_SELECT_CTIN3     (0 << GIMA_SELECT_SHIFT) /* CTIN_3 */
#  define GIMA_CAP11_SELECT_U0TX      (1 << GIMA_SELECT_SHIFT) /* USART0 TX active */
#  define GIMA_CAP11_SELECT_T1CAP1    (2 << GIMA_SELECT_SHIFT) /* T1_CAP1 */
                                                /* Bits 8-31: Reserved */
/* Timer 1 CAP1_2 capture input multiplexer (GIMA output 6) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP12_SELECT_CTIN4     (0 << GIMA_SELECT_SHIFT) /* CTIN_4 */
#  define GIMA_CAP12_SELECT_U0RX      (1 << GIMA_SELECT_SHIFT) /* USART0 RX active */
#  define GIMA_CAP12_SELECT_T1CAP2    (2 << GIMA_SELECT_SHIFT) /* T1_CAP2 */
                                                /* Bits 8-31: Reserved */
/* Timer 1 CAP1_3 capture input multiplexer (GIMA output 7) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP13_SELECT_CTOUT3    (0 << GIMA_SELECT_SHIFT) /* CTOUT_3 or T0_MAT3 */
#  define GIMA_CAP13_SELECT_T1CAP3    (1 << GIMA_SELECT_SHIFT) /* T1_CAP3 */
#  define GIMA_CAP13_SELECT_T0MAT3    (2 << GIMA_SELECT_SHIFT) /* T0_MAT3 */
                                                /* Bits 8-31: Reserved */
/* Timer 2 CAP2_0 capture input multiplexer (GIMA output 8) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP20_SELECT_CTIN0     (0 << GIMA_SELECT_SHIFT) /* CTIN_0 */
#  define GIMA_CAP20_SELECT_SGPIO12D  (1 << GIMA_SELECT_SHIFT) /* SGPIO12_DIV */
#  define GIMA_CAP20_SELECT_T2CAP0    (2 << GIMA_SELECT_SHIFT) /* T2_CAP0 */
                                                /* Bits 8-31: Reserved */
/* Timer 2 CAP2_1 capture input multiplexer (GIMA output 9) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP21_SELECT_CTIN1     (0 << GIMA_SELECT_SHIFT) /* CTIN_1 */
#  define GIMA_CAP21_SELECT_U2TX      (1 << GIMA_SELECT_SHIFT) /* USART2 TX active */
#  define GIMA_CAP21_SELECT_T2CAP1    (2 << GIMA_SELECT_SHIFT) /* T2_CAP1 */
                                                /* Bits 8-31: Reserved */
/* Timer 2 CAP2_2 capture input multiplexer (GIMA output 10) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP22_SELECT_CTIN5     (0 << GIMA_SELECT_SHIFT) /* CTIN_5 */
#  define GIMA_CAP22_SELECT_U2RX      (1 << GIMA_SELECT_SHIFT) /* USART2 RX active */
#  define GIMA_CAP22_SELECT_I2S1TX    (2 << GIMA_SELECT_SHIFT) /* I2S1_TX_MWS */
#  define GIMA_CAP22_SELECT_T2CAP2    (3 << GIMA_SELECT_SHIFT) /* T2_CAP2 */
                                                /* Bits 8-31: Reserved */
/* Timer 2 CAP2_3 capture input multiplexer (GIMA output 11) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP23_SELECT_CTOUT7    (0 << GIMA_SELECT_SHIFT) /* CTOUT_7 or T1_MAT3 */
#  define GIMA_CAP23_SELECT_T2CAP3    (1 << GIMA_SELECT_SHIFT) /* T2_CAP3 */
#  define GIMA_CAP23_SELECT_T1MAT3    (2 << GIMA_SELECT_SHIFT) /* T1_MAT3 */
                                                /* Bits 8-31: Reserved */
/* Timer 3 CAP3_0 capture input multiplexer (GIMA output 12) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP30_SELECT_CTIN0     (0 << GIMA_SELECT_SHIFT) /* CTIN_0 */
#  define GIMA_CAP30_SELECT_I2S0RX    (1 << GIMA_SELECT_SHIFT) /* I2S0_RX_MWS */
#  define GIMA_CAP30_SELECT_T3CAP0    (2 << GIMA_SELECT_SHIFT) /* T3_CAP0 */
                                                /* Bits 8-31: Reserved */
/* Timer 3 CAP3_1 capture input multiplexer (GIMA output 13) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP31_SELECT_CTIN6     (0 << GIMA_SELECT_SHIFT) /* CTIN_6 */
#  define GIMA_CAP31_SELECT_U3TX      (1 << GIMA_SELECT_SHIFT) /* USART3 TX active */
#  define GIMA_CAP31_SELECT_I2S0TX    (2 << GIMA_SELECT_SHIFT) /* I2S0_TX_MWS */
#  define GIMA_CAP31_SELECT_T3CAP1    (3 << GIMA_SELECT_SHIFT) /* T3_CAP1 */
                                                /* Bits 8-31: Reserved */
/* Timer 3 CAP3_2 capture input multiplexer (GIMA output 14) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP32_SELECT_CTIN7     (0 << GIMA_SELECT_SHIFT) /* CTIN_7 */
#  define GIMA_CAP32_SELECT_U3RX      (1 << GIMA_SELECT_SHIFT) /* USART3 RX active */
#  define GIMA_CAP32_SELECT_SOF0      (2 << GIMA_SELECT_SHIFT) /* SOF0 (Start-Of-Frame USB0) */
#  define GIMA_CAP32_SELECT_T3CAP2    (3 << GIMA_SELECT_SHIFT) /* T3_CAP2 */
                                                /* Bits 8-31: Reserved */
/* Timer 3 CAP3_3 capture input multiplexer (GIMA output 15) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CAP33_SELECT_CTOUT11   (0 << GIMA_SELECT_SHIFT) /* CTOUT11 or T2_MAT3 */
#  define GIMA_CAP33_SELECT_SOF1      (1 << GIMA_SELECT_SHIFT) /* SOF1 (Start-Of-Frame USB1) */
#  define GIMA_CAP33_SELECT_T3CAP3    (2 << GIMA_SELECT_SHIFT) /* T3_CAP3 */
#  define GIMA_CAP33_SELECT_T2MAT3    (3 << GIMA_SELECT_SHIFT) /* T2_MAT3 */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_0 capture input multiplexer (GIMA output 16) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN0_SELECT_CTIN0     (0 << GIMA_SELECT_SHIFT) /* CTIN_0 */
#  define GIMA_CTIN0_SELECT_SGPIO3    (1 << GIMA_SELECT_SHIFT) /* SGPIO3 */
#  define GIMA_CTIN0_SELECT_SGPIO3D   (2 << GIMA_SELECT_SHIFT) /* SGPIO3_DIV */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_1 capture input multiplexer (GIMA output 17) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN1_SELECT_CTIN1     (0 << GIMA_SELECT_SHIFT) /* CTIN_1 */
#  define GIMA_CTIN1_SELECT_U2TX      (1 << GIMA_SELECT_SHIFT) /* USART2 TX active */
#  define GIMA_CTIN1_SELECT_SGPIO12   (2 << GIMA_SELECT_SHIFT) /* SGPIO12 */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_2 capture input multiplexer (GIMA output 18) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN2_SELECT_CTIN2     (0 << GIMA_SELECT_SHIFT) /* CTIN_2 */
#  define GIMA_CTIN2_SELECT_SGPIO12   (1 << GIMA_SELECT_SHIFT) /* SGPIO12 */
#  define GIMA_CTIN2_SELECT_SGPIO12D  (2 << GIMA_SELECT_SHIFT) /* SGPIO12_DIV */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_3 capture input multiplexer (GIMA output 19) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN3_SELECT_CTIN3     (0 << GIMA_SELECT_SHIFT) /* CTIN_3 */
#  define GIMA_CTIN3_SELECT_U0TX      (1 << GIMA_SELECT_SHIFT) /* USART0 TX active */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_4 capture input multiplexer (GIMA output 20) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN4_SELECT_CTIN4     (0 << GIMA_SELECT_SHIFT) /* CTIN_4*/
#  define GIMA_CTIN4_SELECT_U0RX      (1 << GIMA_SELECT_SHIFT) /* USART0 RX active */
#  define GIMA_CTIN4_SELECT_I2S1RX    (2 << GIMA_SELECT_SHIFT) /* I2S1_RX_MWS1 */
#  define GIMA_CTIN4_SELECT_I2S1TX    (3 << GIMA_SELECT_SHIFT) /* I2S1_TX_MWS1 */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_5 capture input multiplexer (GIMA output 21) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN5_SELECT_CTIN5     (0 << GIMA_SELECT_SHIFT) /* CTIN_5 */
#  define GIMA_CTIN5_SELECT_U2RX      (1 << GIMA_SELECT_SHIFT) /* USART2 RX active */
#  define GIMA_CTIN5_SELECT_SGPIO12D  (2 << GIMA_SELECT_SHIFT) /* SGPIO12_DIV */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_6 capture input multiplexer (GIMA output 22) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN6_SELECT_CTIN6     (0 << GIMA_SELECT_SHIFT) /* CTIN_6 */
#  define GIMA_CTIN6_SELECT_U3TX      (1 << GIMA_SELECT_SHIFT) /* USART3 TX active */
#  define GIMA_CTIN6_SELECT_I2S0RX    (2 << GIMA_SELECT_SHIFT) /* I2S0_RX_MWS */
#  define GIMA_CTIN6_SELECT_I2S0TX    (3 << GIMA_SELECT_SHIFT) /* I2S0_TX_MWS */
                                                /* Bits 8-31: Reserved */
/* SCT CTIN_7 capture input multiplexer (GIMA output 23) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_CTIN7_SELECT_CTIN7     (0 << GIMA_SELECT_SHIFT) /* CTIN_7 */
#  define GIMA_CTIN7_SELECT_U3RX      (1 << GIMA_SELECT_SHIFT) /* USART3 RX active */
#  define GIMA_CTIN7_SELECT_SOF0      (2 << GIMA_SELECT_SHIFT) /* SOF0 (Start-Of-Frame USB0) */
#  define GIMA_CTIN7_SELECT_SOF1      (3 << GIMA_SELECT_SHIFT) /* SOF1 (Start-Of-Frame USB1) */
                                                /* Bits 8-31: Reserved */
/* VADC trigger input multiplexer (GIMA output 24) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_VADC_SELECT_GPIO6p28   (0 << GIMA_SELECT_SHIFT) /* GPIO6[28] */
#  define GIMA_VADC_SELECT_GPIO5p3    (1 << GIMA_SELECT_SHIFT) /* GPIO5[3] */
#  define GIMA_VADC_SELECT_SGPIO10    (2 << GIMA_SELECT_SHIFT) /* SGPIO10 */
#  define GIMA_VADC_SELECT_SGPIO12    (3 << GIMA_SELECT_SHIFT) /* SGPIO12 */
#  define GIMA_VADC_SELECT_MCOB2      (5 << GIMA_SELECT_SHIFT) /* MCOB2 */
#  define GIMA_VADC_SELECT_CTOUT0     (6 << GIMA_SELECT_SHIFT) /* CTOUT_0 or T0_MAT0 */
#  define GIMA_VADC_SELECT_CTOUT8     (7 << GIMA_SELECT_SHIFT) /* CTOUT_8 or T2_MAT0 */
#  define GIMA_VADC_SELECT_T0MAT0     (8 << GIMA_SELECT_SHIFT) /* T0_MAT0 */
#  define GIMA_VADC_SELECT_T2MAT0     (9 << GIMA_SELECT_SHIFT) /* T2_MAT0 */
                                                /* Bits 8-31: Reserved */
/* Event router input 13 multiplexer (GIMA output 25) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_EVNTRTR_SELECT_CTOUT2  (0 << GIMA_SELECT_SHIFT) /* CTOUT_2 or T0_MAT2 */
#  define GIMA_EVNTRTR_SELECT_SGPIO3  (1 << GIMA_SELECT_SHIFT) /* SGPIO3 */
#  define GIMA_EVNTRTR_SELECT_T0MAT2  (2 << GIMA_SELECT_SHIFT) /* T0_MAT2 */
                                                /* Bits 8-31: Reserved */
/* Event router input 14 multiplexer (GIMA output 26) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_EVNTRTR_SELECT_CTOUT6  (0 << GIMA_SELECT_SHIFT) /* CTOUT_6 or T1_MAT2 */
#  define GIMA_EVNTRTR_SELECT_SGPIO12 (1 << GIMA_SELECT_SHIFT) /* SGPIO12 */
#  define GIMA_EVNTRTR_SELECT_T1MAT2  (2 << GIMA_SELECT_SHIFT) /* T1_MAT2 */
                                                /* Bits 8-31: Reserved */
/* Event router input 16 multiplexer (GIMA output 27) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_EVNTRTR_SELECT_CTOUT14 (0 << GIMA_SELECT_SHIFT) /* CTOUT_14 or T3_MAT2 */
#  define GIMA_EVNTRTR_SELECT_T3MAT2  (1 << GIMA_SELECT_SHIFT) /* T3_MAT2 */
                                                /* Bits 8-31: Reserved */
/* ADC start0 input multiplexer (GIMA output 28) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_ADC0_SELECT_CTOUT15    (0 << GIMA_SELECT_SHIFT) /* CTOUT_15 or T3_MAT3 */
#  define GIMA_ADC0_SELECT_T3MAT2     (1 << GIMA_SELECT_SHIFT) /* T3_MAT2 */
                                                /* Bits 8-31: Reserved */
/* ADC start1 input multiplexer (GIMA output 29) */
                                                /* Bits 4-7:  Same as the common definitions */
#  define GIMA_ADC1_SELECT_CTOUT8     (0 << GIMA_SELECT_SHIFT) /* CTOUT_8 or T2_MAT0 */
#  define GIMA_ADC1_SELECT_T2MAT0     (1 << GIMA_SELECT_SHIFT) /* T2_MAT0 */
                                                /* Bits 8-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_GIMA_H */
