/************************************************************************************
 * arch/arm/src/str71x/str71x_apb.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_APB_H
#define __ARCH_ARM_SRC_STR71X_STR71X_APB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* APB register offsets *************************************************************/

#define STR71X_APB_CKDIS_OFFSET   (0x0010) /* 32-bits wide */
#define STR71X_APB_SWRES_OFFSET   (0x0014) /* 32-bits wide */

/* APB register addresses ***********************************************************/

#define STR71X_APB1_CKDIS         (STR71X_APB1_BASE + STR71X_APB_CKDIS_OFFSET)
#define STR71X_APB1_SWRES         (STR71X_APB1_BASE + STR71X_APB_SWRES_OFFSET)

#define STR71X_APB2_CKDIS         (STR71X_APB2_BASE + STR71X_APB_CKDIS_OFFSET)
#define STR71X_APB2_SWRES         (STR71X_APB2_BASE + STR71X_APB_SWRES_OFFSET)

/* Register bit settings ***********************************************************/

/* APB1 periperals */

#define STR71X_APB1_I2C0          (0x0001) /* Bit 0:  I2C0 */
#define STR71X_APB1_I2C1          (0x0002) /* Bit 1:  I2C1 */
#define STR71X_APB1_UART0         (0x0008) /* Bit 3:  UART0 */
#define STR71X_APB1_UART1         (0x0010) /* Bit 4:  UART1 */
#define STR71X_APB1_UART2         (0x0020) /* Bit 5:  UART2 */
#define STR71X_APB1_UART3         (0x0040) /* Bit 6:  UART3 */
#define STR71X_APB1_USB           (0x0080) /* Bit 7:  USB */
#define STR71X_APB1_CAN           (0x0100) /* Bit 8:  CAN */
#define STR71X_APB1_BSPI0         (0x0200) /* Bit 9:  BSPI0 */
#define STR71X_APB1_BSPI1         (0x0400) /* Bit 10: BSPI1 */
#define STR71X_APB1_HDLC          (0x2000) /* Bit 13: HDLC */
#define STR71X_APB1_APB1ALL       (0x27fb)

/* APB2 Peripherals */

#define STR71X_APB2_XTI           (0x0001) /* Bit 0:  XTI */
#define STR71X_APB2_GPIO0         (0x0004) /* Bit 2:  IOPORT0 */
#define STR71X_APB2_GPIO1         (0x0008) /* Bit 3:  IOPORT1 */
#define STR71X_APB2_GPIO2         (0x0010) /* Bit 4:  IOPORT2 */
#define STR71X_APB2_ADC12         (0x0040) /* Bit 6:  ADC */
#define STR71X_APB2_CKOUT         (0x0080) /* Bit 7:  CKOUT */
#define STR71X_APB2_TIM0          (0x0100) /* Bit 8:  TIMER0 */
#define STR71X_APB2_TIM1          (0x0200) /* Bit 9:  TIMER1 */
#define STR71X_APB2_TIM2          (0x0400) /* Bit 10: TIMER2 */
#define STR71X_APB2_TIM3          (0x0800) /* Bit 11: TIMER3 */
#define STR71X_APB2_RTC           (0x1000) /* Bit 12: RTC */
#define STR71X_APB2_EIC           (0x4000) /* Bit 14: EIC */
#define STR71X_APB2_APB2ALL       (0x5fdd)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_APB_H */
