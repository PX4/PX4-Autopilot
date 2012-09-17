/************************************************************************************
 * arch/arm/src/str71x/str71x_map.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_MAP_H
#define __ARCH_ARM_SRC_STR71X_STR71X_MAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Memory Map ***********************************************************************/

#define STR71X_FLASHRAMEMI_BASE (0x00000000) /* Flash alias for booting */
#define STR71X_RAM_BASE         (0x20000000)
#define STR71X_FLASH_BASE       (0x40000000)
#define STR71X_FLASHREG_BASE    (0x40100000)
#define STR71X_EXTMEM_BASE      (0x60000000)
#define STR71X_EMI_BASE         (STR71X_EXTMEM_BASE + 0x0c000000)
#define STR71X_RCCU_BASE        (0xa0000000)
#define STR71X_PCU_BASE         (0xa0000040)
#define STR71X_APB1_BASE        (0xc0000000)
#define STR71X_I2C0_BASE        (STR71X_APB1_BASE + 0x1000)
#define STR71X_I2C1_BASE        (STR71X_APB1_BASE + 0x2000)
#define STR71X_UART0_BASE       (STR71X_APB1_BASE + 0x4000)
#define STR71X_UART1_BASE       (STR71X_APB1_BASE + 0x5000)
#define STR71X_UART2_BASE       (STR71X_APB1_BASE + 0x6000)
#define STR71X_UART3_BASE       (STR71X_APB1_BASE + 0x7000)
#define STR71X_USBRAM_BASE      (STR71X_APB1_BASE + 0x8000)
#define STR71X_USB_BASE         (STR71X_APB1_BASE + 0x8800)
#define STR71X_CAN_BASE         (STR71X_APB1_BASE + 0x9000)
#define STR71X_BSPI0_BASE       (STR71X_APB1_BASE + 0xa000)
#define STR71X_BSPI1_BASE       (STR71X_APB1_BASE + 0xb000)
#define STR71X_HDLCRAM_BASE     (STR71X_APB1_BASE + 0xe000)
#define STR71X_APB2_BASE        (0xe0000000)
#define STR71X_XTI_BASE         (STR71X_APB2_BASE + 0x1000)
#define STR71X_GPIO0_BASE       (STR71X_APB2_BASE + 0x3000)
#define STR71X_GPIO1_BASE       (STR71X_APB2_BASE + 0x4000)
#define STR71X_GPIO2_BASE       (STR71X_APB2_BASE + 0x5000)
#define STR71X_ADC12_BASE       (STR71X_APB2_BASE + 0x7000)
#define STR71X_CLKOUT_BASE      (STR71X_APB2_BASE + 0x8000)
#define STR71X_TIMER0_BASE      (STR71X_APB2_BASE + 0x9000)
#define STR71X_TIMER1_BASE      (STR71X_APB2_BASE + 0xa000)
#define STR71X_TIMER2_BASE      (STR71X_APB2_BASE + 0xb000)
#define STR71X_TIMER3_BASE      (STR71X_APB2_BASE + 0xc000)
#define STR71X_RTC_BASE         (STR71X_APB2_BASE + 0xd000)
#define STR71X_WDOG_BASE        (STR71X_APB2_BASE + 0xe000)
#define STR71X_EIC_BASE         (0xfffff800)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif // __ARCH_ARM_SRC_STR71X_STR71X_MAP_H
