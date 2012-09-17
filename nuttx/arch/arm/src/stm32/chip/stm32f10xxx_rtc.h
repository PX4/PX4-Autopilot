/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f10xxx_rtc.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RTC_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RTC_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_RTC_CRH_OFFSET    0x0000    /* RTC control register High (16-bit) */
#define STM32_RTC_CRL_OFFSET    0x0004    /* RTC control register low (16-bit) */
#define STM32_RTC_PRLH_OFFSET   0x0008    /* RTC prescaler load register high (16-bit) */ 
#define STM32_RTC_PRLL_OFFSET   0x000c    /* RTC prescaler load register low (16-bit) */
#define STM32_RTC_DIVH_OFFSET   0x0010    /* RTC prescaler divider register high (16-bit) */
#define STM32_RTC_DIVL_OFFSET   0x0014    /* RTC prescaler divider register low (16-bit) */
#define STM32_RTC_CNTH_OFFSET   0x0018    /* RTC counter register high (16-bit) */
#define STM32_RTC_CNTL_OFFSET   0x001c    /* RTC counter register low (16-bit) */
#define STM32_RTC_ALRH_OFFSET   0x0020    /* RTC alarm register high (16-bit) */
#define STM32_RTC_ALRL_OFFSET   0x0024    /* RTC alarm register low (16-bit) */

/* Register Addresses ***************************************************************/

#define STM32_RTC_CRH           (STM32_RTC_BASE+STM32_RTC_CRH_OFFSET)
#define STM32_RTC_CRL           (STM32_RTC_BASE+STM32_RTC_CRL_OFFSET)
#define STM32_RTC_PRLH          (STM32_RTC_BASE+STM32_RTC_PRLH_OFFSET)
#define STM32_RTC_PRLL          (STM32_RTC_BASE+STM32_RTC_PRLL_OFFSET)
#define STM32_RTC_DIVH          (STM32_RTC_BASE+STM32_RTC_DIVH_OFFSET)
#define STM32_RTC_DIVL          (STM32_RTC_BASE+STM32_RTC_DIVL_OFFSET)
#define STM32_RTC_CNTH          (STM32_RTC_BASE+STM32_RTC_CNTH_OFFSET)
#define STM32_RTC_CNTL          (STM32_RTC_BASE+STM32_RTC_CNTL_OFFSET)
#define STM32_RTC_ALRH          (STM32_RTC_BASE+STM32_RTC_ALRH_OFFSET)
#define STM32_RTC_ALRL          (STM32_RTC_BASE+STM32_RTC_ALRL_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* RTC control register High (16-bit) */

#define RTC_CRH_SECIE           (1 << 0)  /* Bit 0 : Second Interrupt Enable */
#define RTC_CRH_ALRIE           (1 << 1)  /* Bit 1: Alarm Interrupt Enable */
#define RTC_CRH_OWIE            (1 << 2)  /* Bit 2: OverfloW Interrupt Enable */

/* RTC control register low (16-bit) */

#define RTC_CRL_SECF            (1 << 0)  /* Bit 0: Second Flag */
#define RTC_CRL_ALRF            (1 << 1)  /* Bit 1: Alarm Flag */
#define RTC_CRL_OWF             (1 << 2)  /* Bit 2: Overflow Flag */
#define RTC_CRL_RSF             (1 << 3)  /* Bit 3: Registers Synchronized Flag */
#define RTC_CRL_CNF             (1 << 4)  /* Bit 4: Configuration Flag */
#define RTC_CRL_RTOFF           (1 << 5)  /* Bit 5: RTC operation OFF */

/* RTC prescaler load register high (16-bit) */

#define RTC_PRLH_PRL_SHIFT      (0)      /* Bits 3-0: RTC Prescaler Reload Value High */
#define RTC_PRLH_PRL_MASK       (0x0f << RTC_PRLH_PRL_SHIFT)

/* RTC prescaler divider register high (16-bit) */

#define RTC_DIVH_RTC_DIV_SHIFT  (0)      /* Bits 3-0: RTC Clock Divider High */
#define RTC_DIVH_RTC_DIV_MASK   (0x0f << RTC_DIVH_RTC_DIV_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RTC_H */
