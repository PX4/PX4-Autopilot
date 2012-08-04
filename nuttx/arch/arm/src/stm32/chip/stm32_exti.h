/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_exti.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_STM32F10XX)
#  ifdef CONFIG_STM32_CONNECTIVITYLINE
#    define STM32_NEXTI          20
#    define STM32_EXTI_MASK      0x000fffff
#  else
#    define STM32_NEXTI          19
#    define STM32_EXTI_MASK      0x0007ffff
#  endif
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_NEXTI            23
#  define STM32_EXTI_MASK        0x007fffff
#endif

#define STM32_EXTI_BIT(n)        (1 << (n))

/* Register Offsets *****************************************************************/

#define STM32_EXTI_IMR_OFFSET    0x0000  /* Interrupt mask register */
#define STM32_EXTI_EMR_OFFSET    0x0004  /* Event mask register */
#define STM32_EXTI_RTSR_OFFSET   0x0008  /* Rising Trigger selection register */
#define STM32_EXTI_FTSR_OFFSET   0x000c  /* Falling Trigger selection register */
#define STM32_EXTI_SWIER_OFFSET  0x0010  /* Software interrupt event register */
#define STM32_EXTI_PR_OFFSET     0x0014  /* Pending register */

/* Register Addresses ***************************************************************/

#define STM32_EXTI_IMR          (STM32_EXTI_BASE+STM32_EXTI_IMR_OFFSET)
#define STM32_EXTI_EMR          (STM32_EXTI_BASE+STM32_EXTI_EMR_OFFSET)
#define STM32_EXTI_RTSR         (STM32_EXTI_BASE+STM32_EXTI_RTSR_OFFSET)
#define STM32_EXTI_FTSR         (STM32_EXTI_BASE+STM32_EXTI_FTSR_OFFSET)
#define STM32_EXTI_SWIER        (STM32_EXTI_BASE+STM32_EXTI_SWIER_OFFSET)
#define STM32_EXTI_PR           (STM32_EXTI_BASE+STM32_EXTI_PR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* EXTI lines > 15 are associated with internal devices: */

#if defined(CONFIG_STM32_STM32F10XX)
#  define EXTI_PVD_LINE          (1 << 16) /* EXTI line 16 is connected to the PVD output */
#  define EXTI_RTC_ALARM         (1 << 17) /* EXTI line 17 is connected to the RTC Alarm event */
#  define EXTI_USB_WAKEUP        (1 << 18) /* EXTI line 18 is connected to the USB Wakeup event */
#  ifdef CONFIG_STM32_CONNECTIVITYLINE
#    define EXTI_ETH_WAKEUP      (1 << 19) /* EXTI line 19 is connected to the Ethernet Wakeup event */
#  endif
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define EXTI_PVD_LINE          (1 << 16) /* EXTI line 16 is connected to the PVD output */
#  define EXTI_RTC_ALARM         (1 << 17) /* EXTI line 17 is connected to the RTC Alarm event */
#  define EXTI_OTGFS_WAKEUP      (1 << 18) /* EXTI line 18 is connected to the USB OTG FS Wakeup event */
#  define EXTI_ETH_WAKEUP        (1 << 19) /* EXTI line 19 is connected to the Ethernet Wakeup event */
#  define EXTI_OTGHS_WAKEUP      (1 << 20) /* EXTI line 20 is connected to the USB OTG HS Wakeup event */
#  define EXTI_RTC_TAMPER        (1 << 21) /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_TIMESTAMP     (1 << 22) /* EXTI line 21 is connected to the RTC Tamper and TimeStamp events */
#  define EXTI_RTC_WAKEUP        (1 << 23) /* EXTI line 22 is connected to the RTC Wakeup event */
#endif

/* Interrupt mask register */

#define EXTI_IMR_BIT(n)          STM32_EXTI_BIT(n) /* 1=Interrupt request from line x is not masked */
#define EXTI_IMR_SHIFT           (0)               /* Bits 0-X: Interrupt Mask for all lines */
#define EXTI_IMR_MASK            STM32_EXTI_MASK

/* Event mask register */

#define EXTI_EMR_BIT(n)          STM32_EXTI_BIT(n) /* 1=Event request from line x is not mask */
#define EXTI_EMR_SHIFT           (0)               /* Bits Bits 0-X:  Event Mask for all lines */
#define EXTI_EMR_MASK            STM32_EXTI_MASK

/* Rising Trigger selection register */

#define EXTI_RTSR_BIT(n)         STM32_EXTI_BIT(n) /* 1=Rising trigger enabled (for Event and Interrupt) for input line */
#define EXTI_RTSR_SHIFT          (0)               /* Bits 0-X: Rising trigger event configuration bit for all lines */
#define EXTI_RTSR_MASK           STM32_EXTI_MASK

/* Falling Trigger selection register */

#define EXTI_FTSR_BIT(n)         STM32_EXTI_BIT(n)  /* 1=Falling trigger enabled (for Event and Interrupt) for input line */
#define EXTI_FTSR_SHIFT          (0)                /* Bits 0-X: Falling trigger event configuration bitfor all lines */
#define EXTI_FTSR_MASK           STM32_EXTI_MASK

/* Software interrupt event register  */

#define EXTI_SWIER_BIT(n)        STM32_EXTI_BIT(n)  /* 1=Sets the corresponding pending bit in EXTI_PR */
#define EXTI_SWIER_SHIFT         (0)                /* Bits 0-X: Software Interrupt for all lines */
#define EXTI_SWIER_MASK          STM32_EXTI_MASK

/* Pending register */

#define EXTI_IMR_BIT(n)          STM32_EXTI_BIT(n)  /* 1=Selected trigger request occurred */
#define EXTI_IMR_SHIFT           (0)                /* Bits 0-X: Pending bit for all lines */
#define EXTI_IMR_MASK            STM32_EXTI_MASK

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_EXTI_H */
