/************************************************************************************
 * arch/arm/include/lm/chip.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jose Pablo Carballo <jcarballo@nx-engineering.com>
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

#ifndef __ARCH_ARM_INCLUDE_LM_CHIP_H
#define __ARCH_ARM_INCLUDE_LM_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip (only the LM3S6918 and 65 right now) */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
#  define LM3S               1  /* LM3S family */
#  undef  LM4F                  /* Not LM4F family */
#  define LM_NTIMERS         4  /* Four general purpose timers */
#  define LM_NWIDETIMERS     0  /* No general purpose wide timers */
#  define LM_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM_ETHTS              /* No timestamp register */
#  define LM_NSSI            2  /* Two SSI modules */
#  define LM_NUARTS          2  /* Two UART modules */
#  define LM_NI2C            2  /* Two I2C modules */
#  define LM_NADC            1  /* One ADC module */
#  define LM_NPWM            0  /* No PWM generator modules */
#  define LM_NQEI            0  /* No quadrature encoders */
#  define LM_NPORTS          8  /* 8 Ports (GPIOA-H) 5-38 GPIOs */
#  define LM_NCANCONTROLLER  0  /* No CAN controllers */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
#  define LM3S               1  /* LM3S family */
#  undef  LM4F                  /* Not LM4F family */
#  define LM_NTIMERS         3  /* Three general purpose timers */
#  define LM_NWIDETIMERS     0  /* No general purpose wide timers */
#  define LM_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM_ETHTS              /* No timestamp register */
#  define LM_NSSI            1  /* One SSI module */
#  define LM_NUARTS          2  /* Two UART modules */
#  define LM_NI2C            1  /* Two I2C modules */
#  define LM_NADC            1  /* One ADC module */
#  define LM_NPWM            1  /* One PWM generator module */
#  define LM_NQEI            0  /* No quadrature encoders */
#  define LM_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#  define LM_NCANCONTROLLER  0  /* No CAN controllers */
#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
#  define LM3S               1  /* LM3S family */
#  undef  LM4F                  /* Not LM4F family */
#  define LM_NTIMERS         4  /* Four general purpose timers */
#  define LM_NWIDETIMERS     0  /* No general purpose wide timers */
#  define LM_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM_ETHTS              /* No timestamp register */
#  define LM_NSSI            1  /* One SSI module */
#  define LM_NUARTS          3  /* Three UART modules */
#  define LM_NI2C            2  /* Two I2C modules */
#  define LM_NADC            1  /* One ADC module */
#  define LM_NPWM            3  /* Three PWM generator modules */
#  define LM_NQEI            2  /* Two quadrature encoders */
#  define LM_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#  define LM_NCANCONTROLLER  0  /* No CAN controllers */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96) 
#  define LM3S               1  /* LM3S family */
#  undef  LM4F                  /* Not LM4F family */
#  define LM_NTIMERS         4  /* Four general purpose timers */
#  define LM_NWIDETIMERS     0  /* No general purpose wide timers */
#  define LM_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM_ETHTS              /* No timestamp register */
#  define LM_NSSI            2  /* Two SSI modules */
#  define LM_NUARTS          3  /* Three UART modules */
#  define LM_NI2C            2  /* Two I2C modules */
#  define LM_NADC            2  /* Two ADC module */
#  define LM_CAN             2  /* Two CAN module */
#  define LM_NPWM            4  /* Four PWM generator modules */
#  define LM_NQEI            2  /* Two quadrature encoders */
#  define LM_NPORTS          9  /* 9 Ports (GPIOA-H,J) 0-65 GPIOs */
#  define LM_NCANCONTROLLER  0  /* No CAN controllers */
#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define LM3S               1  /* LM3S family */
#  undef  LM4F                  /* Not LM4F family */
#  define LM_NTIMERS         6  /* Four general purpose timers */
#  define LM_NWIDETIMERS     0  /* No general purpose wide timers */
#  define LM_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LM_NSSI            1  /* One SSI module */
#  define LM_NUARTS          3  /* Two UART modules */
#  define LM_NI2C            2  /* One I2C module */
#  define LM_NADC            1  /* One ADC module */
#  define LM_NPWM            3  /* Three PWM generator modules */
#  define LM_NQEI            2  /* Two quadrature encoders */
#  define LM_NPORTS          7  /* 7 Ports (GPIOA-G), 5-42 GPIOs */
#  define LM_NCANCONTROLLER  1  /* One CAN controller */
#elif defined(CONFIG_ARCH_CHIP_LM4F120)
#  undef  LM3S                  /* Not LM3S family */
#  define LM4F               1  /* LM4F family */
#  define LM_NTIMERS         6  /* Six general purpose timers */
#  define LM_NWIDETIMERS     6  /* Six general purpose wide timers */
#  define LM_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LM_NSSI            4  /* Four SSI module */
#  define LM_NUARTS          8  /* Eight UART modules */
#  define LM_NI2C            4  /* Four I2C modules */
#  define LM_NADC            2  /* Two ADC modules */
#  define LM_NPWM            0  /* No PWM generator modules */
#  define LM_NQEI            0  /* No quadrature encoders */
#  define LM_NPORTS          6  /* 6 Ports (GPIOA-F), 0-43 GPIOs */
#  define LM_NCANCONTROLLER  1  /* One CAN controller */
#else
#  error "Capabilities not specified for this Stellaris chip"
#endif

/* The LM3S69xx only supports 8 priority levels.  The hardware priority mechanism
 * will only look at the upper N bits of the 8-bit priority level (where N is 3 for
 * the Stellaris family), so any prioritization must be performed in those bits.
 * The default priority level is set to the middle value
 */

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* Bits [5:7] set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x20 /* Three bits of interrupt priority used */

#define NVIC_SYSH_DISABLE_PRIORITY (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_SVCALL_PRIORITY  NVIC_SYSH_PRIORITY_MAX

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_LM_CHIP_H */
