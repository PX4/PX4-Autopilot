/************************************************************************************
 * arch/arm/include/lm3s/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_LM3S_CHIP_H
#define __ARCH_ARM_INCLUDE_LM3S_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip (only the LM3S6918 and 65 right now) */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
#  define LM3S_NTIMERS         4  /* Four general purpose timers */
#  define LM3S_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM3S_ETHTS              /* No timestamp register */
#  define LM3S_NSSI            2  /* Two SSI modules */
#  define LM3S_NUARTS          2  /* Two UART modules */
#  define LM3S_NI2C            2  /* Two I2C modules */
#  define LM3S_NADC            1  /* One ADC module */
#  define LM2S_NPWM            0  /* No PWM generator modules */
#  define LM3S_NQEI            0  /* No quadrature encoders */
#  define LM3S_NPORTS          8  /* 8 Ports (GPIOA-H) 5-38 GPIOs */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
#  define LM3S_NTIMERS         3  /* Three general purpose timers */
#  define LM3S_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM3S_ETHTS              /* No timestamp register */
#  define LM3S_NSSI            1  /* One SSI module */
#  define LM3S_NUARTS          2  /* Two UART modules */
#  define LM3S_NI2C            1  /* Two I2C modules */
#  define LM3S_NADC            1  /* One ADC module */
#  define LM2S_NPWM            1  /* One PWM generator module */
#  define LM3S_NQEI            0  /* No quadrature encoders */
#  define LM3S_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
#  define LM3S_NTIMERS         4  /* Four general purpose timers */
#  define LM3S_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM3S_ETHTS              /* No timestamp register */
#  define LM3S_NSSI            1  /* One SSI module */
#  define LM3S_NUARTS          3  /* Three UART modules */
#  define LM3S_NI2C            2  /* Two I2C modules */
#  define LM3S_NADC            1  /* One ADC module */
#  define LM2S_NPWM            3  /* Three PWM generator modules */
#  define LM3S_NQEI            2  /* Two quadrature encoders */
#  define LM3S_NPORTS          7  /* 7 Ports (GPIOA-G), 0-42 GPIOs */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96) 
#  define LM3S_NTIMERS         4  /* Four general purpose timers */
#  define LM3S_NETHCONTROLLERS 1  /* One Ethernet controller */
#  undef  LM3S_ETHTS              /* No timestamp register */
#  define LM3S_NSSI            2  /* Two SSI modules */
#  define LM3S_NUARTS          3  /* Three UART modules */
#  define LM3S_NI2C            2  /* Two I2C modules */
#  define LM3S_NADC            2  /* Two ADC module */
#  define LM3S_CAN             2  /* Two CAN module */
#  define LM3S_NPWM            4  /* Four PWM generator modules */
#  define LM3S_NQEI            2  /* Two quadrature encoders */
#  define LM3S_NPORTS          9  /* 9 Ports (GPIOA-H,J) 0-65 GPIOs */
#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define LM3S_NTIMERS         4  /* Four general purpose timers */
#  define LM3S_NETHCONTROLLERS 1  /* One Ethernet controller */
#  define LM3S_NSSI            1  /* One SSI module */
#  define LM3S_NUARTS          3  /* Two UART modules */
#  define LM3S_NI2C            2  /* One I2C module */
#  define LM3S_NADC            1  /* One ADC module */
#  define LM2S_NPWM            3  /* Three PWM generator modules */
#  define LM3S_NQEI            2  /* Two quadrature encoders */
#  define LM3S_NPORTS          7  /* 7 Ports (GPIOA-G), 5-42 GPIOs */
#  define LC3S_CANCONTROLLER   1  /* One CAN controller */
#else
#  error "Capabilities not specified for this LM3S chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_LM3S_CHIP_H */
