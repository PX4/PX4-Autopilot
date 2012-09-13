/************************************************************************************
 * arch/arm/src/kinetis/kinetis_gpio.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_GPIO_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_GPIO_PDOR_OFFSET 0x0000 /* Port Data Output Register */
#define KINETIS_GPIO_PSOR_OFFSET 0x0004 /* Port Set Output Register */
#define KINETIS_GPIO_PCOR_OFFSET 0x0008 /* Port Clear Output Register */
#define KINETIS_GPIO_PTOR_OFFSET 0x000c /* Port Toggle Output Register */
#define KINETIS_GPIO_PDIR_OFFSET 0x0010 /* Port Data Input Register */
#define KINETIS_GPIO_PDDR_OFFSET 0x0014 /* Port Data Direction Register */

/* Register Addresses ***************************************************************/

#define KINETIS_GPIO_PDOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIO_PSOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIO_PCOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIO_PTOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIO_PDIR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIO_PDDR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOA_PDOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOA_PSOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOA_PCOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOA_PTOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOA_PDIR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOA_PDDR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOB_PDOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOB_PSOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOB_PCOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOB_PTOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOB_PDIR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOB_PDDR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOC_PDOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOC_PSOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOC_PCOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOC_PTOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOC_PDIR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOC_PDDR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOD_PDOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOD_PSOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOD_PCOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOD_PTOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOD_PDIR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOD_PDDR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOE_PDOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOE_PSOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOE_PCOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOE_PTOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOE_PDIR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOE_PDDR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDDR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Port Data Output Register */

#define GPIO_PDOR(n)            (1 << (n))

/* Port Set Output Register */

#define GPIO_PSOR(n)            (1 << (n))

/* Port Clear Output Register */

#define GPIO_PCOR(n)            (1 << (n))

/* Port Toggle Output Register */

#define GPIO_PTOR(n)            (1 << (n))

/* Port Data Input Register */

#define GPIO_PDIR(n)            (1 << (n))

/* Port Data Direction Register */

#define GPIO_PDDR(n)            (1 << (n))

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_GPIO_H */
