/************************************************************************************
 * configs/mbed/src/mbed_internal.h
 * arch/arm/src/board/mbed_internal.n
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef _CONFIGS_MBED_SRC_MBED_INTERNAL_H
#define _CONFIGS_MBED_SRC_MBED_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* MBED GPIO Pin Definitions ********************************************************/

#define MBED_LED1             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)
#define MBED_LED1_OFF          MBED_LED1
#define MBED_LED1_ON          (MBED_LED1 | GPIO_VALUE_ONE)
#define MBED_LED2             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN20)
#define MBED_LED2_OFF          MBED_LED2
#define MBED_LED2_ON          (MBED_LED2 | GPIO_VALUE_ONE)
#define MBED_LED3             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN21)
#define MBED_LED3_OFF          MBED_LED3
#define MBED_LED3_ON          (MBED_LED3 | GPIO_VALUE_ONE)
#define MBED_LED4             (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN23)
#define MBED_LED4_OFF         MBED_LED4
#define MBED_LED4_ON          (MBED_LED 4| GPIO_VALUE_ONE)

#define MBED_HEARTBEAT        MBED_LED4

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ************************************************************************************/

extern void weak_function lpc17_sspinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_MBED_SRC_MBED_INTERNAL_H */

