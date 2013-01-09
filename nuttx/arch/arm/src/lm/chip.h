/************************************************************************************
 * arch/arm/src/lm/chip.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_H
#define __ARCH_ARM_SRC_LM_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/lm/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Then get all of the register definitions */

#include "chip/lm_memorymap.h"  /* Memory map */
#include "chip/lm_syscontrol.h" /* System control module */
#include "chip/lm_gpio.h"       /* GPIO modules */
#include "chip/lm_uart.h"       /* UART modules */
#include "chip/lm_i2c.h"        /* I2C modules */
#include "chip/lm_ssi.h"        /* SSI modules */
#include "chip/lm_ethernet.h"   /* Ethernet MAC and PHY */
#include "chip/lm_flash.h"      /* FLASH */

/* The LM3S69xx only supports 8 priority levels.  The hardware priority mechanism
 * will only look at the upper N bits of the 8-bit priority level (where N is 3 for
 * the Stellaris family), so any prioritization must be performed in those bits.
 * The default priority level is set to the middle value
 */

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_H */
