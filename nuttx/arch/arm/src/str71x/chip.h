/************************************************************************************
 * arch/arm/src/str71x/chip.h
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

#ifndef __ARCH_ARM_SRC_STR71X_CHIP_H
#define __ARCH_ARM_SRC_STR71X_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"     /* Memory map */
#include "str71x_emi.h"     /* External memory interface */
#include "str71x_rccu.h"    /* Reset and clock control unit */
#include "str71x_pcu.h"     /* Power control unit */
#include "str71x_gpio.h"    /* I/O ports */
#include "str71x_eic.h"     /* Enhanced interrupt controller */
#include "str71x_xti.h"     /* External interrupts (XTI) */
#include "str71x_rtc.h"     /* Real Time Clock (RTC) */
#include "str71x_wdog.h"    /* Watchdog timer */
#include "str71x_timer.h"   /* Timers */
#include "str71x_can.h"     /* Controller Area Network (CAN) */
#include "str71x_i2c.h"     /* I2C */
#include "str71x_bspi.h"    /* Buffered SPI (BSPI) */
#include "str71x_uart.h"    /* UART */
#include "str71x_usb.h"     /* USB */
#include "str71x_adc12.h"   /* ADC */
#include "str71x_apb.h"     /* USB */
#include "str71x_flash.h"   /* Flash */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_CHIP_H */
