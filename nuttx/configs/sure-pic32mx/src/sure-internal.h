/****************************************************************************
 * configs/sure-pic32mx/src/sure-internal.h
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
 ****************************************************************************/

#ifndef __CONFIGS_SURE_PIC32MX_SRC_SURE_INTERNAL_H
#define __CONFIGS_SURE_PIC32MX_SRC_SURE_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: pic32mx_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX Logic board.
 *
 ************************************************************************************/

#if defined(CONFIG_PIC32MX_SPI2)
EXTERN void weak_function pic32mx_spiinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mx_usdbinitialize
 *
 * Description:
 *   Called to configure the mini-B PHY on the Sure PIC32MX board for the USB device
 *
 ************************************************************************************/

#if defined(CONFIG_PIC32MX_USBDEV)
EXTERN void weak_function pic32mx_usdbinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mx_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
EXTERN void pic32mx_ledinit(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SURE_PIC32MX_SRC_SURE_INTERNAL_H */
