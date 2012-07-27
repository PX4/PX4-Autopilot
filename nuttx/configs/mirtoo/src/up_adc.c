/****************************************************************************
 * config/mirtoo/src/up_adc.c
 * arch/arm/src/board/up_adc.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "pic32mx-internal.h"
#include "mirtoo-internal.h"

#ifdef CONFIG_PIC32MX_ADC

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The Mirtoo features a PGA117 amplifier/multipexer that can be configured to
 * bring any analog signal from PORT0,.. PORT7 to pin 19 of the PIC32MX:
 *
 * --- ------------------------------------------------ ----------------------------
 * PIN PIC32 SIGNAL(s)                                  BOARD SIGNAL/USAGE
 * --- ------------------------------------------------ ----------------------------
 * 19  PGED3/VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/PMD7/RA0 AIN PGA117 Vout
  --- ------------------------------------------------ ----------------------------
 *
 * The PGA117 driver can be enabled by setting the following the the nsh
 * configuration:
 *
 *   CONFIG_ADC=y         : Enable support for analog input devices
 *   CONFIG_PIC32MX_ADC=y : Enable support the PIC32 ADC driver
 *   CONFIG_SPI_OWNBUS=n  : The PGA117 is *not* the only device on the bus
 *   CONFIG_ADC_PGA11X=y  : Enable support for the PGA117
 *
 * When CONFIG_PIC32MX_ADC=y is defined, the Mirtoo boot up logic will automatically
 * configure pin 18 (AN0) as an analog input.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_adcinitialize
 *
 * Description:
 *   Perform architecture specific ADC initialization
 *
 ****************************************************************************/

#if 0 /* Not used */
int pic32mx_adcinitialize(void)
{
  /* Configure the pin 19 as an analog input */
#warning "Missing logic"

  /* Initialize the PGA117 amplifier multiplexer */
#warning "Missing logic"

  /* Register the ADC device driver */
#warning "Missing logic"

  return OK;
}
#endif

#endif /* CONFIG_PIC32MX_ADC */