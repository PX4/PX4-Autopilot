/****************************************************************************
 * arch/arm/src/stm32/stm32_lse.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
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

#include "up_arch.h"

#include "stm32_rcc.h"
#include "stm32_waste.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) Oscillator and, if the RTC is
 *   configured, setup the LSE as the RTC clock source, and enable the RTC.
 *
 * Todo:
 *   Check for LSE good timeout and return with -1,
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void)
{
  /* Enable the External Low-Speed (LSE) Oscillator by setting the LSEON bit
   * the RCC BDCR register.
   */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_LSEON);

  /* Wait for the LSE clock to be ready */

  while ((getreg16(STM32_RCC_BDCR) & RCC_BDCR_LSERDY) == 0)
    {
      up_waste();
    }
    
  /* The primariy purpose of the LSE clock is to drive the RTC.  The RTC could
   * also be driven by the LSI (but that would be very inaccurate) or by the
   * HSE (but that would prohibit low-power operation)
   *
   * Select LSE as RTC Clock Source by setting the RTCSEL field of the RCC BDCR
   * register.
   */

#ifdef CONFIG_RTC
  modifyreg16(STM32_RCC_BDCR, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RTCSEL_LSE);

  /* Enable the RTC Clock by setting the RTCEN bit in the RCC BDCR register */

  modifyreg16(STM32_RCC_BDCR, 0, RCC_BDCR_RTCEN);    
#endif
}
