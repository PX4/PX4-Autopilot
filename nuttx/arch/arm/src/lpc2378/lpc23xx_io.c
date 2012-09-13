/***********************************************************************
 * arch/arm/src/arm/lpc2378/lpc23xx_head.S
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ***********************************************************************/

/***********************************************************************
 * Included files
 ***********************************************************************/

#include "up_arch.h"
#include <sys/types.h>
#include "lpc23xx_scb.h"
#include "lpc23xx_pinsel.h"
#include "lpc23xx_uart.h"
#include "lpc23xx_gpio.h"

/***********************************************************************
 * Definitions
 ***********************************************************************/

/***********************************************************************
* Name: IO_Init()
*
* Descriptions: Initialize the target board before running the main() 
*
************************************************************************/
void IO_Init( void )
{
  uint32_t regval;

  /* Reset all GPIO pins to default */

  pinsel_putreg(0, PINSEL0_OFFSET);
  pinsel_putreg(0, PINSEL1_OFFSET);
  pinsel_putreg(0, PINSEL2_OFFSET);
  pinsel_putreg(0, PINSEL3_OFFSET);
  pinsel_putreg(0, PINSEL4_OFFSET);
  pinsel_putreg(0, PINSEL5_OFFSET);
  pinsel_putreg(0, PINSEL6_OFFSET);
  pinsel_putreg(0, PINSEL7_OFFSET);
  pinsel_putreg(0, PINSEL8_OFFSET);
  pinsel_putreg(0, PINSEL9_OFFSET);
  pinsel_putreg(0, PINSEL10_OFFSET);

/*
  regval = scb_getreg(SCB_PCONP_OFFSET) & ~(PCSDC | PCUART1 | PCI2C0 | PCSSP1 | PCEMC | );
  scb_getreg(regval, SCB_PCONP_OFFSET );
*/

  /* Turn off all peripheral power */

  scb_putreg(0, SCB_PCONP_OFFSET );
        
  /* Turn on UART0/2 / Timer0 */
  /* regval = PCUART0 | PCUART2 | PCTIM0 | PCRTC ; */

  regval = PCUART0 | PCUART2 | PCTIM0 ;
  scb_putreg(regval , SCB_PCONP_OFFSET );        
        
  /* Status LED P1.19 */

  dir_putreg8((1 << 3), FIO1DIR2_OFFSET);

  /* other io setup here */

  return;        
}
