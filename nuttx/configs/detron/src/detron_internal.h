/************************************************************************************
 * configs/detron/src/detron_internal.h
 * arch/arm/src/board/detron_internal.n
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

/************************************************************************************
 *
 * Pin used in the Internet Radio Detron Board
 *
 * Graphic Display
 *
 * Pin 	Port	Function
 * 58   	P0(20)	DI
 * 59   	P0(19)	RW
 * 49    P0(11)  ENABLE
 * 78    P0(7)   D0
 * 79    P0(6)   D1
 * 78    P0(5)   D2
 * 81    P0(4)   D3
 * 94    P1(1)   D4
 * 95    P1(0)   D5
 * 47    P0(1)   D6
 * 46    P0(0)   D7
 *
 * VS1003
 *
 * Pin	Port	Function
 * 65	P2(8)	xreset
 * 85	P4(29)	dreq
 * 82	P4(28)	xdcs
 * 63	P0(16)	xcs
 * 62	P0(15)	sclk
 * 60	P0(18)	si
 * 61	P0(17)	so
 *
 * Botoes do painel
 *
 * 85	Chave pulsada 1  		P4(29)
 * 87	Chave Pulsada 2 	 	P1(16)
 * 88   Chave pulsada centro	P1(15)
 *
 * USB
 *
 * Pin	Port	Function
 * 29			D+
 * 30			D-
 */

#ifndef _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H
#define _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <arch/lpc17xx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Detron GPIO Pin Definitions ******************************************************/
/* Pinos do Display grafico */

#define pin_di              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN20)
#define pin_rw              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN19)
#define pin_enable          (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN11)
#define pin_d0              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN7)
#define pin_d1              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN6)
#define pin_d2              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN5)
#define pin_d3              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN4)
#define pin_d4              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN1)
#define pin_d5              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT1 | GPIO_PIN0)
#define pin_d6              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN1)
#define pin_d7              (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN0)
#define pin_cs1             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN10)
#define pin_cs2             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN9)
#define pin_rst             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN8)

/* VS1003 pins
 * xdcs = SPI in mode SDI (data)
 * xcs =  SPI in mode SCI (comand)
 *
 * sclk = SPI clock
 * si   = SPI in
 * so   = SPI out
 *
 * dreq = SPI status   1 - free   0 - busy
 * xreset = hardware reset
 */

#define pin_vs1003_xreset   (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN8)
#define pin_vs1003_dreq     (GPIO_INPUT  | GPIO_VALUE_ZERO | GPIO_PORT4 | GPIO_PIN29)
#define pin_vs1003_xdcs     (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT4 | GPIO_PIN28)
#define pin_vs1003_xcs      (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN16)
#define pin_vs1003_sclk     (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN15)
#define pin_vs1003_si       (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN18)
#define pin_vs1003_so       (GPIO_INPUT  | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN17)

/* Pinos do painel  */

#define pin_panel_center		  		(GPIO_INPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN3)
#define pin_panel_pulse_up   			(GPIO_INPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN4)
#define pin_panel_pulse_down	   		(GPIO_INPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN5)

#define IRQ_pin_panel_center 			LPC17_IRQ_P2p3
#define IRQ_pin_panel_pulse_up   		LPC17_IRQ_P2p4
#define IRQ_pin_panel_pulse_down   		LPC17_IRQ_P2p5

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
 
#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_DETRON_SRC_DETRON_INTERNAL_H */

