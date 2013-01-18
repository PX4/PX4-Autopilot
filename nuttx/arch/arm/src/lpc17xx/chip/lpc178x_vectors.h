/************************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc178x_vectors.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Rommel Marcelo
 *           Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

/*********************************************************************************
 * Preprocessor Definitions
 ********************************************************************************/
/* This file is included by lpc17_vectors.S.  It provides the macro VECTOR that
 * supplies each LPC17xx vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/lpc17/lpc17xx_irq.h.
 * lpc17_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 41 interrupt table entries for I/O interrupts. */

#  define ARMV7M_PERIPHERAL_INTERRUPTS 41

#else

VECTOR(lpc17_wdt, LPC17_IRQ_WDT)       /* Vector 16+0:  Watchdog timer */
VECTOR(lpc17_tmr0, LPC17_IRQ_TMR0)     /* Vector 16+1:  Timer 0 */
VECTOR(lpc17_tmr1, LPC17_IRQ_TMR1)     /* Vector 16+2:  Timer 1 */
VECTOR(lpc17_tmr2, LPC17_IRQ_TMR2)     /* Vector 16+3:  Timer 2 */
VECTOR(lpc17_tmr3, LPC17_IRQ_TMR3)     /* Vector 16+4:  Timer 3 */
VECTOR(lpc17_uart0, LPC17_IRQ_UART0)   /* Vector 16+5:  UART 0 */
VECTOR(lpc17_uart1, LPC17_IRQ_UART1)   /* Vector 16+6:  UART 1 */
VECTOR(lpc17_uart2, LPC17_IRQ_UART2)   /* Vector 16+7:  UART 2 */
VECTOR(lpc17_uart3, LPC17_IRQ_UART3)   /* Vector 16+8:  UART 3 */
VECTOR(lpc17_pwm1, LPC17_IRQ_PWM1)     /* Vector 16+9:  PWM 1 */
VECTOR(lpc17_i2c0, LPC17_IRQ_I2C0)     /* Vector 16+10: I2C 0 */
VECTOR(lpc17_i2c1, LPC17_IRQ_I2C1)     /* Vector 16+11: I2C 1 */
VECTOR(lpc17_i2c2, LPC17_IRQ_I2C2)     /* Vector 16+12: I2C 2 */
UNUSED(LPC17_IRQ_RESERVED29)           /* Vector 16+13: Reserved */
VECTOR(lpc17_ssp0, LPC17_IRQ_SSP0)     /* Vector 16+14: SSP 0 */
VECTOR(lpc17_ssp1, LPC17_IRQ_SSP1)     /* Vector 16+15: SSP 1 */
VECTOR(lpc17_pll0, LPC17_IRQ_PLL0)     /* Vector 16+16: PLL 0 */
VECTOR(lpc17_rtc, LPC17_IRQ_RTC)       /* Vector 16+17: Real time clock */
VECTOR(lpc17_eint0, LPC17_IRQ_EINT0)   /* Vector 16+18: External interrupt 0 */
VECTOR(lpc17_eint1, LPC17_IRQ_EINT1)   /* Vector 16+19: External interrupt 1 */
VECTOR(lpc17_eint2, LPC17_IRQ_EINT2)   /* Vector 16+20: External interrupt 2 */
VECTOR(lpc17_eint3, LPC17_IRQ_EINT3)   /* Vector 16+21: External interrupt 3 */
VECTOR(lpc17_adc, LPC17_IRQ_ADC)       /* Vector 16+22: A/D Converter */
VECTOR(lpc17_bod, LPC17_IRQ_BOD)       /* Vector 16+23: Brown Out detect */
VECTOR(lpc17_usb, LPC17_IRQ_USB)       /* Vector 16+24: USB */
VECTOR(lpc17_can, LPC17_IRQ_CAN)       /* Vector 16+25: CAN */
VECTOR(lpc17_gpdma, LPC17_IRQ_GPDMA)   /* Vector 16+26: GPDMA */
VECTOR(lpc17_i2s, LPC17_IRQ_I2S)       /* Vector 16+27: I2S */
VECTOR(lpc17_eth, LPC17_IRQ_ETH)       /* Vector 16+28: Ethernet */
VECTOR(lpc17_mci, LPC17_IRQ_MCI)       /* Vector 16+29: MMC/SD */
VECTOR(lpc17_mcpwm, LPC17_IRQ_MCPWM)   /* Vector 16+30: Motor Control PWM */
VECTOR(lpc17_qei, LPC17_IRQ_QEI)       /* Vector 16+31: Quadrature Encoder */
VECTOR(lpc17_pll1, LPC17_IRQ_PLL1)     /* Vector 16+32: PLL 1 */
VECTOR(lpc17_usbact, LPC17_IRQ_USBACT) /* Vector 16+33: USB Activity Interrupt */
VECTOR(lpc17_canact, LPC17_IRQ_CANACT) /* Vector 16+34: CAN Activity Interrupt */
VECTOR(lpc17_uart4, LPC17_IRQ_UART4)   /* Vector 16+35:  UART 4 */
VECTOR(lpc17_ssp2, LPC17_IRQ_SSP2)     /* Vector 16+36: SSP 2 */
VECTOR(lpc17_lcd, LPC17_IRQ_LCD)       /* Vector 16+37: LCD */
VECTOR(lpc17_gpio, LPC17_IRQ_GPIO)     /* Vector 16+38: GPIO */
VECTOR(lpc17_pwm0, LPC17_IRQ_PWM0)     /* Vector 16+39: PWM0 */
VECTOR(lpc17_eeprom, LPC17_IRQ_EEPROM) /* Vector 16+40: EEPROM */

#endif

/********************************************************************************
 * Public Types
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
