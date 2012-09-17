/**************************************************************************
 * arch/z80/src/z8/z8_xdef.S
 * Interrupt Handling
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS or IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER or CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, or CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS or SERVICES; LOSS
 * OF USE, DATA, or PROFITS; or BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, or TORT (INCLUDING NEGLIGENCE or OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

#include <ez8.inc>
#include <vect.inc>

/**************************************************************************
 * Definitions
 **************************************************************************/

/**************************************************************************
 * External References / External Definitions
 **************************************************************************/

	xref	_up_doirq:ROM

#if defined(ENCORE_VECTORS)
	xdef	_z8_wdt_handler
	xdef	_z8_trap_handler
if EZ8_TIMER3=1
	xdef	_z8_timer2_handler
endif
	xdef	_z8_timer1_handler
	xdef	_z8_timer0_handler
if EZ8_UART0=1
	xdef	_z8_uart0rx_handler
	xdef	_z8_uart0tx_handler
endif
if EZ8_I2C=1
	xdef	_z8_i2c_handler
endif
if EZ8_SPI=1
	xdef	_z8_spi_handler
endif
if EZ8_ADC=1
	xdef	_z8_adc_handler
endif
	xdef	_z8_p7ad_handler
	xdef	_z8_p6ad_handler
 	xdef	_z8_p5ad_handler
	xdef	_z8_p4ad_handler
	xdef	_z8_p3ad_handler
	xdef	_z8_p2ad_handler
	xdef	_z8_p1ad_handler
	xdef	_z8_p0ad_handler
if EZ8_TIMER4=1
	xdef	_z8_timer3_handler
endif
if EZ8_UART1=1
	xdef	_z8_uart1rx_handler
	xdef	_z8_uart1tx_handler
endif
if EZ8_DMA=1
	xdef	_z8_dma_handler
endif
if EZ8_PORT1=0
	xdef	_z8_c3_handler
	xdef	_z8_c2_handler
	xdef	_z8_c1_handler
	xdef	_z8_c0_handler
endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
if EZ8_TIMER3=1
	xdef _z8_timer2_handler
endif
	xdef _z8_timer1_handler
	xdef _z8_timer0_handler
if EZ8_UART0=1
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
endif
if EZ8_I2C=1
	xdef _z8_i2c_handler
endif
if EZ8_SPI=1
	xdef _z8_spi_handler
endif
if (EZ8_ADC=1) || (EZ8_ADC_NEW=1)
	xdef _z8_adc_handler
endif
	xdef _z8_p7ad_handler
	xdef _z8_p6ad_handler
 	xdef _z8_p5ad_handler
	xdef _z8_p4ad_handler
	xdef _z8_p3ad_handler
	xdef _z8_p2ad_handler
	xdef _z8_p1ad_handler
	xdef _z8_p0ad_handler
if EZ8_TIMER4=1
	xdef _z8_timer3_handler
endif
if EZ8_UART1=1
	xdef _z8_uart1rx_handler
	xdef _z8_uart1tx_handler
endif
if EZ8_DMA=1
	xdef _z8_dma_handler
endif
if (EZ8_PORT1=0)
	xdef _z8_c3_handler
	xdef _z8_c2_handler
	xdef _z8_c1_handler
	xdef _z8_c0_handler
endif
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
if EZ8_TIMER3=1
	xdef _z8_timer2_handler
endif
	xdef _z8_timer1_handler
	xdef _z8_timer0_handler
if EZ8_UART0=1
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
endif
if EZ8_I2C=1
	xdef _z8_i2c_handler
endif
if EZ8_ESPI=1
	xdef _z8_spi_handler
endif
if EZ8_ADC_NEW=1
	xdef _z8_adc_handler
endif
	xdef _z8_p7ad_handler
	xdef _z8_p6ad_handler
 	xdef _z8_p5ad_handler
	xdef _z8_p4ad_handler
	xdef _z8_p3ad_handler
	xdef _z8_p2ad_handler
	xdef _z8_p1ad_handler
	xdef _z8_p0ad_handler
if EZ8_MCT=1
	xdef _z8_mct_handler
endif
if EZ8_UART1=1
	xdef _z8_uart1rx_handler
	xdef _z8_uart1tx_handler
endif
	xdef _z8_c3_handler
	xdef _z8_c2_handler
	xdef _z8_c1_handler
	xdef _z8_c0_handler
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

	xdef _z8_wdt_handler
	xdef _z8_trap_handler
	xdef _z8_pwmtimer_handler
	xdef _z8_pwmfault_handler
if EZ8_ADC_NEW=1
	xdef _z8_adc_handler
endif
	xdef _z8_cmp_handler
	xdef _z8_timer0_handler
if EZ8_UART0
	xdef _z8_uart0rx_handler
	xdef _z8_uart0tx_handler
endif
if EZ8_SPI=1
	xdef _z8_spi_handler
endif
if EZ8_I2C=1
	xdef _z8_i2c_handler
endif
	xdef _z8_c0_handler
	xdef _z8_pb_handler
	xdef _z8_p7ap3a_handler
	xdef _z8_p6ap2a_handler
	xdef _z8_p5ap1a_handler
	xdef _z8_p4ap0a_handler
	xdef _z8_potrap_handler
	xdef _z8_wotrap_handler
#endif 

/**************************************************************************
 * Macros
 **************************************************************************/

ENTER :	MACRO	val
	pushx	rp			/* Save the current RP value in the stack */
	srp	#%f0			/* Load the interrupt register pointer */
	ld	r0, #val		/* Pass the new value in r0 */
	jr	_z8_common_handler	/* The rest of the handling is common */
	ENDMAC	ENTER

LEAVE :	MACRO
	popx	rp			/* Restore the user register pointer */
	iret				/* And return from interrupt */
	ENDMAC	LEAVE

/**************************************************************************
 * Code
 **************************************************************************/

	segment CODE

/**************************************************************************
 * Interrupt Vectors
 **************************************************************************/

#if defined(ENCORE_VECTORS)
	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
if EZ8_TIMER3=1
	vector TIMER2	= _z8_timer2_handler
endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
if EZ8_UART0=1
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
endif
if EZ8_I2C=1
	vector I2C	= _z8_i2c_handler
endif
if EZ8_SPI=1
	vector SPI	= _z8_spi_handler
endif
if EZ8_ADC=1
	vector ADC	= _z8_adc_handler
endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
if EZ8_TIMER4=1
	vector TIMER3	= _z8_timer3_handler
endif
if EZ8_UART1=1
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
endif
if EZ8_DMA=1
	vector DMA	= _z8_dma_handler
endif
if EZ8_PORT1=0
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
if EZ8_TIMER3=1
	vector TIMER2	= _z8_timer2_handler
endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
if EZ8_UART0=1
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
endif
if EZ8_I2C=1
	vector I2C	= _z8_i2c_handler
endif
if EZ8_SPI=1
	vector SPI	= _z8_spi_handler
endif
if (EZ8_ADC=1) || (EZ8_ADC_NEW=1)
	vector ADC	= _z8_adc_handler
endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
if EZ8_TIMER4=1
	vector TIMER3	= _z8_timer3_handler
endif
if EZ8_UART1=1
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
endif
if EZ8_DMA=1
	vector DMA	= _z8_dma_handler
endif
if EZ8_PORT1=0
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
endif
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
if EZ8_TIMER3=1
	vector TIMER2	= _z8_timer2_handler
endif
	vector TIMER1	= _z8_timer1_handler
	vector TIMER0	= _z8_timer0_handler
if EZ8_UART0=1
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
endif
if EZ8_I2C=1
	vector I2C	= _z8_i2c_handler
endif
if EZ8_ESPI=1
	vector SPI	= _z8_spi_handler
endif
if EZ8_ADC_NEW=1
	vector ADC	= _z8_adc_handler
endif
	vector P7AD	= _z8_p7ad_handler
	vector P6AD	= _z8_p6ad_handler
 	vector P5AD	= _z8_p5ad_handler
	vector P4AD	= _z8_p4ad_handler
	vector P3AD	= _z8_p3ad_handler
	vector P2AD	= _z8_p2ad_handler
	vector P1AD	= _z8_p1ad_handler
	vector P0AD	= _z8_p0ad_handler
if EZ8_MCT=1
	vector MCT	= _z8_mct_handler
endif
if EZ8_UART1=1
	vector UART1_RX	= _z8_uart1rx_handler
	vector UART1_TX	= _z8_uart1tx_handler
endif
	vector C3	= _z8_c3_handler
	vector C2	= _z8_c2_handler
	vector C1	= _z8_c1_handler
	vector C0	= _z8_c0_handler
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

	vector WDT	= _z8_wdt_handler
	vector TRAP	= _z8_trap_handler
	vector PWMTIMER	= _z8_pwmtimer_handler
	vector PWMFAULT	= _z8_pwmfault_handler
if EZ8_ADC_NEW=1
	vector ADC	= _z8_adc_handler
endif
	vector CMP	= _z8_cmp_handler
	vector TIMER0	= _z8_timer0_handler
if EZ8_UART0
	vector UART0_RX	= _z8_uart0rx_handler
	vector UART0_TX	= _z8_uart0tx_handler
endif
if EZ8_SPI=1
	vector SPI	= _z8_spi_handler
endif
if EZ8_I2C=1
	vector I2C	= _z8_i2c_handler
endif
	vector C0	= _z8_c0_handler
	vector PB	= _z8_pb_handler
	vector P7A	= _z8_p7ap3a_handler
	vector P6A	= _z8_p6ap2a_handler
	vector P5A	= _z8_p5ap1a_handler
	vector P4A	= _z8_p4ap0a_handler
	vector POTRAP	= _z8_potrap_handler
	vector WOTRAP	= _z8_wotrap_handler
#endif 

/**************************************************************************
 * Name: _z16f_*_handler
 *
 * Description:
 *   Map individual interrupts into interrupt number and branch to common
 *   interrupt handling logic.  If higher interrupt handling performance
 *   for particular interrupts is required, then those interrupts should
 *   be picked off here and handled outside of the common logic.
 *
 *   On entry to any of these handlers, the stack contains the following:
 *
 *         TOS before interrupt
 *         PC[7:0]
 *         PC[15:8]
 *   SP -> Flags Register
 *
 **************************************************************************/

#if defined(ENCORE_VECTORS)
_z8_wdt_handler:
	ENTER(Z8_WDT_IRQ)
_z8_trap_handler:
	ENTER(Z8_TRAP_IRQ)
if EZ8_TIMER3=1
_z8_timer2_handler:
	ENTER(Z8_TIMER2_IRQ)
endif
_z8_timer1_handler:
	ENTER(Z8_TIMER1_IRQ)
_z8_timer0_handler:
	ENTER(Z8_TIMER0_IRQ)
if EZ8_UART0=1
_z8_uart0rx_handler:
	ENTER(Z8_UART0_RX_IRQ)
_z8_uart0tx_handler:
	ENTER(Z8_UART0_TX_IRQ)
endif
if EZ8_I2C=1
_z8_i2c_handler:
	ENTER(Z8_I2C_IRQ)
endif
if EZ8_SPI=1
_z8_spi_handler:
	ENTER(Z8_SPI_IRQ)
endif
if EZ8_ADC=1
_z8_adc_handler:
	ENTER(Z8_ADC_IRQ)
endif
_z8_p7ad_handler:
	ENTER(Z8_P7AD_IRQ)
_z8_p6ad_handler:
	ENTER(Z8_P6AD_IRQ)
_z8_p5ad_handler:
	ENTER(Z8_P5AD_IRQ)
_z8_p4ad_handler:
	ENTER(Z8_P4AD_IRQ)
_z8_p3ad_handler:
	ENTER(Z8_P3AD_IRQ)
_z8_p2ad_handler:
	ENTER(Z8_P2AD_IRQ)
_z8_p1ad_handler:
	ENTER(Z8_P1AD_IRQ)
_z8_p0ad_handler:
	ENTER(Z8_P0AD_IRQ)
if EZ8_TIMER4=1
_z8_timer3_handler:
	ENTER(Z8_TIMER3_IRQ)
endif
if EZ8_UART1=1
_z8_uart1rx_handler:
	ENTER(Z8_UART1_RX_IRQ)
_z8_uart1tx_handler:
	ENTER(Z8_UART1_TX_IRQ)
endif
if EZ8_DMA=1
_z8_dma_handler:
	ENTER(Z8_DMA_IRQ)
endif
if EZ8_PORT1=0
_z8_c3_handler:
	ENTER(Z8_C3_IRQ)
_z8_c2_handler:
	ENTER(Z8_C2_IRQ)
_z8_c1_handler:
	ENTER(Z8_C1_IRQ)
_z8_c0_handler:
	ENTER(Z8_C0_IRQ)
endif

/**************************************************************************/

#elif defined(ENCORE_XP_VECTORS)

_z8_wdt_handler:
	ENTER(Z8_WDT_IRQ)
_z8_trap_handler:
	ENTER(Z8_TRAP_IRQ)
if EZ8_TIMER3=1
_z8_timer2_handler:
	ENTER(Z8_TIMER2_IRQ)
endif
_z8_timer1_handler:
	ENTER(Z8_TIMER1_IRQ)
_z8_timer0_handler:
	ENTER(Z8_TIMER0_IRQ)
if EZ8_UART0=1
_z8_uart0rx_handler:
	ENTER(Z8_UART0_RX_IRQ)
_z8_uart0tx_handler:
	ENTER(Z8_UART0_TX_IRQ)
endif
if EZ8_I2C=1
_z8_i2c_handler:
	ENTER(Z8_I2C_IRQ)
endif
if EZ8_SPI=1
_z8_spi_handler:
	ENTER(Z8_SPI_IRQ)
endif
if (EZ8_ADC=1) || (EZ8_ADC_NEW=1)
_z8_adc_handler:
	ENTER(Z8_ADC_IRQ)
endif
_z8_p7ad_handler:
	ENTER(Z8_P7AD_IRQ)
_z8_p6ad_handler:
	ENTER(Z8_P6AD_IRQ)
_z8_p5ad_handler:
	ENTER(Z8_P5AD_IRQ)
_z8_p4ad_handler:
	ENTER(Z8_P4AD_IRQ)
_z8_p3ad_handler:
	ENTER(Z8_P3AD_IRQ)
_z8_p2ad_handler:
	ENTER(Z8_P2AD_IRQ)
_z8_p1ad_handler:
	ENTER(Z8_P1AD_IRQ)
_z8_p0ad_handler:
	ENTER(Z8_P0AD_IRQ)
if EZ8_TIMER4=1
_z8_timer3_handler:
	ENTER(Z8_TIMER3_IRQ)
endif
if EZ8_UART1=1
_z8_uart1rx_handler:
	ENTER(Z8_UART1_RX_IRQ)
_z8_uart1tx_handler:
	ENTER(Z8_UART1_TX_IRQ)
endif
if EZ8_DMA=1
_z8_dma_handler:
	ENTER(Z8_DMA_IRQ)
endif
if EZ8_PORT1=0
_z8_c3_handler:
	ENTER(Z8_C3_IRQ)
_z8_c2_handler:
	ENTER(Z8_C2_IRQ)
_z8_c1_handler:
	ENTER(Z8_C1_IRQ)
_z8_c0_handler:
	ENTER(Z8_C0_IRQ)
endif
_z8_potrap_handler:
	ENTER(Z8_POTRAP_IRQ)
_z8_wotrap_handler:
	ENTER(Z8_WOTRAP_IRQ)

/**************************************************************************/

#elif defined(ENCORE_XP16K_VECTORS)

_z8_wdt_handler:
	ENTER(Z8_WDT_IRQ)
_z8_trap_handler:
	ENTER(Z8_TRAP_IRQ)
if EZ8_TIMER3=1
_z8_timer2_handler:
	ENTER(Z8_TIMER2_IRQ)
endif
_z8_timer1_handler:
	ENTER(Z8_TIMER1_IRQ)
_z8_timer0_handler:
	ENTER(Z8_TIMER0_IRQ)
if EZ8_UART0=1
_z8_uart0rx_handler:
	ENTER(Z8_UART0_RX_IRQ)
_z8_uart0tx_handler:
	ENTER(Z8_UART0_TX_IRQ)
endif
if EZ8_I2C=1
_z8_i2c_handler:
	ENTER(Z8_I2C_IRQ)
endif
if EZ8_ESPI=1
_z8_spi_handler:
	ENTER(Z8_SPI_IRQ)
endif
if EZ8_ADC_NEW=1
_z8_adc_handler:
	ENTER(Z8_ADC_IRQ)
endif
_z8_p7ad_handler:
	ENTER(Z8_P7AD_IRQ)
_z8_p6ad_handler:
	ENTER(Z8_P6AD_IRQ)
_z8_p5ad_handler:
	ENTER(Z8_P5AD_IRQ)
_z8_p4ad_handler:
	ENTER(Z8_P4AD_IRQ)
_z8_p3ad_handler:
	ENTER(Z8_P3AD_IRQ)
_z8_p2ad_handler:
	ENTER(Z8_P2AD_IRQ)
_z8_p1ad_handler:
	ENTER(Z8_P1AD_IRQ)
_z8_p0ad_handler:
	ENTER(Z8_P0AD_IRQ)
if EZ8_MCT=1
_z8_mct_handler:
	ENTER(Z8_MCT_IRQ)
endif
if EZ8_UART1=1
_z8_uart1rx_handler:
	ENTER(Z8_UART1_RX_IRQ)
_z8_uart1tx_handler:
	ENTER(Z8_UART1_TX_IRQ)
endif
_z8_c3_handler:
	ENTER(Z8_C3_IRQ)
_z8_c2_handler:
	ENTER(Z8_C2_IRQ)
_z8_c1_handler:
	ENTER(Z8_C1_IRQ)
_z8_c0_handler:
	ENTER(Z8_C0_IRQ)
_z8_potrap_handler:
	ENTER(Z8_POTRAP_IRQ)
_z8_wotrap_handler:
	ENTER(Z8_WOTRAP_IRQ)

/**************************************************************************/

#elif defined(ENCORE_MC_VECTORS)

_z8_wdt_handler:
	ENTER(Z8_WDT_IRQ)
_z8_trap_handler:
	ENTER(Z8_TRAP_IRQ)
_z8_pwmtimer_handler:
	ENTER(Z8_PWMTIMER_IRQ)
_z8_pwmfault_handler:
	ENTER(Z8_PWMFAULT_IRQ)
if EZ8_ADC_NEW=1
_z8_adc_handler:
	ENTER(Z8_ADC_IRQ)
endif
_z8_cmp_handler:
	ENTER(Z8_CMP_IRQ)
_z8_timer0_handler:
	ENTER(Z8_TIMER0_IRQ)
if EZ8_UART0
_z8_uart0rx_handler:
	ENTER(Z8_UART0_RX_IRQ)
_z8_uart0tx_handler:
	ENTER(Z8_UART0_TX_IRQ)
endif
if EZ8_SPI=1
_z8_spi_handler:
	ENTER(Z8_SPI_IRQ)
endif
if EZ8_I2C=1
_z8_i2c_handler:
	ENTER(Z8_I2C_IRQ)
endif
_z8_c0_handler:
	ENTER(Z8_C0_IRQ)
_z8_pb_handler:
	ENTER(Z8_PB_IRQ)
_z8_p7ap3a_handler:
	ENTER(Z8_P7A_IRQ)
_z8_p6ap2a_handler:
	ENTER(Z8_P6AP2A_IRQ)
_z8_p5ap1a_handler:
	ENTER(Z8_P5AP1A_IRQ)
_z8_p4ap0a_handler:
	ENTER(Z8_P4AP0A_IRQ)
_z8_potrap_handler:
	ENTER(Z8_POTRAP_IRQ)
_z8_wotrap_handler:
	ENTER(Z8_WOTRAP_IRQ)
#endif 

/**************************************************************************
 * Name: _z16f_common_handler
 *
 * Description:
 *   Common IRQ handling logic
 *
 *   On entry, the stack contains the following:
 *
 *         TOS before interrupt
 *         PC[7:0]
 *         PC[15:8]
 *         Flags Register
 *   SP -> RP
 *
 *   R0 holds the IRQ number and the RP has been reset to %f0
 *
 **************************************************************************/

_z8_common_handler:
	/* Pass the address of the IRQ stack frame */

	ldx	r2, sph			/* rr2 = stack pointer */
	ldx	r3, spl
	push	r3			/* Pass as a parameter */
	push	r2

	/* Pass the IRQ number */

	push	r0
	
	/* Process the interrupt */

	call	_up_doirq		/* Call the IRQ handler */

	/* Release arguments from the stack */

	pop	r4			/* Discard the IRQ argument */
	pop	r2			/* Recover the stack pointer parameter */
	pop	r3

	/* If a interrupt level context switch occurred, then the
	 * return value will be the same as the input value
	 */

	cp	r0, r2			/* Same as the return value? */
	jr	nz, _z8_switch
	cp	r1, r3
	jr	z, _z8_noswitch

	/* A context switch occurs.  Restore the use context.
	 * rr0 = pointer to context structgure.
	 */

_z8_switch:

	/* Destroy the interrupt return information on the stack */
	
	pop	r4			/* Destroy saved RP */
	pop	r4			/* Destroy saved flags */
	pop	r4			/* Destroy saved return address */
	pop	r4

	/* Copy all registers into the user register area. */

	clr	r2			/* rr2 = destination address */
	ldx	r3, XCPT_RP_OFFS(rr0)
	ld	r4, r0			/* rr4 = source address */
	ld	r5, r1
	ld	r6, #16			/* r6 = number of bytes to copy */

_z8_restore:
	ldx	r7, @rr4
	ldx	@rr2, r7
	incw	rr2
	incw	rr4
	djnz	r6, _z8_restore

	/* Set the new stack pointer */

	ldx	r2, XCPT_SPH_OFFS(rr0)
	ldx	r3, XCPT_SPL_OFFS(rr0)
	ldx	sph, r2
	ldx	spl, r3

	/* Push the return address onto the stack */

	ldx	r2, XCPT_PCH_OFFS(rr0)
	ldx	r3, XCPT_PCL_OFFS(rr0)
	push	r3
	push	r2

	/* Recover the flags and RP settings.. but don't restore them yet */

	ldx	r3, XCPT_FLAGS_OFFS(rr0)
	ldx	r4, XCPT_RP_OFFS(rr0)
	
	/* Determine whether interrupts must be enabled on return.  This
	 * would be nicer to do below, but later we will need to preserve
	 * the condition codes in the flags.
	 */

	ldx	r2, XCPT_IRQCTL_OFFS(rr0)
	tm	r2, #%80
	jr	nz, _z8_returnenabled

	/* Restore the flag settings */

	ldx	flags, r3

	/* Restore the user register page and return with interrupts disabled.
	 * Note that we cannot use the iret instruction because it unconditionally
	 * re-enabled interrupts
	 */

	ldx	rp, r4				/* Does not effect flags */
	ret					/* Does not effect flags */

_z8_returnenabled:
	/* Restore the flag settings */

	ldx	flags, r1

	/* Restore the user register page, re-enable interrupts and return.
	 * Note that we cannot use the iret instruction because it unconditionally
	 * re-enabled interrupts
	 */

	ldx	rp, r4				/* Does not effect flags */
	ei					/* Does not effect flags */
	ret					/* Does not effect flags */

_z8_noswitch:
	LEAVE

/**************************************************************************
 * Data
 **************************************************************************/

	/* Set aside area for interrupt registers */

	define interruptreg, space=rdata, org=%f0
	segment interruptreg
	ds  %10

	end	_z8_common_handler
