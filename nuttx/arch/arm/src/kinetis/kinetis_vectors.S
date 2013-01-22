/************************************************************************************************
 * arch/arm/src/kinetis/kinetis_vectors.S
 * arch/arm/src/chip/kinetis_vectors.S
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
 ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************************/
/* Memory Map:
 *
 * 0x0000:0000 - Beginning of FLASH. Address of vectors
 * 0x1800:0000 - Start of CPU SRAM and start of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap. NOTE
 *               that the ARM uses a decrement before store stack so that the correct initial
 *               value is the end of the stack + 4;
 *             - Heap ends at the configured end of SRAM.
 */

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/* The Cortex-M4 return from interrupt is unusual.  We provide the following special
 * address to the BX instruction.  The particular value also forces a return to
 * thread mode and covers state from the main stack point, the MSP (vs. the MSP).
 */

#define EXC_RETURN		0xfffffff9

/************************************************************************************************
 * Global Symbols
 ************************************************************************************************/

	.globl		__start

	.syntax		unified
	.thumb
	.file		"kinetis_vectors.S"

/************************************************************************************************
 * Macros
 ************************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov		r0, #\irqno
	b		kinetis_common
	.endm

/************************************************************************************************
 * Vectors
 ************************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		kinetis_vectors
	.type		kinetis_vectors, function

kinetis_vectors:

/* Processor Exceptions *************************************************************************/

	.word	IDLE_STACK			/* Vector  0: Reset stack pointer */
	.word	__start				/* Vector  1: Reset vector */
	.word	kinetis_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	kinetis_hardfault	/* Vector  3: Hard fault */
	.word	kinetis_mpu			/* Vector  4: Memory management (MPU) */
	.word	kinetis_busfault	/* Vector  5: Bus fault */
	.word	kinetis_usagefault	/* Vector  6: Usage fault */
	.word	kinetis_reserved	/* Vector  7: Reserved */
	.word	kinetis_reserved	/* Vector  8: Reserved */
	.word	kinetis_reserved	/* Vector  9: Reserved */
	.word	kinetis_reserved	/* Vector 10: Reserved */
	.word	kinetis_svcall		/* Vector 11: SVC call */
	.word	kinetis_dbgmonitor	/* Vector 12: Debug monitor */
	.word	kinetis_reserved	/* Vector 13: Reserved */
	.word	kinetis_pendsv		/* Vector 14: Pendable system service request */
	.word	kinetis_systick		/* Vector 15: System tick */

/* External Interrupts **************************************************************************/
/* K40 Family ***********************************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

#if defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

	.word	kinetis_dmach0		/* Vector 16: DMA channel 0 transfer complete */
	.word	kinetis_dmach1		/* Vector 17: DMA channel 1 transfer complete */
	.word	kinetis_dmach2		/* Vector 18: DMA channel 2 transfer complete */
	.word	kinetis_dmach3		/* Vector 19: DMA channel 3 transfer complete */
	.word	kinetis_dmach4		/* Vector 20: DMA channel 4 transfer complete */
	.word	kinetis_dmach5		/* Vector 21: DMA channel 5 transfer complete */
	.word	kinetis_dmach6		/* Vector 22: DMA channel 6 transfer complete */
	.word	kinetis_dmach7		/* Vector 23: DMA channel 7 transfer complete */
	.word	kinetis_dmach8		/* Vector 24: DMA channel 8 transfer complete */
	.word	kinetis_dmach9		/* Vector 25: DMA channel 9 transfer complete */
	.word	kinetis_dmach10		/* Vector 26: DMA channel 10 transfer complete */
	.word	kinetis_dmach11		/* Vector 27: DMA channel 11 transfer complete */
	.word	kinetis_dmach12		/* Vector 28: DMA channel 12 transfer complete */
	.word	kinetis_dmach13		/* Vector 29: DMA channel 13 transfer complete */
	.word	kinetis_dmach14		/* Vector 30: DMA channel 14 transfer complete */
	.word	kinetis_dmach15		/* Vector 31: DMA channel 15 transfer complete */
	.word	kinetis_dmaerr		/* Vector 32: DMA error interrupt channels 0-15 */
	.word	kinetis_mcm			/* Vector 33: MCM Normal interrupt */
	.word	kinetis_flashcc		/* Vector 34: Flash memory command complete */
	.word	kinetis_flashrc		/* Vector 35: Flash memory read collision */
	.word	kinetis_smclvd		/* Vector 36: Mode Controller low-voltage detect, low-voltage warning */
	.word	kinetis_llwu		/* Vector 37: LLWU Normal Low Leakage Wakeup */
	.word	kinetis_wdog		/* Vector 38: Watchdog */
	.word	kinetis_reserved	/* Vector 39: Reserved */
	.word	kinetis_i2c0		/* Vector 40: I2C0 */
	.word	kinetis_i2c1		/* Vector 41: I2C1 */
	.word	kinetis_spi0		/* Vector 42: SPI0 all sources */
	.word	kinetis_spi1		/* Vector 43: SPI1 all sources */
	.word	kinetis_spi2		/* Vector 44: SPI2 all sources */
	.word	kinetis_can0mb		/* Vector 45: CAN0 OR'ed Message buffer (0-15) */
	.word	kinetis_can0bo		/* Vector 46: CAN0 Bus Off */
	.word	kinetis_can0err		/* Vector 47: CAN0 Error */
	.word	kinetis_can0tw		/* Vector 48: CAN0 Transmit Warning */
	.word	kinetis_can0rw		/* Vector 49: CAN0 Receive Warning */
	.word	kinetis_can0wu		/* Vector 50: CAN0 Wake UP */
	.word	kinetis_reserved	/* Vector 51: Reserved */
	.word	kinetis_reserved	/* Vector 52: Reserved */
	.word	kinetis_can1mb		/* Vector 53: CAN1 OR'ed Message buffer (0-15) */
	.word	kinetis_can1bo		/* Vector 54: CAN1 Bus Off */
	.word	kinetis_can1err		/* Vector 55: CAN1 Error */
	.word	kinetis_can1tw		/* Vector 56: CAN1 Transmit Warning */
	.word	kinetis_can1rw		/* Vector 57: CAN1 Receive Warning */
	.word	kinetis_can1wu		/* Vector 58: CAN1 Wake UP */
	.word	kinetis_reserved	/* Vector 59: Reserved */
	.word	kinetis_reserved	/* Vector 60: Reserved */
	.word	kinetis_uart0s		/* Vector 61: UART0 status */
	.word	kinetis_uart0e		/* Vector 62: UART0 error */
	.word	kinetis_uart1s		/* Vector 63: UART1 status */
	.word	kinetis_uart1e		/* Vector 64: UART1 error */
	.word	kinetis_uart2s		/* Vector 65: UART2 status */
	.word	kinetis_uart2e		/* Vector 66: UART2 error */
	.word	kinetis_uart3s		/* Vector 67: UART3 status */
	.word	kinetis_uart3e		/* Vector 68: UART3 error */
	.word	kinetis_uart4s		/* Vector 69: UART4 status */
	.word	kinetis_uart4e		/* Vector 70: UART4 error */
	.word	kinetis_uart5s		/* Vector 71: UART5 status */
	.word	kinetis_uart5e		/* Vector 72: UART5 error */
	.word	kinetis_adc0		/* Vector 73: ADC0 */
	.word	kinetis_adc1		/* Vector 74: ADC1 */
	.word	kinetis_cmp0		/* Vector 75: CMP0 */
	.word	kinetis_cmp1		/* Vector 76: CMP1 */
	.word	kinetis_cmp2		/* Vector 77: CMP2 */
	.word	kinetis_ftm0		/* Vector 78: FTM0 all sources */
	.word	kinetis_ftm1		/* Vector 79: FTM1 all sources */
	.word	kinetis_ftm2		/* Vector 80: FTM2 all sources */
	.word	kinetis_cmt			/* Vector 81: CMT */
	.word	kinetis_rtc			/* Vector 82: RTC alarm interrupt */
	.word	kinetis_reserved	/* Vector 83: Reserved */
	.word	kinetis_pitch0		/* Vector 84: PIT channel 0 */
	.word	kinetis_pitch1		/* Vector 85: PIT channel 1 */
	.word	kinetis_pitch2		/* Vector 86: PIT channel 2 */
	.word	kinetis_pitch3		/* Vector 87: PIT channel 3 */
	.word	kinetis_pdb			/* Vector 88: PDB */
	.word	kinetis_usbotg		/* Vector 88: USB OTG */
	.word	kinetis_usbcd		/* Vector 90: USB charger detect */
	.word	kinetis_reserved	/* Vector 91: Reserved */
	.word	kinetis_reserved	/* Vector 92: Reserved */
	.word	kinetis_reserved	/* Vector 93: Reserved */
	.word	kinetis_reserved	/* Vector 94: Reserved */
	.word	kinetis_i2s0		/* Vector 95: I2S0 */
	.word	kinetis_sdhc		/* Vector 96: SDHC */
	.word	kinetis_dac0		/* Vector 97: DAC0 */
	.word	kinetis_dac1		/* Vector 98: DAC1 */
	.word	kinetis_tsi			/* Vector 97: TSI all sources */
	.word	kinetis_mcg			/* Vector 100: MCG */
	.word	kinetis_lpt			/* Vector 101: Low power timer */
	.word	kinetis_slcd		/* Vector 102: Segment LCD all sources */
	.word	kinetis_porta		/* Vector 103: Pin detect port A */
	.word	kinetis_portb		/* Vector 104: Pin detect port B */
	.word	kinetis_portc		/* Vector 105: Pin detect port C */
	.word	kinetis_portd		/* Vector 106: Pin detect port D */
	.word	kinetis_porte		/* Vector 107: Pin detect port E */
	.word	kinetis_reserved	/* Vector 108: Reserved */
	.word	kinetis_reserved	/* Vector 109: Reserved */
	.word	kinetis_swi			/* Vector 110: Software interrupt */

/* K60 Family ***********************************************************************************
 *
 * The memory map for the following parts is defined in Freescale document
 * K60P144M100SF2RM
 */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100) || defined(CONFIG_ARCH_CHIP_MK60X256VLQ100) || \
      defined(CONFIG_ARCH_CHIP_MK60N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK60N256VMD100) || \
      defined(CONFIG_ARCH_CHIP_MK60X256VMD100) || defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

	.word	kinetis_dmach0		/* Vector 16: DMA channel 0 transfer complete */
	.word	kinetis_dmach1		/* Vector 17: DMA channel 1 transfer complete */
	.word	kinetis_dmach2		/* Vector 18: DMA channel 2 transfer complete */
	.word	kinetis_dmach3		/* Vector 19: DMA channel 3 transfer complete */
	.word	kinetis_dmach4		/* Vector 20: DMA channel 4 transfer complete */
	.word	kinetis_dmach5		/* Vector 21: DMA channel 5 transfer complete */
	.word	kinetis_dmach6		/* Vector 22: DMA channel 6 transfer complete */
	.word	kinetis_dmach7		/* Vector 23: DMA channel 7 transfer complete */
	.word	kinetis_dmach8		/* Vector 24: DMA channel 8 transfer complete */
	.word	kinetis_dmach9		/* Vector 25: DMA channel 9 transfer complete */
	.word	kinetis_dmach10		/* Vector 26: DMA channel 10 transfer complete */
	.word	kinetis_dmach11		/* Vector 27: DMA channel 11 transfer complete */
	.word	kinetis_dmach12		/* Vector 28: DMA channel 12 transfer complete */
	.word	kinetis_dmach13		/* Vector 29: DMA channel 13 transfer complete */
	.word	kinetis_dmach14		/* Vector 30: DMA channel 14 transfer complete */
	.word	kinetis_dmach15		/* Vector 31: DMA channel 15 transfer complete */
	.word	kinetis_dmaerr		/* Vector 32: DMA error interrupt channels 0-15 */
	.word	kinetis_mcm			/* Vector 33: MCM Normal interrupt */
	.word	kinetis_flashcc		/* Vector 34: Flash memory command complete */
	.word	kinetis_flashrc		/* Vector 35: Flash memory read collision */
	.word	kinetis_smclvd		/* Vector 36: Mode Controller low-voltage detect, low-voltage warning */
	.word	kinetis_llwu		/* Vector 37: LLWU Normal Low Leakage Wakeup */
	.word	kinetis_wdog		/* Vector 38: Watchdog */
	.word	kinetis_rngb		/* Vector 39: Random number generator */
	.word	kinetis_i2c0		/* Vector 40: I2C0 */
	.word	kinetis_i2c1		/* Vector 41: I2C1 */
	.word	kinetis_spi0		/* Vector 42: SPI0 all sources */
	.word	kinetis_spi1		/* Vector 43: SPI1 all sources */
	.word	kinetis_spi2		/* Vector 44: SPI2 all sources */
	.word	kinetis_can0mb		/* Vector 45: CAN0 OR'ed Message buffer (0-15) */
	.word	kinetis_can0bo		/* Vector 46: CAN0 Bus Off */
	.word	kinetis_can0err		/* Vector 47: CAN0 Error */
	.word	kinetis_can0tw		/* Vector 48: CAN0 Transmit Warning */
	.word	kinetis_can0rw		/* Vector 49: CAN0 Receive Warning */
	.word	kinetis_can0wu		/* Vector 50: CAN0 Wake UP */
	.word	kinetis_reserved	/* Vector 51: Reserved */
	.word	kinetis_reserved	/* Vector 52: Reserved */
	.word	kinetis_can1mb		/* Vector 53: CAN1 OR'ed Message buffer (0-15) */
	.word	kinetis_can1bo		/* Vector 54: CAN1 Bus Off */
	.word	kinetis_can1err		/* Vector 55: CAN1 Error */
	.word	kinetis_can1tw		/* Vector 56: CAN1 Transmit Warning */
	.word	kinetis_can1rw		/* Vector 57: CAN1 Receive Warning */
	.word	kinetis_can1wu		/* Vector 58: CAN1 Wake UP */
	.word	kinetis_reserved	/* Vector 59: Reserved */
	.word	kinetis_reserved	/* Vector 60: Reserved */
	.word	kinetis_uart0s		/* Vector 61: UART0 status */
	.word	kinetis_uart0e		/* Vector 62: UART0 error */
	.word	kinetis_uart1s		/* Vector 63: UART1 status */
	.word	kinetis_uart1e		/* Vector 64: UART1 error */
	.word	kinetis_uart2s		/* Vector 65: UART2 status */
	.word	kinetis_uart2e		/* Vector 66: UART2 error */
	.word	kinetis_uart3s		/* Vector 67: UART3 status */
	.word	kinetis_uart3e		/* Vector 68: UART3 error */
	.word	kinetis_uart4s		/* Vector 69: UART4 status */
	.word	kinetis_uart4e		/* Vector 70: UART4 error */
	.word	kinetis_uart5s		/* Vector 71: UART5 status */
	.word	kinetis_uart5e		/* Vector 72: UART5 error */
	.word	kinetis_adc0		/* Vector 73: ADC0 */
	.word	kinetis_adc1		/* Vector 74: ADC1 */
	.word	kinetis_cmp0		/* Vector 75: CMP0 */
	.word	kinetis_cmp1		/* Vector 76: CMP1 */
	.word	kinetis_cmp2		/* Vector 77: CMP2 */
	.word	kinetis_ftm0		/* Vector 78: FTM0 all sources */
	.word	kinetis_ftm1		/* Vector 79: FTM1 all sources */
	.word	kinetis_ftm2		/* Vector 80: FTM2 all sources */
	.word	kinetis_cmt			/* Vector 81: CMT */
	.word	kinetis_rtc			/* Vector 82: RTC alarm interrupt */
	.word	kinetis_reserved	/* Vector 83: Reserved */
	.word	kinetis_pitch0		/* Vector 84: PIT channel 0 */
	.word	kinetis_pitch1		/* Vector 85: PIT channel 1 */
	.word	kinetis_pitch2		/* Vector 86: PIT channel 2 */
	.word	kinetis_pitch3		/* Vector 87: PIT channel 3 */
	.word	kinetis_pdb			/* Vector 88: PDB */
	.word	kinetis_usbotg		/* Vector 88: USB OTG */
	.word	kinetis_usbcd		/* Vector 90: USB charger detect */
	.word	kinetis_emactmr		/* Vector 91: Ethernet MAC IEEE 1588 timer interrupt */
	.word	kinetis_emactx		/* Vector 92: Ethernet MAC transmit interrupt */
	.word	kinetis_emacrx		/* Vector 93: Ethernet MAC receive interrupt */
	.word	kinetis_emacmisc	/* Vector 94: Ethernet MAC error and misc interrupt */
	.word	kinetis_i2s0		/* Vector 95: I2S0 */
	.word	kinetis_sdhc		/* Vector 96: SDHC */
	.word	kinetis_dac0		/* Vector 97: DAC0 */
	.word	kinetis_dac1		/* Vector 98: DAC1 */
	.word	kinetis_tsi			/* Vector 97: TSI all sources */
	.word	kinetis_mcg			/* Vector 100: MCG */
	.word	kinetis_lpt			/* Vector 101: Low power timer */
	.word	kinetis_reserved	/* Vector 102: Reserved */
	.word	kinetis_porta		/* Vector 103: Pin detect port A */
	.word	kinetis_portb		/* Vector 104: Pin detect port B */
	.word	kinetis_portc		/* Vector 105: Pin detect port C */
	.word	kinetis_portd		/* Vector 106: Pin detect port D */
	.word	kinetis_porte		/* Vector 107: Pin detect port E */
	.word	kinetis_reserved	/* Vector 108: Reserved */
	.word	kinetis_reserved	/* Vector 109: Reserved */
	.word	kinetis_reserved	/* Vector 110: Reserved */
	.word	kinetis_reserved	/* Vector 111: Reserved */
	.word	kinetis_reserved	/* Vector 112: Reserved */
	.word	kinetis_reserved	/* Vector 113: Reserved */
	.word	kinetis_reserved	/* Vector 114: Reserved */
	.word	kinetis_reserved	/* Vector 115: Reserved */
	.word	kinetis_reserved	/* Vector 116: Reserved */
	.word	kinetis_reserved	/* Vector 117: Reserved */
	.word	kinetis_reserved	/* Vector 118: Reserved */
	.word	kinetis_reserved	/* Vector 119: Reserved */
#else
#  error "No vectors for this Kinetis part"
#endif

/************************************************************************************************
 * .text
 ************************************************************************************************/

	.text
	.type	handlers, function
	.thumb_func
handlers:

/* Processor Exceptions *************************************************************************/

	HANDLER	kinetis_reserved, KINETIS_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	kinetis_nmi, KINETIS_IRQ_NMI				/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	kinetis_hardfault, KINETIS_IRQ_HARDFAULT	/* Vector  3: Hard fault */
	HANDLER	kinetis_mpu, KINETIS_IRQ_MEMFAULT			/* Vector  4: Memory management (MPU) */
	HANDLER	kinetis_busfault, KINETIS_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	kinetis_usagefault, KINETIS_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	kinetis_svcall, KINETIS_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	kinetis_dbgmonitor, KINETIS_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	kinetis_pendsv, KINETIS_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	kinetis_systick, KINETIS_IRQ_SYSTICK		/* Vector 15: System tick */

/* External Interrupts **************************************************************************/
/* K40 Family ***********************************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

#if defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

	HANDLER kinetis_dmach0, KINETIS_IRQ_DMACH0		/* Vector 16: DMA channel 0 transfer complete */
	HANDLER kinetis_dmach1, KINETIS_IRQ_DMACH1		/* Vector 17: DMA channel 1 transfer complete */
	HANDLER kinetis_dmach2, KINETIS_IRQ_DMACH2		/* Vector 18: DMA channel 2 transfer complete */
	HANDLER kinetis_dmach3, KINETIS_IRQ_DMACH3		/* Vector 19: DMA channel 3 transfer complete */
	HANDLER kinetis_dmach4, KINETIS_IRQ_DMACH4		/* Vector 20: DMA channel 4 transfer complete */
	HANDLER kinetis_dmach5, KINETIS_IRQ_DMACH5		/* Vector 21: DMA channel 5 transfer complete */
	HANDLER kinetis_dmach6, KINETIS_IRQ_DMACH6		/* Vector 22: DMA channel 6 transfer complete */
	HANDLER kinetis_dmach7, KINETIS_IRQ_DMACH7		/* Vector 23: DMA channel 7 transfer complete */
	HANDLER kinetis_dmach8, KINETIS_IRQ_DMACH8		/* Vector 24: DMA channel 8 transfer complete */
	HANDLER kinetis_dmach9, KINETIS_IRQ_DMACH9		/* Vector 25: DMA channel 9 transfer complete */
	HANDLER kinetis_dmach10, KINETIS_IRQ_DMACH10	/* Vector 26: DMA channel 10 transfer complete */
	HANDLER kinetis_dmach11, KINETIS_IRQ_DMACH11	/* Vector 27: DMA channel 11 transfer complete */
	HANDLER kinetis_dmach12, KINETIS_IRQ_DMACH12	/* Vector 28: DMA channel 12 transfer complete */
	HANDLER kinetis_dmach13, KINETIS_IRQ_DMACH13	/* Vector 29: DMA channel 13 transfer complete */
	HANDLER kinetis_dmach14, KINETIS_IRQ_DMACH14	/* Vector 30: DMA channel 14 transfer complete */
	HANDLER kinetis_dmach15, KINETIS_IRQ_DMACH15	/* Vector 31: DMA channel 15 transfer complete */
	HANDLER kinetis_dmaerr, KINETIS_IRQ_DMAERR		/* Vector 32: DMA error interrupt channels 0-15 */
	HANDLER kinetis_mcm, KINETIS_IRQ_MCM			/* Vector 33: MCM Normal interrupt */
	HANDLER kinetis_flashcc, KINETIS_IRQ_FLASHCC	/* Vector 34: Flash memory command complete */
	HANDLER kinetis_flashrc, KINETIS_IRQ_FLASHRC	/* Vector 35: Flash memory read collision */
	HANDLER kinetis_smclvd, KINETIS_IRQ_SMCLVD		/* Vector 36: Mode Controller low-voltage detect, low-voltage warning */
	HANDLER kinetis_llwu, KINETIS_IRQ_LLWU			/* Vector 37: LLWU Normal Low Leakage Wakeup */
	HANDLER kinetis_wdog, KINETIS_IRQ_WDOG			/* Vector 38: Watchdog */
	HANDLER kinetis_i2c0, KINETIS_IRQ_I2C0			/* Vector 40: I2C0 */
	HANDLER kinetis_i2c1, KINETIS_IRQ_I2C1			/* Vector 41: I2C1 */
	HANDLER kinetis_spi0, KINETIS_IRQ_SPI0			/* Vector 42: SPI0 all sources */
	HANDLER kinetis_spi1, KINETIS_IRQ_SPI1			/* Vector 43: SPI1 all sources */
	HANDLER kinetis_spi2, KINETIS_IRQ_SPI2			/* Vector 44: SPI2 all sources */
	HANDLER kinetis_can0mb, KINETIS_IRQ_CAN0MB		/* Vector 45: CAN0 OR'ed Message buffer (0-15) */
	HANDLER kinetis_can0bo, KINETIS_IRQ_CAN0BO		/* Vector 46: CAN0 Bus Off */
	HANDLER kinetis_can0err, KINETIS_IRQ_CAN0ERR	/* Vector 47: CAN0 Error */
	HANDLER kinetis_can0tw, KINETIS_IRQ_CAN0TW		/* Vector 48: CAN0 Transmit Warning */
	HANDLER kinetis_can0rw, KINETIS_IRQ_CAN0RW		/* Vector 49: CAN0 Receive Warning */
	HANDLER kinetis_can0wu, KINETIS_IRQ_CAN0WU		/* Vector 50: CAN0 Wake UP */
	HANDLER kinetis_can1mb, KINETIS_IRQ_CAN1MB		/* Vector 53: CAN1 OR'ed Message buffer (0-15) */
	HANDLER kinetis_can1bo, KINETIS_IRQ_CAN1BO		/* Vector 54: CAN1 Bus Off */
	HANDLER kinetis_can1err, KINETIS_IRQ_CAN1ERR	/* Vector 55: CAN1 Error */
	HANDLER kinetis_can1tw, KINETIS_IRQ_CAN1TW		/* Vector 56: CAN1 Transmit Warning */
	HANDLER kinetis_can1rw, KINETIS_IRQ_CAN1RW		/* Vector 57: CAN1 Receive Warning */
	HANDLER kinetis_can1wu, KINETIS_IRQ_CAN1WU		/* Vector 58: CAN1 Wake UP */
	HANDLER kinetis_uart0s, KINETIS_IRQ_UART0S		/* Vector 61: UART0 status */
	HANDLER kinetis_uart0e, KINETIS_IRQ_UART0E		/* Vector 62: UART0 error */
	HANDLER kinetis_uart1s, KINETIS_IRQ_UART1S		/* Vector 63: UART1 status */
	HANDLER kinetis_uart1e, KINETIS_IRQ_UART1E		/* Vector 64: UART1 error */
	HANDLER kinetis_uart2s, KINETIS_IRQ_UART2S		/* Vector 65: UART2 status */
	HANDLER kinetis_uart2e, KINETIS_IRQ_UART2E		/* Vector 66: UART2 error */
	HANDLER kinetis_uart3s, KINETIS_IRQ_UART3S		/* Vector 67: UART3 status */
	HANDLER kinetis_uart3e, KINETIS_IRQ_UART3E		/* Vector 68: UART3 error */
	HANDLER kinetis_uart4s, KINETIS_IRQ_UART4S		/* Vector 69: UART4 status */
	HANDLER kinetis_uart4e, KINETIS_IRQ_UART4E		/* Vector 70: UART4 error */
	HANDLER kinetis_uart5s, KINETIS_IRQ_UART5S		/* Vector 71: UART5 status */
	HANDLER kinetis_uart5e, KINETIS_IRQ_UART5E		/* Vector 72: UART5 error */
	HANDLER kinetis_adc0, KINETIS_IRQ_ADC0			/* Vector 73: ADC0 */
	HANDLER kinetis_adc1, KINETIS_IRQ_ADC1			/* Vector 74: ADC1 */
	HANDLER kinetis_cmp0, KINETIS_IRQ_CMP0			/* Vector 75: CMP0 */
	HANDLER kinetis_cmp1, KINETIS_IRQ_CMP1			/* Vector 76: CMP1 */
	HANDLER kinetis_cmp2, KINETIS_IRQ_CMP2			/* Vector 77: CMP2 */
	HANDLER kinetis_ftm0, KINETIS_IRQ_FTM0			/* Vector 78: FTM0 all sources */
	HANDLER kinetis_ftm1, KINETIS_IRQ_FTM1			/* Vector 79: FTM1 all sources */
	HANDLER kinetis_ftm2, KINETIS_IRQ_FTM2			/* Vector 80: FTM2 all sources */
	HANDLER kinetis_cmt, KINETIS_IRQ_CMT			/* Vector 81: CMT */
	HANDLER kinetis_rtc, KINETIS_IRQ_RTC			/* Vector 82: RTC alarm interrupt */
	HANDLER kinetis_pitch0, KINETIS_IRQ_PITCH0		/* Vector 84: PIT channel 0 */
	HANDLER kinetis_pitch1, KINETIS_IRQ_PITCH1		/* Vector 85: PIT channel 1 */
	HANDLER kinetis_pitch2, KINETIS_IRQ_PITCH2		/* Vector 86: PIT channel 2 */
	HANDLER kinetis_pitch3, KINETIS_IRQ_PITCH3		/* Vector 87: PIT channel 3 */
	HANDLER kinetis_pdb, KINETIS_IRQ_PDB			/* Vector 88: PDB */
	HANDLER kinetis_usbotg, KINETIS_IRQ_USBOTG		/* Vector 88: USB OTG */
	HANDLER kinetis_usbcd, KINETIS_IRQ_USBCD		/* Vector 90: USB charger detect */
	HANDLER kinetis_i2s0, KINETIS_IRQ_I2S0			/* Vector 95: I2S0 */
	HANDLER kinetis_sdhc, KINETIS_IRQ_SDHC			/* Vector 96: SDHC */
	HANDLER kinetis_dac0, KINETIS_IRQ_DAC0			/* Vector 97: DAC0 */
	HANDLER kinetis_dac1, KINETIS_IRQ_DAC1			/* Vector 98: DAC1 */
	HANDLER kinetis_tsi, KINETIS_IRQ_TSI			/* Vector 97: TSI all sources */
	HANDLER kinetis_mcg, KINETIS_IRQ_MCG			/* Vector 100: MCG */
	HANDLER kinetis_lpt, KINETIS_IRQ_LPT			/* Vector 101: Low power timer */
	HANDLER kinetis_slcd, KINETIS_IRQ_SLCD			/* Vector 102: Segment LCD all sources */
	HANDLER kinetis_porta, KINETIS_IRQ_PORTA		/* Vector 103: Pin detect port A */
	HANDLER kinetis_portb, KINETIS_IRQ_PORTB		/* Vector 104: Pin detect port B */
	HANDLER kinetis_portc, KINETIS_IRQ_PORTC		/* Vector 105: Pin detect port C */
	HANDLER kinetis_portd, KINETIS_IRQ_PORTD		/* Vector 106: Pin detect port D */
	HANDLER kinetis_porte, KINETIS_IRQ_PORTE		/* Vector 107: Pin detect port E */
	HANDLER kinetis_swi, KINETIS_IRQ_SWI			/* Vector 110: Software interrupt */

/* K60 Family ***********************************************************************************
 *
 * The memory map for the following parts is defined in Freescale document
 * K60P144M100SF2RM
 */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100) || defined(CONFIG_ARCH_CHIP_MK60X256VLQ100) || \
      defined(CONFIG_ARCH_CHIP_MK60N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK60N256VMD100) || \
      defined(CONFIG_ARCH_CHIP_MK60X256VMD100) || defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

	HANDLER kinetis_dmach0, KINETIS_IRQ_DMACH0		/* Vector 16: DMA channel 0 transfer complete */
	HANDLER kinetis_dmach1, KINETIS_IRQ_DMACH1		/* Vector 17: DMA channel 1 transfer complete */
	HANDLER kinetis_dmach2, KINETIS_IRQ_DMACH2		/* Vector 18: DMA channel 2 transfer complete */
	HANDLER kinetis_dmach3, KINETIS_IRQ_DMACH3		/* Vector 19: DMA channel 3 transfer complete */
	HANDLER kinetis_dmach4, KINETIS_IRQ_DMACH4		/* Vector 20: DMA channel 4 transfer complete */
	HANDLER kinetis_dmach5, KINETIS_IRQ_DMACH5		/* Vector 21: DMA channel 5 transfer complete */
	HANDLER kinetis_dmach6, KINETIS_IRQ_DMACH6		/* Vector 22: DMA channel 6 transfer complete */
	HANDLER kinetis_dmach7, KINETIS_IRQ_DMACH7		/* Vector 23: DMA channel 7 transfer complete */
	HANDLER kinetis_dmach8, KINETIS_IRQ_DMACH8		/* Vector 24: DMA channel 8 transfer complete */
	HANDLER kinetis_dmach9, KINETIS_IRQ_DMACH9		/* Vector 25: DMA channel 9 transfer complete */
	HANDLER kinetis_dmach10, KINETIS_IRQ_DMACH10	/* Vector 26: DMA channel 10 transfer complete */
	HANDLER kinetis_dmach11, KINETIS_IRQ_DMACH11	/* Vector 27: DMA channel 11 transfer complete */
	HANDLER kinetis_dmach12, KINETIS_IRQ_DMACH12	/* Vector 28: DMA channel 12 transfer complete */
	HANDLER kinetis_dmach13, KINETIS_IRQ_DMACH13	/* Vector 29: DMA channel 13 transfer complete */
	HANDLER kinetis_dmach14, KINETIS_IRQ_DMACH14	/* Vector 30: DMA channel 14 transfer complete */
	HANDLER kinetis_dmach15, KINETIS_IRQ_DMACH15	/* Vector 31: DMA channel 15 transfer complete */
	HANDLER kinetis_dmaerr, KINETIS_IRQ_DMAERR		/* Vector 32: DMA error interrupt channels 0-15 */
	HANDLER kinetis_mcm, KINETIS_IRQ_MCM			/* Vector 33: MCM Normal interrupt */
	HANDLER kinetis_flashcc, KINETIS_IRQ_FLASHCC	/* Vector 34: Flash memory command complete */
	HANDLER kinetis_flashrc, KINETIS_IRQ_FLASHRC	/* Vector 35: Flash memory read collision */
	HANDLER kinetis_smclvd, KINETIS_IRQ_SMCLVD		/* Vector 36: Mode Controller low-voltage detect, low-voltage warning */
	HANDLER kinetis_llwu, KINETIS_IRQ_LLWU			/* Vector 37: LLWU Normal Low Leakage Wakeup */
	HANDLER kinetis_wdog, KINETIS_IRQ_WDOG			/* Vector 38: Watchdog */
	HANDLER kinetis_rngb, KINETIS_IRQ_RNGB			/* Vector 39: Random number generator */
	HANDLER kinetis_i2c0, KINETIS_IRQ_I2C0			/* Vector 40: I2C0 */
	HANDLER kinetis_i2c1, KINETIS_IRQ_I2C1			/* Vector 41: I2C1 */
	HANDLER kinetis_spi0, KINETIS_IRQ_SPI0			/* Vector 42: SPI0 all sources */
	HANDLER kinetis_spi1, KINETIS_IRQ_SPI1			/* Vector 43: SPI1 all sources */
	HANDLER kinetis_spi2, KINETIS_IRQ_SPI2			/* Vector 44: SPI2 all sources */
	HANDLER kinetis_can0mb, KINETIS_IRQ_CAN0MB		/* Vector 45: CAN0 OR'ed Message buffer (0-15) */
	HANDLER kinetis_can0bo, KINETIS_IRQ_CAN0BO		/* Vector 46: CAN0 Bus Off */
	HANDLER kinetis_can0err, KINETIS_IRQ_CAN0ERR	/* Vector 47: CAN0 Error */
	HANDLER kinetis_can0tw, KINETIS_IRQ_CAN0TW		/* Vector 48: CAN0 Transmit Warning */
	HANDLER kinetis_can0rw, KINETIS_IRQ_CAN0RW		/* Vector 49: CAN0 Receive Warning */
	HANDLER kinetis_can0wu, KINETIS_IRQ_CAN0WU		/* Vector 50: CAN0 Wake UP */
	HANDLER kinetis_can1mb, KINETIS_IRQ_CAN1MB		/* Vector 53: CAN1 OR'ed Message buffer (0-15) */
	HANDLER kinetis_can1bo, KINETIS_IRQ_CAN1BO		/* Vector 54: CAN1 Bus Off */
	HANDLER kinetis_can1err, KINETIS_IRQ_CAN1ERR	/* Vector 55: CAN1 Error */
	HANDLER kinetis_can1tw, KINETIS_IRQ_CAN1TW		/* Vector 56: CAN1 Transmit Warning */
	HANDLER kinetis_can1rw, KINETIS_IRQ_CAN1RW		/* Vector 57: CAN1 Receive Warning */
	HANDLER kinetis_can1wu, KINETIS_IRQ_CAN1WU		/* Vector 58: CAN1 Wake UP */
	HANDLER kinetis_uart0s, KINETIS_IRQ_UART0S		/* Vector 61: UART0 status */
	HANDLER kinetis_uart0e, KINETIS_IRQ_UART0E		/* Vector 62: UART0 error */
	HANDLER kinetis_uart1s, KINETIS_IRQ_UART1S		/* Vector 63: UART1 status */
	HANDLER kinetis_uart1e, KINETIS_IRQ_UART1E		/* Vector 64: UART1 error */
	HANDLER kinetis_uart2s, KINETIS_IRQ_UART2S		/* Vector 65: UART2 status */
	HANDLER kinetis_uart2e, KINETIS_IRQ_UART2E		/* Vector 66: UART2 error */
	HANDLER kinetis_uart3s, KINETIS_IRQ_UART3S		/* Vector 67: UART3 status */
	HANDLER kinetis_uart3e, KINETIS_IRQ_UART3E		/* Vector 68: UART3 error */
	HANDLER kinetis_uart4s, KINETIS_IRQ_UART4S		/* Vector 69: UART4 status */
	HANDLER kinetis_uart4e, KINETIS_IRQ_UART4E		/* Vector 70: UART4 error */
	HANDLER kinetis_uart5s, KINETIS_IRQ_UART5S		/* Vector 71: UART5 status */
	HANDLER kinetis_uart5e, KINETIS_IRQ_UART5E		/* Vector 72: UART5 error */
	HANDLER kinetis_adc0, KINETIS_IRQ_ADC0			/* Vector 73: ADC0 */
	HANDLER kinetis_adc1, KINETIS_IRQ_ADC1			/* Vector 74: ADC1 */
	HANDLER kinetis_cmp0, KINETIS_IRQ_CMP0			/* Vector 75: CMP0 */
	HANDLER kinetis_cmp1, KINETIS_IRQ_CMP1			/* Vector 76: CMP1 */
	HANDLER kinetis_cmp2, KINETIS_IRQ_CMP2			/* Vector 77: CMP2 */
	HANDLER kinetis_ftm0, KINETIS_IRQ_FTM0			/* Vector 78: FTM0 all sources */
	HANDLER kinetis_ftm1, KINETIS_IRQ_FTM1			/* Vector 79: FTM1 all sources */
	HANDLER kinetis_ftm2, KINETIS_IRQ_FTM2			/* Vector 80: FTM2 all sources */
	HANDLER kinetis_cmt, KINETIS_IRQ_CMT			/* Vector 81: CMT */
	HANDLER kinetis_rtc, KINETIS_IRQ_RTC			/* Vector 82: RTC alarm interrupt */
	HANDLER kinetis_pitch0, KINETIS_IRQ_PITCH0		/* Vector 84: PIT channel 0 */
	HANDLER kinetis_pitch1, KINETIS_IRQ_PITCH1		/* Vector 85: PIT channel 1 */
	HANDLER kinetis_pitch2, KINETIS_IRQ_PITCH2		/* Vector 86: PIT channel 2 */
	HANDLER kinetis_pitch3, KINETIS_IRQ_PITCH3		/* Vector 87: PIT channel 3 */
	HANDLER kinetis_pdb, KINETIS_IRQ_PDB			/* Vector 88: PDB */
	HANDLER kinetis_usbotg, KINETIS_IRQ_USBOTG		/* Vector 88: USB OTG */
	HANDLER kinetis_usbcd, KINETIS_IRQ_USBCD		/* Vector 90: USB charger detect */
	HANDLER kinetis_emactmr, KINETIS_IRQ_EMACTMR	/* Vector 91: Ethernet MAC IEEE 1588 timer interrupt */
	HANDLER kinetis_emactx, KINETIS_IRQ_EMACTX		/* Vector 92: Ethernet MAC transmit interrupt */
	HANDLER kinetis_emacrx, KINETIS_IRQ_EMACRX		/* Vector 93: Ethernet MAC receive interrupt */
	HANDLER kinetis_emacmisc, KINETIS_IRQ_EMACMISC	/* Vector 94: Ethernet MAC error and misc interrupt */
	HANDLER kinetis_i2s0, KINETIS_IRQ_I2S0			/* Vector 95: I2S0 */
	HANDLER kinetis_sdhc, KINETIS_IRQ_SDHC			/* Vector 96: SDHC */
	HANDLER kinetis_dac0, KINETIS_IRQ_DAC0			/* Vector 97: DAC0 */
	HANDLER kinetis_dac1, KINETIS_IRQ_DAC1			/* Vector 98: DAC1 */
	HANDLER kinetis_tsi, KINETIS_IRQ_TSI			/* Vector 97: TSI all sources */
	HANDLER kinetis_mcg, KINETIS_IRQ_MCG			/* Vector 100: MCG */
	HANDLER kinetis_lpt, KINETIS_IRQ_LPT			/* Vector 101: Low power timer */
	HANDLER kinetis_porta, KINETIS_IRQ_PORTA		/* Vector 103: Pin detect port A */
	HANDLER kinetis_portb, KINETIS_IRQ_PORTB		/* Vector 104: Pin detect port B */
	HANDLER kinetis_portc, KINETIS_IRQ_PORTC		/* Vector 105: Pin detect port C */
	HANDLER kinetis_portd, KINETIS_IRQ_PORTD		/* Vector 106: Pin detect port D */
	HANDLER kinetis_porte, KINETIS_IRQ_PORTE		/* Vector 107: Pin detect port E */

#else
#  error "No handlers for this Kinetis part"
#endif

/* Common IRQ handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      R0 contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

kinetis_common:

	/* Complete the context save */

#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the state is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	adds	r2, r14, #3				/* If R14=0xfffffffd, then r2 == 0 */
	ite		ne						/* Next two instructions are condition */
	mrsne	r1, msp					/* R1=The main stack pointer */
	mrseq	r1, psp					/* R1=The process stack pointer */
#else
	mrs		r1, msp					/* R1=The main stack pointer */
#endif

	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
#ifdef CONFIG_ARMV7M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif
#ifdef CONFIG_NUTTX_KERNEL 
	stmdb	r1!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	r1!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will re-use the main stack for interrupt level processing.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr		sp, =g_intstackbase
	str		r1, [sp, #-4]!			/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	ldr		r1, [sp, #+4]!			/* Recover R1=main stack pointer */
#else
	mov		sp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, sp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		1f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather, are within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add		r1, r0, #SW_XCPT_SIZE	/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
#ifdef CONFIG_NUTTX_KERNEL 
	ldmia	r0, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
	b		2f						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 */
1:
#ifdef CONFIG_NUTTX_KERNEL 
	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
2:
#ifdef CONFIG_NUTTX_KERNEL
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the state is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	adds	r0, r14, #3				/* If R14=0xfffffffd, then r0 == 0 */
	ite		ne						/* Next two instructions are condition */
	msrne	msp, r1					/* R1=The main stack pointer */
	msreq	psp, r1					/* R1=The process stack pointer */
#else
	msr		msp, r1					/* Recover the return MSP value */

	/* Preload r14 with the special return value first (so that the return
	 * actually occurs with interrupts still disabled).
	 */

	ldr		r14, =EXC_RETURN		/* Load the special value */
#endif

    /* Restore the interrupt state */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking*/
	cpsie	i						/* Re-enable interrupts */
#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */
	.size	handlers, .-handlers

/************************************************************************************************
 *  Name: up_interruptstack/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackbase
	.align	4
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif

/************************************************************************************************
 * .rodata
 ************************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_heapbase below.
 */

	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.word	HEAP_BASE
	.size	g_heapbase, .-g_heapbase

	.end
