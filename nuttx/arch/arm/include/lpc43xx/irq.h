/********************************************************************************************
 * arch/arm/include/lpc43xxx/irq.h
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
 ********************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LPC43XX_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC43XX_IRQ_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
 
/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to bits in
 * the NVIC.  This does, however, waste several words of memory in the IRQ to handle mapping
 * tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define LPC43_IRQ_RESERVED         (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                       /* Vector  0: Reset stack pointer value */
                                       /* Vector  1: Reset (not handler as an IRQ) */
#define LPC43_IRQ_NMI              (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LPC43_IRQ_HARDFAULT        (3) /* Vector  3: Hard fault */
#define LPC43_IRQ_MEMFAULT         (4) /* Vector  4: Memory management (MPU) */
#define LPC43_IRQ_BUSFAULT         (5) /* Vector  5: Bus fault */
#define LPC43_IRQ_USAGEFAULT       (6) /* Vector  6: Usage fault */
#define LPC43_IRQ_SIGNVALUE        (7) /* Vector  7: Sign value */
#define LPC43_IRQ_SVCALL          (11) /* Vector 11: SVC call */
#define LPC43_IRQ_DBGMONITOR      (12) /* Vector 12: Debug Monitor */
                                       /* Vector 13: Reserved */
#define LPC43_IRQ_PENDSV          (14) /* Vector 14: Pendable system service request */
#define LPC43_IRQ_SYSTICK         (15) /* Vector 15: System tick */
#define LPC43_IRQ_EXTINT          (16) /* Vector 16: Vector number of the first external interrupt */

/* Cortex-M4 External interrupts (vectors >= 16) */

#define LPC43M4_IRQ_DAC           (LPC43_IRQ_EXTINT+0)  /* D/A */
#define LPC43M4_IRQ_M0CORE        (LPC43_IRQ_EXTINT+1)  /* M0 Core */
#define LPC43M4_IRQ_DMA           (LPC43_IRQ_EXTINT+2)  /* DMA */
#define LPC43M4_IRQ_FLASHEEPROM   (LPC43_IRQ_EXTINT+4)  /* EEPROM interrupts (BankA or BankB) */
#define LPC43M4_IRQ_ETHERNET      (LPC43_IRQ_EXTINT+5)  /* Ethernet interrupt */
#define LPC43M4_IRQ_SDIO          (LPC43_IRQ_EXTINT+6)  /* SD/MMC interrupt */
#define LPC43M4_IRQ_LCD           (LPC43_IRQ_EXTINT+7)  /* LCD */
#define LPC43M4_IRQ_USB0          (LPC43_IRQ_EXTINT+8)  /* USB0 OTG interrupt */
#define LPC43M4_IRQ_USB1          (LPC43_IRQ_EXTINT+9)  /* USB1 interrupt */
#define LPC43M4_IRQ_SCT           (LPC43_IRQ_EXTINT+10) /* SCT combined interrupt */
#define LPC43M4_IRQ_RITIMER       (LPC43_IRQ_EXTINT+11) /* RITIMER interrupt */
#define LPC43M4_IRQ_TIMER0        (LPC43_IRQ_EXTINT+12) /* TIMER0 interrupt */
#define LPC43M4_IRQ_TIMER1        (LPC43_IRQ_EXTINT+13) /* TIMER1 interrupt */
#define LPC43M4_IRQ_TIMER2        (LPC43_IRQ_EXTINT+14) /* TIMER2 interrupt */
#define LPC43M4_IRQ_TIMER3        (LPC43_IRQ_EXTINT+15) /* TIMER3 interrupt */
#define LPC43M4_IRQ_MCPWM         (LPC43_IRQ_EXTINT+16) /* Motor control PWM interrupt */
#define LPC43M4_IRQ_ADC0          (LPC43_IRQ_EXTINT+17) /* ADC0 interrupt */
#define LPC43M4_IRQ_I2C0          (LPC43_IRQ_EXTINT+18) /* I2C0 interrupt */
#define LPC43M4_IRQ_I2C1          (LPC43_IRQ_EXTINT+19) /* I2C1 interrupt */
#define LPC43M4_IRQ_SPI           (LPC43_IRQ_EXTINT+20) /* SPI interrupt */
#define LPC43M4_IRQ_ADC1          (LPC43_IRQ_EXTINT+21) /* ADC1 interrupt */
#define LPC43M4_IRQ_SSP0          (LPC43_IRQ_EXTINT+22) /* SSP0 interrupt */
#define LPC43M4_IRQ_SSP1          (LPC43_IRQ_EXTINT+23) /* SSP1 interrupt */
#define LPC43M4_IRQ_USART0        (LPC43_IRQ_EXTINT+24) /* USART0 interrupt */
#define LPC43M4_IRQ_UART1         (LPC43_IRQ_EXTINT+25) /* UART1/Modem interrupt */
#define LPC43M4_IRQ_USART2        (LPC43_IRQ_EXTINT+26) /* USART2 interrupt */
#define LPC43M4_IRQ_USART3        (LPC43_IRQ_EXTINT+27) /* USART3/IrDA interrupt */
#define LPC43M4_IRQ_I2S0          (LPC43_IRQ_EXTINT+28) /* I2S0 interrupt */
#define LPC43M4_IRQ_I2S1          (LPC43_IRQ_EXTINT+29) /* I2S1 interrupt */
#define LPC43M4_IRQ_SPIFI         (LPC43_IRQ_EXTINT+30) /* SPIFI interrupt */
#define LPC43M4_IRQ_SGPIO         (LPC43_IRQ_EXTINT+31) /* SGPIO interrupt */
#define LPC43M4_IRQ_PININT0       (LPC43_IRQ_EXTINT+32) /* GPIO pin interrupt 0 */
#define LPC43M4_IRQ_PININT1       (LPC43_IRQ_EXTINT+33) /* GPIO pin interrupt 1 */
#define LPC43M4_IRQ_PININT2       (LPC43_IRQ_EXTINT+34) /* GPIO pin interrupt 2 */
#define LPC43M4_IRQ_PININT3       (LPC43_IRQ_EXTINT+35) /* GPIO pin interrupt 3 */
#define LPC43M4_IRQ_PININT4       (LPC43_IRQ_EXTINT+36) /* GPIO pin interrupt 4 */
#define LPC43M4_IRQ_PININT5       (LPC43_IRQ_EXTINT+37) /* GPIO pin interrupt 5 */
#define LPC43M4_IRQ_PININT6       (LPC43_IRQ_EXTINT+38) /* GPIO pin interrupt 6 */
#define LPC43M4_IRQ_PININT7       (LPC43_IRQ_EXTINT+39) /* GPIO pin interrupt 7 */
#define LPC43M4_IRQ_GINT0         (LPC43_IRQ_EXTINT+40) /* GPIO global interrupt 0 */
#define LPC43M4_IRQ_GINT1         (LPC43_IRQ_EXTINT+41) /* GPIO global interrupt 1 */
#define LPC43M4_IRQ_EVENTROUTER   (LPC43_IRQ_EXTINT+42) /* Event router interrupt */
#define LPC43M4_IRQ_CAN1          (LPC43_IRQ_EXTINT+43) /* C_CAN1 interrupt */
#define LPC43M4_IRQ_ATIMER        (LPC43_IRQ_EXTINT+46) /* ATIMER Alarm timer interrupt */
#define LPC43M4_IRQ_RTC           (LPC43_IRQ_EXTINT+47) /* RTC interrupt */
#define LPC43M4_IRQ_WWDT          (LPC43_IRQ_EXTINT+49) /* WWDT interrupt */
#define LPC43M4_IRQ_CAN0          (LPC43_IRQ_EXTINT+51) /* C_CAN0 interrupt */
#define LPC43M4_IRQ_QEI           (LPC43_IRQ_EXTINT+52) /* QEI interrupt */

#define LPC43M4_IRQ_NEXTINT       (53)
#define LPC43M4_IRQ_NIRQS         (LPC43_IRQ_EXTINT+LPC43M4_IRQ_NEXTINT)

/* Cortex-M4 GPIO interrupts.  The LPC43xx supports several interrupts on ports 0 and 2
 * (only).  We go through some special efforts to keep the number of IRQs to a minimum in
 * this sparse interrupt case.
 *
 * 28 interrupts on Port 0:  p0.0 - p0.11, p0.15-p0.30
 * 14 interrupts on Port 2:  p2.0 - p2.13
 * --
 * 42
 */

#ifdef CONFIG_GPIO_IRQ
#  warning "REVISIT"
#  define LPC43M4_VALID_GPIOINT0  (0x7fff8ffful) /* GPIO port 0 interrrupt set */
#  define LPC43M4_VALID_GPIOINT2  (0x00003ffful) /* GPIO port 2 interrupt set */

   /* Set 1: 12 interrupts p0.0-p0.11 */

#  define LPC43M4_VALID_GPIOINT0L (0x00000ffful)
#  define LPC43M4_VALID_SHIFT0L   (0)
#  define LPC43M4_VALID_FIRST0L   (LPC43_IRQ_EXTINT+LPC43M4_IRQ_NEXTINT)

#  define LPC43M4_IRQ_P0p0        (LPC43M4_VALID_FIRST0L+0)
#  define LPC43M4_IRQ_P0p1        (LPC43M4_VALID_FIRST0L+1)
#  define LPC43M4_IRQ_P0p2        (LPC43M4_VALID_FIRST0L+2)
#  define LPC43M4_IRQ_P0p3        (LPC43M4_VALID_FIRST0L+3)
#  define LPC43M4_IRQ_P0p4        (LPC43M4_VALID_FIRST0L+4)
#  define LPC43M4_IRQ_P0p5        (LPC43M4_VALID_FIRST0L+5)
#  define LPC43M4_IRQ_P0p6        (LPC43M4_VALID_FIRST0L+6)
#  define LPC43M4_IRQ_P0p7        (LPC43M4_VALID_FIRST0L+7)
#  define LPC43M4_IRQ_P0p8        (LPC43M4_VALID_FIRST0L+8)
#  define LPC43M4_IRQ_P0p9        (LPC43M4_VALID_FIRST0L+9)
#  define LPC43M4_IRQ_P0p10       (LPC43M4_VALID_FIRST0L+10)
#  define LPC43M4_IRQ_P0p11       (LPC43M4_VALID_FIRST0L+11)
#  define LPC43M4_VALID_NIRQS0L   (12)

   /* Set 2: 16 interrupts p0.15-p0.30 */

#  define LPC43M4_VALID_GPIOINT0H (0x7fff8000ull)
#  define LPC43M4_VALID_SHIFT0H   (15)
#  define LPC43M4_VALID_FIRST0H   (LPC43M4_VALID_FIRST0L+LPC43M4_VALID_NIRQS0L)

#  define LPC43M4_IRQ_P0p15       (LPC43M4_VALID_FIRST0H+0)
#  define LPC43M4_IRQ_P0p16       (LPC43M4_VALID_FIRST0H+1)
#  define LPC43M4_IRQ_P0p17       (LPC43M4_VALID_FIRST0H+2)
#  define LPC43M4_IRQ_P0p18       (LPC43M4_VALID_FIRST0H+3)
#  define LPC43M4_IRQ_P0p19       (LPC43M4_VALID_FIRST0H+4)
#  define LPC43M4_IRQ_P0p20       (LPC43M4_VALID_FIRST0H+5)
#  define LPC43M4_IRQ_P0p21       (LPC43M4_VALID_FIRST0H+6)
#  define LPC43M4_IRQ_P0p22       (LPC43M4_VALID_FIRST0H+7)
#  define LPC43M4_IRQ_P0p23       (LPC43M4_VALID_FIRST0H+8)
#  define LPC43M4_IRQ_P0p24       (LPC43M4_VALID_FIRST0H+9)
#  define LPC43M4_IRQ_P0p25       (LPC43M4_VALID_FIRST0H+10)
#  define LPC43M4_IRQ_P0p26       (LPC43M4_VALID_FIRST0H+11)
#  define LPC43M4_IRQ_P0p27       (LPC43M4_VALID_FIRST0H+12)
#  define LPC43M4_IRQ_P0p28       (LPC43M4_VALID_FIRST0H+13)
#  define LPC43M4_IRQ_P0p29       (LPC43M4_VALID_FIRST0H+14)
#  define LPC43M4_IRQ_P0p30       (LPC43M4_VALID_FIRST0H+15)
#  define LPC43M4_VALID_NIRQS0H   (16)

   /* Set 3: 14 interrupts p2.0-p2.13 */

#  define LPC43M4_VALID_GPIOINT2  (0x00003ffful)
#  define LPC43M4_VALID_SHIFT2    (0)
#  define LPC43M4_VALID_FIRST2    (LPC43M4_VALID_FIRST0H+LPC43M4_VALID_NIRQS0H)

#  define LPC43M4_IRQ_P2p0        (LPC43M4_VALID_FIRST2+0)
#  define LPC43M4_IRQ_P2p1        (LPC43M4_VALID_FIRST2+1)
#  define LPC43M4_IRQ_P2p2        (LPC43M4_VALID_FIRST2+2)
#  define LPC43M4_IRQ_P2p3        (LPC43M4_VALID_FIRST2+3)
#  define LPC43M4_IRQ_P2p4        (LPC43M4_VALID_FIRST2+4)
#  define LPC43M4_IRQ_P2p5        (LPC43M4_VALID_FIRST2+5)
#  define LPC43M4_IRQ_P2p6        (LPC43M4_VALID_FIRST2+6)
#  define LPC43M4_IRQ_P2p7        (LPC43M4_VALID_FIRST2+7)
#  define LPC43M4_IRQ_P2p8        (LPC43M4_VALID_FIRST2+8)
#  define LPC43M4_IRQ_P2p9        (LPC43M4_VALID_FIRST2+9)
#  define LPC43M4_IRQ_P2p10       (LPC43M4_VALID_FIRST2+10)
#  define LPC43M4_IRQ_P2p11       (LPC43M4_VALID_FIRST2+11)
#  define LPC43M4_IRQ_P2p12       (LPC43M4_VALID_FIRST2+12)
#  define LPC43M4_IRQ_P2p13       (LPC43M4_VALID_FIRST2+13)
#  define LPC43M4_VALID_NIRQS2    (14)
#  define LPC43M4_NGPIOAIRQS      (LPC43M4_VALID_NIRQS0L+LPC43M4_VALID_NIRQS0H+LPC43M4_VALID_NIRQS2)
#else
#  define LPC43M4_NGPIOAIRQS      (0)
#endif

/* Total number of IRQ numbers (This will need to be revisited if/when the Cortex-M0 is
 * supported
 */

#define NR_IRQS                   (LPC43_IRQ_EXTINT+LPC43M4_IRQ_NEXTINT+LPC43M4_NGPIOAIRQS)

/* Cortex-M0 External interrupts (vectors >= 16) */

#define LPC43M0_IRQ_RTC           (LPC43_IRQ_EXTINT+0)  /* RT interrupt */
#define LPC43M0_IRQ_M4CORE        (LPC43_IRQ_EXTINT+1)  /* Interrupt from the M4 core */
#define LPC43M0_IRQ_DMA           (LPC43_IRQ_EXTINT+2)  /* DMA interrupt */
#define LPC43M0_IRQ_FLASHEEPROM   (LPC43_IRQ_EXTINT+4)  /* EEPROM (Bank A or B) | A Timer */
#define LPC43M0_IRQ_ATIMER        (LPC43_IRQ_EXTINT+4)  /* EEPROM (Bank A or B) | A Timer */
#define LPC43M0_IRQ_ETHERNET      (LPC43_IRQ_EXTINT+5)  /* Ethernet interrupt */
#define LPC43M0_IRQ_SDIO          (LPC43_IRQ_EXTINT+6)  /* SDIO interrupt */
#define LPC43M0_IRQ_LCD           (LPC43_IRQ_EXTINT+7)  /* LCD interrupt */
#define LPC43M0_IRQ_USB0          (LPC43_IRQ_EXTINT+8)  /* USB0 OTG interrupt */
#define LPC43M0_IRQ_USB1          (LPC43_IRQ_EXTINT+9)  /* USB1 interrupt */
#define LPC43M0_IRQ_SCT           (LPC43_IRQ_EXTINT+10) /* SCT combined interrupt */
#define LPC43M0_IRQ_RITIMER       (LPC43_IRQ_EXTINT+11) /* RI Timer | WWDT interrupt */
#define LPC43M0_IRQ_WWDT          (LPC43_IRQ_EXTINT+11) /* RI Timer | WWDT interrupt */
#define LPC43M0_IRQ_TIMER0        (LPC43_IRQ_EXTINT+12) /* TIMER0 interrupt */
#define LPC43M0_IRQ_GINT1         (LPC43_IRQ_EXTINT+13) /* GINT1 GPIO global interrupt 1 */
#define LPC43M0_IRQ_PININT4       (LPC43_IRQ_EXTINT+14) /* GPIO pin interrupt 4 */
#define LPC43M0_IRQ_TIMER3        (LPC43_IRQ_EXTINT+15) /* TIMER interrupt */
#define LPC43M0_IRQ_MCPWM         (LPC43_IRQ_EXTINT+16) /* Motor control PWM interrupt */
#define LPC43M0_IRQ_ADC0          (LPC43_IRQ_EXTINT+17) /* ADC0 interrupt */
#define LPC43M0_IRQ_I2C0          (LPC43_IRQ_EXTINT+18) /* I2C0 | I2C1 interrupt */
#define LPC43M0_IRQ_I2C1          (LPC43_IRQ_EXTINT+18) /* I2C0 | I2C1 interrupt */
#define LPC43M0_IRQ_SGPIO         (LPC43_IRQ_EXTINT+19) /* SGPIO interrupt */
#define LPC43M0_IRQ_SPI           (LPC43_IRQ_EXTINT+20) /* SPI | DAC interrupt */
#define LPC43M0_IRQ_DAC           (LPC43_IRQ_EXTINT+20) /* SPI | DAC interrupt */
#define LPC43M0_IRQ_ADC1          (LPC43_IRQ_EXTINT+21) /* ADC1 interrupt */
#define LPC43M0_IRQ_SSP0          (LPC43_IRQ_EXTINT+22) /* SSP0 | SSP1 interrupt */
#define LPC43M0_IRQ_SSP1          (LPC43_IRQ_EXTINT+22) /* SSP0 | SSP1 interrupt */
#define LPC43M0_IRQ_EVENTROUTER   (LPC43_IRQ_EXTINT+23) /* Event router interrupt */
#define LPC43M0_IRQ_USART0        (LPC43_IRQ_EXTINT+24) /* USART0 interrupt */
#define LPC43M0_IRQ_UART1         (LPC43_IRQ_EXTINT+25) /* UART1 Modem/UART1 interrupt */
#define LPC43M0_IRQ_USART2        (LPC43_IRQ_EXTINT+26) /* USART2 | C_CAN1 interrupt */
#define LPC43M0_IRQ_CAN1          (LPC43_IRQ_EXTINT+26) /* USART2 | C_CAN1 interrupt */
#define LPC43M0_IRQ_USART3        (LPC43_IRQ_EXTINT+27) /* USART3 interrupt */
#define LPC43M0_IRQ_I2S0          (LPC43_IRQ_EXTINT+28) /* I2S0 | I2S1 | QEI interrupt */
#define LPC43M0_IRQ_I2S1          (LPC43_IRQ_EXTINT+28) /* I2S0 | I2S1 | QEI interrupt */
#define LPC43M0_IRQ_QEI           (LPC43_IRQ_EXTINT+28) /* I2S0 | I2S1 | QEI interrupt */
#define LPC43M0_IRQ_CAN0          (LPC43_IRQ_EXTINT+29) /* C_CAN0 interrupt */

#define LPC43M0_IRQ_NEXTINT       (30)
#define LPC43M0_IRQ_NIRQS         (LPC43_IRQ_EXTINT+LPC43M0_IRQ_NEXTINT)

/* Cortex-M0 GPIO interrupts */

#ifdef CONFIG_GPIO_IRQ
#  warning "REVISIT"
#  define LPC43M0_NGPIOAIRQS      (0)
#else
#  define LPC43M0_NGPIOAIRQS      (0)
#endif

/* Total number of IRQ numbers (This will need to be revisited if/when the Cortex-M0 is
 * supported)
 */

#if 0
#define NR_IRQS                   (LPC43_IRQ_EXTINT+LPC43M0_IRQ_NEXTINT+LPC43M0_NGPIOAIRQS)
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

/********************************************************************************************
 * Inline functions
 ********************************************************************************************/

/********************************************************************************************
 * Public Variables
 ********************************************************************************************/

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LPC43XX_IRQ_H */

