/****************************************************************************
 * arch/lpc17xx/lpc176x_irq.h
 *
 *   Copyright (C) 2010-2011, 2013 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LPC17XX_LPC176X_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC17XX_LPC176X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IRQ numbers.  The IRQ number corresponds vector number and hence map
 * directly to bits in the NVIC.  This does, however, waste several words of
 * memory in the IRQ to handle mapping tables.
 */

/* External interrupts (vectors >= 16) */

#define LPC17_IRQ_WDT           (LPC17_IRQ_EXTINT+0)  /* WDT Watchdog Interrupt (WDINT) */
#define LPC17_IRQ_TMR0          (LPC17_IRQ_EXTINT+1)  /* Timer 0 Match 0 - 1 (MR0, MR1)
                                                       * Capture 0 - 1 (CR0, CR1) */
#define LPC17_IRQ_TMR1          (LPC17_IRQ_EXTINT+2)  /* Timer 1 Match 0 - 2 (MR0, MR1, MR2)
                                                       * Capture 0 - 1 (CR0, CR1) */
#define LPC17_IRQ_TMR2          (LPC17_IRQ_EXTINT+3)  /* Timer 2 Match 0-3
                                                       * Capture 0-1 */
#define LPC17_IRQ_TMR3          (LPC17_IRQ_EXTINT+4)  /* Timer 3 Match 0-3
                                                       * Capture 0-1 */
#define LPC17_IRQ_UART0         (LPC17_IRQ_EXTINT+5)  /* UART 0 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART1         (LPC17_IRQ_EXTINT+6)  /* UART 1 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * Modem Control Change
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART2         (LPC17_IRQ_EXTINT+7)  /* UART 2 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_UART3         (LPC17_IRQ_EXTINT+8)  /* UART 3 Rx Line Status (RLS)
                                                       * Transmit Holding Register Empty (THRE)
                                                       * Rx Data Available (RDA)
                                                       * Character Time-out Indicator (CTI)
                                                       * End of Auto-Baud (ABEO)
                                                       * Auto-Baud Time-Out (ABTO) */
#define LPC17_IRQ_PWM1          (LPC17_IRQ_EXTINT+9)  /* PWM1 Match 0 - 6 of PWM1
                                                       * Capture 0-1 of PWM1 */
#define LPC17_IRQ_I2C0          (LPC17_IRQ_EXTINT+10) /* I2C0 SI (state change) */
#define LPC17_IRQ_I2C1          (LPC17_IRQ_EXTINT+11) /* I2C1 SI (state change) */
#define LPC17_IRQ_I2C2          (LPC17_IRQ_EXTINT+12) /* I2C2 SI (state change) */
#define LPC17_IRQ_SPIF          (LPC17_IRQ_EXTINT+13) /* SPI SPI Interrupt Flag (SPIF)
                                                       * Mode Fault (MODF) */
#define LPC17_IRQ_SSP0          (LPC17_IRQ_EXTINT+14) /* SSP0 Tx FIFO half empty of SSP0
                                                       * Rx FIFO half full of SSP0
                                                       * Rx Timeout of SSP0
                                                       * Rx Overrun of SSP0 */
#define LPC17_IRQ_SSP1          (LPC17_IRQ_EXTINT+15) /* SSP 1 Tx FIFO half empty
                                                       * Rx FIFO half full
                                                       * Rx Timeout
                                                       * Rx Overrun */
#define LPC17_IRQ_PLL0          (LPC17_IRQ_EXTINT+16) /* PLL0 (Main PLL) PLL0 Lock (PLOCK0) */
#define LPC17_IRQ_RTC           (LPC17_IRQ_EXTINT+17) /* RTC Counter Increment (RTCCIF)
                                                       * Alarm (RTCALF) */
#define LPC17_IRQ_EINT0         (LPC17_IRQ_EXTINT+18) /* External Interrupt 0 (EINT0) */
#define LPC17_IRQ_EINT1         (LPC17_IRQ_EXTINT+19) /* External Interrupt 1 (EINT1) */
#define LPC17_IRQ_EINT2         (LPC17_IRQ_EXTINT+20) /* External Interrupt 2 (EINT2) */
#define LPC17_IRQ_EINT3         (LPC17_IRQ_EXTINT+21) /* External Interrupt 3 (EINT3)
                                                       * Note: EINT3 channel is shared with GPIO interrupts */
#define LPC17_IRQ_ADC           (LPC17_IRQ_EXTINT+22) /* ADC A/D Converter end of conversion */
#define LPC17_IRQ_BOD           (LPC17_IRQ_EXTINT+23) /* BOD Brown Out detect */
#define LPC17_IRQ_USB           (LPC17_IRQ_EXTINT+24) /* USB USB_INT_REQ_LP, USB_INT_REQ_HP,
                                                       * USB_INT_REQ_DMA */
#define LPC17_IRQ_CAN           (LPC17_IRQ_EXTINT+25) /* CAN CAN Common, CAN 0 Tx, CAN 0 Rx,
                                                       *                 CAN 1 Tx, CAN 1 Rx */
#define LPC17_IRQ_GPDMA         (LPC17_IRQ_EXTINT+26) /* GPDMA IntStatus of DMA channel 0,
                                                       *       IntStatus of DMA channel 1 */
#define LPC17_IRQ_I2S           (LPC17_IRQ_EXTINT+27) /* I2S irq, dmareq1, dmareq2 */
#define LPC17_IRQ_ETH           (LPC17_IRQ_EXTINT+28) /* Ethernet WakeupInt, SoftInt, TxDoneInt,
                                                       * TxFinishedInt, TxErrorInt,* TxUnderrunInt,
                                                       * RxDoneInt, RxFinishedInt, RxErrorInt,
                                                       * RxOverrunInt */
#define LPC17_IRQ_RITINT        (LPC17_IRQ_EXTINT+29) /* Repetitive Interrupt Timer (RITINT) */
#define LPC17_IRQ_MCPWM         (LPC17_IRQ_EXTINT+30) /* Motor Control PWM IPER[2:0], IPW[2:0],
                                                       * ICAP[2:0], FES */
#define LPC17_IRQ_QEI           (LPC17_IRQ_EXTINT+31) /* Quadrature Encoder INX_Int, TIM_Int, VELC_Int,
                                                       * DIR_Int, ERR_Int, ENCLK_Int, POS0_Int, POS1_Int
                                                       * POS2_Int, REV_Int, POS0REV_Int, OS1REV_Int,
                                                       * POS2REV_Int */
#define LPC17_IRQ_PLL1          (LPC17_IRQ_EXTINT+32) /* PLL1 (USB PLL) PLL1 Lock (PLOCK1) */
#define LPC17_IRQ_USBACT        (LPC17_IRQ_EXTINT+33) /* USB Activity Interrupt USB_NEED_CLK */
#define LPC17_IRQ_CANACT        (LPC17_IRQ_EXTINT+34) /* CAN Activity Interrupt CAN1WAKE, CAN2WAKE */
#define LPC17_IRQ_NEXTINT       (35)
#define LPC17_IRQ_NIRQS         (LPC17_IRQ_EXTINT+LPC17_IRQ_NEXTINT)

/* GPIO interrupts.  The LPC17xx supports several interrupts on ports 0 and
 * 2 (only).  We go through some special efforts to keep the number of IRQs
 * to a minimum in this sparse interrupt case.
 *
 * 28 interrupts on Port 0:  p0.0 - p0.11, p0.15-p0.30
 * 14 interrupts on Port 2:  p2.0 - p2.13
 * --
 * 42
 */

#ifdef CONFIG_GPIO_IRQ
#  define LPC17_VALID_GPIOINT0  (0x7fff8ffful) /* GPIO port 0 interrrupt set */
#  define LPC17_VALID_GPIOINT2  (0x00003ffful) /* GPIO port 2 interrupt set */

   /* Set 1: 12 interrupts p0.0-p0.11 */

#  define LPC17_VALID_GPIOINT0L (0x00000ffful)
#  define LPC17_VALID_SHIFT0L   (0)
#  define LPC17_VALID_FIRST0L   (LPC17_IRQ_EXTINT+LPC17_IRQ_NEXTINT)

#  define LPC17_IRQ_P0p0        (LPC17_VALID_FIRST0L+0)
#  define LPC17_IRQ_P0p1        (LPC17_VALID_FIRST0L+1)
#  define LPC17_IRQ_P0p2        (LPC17_VALID_FIRST0L+2)
#  define LPC17_IRQ_P0p3        (LPC17_VALID_FIRST0L+3)
#  define LPC17_IRQ_P0p4        (LPC17_VALID_FIRST0L+4)
#  define LPC17_IRQ_P0p5        (LPC17_VALID_FIRST0L+5)
#  define LPC17_IRQ_P0p6        (LPC17_VALID_FIRST0L+6)
#  define LPC17_IRQ_P0p7        (LPC17_VALID_FIRST0L+7)
#  define LPC17_IRQ_P0p8        (LPC17_VALID_FIRST0L+8)
#  define LPC17_IRQ_P0p9        (LPC17_VALID_FIRST0L+9)
#  define LPC17_IRQ_P0p10       (LPC17_VALID_FIRST0L+10)
#  define LPC17_IRQ_P0p11       (LPC17_VALID_FIRST0L+11)
#  define LPC17_VALID_NIRQS0L   (12)

   /* Set 2: 16 interrupts p0.15-p0.30 */

#  define LPC17_VALID_GPIOINT0H (0x7fff8000ull)
#  define LPC17_VALID_SHIFT0H   (15)
#  define LPC17_VALID_FIRST0H   (LPC17_VALID_FIRST0L+LPC17_VALID_NIRQS0L)

#  define LPC17_IRQ_P0p15       (LPC17_VALID_FIRST0H+0)
#  define LPC17_IRQ_P0p16       (LPC17_VALID_FIRST0H+1)
#  define LPC17_IRQ_P0p17       (LPC17_VALID_FIRST0H+2)
#  define LPC17_IRQ_P0p18       (LPC17_VALID_FIRST0H+3)
#  define LPC17_IRQ_P0p19       (LPC17_VALID_FIRST0H+4)
#  define LPC17_IRQ_P0p20       (LPC17_VALID_FIRST0H+5)
#  define LPC17_IRQ_P0p21       (LPC17_VALID_FIRST0H+6)
#  define LPC17_IRQ_P0p22       (LPC17_VALID_FIRST0H+7)
#  define LPC17_IRQ_P0p23       (LPC17_VALID_FIRST0H+8)
#  define LPC17_IRQ_P0p24       (LPC17_VALID_FIRST0H+9)
#  define LPC17_IRQ_P0p25       (LPC17_VALID_FIRST0H+10)
#  define LPC17_IRQ_P0p26       (LPC17_VALID_FIRST0H+11)
#  define LPC17_IRQ_P0p27       (LPC17_VALID_FIRST0H+12)
#  define LPC17_IRQ_P0p28       (LPC17_VALID_FIRST0H+13)
#  define LPC17_IRQ_P0p29       (LPC17_VALID_FIRST0H+14)
#  define LPC17_IRQ_P0p30       (LPC17_VALID_FIRST0H+15)
#  define LPC17_VALID_NIRQS0H   (16)

   /* Set 3: 14 interrupts p2.0-p2.13 */

#  define LPC17_VALID_GPIOINT2  (0x00003ffful)
#  define LPC17_VALID_SHIFT2    (0)
#  define LPC17_VALID_FIRST2    (LPC17_VALID_FIRST0H+LPC17_VALID_NIRQS0H)

#  define LPC17_IRQ_P2p0        (LPC17_VALID_FIRST2+0)
#  define LPC17_IRQ_P2p1        (LPC17_VALID_FIRST2+1)
#  define LPC17_IRQ_P2p2        (LPC17_VALID_FIRST2+2)
#  define LPC17_IRQ_P2p3        (LPC17_VALID_FIRST2+3)
#  define LPC17_IRQ_P2p4        (LPC17_VALID_FIRST2+4)
#  define LPC17_IRQ_P2p5        (LPC17_VALID_FIRST2+5)
#  define LPC17_IRQ_P2p6        (LPC17_VALID_FIRST2+6)
#  define LPC17_IRQ_P2p7        (LPC17_VALID_FIRST2+7)
#  define LPC17_IRQ_P2p8        (LPC17_VALID_FIRST2+8)
#  define LPC17_IRQ_P2p9        (LPC17_VALID_FIRST2+9)
#  define LPC17_IRQ_P2p10       (LPC17_VALID_FIRST2+10)
#  define LPC17_IRQ_P2p11       (LPC17_VALID_FIRST2+11)
#  define LPC17_IRQ_P2p12       (LPC17_VALID_FIRST2+12)
#  define LPC17_IRQ_P2p13       (LPC17_VALID_FIRST2+13)
#  define LPC17_VALID_NIRQS2    (14)
#  define LPC17_NGPIOAIRQS      (LPC17_VALID_NIRQS0L+LPC17_VALID_NIRQS0H+LPC17_VALID_NIRQS2)
#else
#  define LPC17_NGPIOAIRQS      (0)
#endif

/* Total number of IRQ numbers */

#define NR_IRQS                 (LPC17_IRQ_EXTINT+LPC17_IRQ_NEXTINT+LPC17_NGPIOAIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_LPC17XX_LPC176X_IRQ_H */

