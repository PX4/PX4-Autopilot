/************************************************************************************
 * arch/arm/include/str71x/irq.h
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STR71X_IRQ_H
#define __ARCH_ARM_INCLUDE_STR71X_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ channels */

#define STR71X_IRQ_T0TIMI      (0) /* IRQ 0:  T0.EFTI Timer 0 global interrupt */
#define STR71X_IRQ_FLASH       (1) /* IRQ 1:  FLASH FLASH global interrupt */
#define STR71X_IRQ_RCCU        (2) /* IRQ 2:  PRCCU PRCCU global interrupt */
#define STR71X_IRQ_RTC         (3) /* IRQ 3:  RTC Real Time Clock global interrupt */
#define STR71X_IRQ_WDG         (4) /* IRQ 4:  WDG.IRQ Watchdog timer interrupt */
#define STR71X_IRQ_XTI         (5) /* IRQ 5:  XTI.IRQ XTI external interrupt */
#define STR71X_IRQ_USBHP       (6) /* IRQ 6:  USB.HPIRQ USB high priority event interrupt */
#define STR71X_IRQ_I2C0ITERR   (7) /* IRQ 7:  I2C0.ITERR I2C 0 error interrupt */
#define STR71X_IRQ_I2C1ITERR   (8) /* IRQ 8:  I2C1.ITERR I2C 1 error interrupt */
#define STR71X_IRQ_UART0       (9) /* IRQ 9:  UART0.IRQ UART 0 global interrupt */
#define STR71X_IRQ_UART1      (10) /* IRQ 10: UART1.IRQ UART 1 global interrupt */
#define STR71X_IRQ_UART2      (11) /* IRQ 11: UART2.IRQ UART 2 global interrupt */
#define STR71X_IRQ_UART3      (12) /* IRQ 12: UART3.IRQ UART 3 global interrupt */
#define STR71X_IRQ_SPI0       (13) /* IRQ 13: BSPI0.IRQ BSPI 0 global interrupt */
#define STR71X_IRQ_SPI1       (14) /* IRQ 14: BSPI1.IRQ BSPI 1 global interrupt */
#define STR71X_IRQ_I2C0       (15) /* IRQ 15: I2C0.IRQ I2C 0 tx/rx interrupt */
#define STR71X_IRQ_I2C1       (16) /* IRQ 16: I2C1.IRQ I2C 1 tx/rx interrupt */
#define STR71X_IRQ_CAN        (17) /* IRQ 17: CAN.IRQ CAN module global interrupt */
#define STR71X_IRQ_ADC        (18) /* IRQ 18: ADC.IRQ ADC sample ready interrupt */
#define STR71X_IRQ_T1TIMI     (19) /* IRQ 19: T1.GI Timer 1 global interrupt */
#define STR71X_IRQ_T2TIMI     (20) /* IRQ 20: T2.GI Timer 2 global interrupt */
#define STR71X_IRQ_T3TIMI     (21) /* IRQ 21: T3.GI Timer 3 global interrupt */
                                   /* IRQ 22-24: Reserved */
#define STR71X_IRQ_HDLC       (25) /* IRQ 25: HDLC.IRQ HDLC global interrupt */
#define STR71X_IRQ_USBLP      (26) /* IRQ 26: USB.LPIRQ USB low priority event interrupt */
                                   /* IRQ 27-28: Reserved */
#define STR71X_IRQ_T0TOI      (29) /* IRQ 29: T0.TOI Timer 0 Overflow interrupt */
#define STR71X_IRQ_T0OC1      (30) /* IRQ 30: T0.OCMPA Timer 0 Output Compare A interrupt */
#define STR71X_IRQ_T0OC2      (31) /* IRQ 31: T0.OCMPB Timer 0 Output Compare B interrupt */

#define STR71X_NBASEIRQS      (32)

#ifdef CONFIG_STR71X_XTI
#  define STR71X_IRQ_FIRSTXTI (32)
#  define STR71X_IRQ_SW       (32) /* Line 0:  SW interrupt - no HW connection */
#  define STR71X_IRQ_USBWKUP  (33) /* Line 1:  USB wake-up event: generated while exiting
                                    *          from suspend mode */
#  define STR71X_IRQ_PORT2p8  (34) /* Line 2:  Port 2.8 - External Interrupt */
#  define STR71X_IRQ_PORT2p9  (35) /* Line 3:  Port 2.9 - External Interrupt */
#  define STR71X_IRQ_PORT2p10 (36) /* Line 4:  Port 2.10 - External Interrupt */
#  define STR71X_IRQ_PORT2p11 (37) /* Line 5:  Port 2.11 - External Interrupt */
#  define STR71X_IRQ_PORT1p11 (38) /* Line 6:  Port 1.11 - CAN module receive pin (CANRX). */
#  define STR71X_IRQ_PORT1p13 (39) /* Line 7:  Port 1.13 - HDLC clock (HCLK) or I2C.0 Clock
                                    *          (I0.SCL) */
#  define STR71X_IRQ_PORT1p14 (40) /* Line 8:  Port 1.14 - HDLC receive pin (HRXD) or I2C.0
                                    *          Data (SDA) */
#  define STR71X_IRQ_PORT0p1  (41) /* Line 9:  Port 0.1 - BSPI0 Slave Input data (S0.MOSI)
                                    *          or UART3 Receive Data Input (U3.Rx) */
#  define STR71X_IRQ_PORT0p2  (42) /* Line 10: Port 0.2 - BSPI0 Slave Input serial clock
                                    *          (S0.SCLK) or I2C.1 Clock (I1.SCL) */
#  define STR71X_IRQ_PORT0p6  (43) /* Line 11: Port 0.6 - BSPI1 Slave Input serial clock
                                    *          (S1.SCLK) */
#  define STR71X_IRQ_PORT0p8  (44) /* Line 12: Port 0.8 - UART0 Receive Data Input (U0.Rx) */
#  define STR71X_IRQ_PORT0p10 (45) /* Line 13: Port 0.10 - UART1 Receive Data Input (U1.Rx) */
#  define STR71X_IRQ_PORT0p13 (46) /* Line 14: Port 0.13 - UART2 Receive Data Input (U2.Rx) */
#  define STR71X_IRQ_PORT0p15 (47) /* Line 15: Port 0.15 - WAKEUP pin or RTC ALARM */
#  define NR_IRQS             (48)
#else
#  define NR_IRQS             (32)
#endif

#define STR71X_IRQ_SYSTIMER  STR71X_IRQ_T0TIMI

/* FIQ channels */

#define STR71X_FIQ_T0TIMI     (0X00000001)
#define STR71X_FIQ_WDG        (0X00000002)
#define STR71X_FIQ_WDGT0TIMIS (0X00000003)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_STR71X_IRQ_H */

