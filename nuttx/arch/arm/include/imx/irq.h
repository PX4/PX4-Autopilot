/****************************************************************************
 * arch/arm/include/imx/irq.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_IMX_IRQ_H
#define __ARCH_ARM_INCLUDE_IMX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* i.MX1 Interrupts */

#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_UART3PFERR         ( 0)
#  define IMX_IRQ_UART3RTS           ( 1)
#  define IMX_IRQ_UART3DTR           ( 2)
#  define IMX_IRQ_UART3UARTC         ( 3)
#  define IMX_IRQ_UART3TX            ( 4)
#  define IMX_IRQ_PENUP              ( 5)
#endif
#define IMX_IRQ_CSI                  ( 6)
#define IMX_IRQ_MMAMAC               ( 7)
#define IMX_IRQ_MMA                  ( 8)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_COMP               ( 9)
#endif
#define IMX_IRQ_MSHCXINT             (10)
#define IMX_IRQ_GPIOPORTA            (11)
#define IMX_IRQ_GPIOPORTB            (12)
#define IMX_IRQ_GPIOPORTC            (13)
#define IMX_IRQ_LCDC                 (14)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_SIM                (15)
#  define IMX_IRQ_SIMDATA            (16)
#endif
#define IMX_IRQ_RTC                  (17)
#define IMX_IRQ_RTCSAMINT            (18)
#define IMX_IRQ_UART2PFERR           (19)
#define IMX_IRQ_UART2RTS             (20)
#define IMX_IRQ_UART2DTR             (21)
#define IMX_IRQ_UART2UARTC           (22)
#define IMX_IRQ_UART2TX              (23)
#define IMX_IRQ_UART2RX              (24)
#define IMX_IRQ_UART1PFERR           (25)
#define IMX_IRQ_UART1RTS             (26)
#define IMX_IRQ_UART1DTR             (27)
#define IMX_IRQ_UART1UARTC           (28)
#define IMX_IRQ_UART1TX              (29)
#define IMX_IRQ_UART1RX              (30)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_PENDATA            (33)
#endif
#define IMX_IRQ_PWM                  (34)
#define IMX_IRQ_MMCSD                (35)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_SSI2TX             (36)
#  define IMX_IRQ_SSI2RX             (37)
#  define IMX_IRQ_SSI2ERR            (38)
#endif
#define IMX_IRQ_I2C                  (39)
#define IMX_IRQ_CSPI2                (40)
#define IMX_IRQ_CSPI1                (41)
#define IMX_IRQ_SSITX                (42)
#define IMX_IRQ_SSITXERR             (43)
#define IMX_IRQ_SSIRX                (44)
#define IMX_IRQ_SSIRXERR             (45)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_TOUCH              (46)
#endif
#define IMX_IRQ_USBD0                (47)
#define IMX_IRQ_USBD1                (48)
#define IMX_IRQ_USBD2                (49)
#define IMX_IRQ_USBD3                (50)
#define IMX_IRQ_USBD4                (51)
#define IMX_IRQ_USBD5                (52)
#define IMX_IRQ_USBD6                (53)
#ifndef CONFIG_ARCH_CHIP_IMXL
#  define IMX_IRQ_UART3RX            (54)
#  define IMX_IRQ_BTSYS              (55)
#  define IMX_IRQ_BTTIM              (56)
#  define IMX_IRQ_BTWUI              (57)
#endif
#define IMX_IRQ_TIMER2               (58)
#define IMX_IRQ_TIMER1               (59)
#define IMX_IRQ_DMAERR               (60)
#define IMX_IRQ_DMA                  (61)
#define IMX_IRQ_GPIOPORTD            (62)
#define IMX_IRQ_WDT                  (63)

#define IMX_IRQ_SYSTIMER             IMX_IRQ_TIMER1
#define NR_IRQS                      (64)

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

#endif /* __ARCH_ARM_INCLUDE_IMX_IRQ_H */

