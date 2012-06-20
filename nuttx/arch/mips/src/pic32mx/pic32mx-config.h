/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-config.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PIC32_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PIC32_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include "chip.h"
#include "pic32mx-memorymap.h"
#include "pic32mx-uart.h"
#include "pic32mx-int.h"
#include "pic32mx-devcfg.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Interrupt Priorities *************************************************************/

#ifndef CONFIG_PIC32MX_CTPRIO         /* Core Timer Interrupt */
#  define CONFIG_PIC32MX_CTPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CTPRIO < 4
#  error "CONFIG_PIC32MX_CTPRIO is too small"
#endif
#if CONFIG_PIC32MX_CTPRIO > 31
#  error "CONFIG_PIC32MX_CTPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CS0PRIO       /* Core Software Interrupt 0 */
#  define CONFIG_PIC32MX_CS0PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CS0PRIO < 4
#  error "CONFIG_PIC32MX_CS0PRIO is too small"
#endif
#if CONFIG_PIC32MX_CS0PRIO > 31
#  error "CONFIG_PIC32MX_CS0PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CS1PRIO      /* Core Software Interrupt 1 */
#  define CONFIG_PIC32MX_CS1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CS1PRIO < 4
#  error "CONFIG_PIC32MX_CS1PRIO is too small"
#endif
#if CONFIG_PIC32MX_CS1PRIO > 31
#  error "CONFIG_PIC32MX_CS1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_INT0PRIO      /* External interrupt 0 */
#  define CONFIG_PIC32MX_INT0PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_INT0PRIO < 4
#  error "CONFIG_PIC32MX_INT0PRIO is too small"
#endif
#if CONFIG_PIC32MX_INT0PRIO > 31
#  error "CONFIG_PIC32MX_INT0PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_INT1PRIO      /* External interrupt 1 */
#  define CONFIG_PIC32MX_INT1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_INT1PRIO < 4
#  error "CONFIG_PIC32MX_INT1PRIO is too small"
#endif
#if CONFIG_PIC32MX_INT1PRIO > 31
#  error "CONFIG_PIC32MX_INT1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_INT2PRIO      /* External interrupt 2 */
#  define CONFIG_PIC32MX_INT2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_INT2PRIO < 4
#  error "CONFIG_PIC32MX_INT2PRIO is too small"
#endif
#if CONFIG_PIC32MX_INT2PRIO > 31
#  error "CONFIG_PIC32MX_INT2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_INT3PRIO      /* External interrupt 3 */
#  define CONFIG_PIC32MX_INT3PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_INT3PRIO < 4
#  error "CONFIG_PIC32MX_INT3PRIO is too small"
#endif
#if CONFIG_PIC32MX_INT3PRIO > 31
#  error "CONFIG_PIC32MX_INT3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_INT4PRIO      /* External interrupt 4 */
#  define CONFIG_PIC32MX_INT4PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_INT4PRIO < 4
#  error "CONFIG_PIC32MX_INT4PRIO is too small"
#endif
#if CONFIG_PIC32MX_INT4PRIO > 31
#  error "CONFIG_PIC32MX_INT4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_FSCMPRIO      /* Fail-Safe Clock Monitor */
#  define CONFIG_PIC32MX_FSCMPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_FSCMPRIO < 4
#  error "CONFIG_PIC32MX_FSCMPRIO is too small"
#endif
#if CONFIG_PIC32MX_FSCMPRIO > 31
#  error "CONFIG_PIC32MX_FSCMPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T1PRIO        /* Timer 1 (System timer) priority */
#  define CONFIG_PIC32MX_T1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T1PRIO < 4
#  error "CONFIG_PIC32MX_T1PRIO is too small"
#endif
#if CONFIG_PIC32MX_T1PRIO > 31
#  error "CONFIG_PIC32MX_T1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T2PRIO        /* Timer 2 priority */
#  define CONFIG_PIC32MX_T2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T2PRIO < 4
#  error "CONFIG_PIC32MX_T2PRIO is too small"
#endif
#if CONFIG_PIC32MX_T2PRIO > 31
#  error "CONFIG_PIC32MX_T2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T3PRIO        /* Timer 3 priority */
#  define CONFIG_PIC32MX_T3PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T3PRIO < 4
#  error "CONFIG_PIC32MX_T3PRIO is too small"
#endif
#if CONFIG_PIC32MX_T3PRIO > 31
#  error "CONFIG_PIC32MX_T3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T4PRIO        /* Timer 4 priority */
#  define CONFIG_PIC32MX_T4PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T4PRIO < 4
#  error "CONFIG_PIC32MX_T4PRIO is too small"
#endif
#if CONFIG_PIC32MX_T4PRIO > 31
#  error "CONFIG_PIC32MX_T4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_T5PRIO        /* Timer 5 priority */
#  define CONFIG_PIC32MX_T5PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_T5PRIO < 4
#  error "CONFIG_PIC32MX_T5PRIO is too small"
#endif
#if CONFIG_PIC32MX_T5PRIO > 31
#  error "CONFIG_PIC32MX_T5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC1PRIO       /* Input Capture 1 */
#  define CONFIG_PIC32MX_IC1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC1PRIO < 4
#  error "CONFIG_PIC32MX_IC1PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC1PRIO > 31
#  error "CONFIG_PIC32MX_IC1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC2PRIO       /* Input Capture 2 */
#  define CONFIG_PIC32MX_IC2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC2PRIO < 4
#  error "CONFIG_PIC32MX_IC2PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC2PRIO > 31
#  error "CONFIG_PIC32MX_IC2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC3PRIO       /* Input Capture 3 */
#  define CONFIG_PIC32MX_IC3PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC3PRIO < 4
#  error "CONFIG_PIC32MX_IC3PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC3PRIO > 31
#  error "CONFIG_PIC32MX_IC3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC4PRIO       /* Input Capture 4 */
#  define CONFIG_PIC32MX_IC4PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC4PRIO < 4
#  error "CONFIG_PIC32MX_IC4PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC4PRIO > 31
#  error "CONFIG_PIC32MX_IC4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_IC5PRIO       /* Input Capture 5 */
#  define CONFIG_PIC32MX_IC5PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_IC5PRIO < 4
#  error "CONFIG_PIC32MX_IC5PRIO is too small"
#endif
#if CONFIG_PIC32MX_IC5PRIO > 31
#  error "CONFIG_PIC32MX_IC5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC1PRIO       /* Output Compare 1 */
#  define CONFIG_PIC32MX_OC1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC1PRIO < 4
#  error "CONFIG_PIC32MX_OC1PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC1PRIO > 31
#  error "CONFIG_PIC32MX_OC1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC2PRIO       /* Output Compare 2 */
#  define CONFIG_PIC32MX_OC2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC2PRIO < 4
#  error "CONFIG_PIC32MX_OC2PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC2PRIO > 31
#  error "CONFIG_PIC32MX_OC2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC3PRIO       /* Output Compare 3 */
#  define CONFIG_PIC32MX_OC3PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC3PRIO < 4
#  error "CONFIG_PIC32MX_OC3PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC3PRIO > 31
#  error "CONFIG_PIC32MX_OC3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC4PRIO       /* Output Compare 4 */
#  define CONFIG_PIC32MX_OC4PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC4PRIO < 4
#  error "CONFIG_PIC32MX_OC4PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC4PRIO > 31
#  error "CONFIG_PIC32MX_OC4PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_OC5PRIO       /* Output Compare 5 */
#  define CONFIG_PIC32MX_OC5PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_OC5PRIO < 4
#  error "CONFIG_PIC32MX_OC5PRIO is too small"
#endif
#if CONFIG_PIC32MX_OC5PRIO > 31
#  error "CONFIG_PIC32MX_OC5PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_I2C1PRIO      /* I2C 1 */
#  define CONFIG_PIC32MX_I2C1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_I2C1PRIO < 4
#  error "CONFIG_PIC32MX_I2C1PRIO is too small"
#endif
#if CONFIG_PIC32MX_I2C1PRIO > 31
#  error "CONFIG_PIC32MX_I2C1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_I2C2PRIO      /* I2C 2 */
#  define CONFIG_PIC32MX_I2C2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_I2C2PRIO < 4
#  error "CONFIG_PIC32MX_I2C2PRIO is too small"
#endif
#if CONFIG_PIC32MX_I2C2PRIO > 31
#  error "CONFIG_PIC32MX_I2C2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_SPI1PRIO      /* SPI 1 */
#  define CONFIG_PIC32MX_SPI1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_SPI1PRIO < 4
#  error "CONFIG_PIC32MX_SPI1PRIO is too small"
#endif
#if CONFIG_PIC32MX_SPI1PRIO > 31
#  error "CONFIG_PIC32MX_SPI1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_SPI2PRIO      /* SPI 2 */
#  define CONFIG_PIC32MX_SPI2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_SPI2PRIO < 4
#  error "CONFIG_PIC32MX_SPI2PRIO is too small"
#endif
#if CONFIG_PIC32MX_SPI2PRIO > 31
#  error "CONFIG_PIC32MX_SPI2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_UART1PRIO      /* UART 1 */
#  define CONFIG_PIC32MX_UART1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_UART1PRIO < 4
#  error "CONFIG_PIC32MX_UART1PRIO is too small"
#endif
#if CONFIG_PIC32MX_UART1PRIO > 31
#  error "CONFIG_PIC32MX_UART1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_UART2PRIO      /* UART 2 */
#  define CONFIG_PIC32MX_UART2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_UART2PRIO < 4
#  error "CONFIG_PIC32MX_UART2PRIO is too small"
#endif
#if CONFIG_PIC32MX_UART2PRIO > 31
#  error "CONFIG_PIC32MX_UART2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CNPRIO            /* Input Change Interrupt */
#  define CONFIG_PIC32MX_CNPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CNPRIO < 4
#  error "CONFIG_PIC32MX_CNPRIO is too small"
#endif
#if CONFIG_PIC32MX_CNPRIO > 31
#  error "CONFIG_PIC32MX_CNPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_ADCPRIO       /* ADC1 Convert Done */
#  define CONFIG_PIC32MX_ADCPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_ADCPRIO < 4
#  error "CONFIG_PIC32MX_ADCPRIO is too small"
#endif
#if CONFIG_PIC32MX_ADCPRIO > 31
#  error "CONFIG_PIC32MX_ADCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_PMPPRIO       /* Parallel Master Port */
#  define CONFIG_PIC32MX_PMPPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_PMPPRIO < 4
#  error "CONFIG_PIC32MX_PMPPRIO is too small"
#endif
#if CONFIG_PIC32MX_PMPPRIO > 31
#  error "CONFIG_PIC32MX_PMPPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CM1PRIO       /* Comparator 1 */
#  define CONFIG_PIC32MX_CM1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CM1PRIO < 4
#  error "CONFIG_PIC32MX_CM1PRIO is too small"
#endif
#if CONFIG_PIC32MX_CM1PRIO > 31
#  error "CONFIG_PIC32MX_CM1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_CM2PRIO       /* Comparator 2 */
#  define CONFIG_PIC32MX_CM2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_CM2PRIO < 4
#  error "CONFIG_PIC32MX_CM2PRIO is too small"
#endif
#if CONFIG_PIC32MX_CM2PRIO > 31
#  error "CONFIG_PIC32MX_CM2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_FSCMPRIO      /* Fail-Safe Clock Monitor */
#  define CONFIG_PIC32MX_FSCMPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_FSCMPRIO < 4
#  error "CONFIG_PIC32MX_FSCMPRIO is too small"
#endif
#if CONFIG_PIC32MX_FSCMPRIO > 31
#  error "CONFIG_PIC32MX_FSCMPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_RTCCPRIO      /* Real-Time Clock and Calendar */
#  define CONFIG_PIC32MX_RTCCPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_RTCCPRIO < 4
#  error "CONFIG_PIC32MX_RTCCPRIO is too small"
#endif
#if CONFIG_PIC32MX_RTCCPRIO > 31
#  error "CONFIG_PIC32MX_RTCCPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DMA0PRIO      /* DMA Channel 0 */
#  define CONFIG_PIC32MX_DMA0PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DMA0PRIO < 4
#  error "CONFIG_PIC32MX_DMA0PRIO is too small"
#endif
#if CONFIG_PIC32MX_DMA0PRIO > 31
#  error "CONFIG_PIC32MX_DMA0PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DMA1PRIO      /* DMA Channel 1 */
#  define CONFIG_PIC32MX_DMA1PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DMA1PRIO < 4
#  error "CONFIG_PIC32MX_DMA1PRIO is too small"
#endif
#if CONFIG_PIC32MX_DMA1PRIO > 31
#  error "CONFIG_PIC32MX_DMA1PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DMA2PRIO      /* DMA Channel 2 */
#  define CONFIG_PIC32MX_DMA2PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DMA2PRIO < 4
#  error "CONFIG_PIC32MX_DMA2PRIO is too small"
#endif
#if CONFIG_PIC32MX_DMA2PRIO > 31
#  error "CONFIG_PIC32MX_DMA2PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_DMA3PRIO      /* DMA Channel 3 */
#  define CONFIG_PIC32MX_DMA3PRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_DMA3PRIO < 4
#  error "CONFIG_PIC32MX_DMA3PRIO is too small"
#endif
#if CONFIG_PIC32MX_DMA3PRIO > 31
#  error "CONFIG_PIC32MX_DMA3PRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_FCEPRIO       /* Flash Control Event */
#  define CONFIG_PIC32MX_FCEPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_FCEPRIO < 4
#  error "CONFIG_PIC32MX_FCEPRIO is too small"
#endif
#if CONFIG_PIC32MX_FCEPRIO > 31
#  error "CONFIG_PIC32MX_FCEPRIO is too large"
#endif

#ifndef CONFIG_PIC32MX_USBPRIO       /* USB */
#  define CONFIG_PIC32MX_USBPRIO (INT_IPC_MID_PRIORITY << 2)
#endif
#if CONFIG_PIC32MX_USBPRIO < 4
#  error "CONFIG_PIC32MX_USBPRIO is too small"
#endif
#if CONFIG_PIC32MX_USBPRIO > 31
#  error "CONFIG_PIC32MX_USBPRIO is too large"
#endif

/* SYS calls ************************************************************************/
/* SYS call 0 and 1 are defined for internal use by the PIC32MX port (see
 * arch/mips/include/mips32/syscall.h
 */

#ifdef CONFIG_NUTTX_KERNEL
#  if !defined(CONFIG_SYS_RESERVED) || CONFIG_SYS_RESERVED < 2
#    error "CONFIG_SYS_RESERVED must be defined to be 2 for a kernel build"
#  elif CONFIG_SYS_RESERVED > 2
#    warning "CONFIG_SYS_RESERVED should be defined to be 2 for a kernel build"
#  endif
#endif

/* UARTs ****************************************************************************/
/* Don't enable UARTs not supported by the chip. */

#if CHIP_NUARTS < 1
#  undef CONFIG_PIC32MX_UART1
#  undef CONFIG_PIC32MX_UART2
#  undef CONFIG_PIC32MX_UART3
#  undef CONFIG_PIC32MX_UART4
#  undef CONFIG_PIC32MX_UART5
#  undef CONFIG_PIC32MX_UART6
#elif CHIP_NUARTS < 2
#  undef CONFIG_PIC32MX_UART2
#  undef CONFIG_PIC32MX_UART3
#  undef CONFIG_PIC32MX_UART4
#  undef CONFIG_PIC32MX_UART5
#  undef CONFIG_PIC32MX_UART6
#elif CHIP_NUARTS < 3
#  undef CONFIG_PIC32MX_UART3
#  undef CONFIG_PIC32MX_UART4
#  undef CONFIG_PIC32MX_UART5
#  undef CONFIG_PIC32MX_UART6
#elif CHIP_NUARTS < 4
#  undef CONFIG_PIC32MX_UART4
#  undef CONFIG_PIC32MX_UART5
#  undef CONFIG_PIC32MX_UART6
#elif CHIP_NUARTS < 5
#  undef CONFIG_PIC32MX_UART5
#  undef CONFIG_PIC32MX_UART6
#elif CHIP_NUARTS < 6
#  undef CONFIG_PIC32MX_UART6
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART_DEVICE
#if defined(CONFIG_PIC32MX_UART1) || defined(CONFIG_PIC32MX_UART2) || \
    defined(CONFIG_PIC32MX_UART4) || defined(CONFIG_PIC32MX_UART4) || \
    defined(CONFIG_PIC32MX_UART5) || defined(CONFIG_PIC32MX_UART6)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART3_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART3)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART4)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART5)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART6_SERIAL_CONSOLE) && defined(CONFIG_PIC32MX_UART6)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* Device Configuration *************************************************************/
/* DEVCFG3 */

#ifndef CONFIG_PIC32MX_USERID               /* User ID */
#  define CONFIG_PIC32MX_USERID   0x584e    /* "NutX" */
#endif

#ifndef CONFIG_PIC32MX_PMDL1WAY             /* Peripheral module disable configuration */
#  define CONFIG_PIC32MX_PMDL1WAY 0
#endif

#ifndef CONFIG_PIC32MX_IOL1WAY              /* Peripheral pin select configuration */
#  define CONFIG_PIC32MX_IOL1WAY  0
#endif

#ifndef CONFIG_PIC32MX_SRSSEL               /* Shadow register interrupt priority */
#  define CONFIG_PIC32MX_SRSSEL   INT_IPC_MIN_PRIORITY
#endif

/* Unless overridden in the .config file, all pins are in the default setting */

#ifndef CONFIG_PIC32MX_FMIIEN               /* Ethernet MII enable: 0=RMII 1=MII */
#  define CONFIG_PIC32MX_FMIIEN   1         /* MII enabled */
#endif

#ifndef CONFIG_PIC32MX_FETHIO               /* Ethernet I/O Pins 0=alternate 1=default */
#  define CONFIG_PIC32MX_FETHIO   1         /* Default Ethernet I/O Pins */
#endif

#ifndef CONFIG_PIC32MX_FCANIO               /* SCM1 pin C selection */
#  define CONFIG_PIC32MX_FCANIO   1         /* Default CAN I/O Pins */
#endif

#ifndef CONFIG_PIC32MX_FSCM1IO              /* SCM1 pin C selection */
#  define CONFIG_PIC32MX_FSCM1IO  1         /* Default pin for SCM1C */
#endif

/* USB or Ports? */

#ifdef CONFIG_PIC32MX_USB
#  ifndef CONFIG_PIC32MX_USBIDO            /* USB USBID Selection */
#    define CONFIG_PIC32MX_USBIDO 1        /* USBID pin is controlled by the USB module */
#  endif
#  ifndef CONFIG_PIC32MX_VBUSIO            /* USB VBUSON Selection */
#    define CONFIG_PIC32MX_VBUSIO 1        /* VBUSON pin is controlled by the USB module */
#  endif
#else
#  ifndef CONFIG_PIC32MX_USBIDO            /* USB USBID Selection */
#    define CONFIG_PIC32MX_USBIDO 0        /* USBID pin is controlled by the Port function */
#  endif
#  ifndef CONFIG_PIC32MX_VBUSIO            /* USB VBUSON Selection */
#    define CONFIG_PIC32MX_VBUSIO 0        /* VBUSON pin is controlled by the Port function */
#  endif
#endif

/* DEVCFG2 */

#undef CONFIG_PIC32MX_PLLIDIV
#if BOARD_PLL_IDIV == 1
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV1
#elif BOARD_PLL_IDIV == 2
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV2
#elif BOARD_PLL_IDIV == 3
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV3
#elif BOARD_PLL_IDIV == 4
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV4
#elif BOARD_PLL_IDIV == 5
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV5
#elif BOARD_PLL_IDIV == 6
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV6
#elif BOARD_PLL_IDIV == 10
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV10
#elif BOARD_PLL_IDIV == 12
#  define CONFIG_PIC32MX_PLLIDIV  DEVCFG2_FPLLIDIV_DIV12
#else
#  error "Unsupported BOARD_PLL_IDIV"
#endif

#undef CONFIG_PIC32MX_PLLMULT
#if BOARD_PLL_MULT == 15
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL15
#elif BOARD_PLL_MULT == 16
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL16
#elif BOARD_PLL_MULT == 17
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL17
#elif BOARD_PLL_MULT == 18
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL18
#elif BOARD_PLL_MULT == 19
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL19
#elif BOARD_PLL_MULT == 20
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL20
#elif BOARD_PLL_MULT == 21
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL21
#elif BOARD_PLL_MULT == 24
#  define CONFIG_PIC32MX_PLLMULT  DEVCFG2_FPLLMULT_MUL24
#else
#  error "Unsupported BOARD_PLL_MULT"
#endif

#undef CONFIG_PIC32MX_UPLLIDIV
#if BOARD_UPLL_IDIV == 1
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV1
#elif BOARD_UPLL_IDIV == 2
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV2
#elif BOARD_UPLL_IDIV == 3
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV3
#elif BOARD_UPLL_IDIV == 4
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV4
#elif BOARD_UPLL_IDIV == 5
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV5
#elif BOARD_UPLL_IDIV == 6
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV6
#elif BOARD_UPLL_IDIV == 10
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV10
#elif BOARD_UPLL_IDIV == 12
#  define CONFIG_PIC32MX_UPLLIDIV DEVCFG2_FUPLLIDIV_DIV12
#else
#  error "Unsupported BOARD_UPLL_IDIV"
#endif

#undef CONFIG_PIC32MX_PLLODIV
#if BOARD_PLL_ODIV == 1
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV1
#elif BOARD_PLL_ODIV == 2
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 4
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 8
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 16
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 32
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 64
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#elif BOARD_PLL_ODIV == 128
#  define CONFIG_PIC32MX_PLLODIV  DEVCFG2_FPLLODIV_DIV2
#else
#  error "Unsupported BOARD_PLL_ODIV"
#endif

/* DEVCFG1 */

#ifdef BOARD_SOSC_ENABLE
#  define CONFIG_PIC32MX_FSOSCEN DEVCFG1_FSOSCEN
#else
#  define CONFIG_PIC32MX_FSOSCEN 0
#endif

#ifdef BOARD_SOSC_IESO
#  define CONFIG_PIC32MX_IESO DEVCFG1_IESO
#else
#  define CONFIG_PIC32MX_IESO 0
#endif

#undef CONFIG_PIC32MX_PBDIV
#if BOARD_PBDIV == 1
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV1
#elif BOARD_PBDIV == 2
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV2
#elif BOARD_PBDIV == 4
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV4
#elif BOARD_PBDIV == 8
#  define CONFIG_PIC32MX_PBDIV DEVCFG1_FPBDIV_DIV8
#else
#  error "Unsupported BOARD_PBDIV"
#endif

#undef CONFIG_PIC32MX_POSCMOD
#if defined(BOARD_POSC_ECMODE)
#  define CONFIG_PIC32MX_POSCMOD DEVCFG1_POSCMOD_EC
#elif defined(BOARD_POSC_XTMODE)
#  define CONFIG_PIC32MX_POSCMOD DEVCFG1_POSCMOD_XT
#elif defined(BOARD_POSC_HSMODE)
#  define CONFIG_PIC32MX_POSCMOD DEVCFG1_POSCMOD_HS
#elif defined(BOARD_POSC_DISABLED)
#  define CONFIG_PIC32MX_POSCMOD DEVCFG1_POSCMOD_DIS
#else
#  error "Unknown board POSC mode"
#endif

#undef CONFIG_PIC32MX_FNOSC
#if defined(BOARD_FNOSC_FRC)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_FRC
#elif defined(BOARD_FNOSC_FRCPLL)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_FRCPLL
#elif defined(BOARD_FNOSC_POSC)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_POSC
#elif defined(BOARD_FNOSC_POSCPLL)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_POSCPLL
#elif defined(BOARD_FNOSC_SOSC)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_SOSC
#elif defined(BOARD_FNOSC_LPRC)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_LPRC
#elif defined(BOARD_FNOSC_FRCDIV)
#  define CONFIG_PIC32MX_FNOSC DEVCFG1_FNOSC_FRCDIV
#else
#  error "Unknown board FNOSC selection"
#endif

#undef CONFIG_PIC32MX_FCKSM
#if defined(BOARD_POSC_SWITCH)
#  if defined(BOARD_POSC_FSCM) 
#    define CONFIG_PIC32MX_FCKSM DEVCFG1_FCKSM_BOTH
#  else
#    define CONFIG_PIC32MX_FCKSM DEVCFG1_FCKSM_CSONLY
#  endif
#else
#  define CONFIG_PIC32MX_FCKSM DEVCFG1_FCKSM_NONE
#endif

#undef CONFIG_PIC32MX_WDPS
#if BOARD_WD_PRESCALER == 1
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1
#elif BOARD_WD_PRESCALER == 2
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_2
#elif BOARD_WD_PRESCALER == 4
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_4
#elif BOARD_WD_PRESCALER == 8
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_8
#elif BOARD_WD_PRESCALER == 16
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_16
#elif BOARD_WD_PRESCALER == 32
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_32
#elif BOARD_WD_PRESCALER == 64
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_64
#elif BOARD_WD_PRESCALER == 128
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_128
#elif BOARD_WD_PRESCALER == 256
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_256
#elif BOARD_WD_PRESCALER == 512
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_512
#elif BOARD_WD_PRESCALER == 1024
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1024
#elif BOARD_WD_PRESCALER == 2048
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_2048
#elif BOARD_WD_PRESCALER == 4096
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_4096
#elif BOARD_WD_PRESCALER == 8192
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_8192
#elif BOARD_WD_PRESCALER == 16384
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_16384
#elif BOARD_WD_PRESCALER == 32768
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_32768
#elif BOARD_WD_PRESCALER == 65536
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_65536
#elif BOARD_WD_PRESCALER == 131072
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_131072
#elif BOARD_WD_PRESCALER == 262144
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_262144
#elif BOARD_WD_PRESCALER == 524288
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_524288
#elif BOARD_WD_PRESCALER == 1048576
#  define CONFIG_PIC32MX_WDPS DEVCFG1_WDTPS_1048576
#else
#  error "Unsupported BOARD_WD_PRESCALER"
#endif

#undef CONFIG_PIC32MX_WDENABLE
#if BOARD_WD_ENABLE
#  define CONFIG_PIC32MX_WDENABLE DEVCFG1_FWDTEN
#else
#  define CONFIG_PIC32MX_WDENABLE 0
#endif

/* DEVCFG0 */

#ifndef CONFIG_PIC32MX_DEBUGGER                /* Background Debugger Enable */
#  define CONFIG_PIC32MX_DEBUGGER         3    /* disabled */
#endif

#ifndef CONFIG_PIC32MX_ICESEL                  /* In-Circuit Emulator/Debugger Communication Channel Select */
#  define CONFIG_PIC32MX_ICESEL           1    /* default */
#endif

#ifndef CONFIG_PIC32MX_PROGFLASHWP             /* Program FLASH write protect */
#  define CONFIG_PIC32MX_PROGFLASHWP      0xff /* Disabled */
#endif

#ifndef CONFIG_PIC32MX_BOOTFLASHWP
#  define CONFIG_PIC32MX_BOOTFLASHWP      1    /* Disabled */
#endif

#ifndef CONFIG_PIC32MX_CODEWP
#  define CONFIG_PIC32MX_CODEWP           1    /* Disabled */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_PIC32_H */
