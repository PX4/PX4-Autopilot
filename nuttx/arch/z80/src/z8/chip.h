/************************************************************************************
 * arch/z80/src/z8/chip.h
 * arch/z80/src/chip/chip.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __Z8_CHIP_H
#define __Z8_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Hexadecimal Representation *******************************************************/

#ifdef __ASSEMBLY__
# define _HX(h)   %##h
#else
# define _HX(h)   0x##h
#endif

/* Memory Map
 *
 * 64Kb Program Memory (64K series)
 *  C:0000 - C:0001 : Flash options
 *  C:0002 - C:0037 : Vectors
 *                  : ROM data
 *                  : Code
 *
 * 4Kb Register File (64K series)
 *  R:020 - R:0ff   : 224 byte RDATA
 *  R:0e0 - R:0ef   :  16-byte working register area (RDATA)
 *  E:100 - E:eff   : 3.5 Kbyte EDATA
 *    f00 -   fff   : 256 byte control register area
 */

/* Special Function Registers *******************************************************
 *
 * Because of the many different ez80 configurations, we will rely on the
 * ZDS-II header file, ez8.h, to provide the correct addresses for each register.
 */

/* Timer Register Bit Definitions ***************************************************/

/* Timer control register */

#define Z8_TIMERCTL_TEN      _HX(80) /* Bit 7: Timer enabled */
#define Z8_TIMERCTL_TPOL     _HX(40) /* Bit 6: Timer input/output polarity */
#define Z8_TIMERCTL_DIV1     _HX(00) /* Bits 3-5: Pre-scale divisor */
#define Z8_TIMERCTL_DIV2     _HX(08)
#define Z8_TIMERCTL_DIV4     _HX(10)
#define Z8_TIMERCTL_DIV8     _HX(18)
#define Z8_TIMERCTL_DIV16    _HX(20)
#define Z8_TIMERCTL_DIV32    _HX(28)
#define Z8_TIMERCTL_DIV64    _HX(30)
#define Z8_TIMERCTL_DIV128   _HX(38)
#define Z8_TIMERCTL_ONESHOT  _HX(00) /* Bits 0-2: Timer mode */
#define Z8_TIMERCTL_CONT     _HX(01)
#define Z8_TIMERCTL_COUNTER  _HX(02)
#define Z8_TIMERCTL_PWM      _HX(03)
#define Z8_TIMERCTL_CAPTURE  _HX(04)
#define Z8_TIMERCTL_COMPARE  _HX(05)
#define Z8_TIMERCTL_GATED    _HX(06)
#define Z8_TIMERCTL_CAPCMP   _HX(07)

/* UART Register Offsets *************************************************************/

#define Z8_UART_TXD          _HX(00)        /*  8-bits: UART Transmit Data */
#define Z8_UART_RXD          _HX(00)        /*  8-bits: UART Receive Data */
#define Z8_UART_STAT0        _HX(01)        /*  8-bits: UART Status 0 */
#define Z8_UART_CTL          _HX(02)        /* 16-bits: UART Control */
#define Z8_UART_CTL0         _HX(02)        /*  8-bits: UART Control 0 */
#define Z8_UART_CTL1         _HX(03)        /*  8-bits: UART COntrol 1 */
#if defined(_Z8FMC16) || defined(_Z8F1680)
#  define Z8_UART_MDSTAT     _HX(04)        /*  8-bits: UART Mode Select & Status */
#else
#  define Z8_UART_STAT1      _HX(04)        /*  8-bits: UART Status 1 */
#endif
#define Z8_UART_ADDR         _HX(05)        /*  8-bits: UART Address Compare */
#define Z8_UART_BR           _HX(06)        /* 16-bits: UART Baud Rate */
#define Z8_UART_BRH          _HX(06)        /*  8-bits: UART Baud Rate High Byte */
#define Z8_UART_BRL          _HX(07)        /*  8-bits: UART Baud Rate Low Byte */

/* UART0/1 Base Register Addresses **************************************************/

#ifdef EZ8_UART0
#  define Z8_UART0_BASE       ((uint8_t volatile far*)0xf40)
#endif

#ifdef EZ8_UART1
#  define Z8_UART1_BASE       ((uint8_t volatile far*)0xf48)
#endif

/* UART0/1 Status 0 Register Bit Definitions ****************************************/

#define Z8_UARTSTAT0_RDA     _HX(80)        /* Bit 7: Receive Data Available */
#define Z8_UARTSTAT0_PE      _HX(40)        /* Bit 6: Parity Error */
#define Z8_UARTSTAT0_OE      _HX(20)        /* Bit 5: Overrun Error */
#define Z8_UARTSTAT0_FE      _HX(10)        /* Bit 4: Framing Error */
#define Z8_UARTSTAT0_BRKD    _HX(08)        /* Bit 3: Break Detect */
#define Z8_UARTSTAT0_TDRE    _HX(04)        /* Bit 2: Transmitter Data Register Empty */
#define Z8_UARTSTAT0_TXE     _HX(02)        /* Bit 1: Transmitter Empty */
#define Z8_UARTSTAT0_CTS     _HX(01)        /* Bit 0: Clear To Send */

/* UART0/1 Control 0/1 Register Bit Definitions *************************************/

#define Z8_UARTCTL0_TEN      _HX(80)        /* Bit 7: Transmit Enable */
#define Z8_UARTCTL0_REN      _HX(40)        /* Bit 6: Receive Enable */
#define Z8_UARTCTL0_CTSE     _HX(20)        /* Bit 5: CTS Enable */
#define Z8_UARTCTL0_PEN      _HX(10)        /* Bit 4: Parity Enable */
#define Z8_UARTCTL0_PSEL     _HX(08)        /* Bit 3: Odd Parity Select */
#define Z8_UARTCTL0_SBRK     _HX(04)        /* Bit 2: Send Break */
#define Z8_UARTCTL0_STOP     _HX(02)        /* Bit 1: Stop Bit Select */
#define Z8_UARTCTL0_LBEN     _HX(01)        /* Bit 0: Loopback Enable */

#define Z8_UARTCTL1_MPMD1    _HX(80)        /* Bit 7: Multiprocessor Mode (bit1) */
#define Z8_UARTCTL1_MPEN     _HX(40)        /* Bit 6: Multiprocessor Enable */
#define Z8_UARTCTL1_MPMD0    _HX(20)        /* Bit 5: Multiprocessor Mode (bit0) */
#define Z8_UARTCTL1_MPBT     _HX(10)        /* Bit 4: Multiprocessor Bit Transmit */
#define Z8_UARTCTL1_DEPOL    _HX(08)        /* Bit 3: Driver Enable Polarity */
#define Z8_UARTCTL1_BRGCTL   _HX(04)        /* Bit 2: Baud Rate Generator Control */
#define Z8_UARTCTL1_RDAIRQ   _HX(02)        /* Bit 1: Receive Data Interrupt Enable */
#define Z8_UARTCTL1_IREN     _HX(01)        /* Bit 0: Infrared Encoder/Decoder Eanble */

/* UART0/1 Mode Status/Select Register Bit Definitions ******************************/

#define Z8_UARTMDSEL_NORMAL  _HX(00)        /* Bits 5-7=0: Multiprocessor and Normal Mode */
#define Z8_UARTMDSEL_FILTER   HX(20)        /* Bits 5-7=1: Noise Filter Control/Status */
#define Z8_UARTMDSEL_LINP     HX(40)        /* Bits 5-7=2: LIN protocol Contol/Status */
#define Z8_UARTMDSEL_HWREV    HX(e0)        /* Bits 5-7=7: LIN-UART Hardware Revision */
                                            /* Bits 0-4:   Mode dependent status */

/* I2C Status Register Bit Definitions **********************************************/

#if defined(_Z8FMC16) || defined(_Z8F1680)
#  define I2C_ISTAT_NCKI   (1 << 0) /* Bit 0: 1=NAK Interrupt */
#  define I2C_ISTAT_SPRS   (1 << 1) /* Bit 1: 1=STOP/RESTART condition Interrupt */
#  define I2C_ISTAT_ARBLST (1 << 2) /* Bit 2: 1=Arbitration lost */
#  define I2C_ISTAT_RD     (1 << 3) /* Bit 3: 1=Read */
#  define I2C_ISTAT_GCA    (1 << 4) /* Bit 4: 1=General Call Address */
#  define I2C_ISTAT_SAM    (1 << 5) /* Bit 5: 1=Slave address match */
#  define I2C_ISTAT_RDRF   (1 << 6) /* Bit 6: 1=Receive Data Register Full */
#  define I2C_ISTAT_TDRE   (1 << 7) /* Bit 7: 1=Transmit Data Register Empty */
#else
#  define I2C_STAT_NCKI    (1 << 0) /* Bit 0: 1=NAK Interrupt */
#  define I2C_STAT_DSS     (1 << 1) /* Bit 1: 1=Data Shift State */
#  define I2C_STAT_TAS     (1 << 2) /* Bit 2: 1=Transmit Address State */
#  define I2C_STAT_RD      (1 << 3) /* Bit 3: 1=Read */
#  define I2C_STAT_10B     (1 << 4) /* Bit 4: 1=10-Bit Address */
#  define I2C_STAT_ACK     (1 << 5) /* Bit 5: 1=Acknowledge */
#  define I2C_STAT_RDRF    (1 << 6) /* Bit 6: 1=Receive Data Register Full */
#  define I2C_STAT_TDRE    (1 << 7) /* Bit 7: 1=Transmit Data Register Empty */
#endif

/* I2C Control Register Bit Definitions *********************************************/

#define I2C_CTL_FILTEN     (1 << 0) /* Bit 0: 1=I2C Signal Filter Enable */
#define I2C_CTL_FLUSH      (1 << 1) /* Bit 1: 1=Flush Data */
#define I2C_CTL_NAK        (1 << 2) /* Bit 2: 1=Send NAK */
#define I2C_CTL_TXI        (1 << 3) /* Bit 3: 1=Enable TDRE interrupts */
#define I2C_CTL_BIRQ       (1 << 4) /* Bit 4: 1=Baud Rate Generator Interrupt Request */
#define I2C_CTL_STOP       (1 << 5) /* Bit 5: 1=Send Stop Condition */
#define I2C_CTL_START      (1 << 6) /* Bit 6: 1=Send Start Condition */
#define I2C_CTL_IEN        (1 << 7) /* Bit 7: 1=I2C Enable */

/* Register access macros ***********************************************************
 *
 * The register access mechanism provided in ez8.h differs from the useful in other
 * NuttX architectures.  The following NuttX common macros will at least make the
 * access compatible at the source level (however, strict type check is lost).
 */

#ifndef __ASSEMBLY__
# define getreg8(a)           (a)
# define putreg8(v,a)         (a) = (v)
# define getreg16(a)          (a)
# define putreg16(v,a)        (a) = (v)
# define getreg32(a)          (a)
# define putreg32(v,a)        (a) = (v)
#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

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

#endif  /* __Z8_CHIP_H */
