/************************************************************************************
 * arch/sh/src/m16c/chip.h
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
 ************************************************************************************/

#ifndef __ARCH_SH_SRC_M16C_CHIP_H
#define __ARCH_SH_SRC_M16C_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* FLG register bits */

#define M16C_FLG_C         0x0001    /* Bit 0: Carry flag */
#define M16C_FLG_D         0x0002    /* Bit 1: Debug flag */
#define M16C_FLG_Z         0x0004    /* Bit 2: Zero flag */
#define M16C_FLG_S         0x0008    /* Bit 3: Sign flag */
#define M16C_FLG_B         0x0010    /* Bit 4: Register bank flag */
#define M16C_FLG_O         0x0020    /* Bit 5: Overflow flag */
#define M16C_FLG_I         0x0040    /* Bit 6: Interrupt enable flag */
#define M16C_FLG_U         0x0080    /* Bit 7: Stack pointer select flag */
                                     /* Bits 8-11: Reserved */
#define M16C_FLG_IPLMASK   0x7000    /* Bits 12:14: Processor interrupt priority level */
                                     /* Bit 15: Reserved */
/* Memory Map */

/* Memory-mapped special function registers begin at address 0x00000 */

#define M16C_SFR_BASE      0x00000   /* 00000-003ff: Special Function Registers */

/* Internal, on-chip SRAM begins at address 0x00400 for all chips, regardless of the
 * size of the on-chip SRAM.
 */

#define M16C_IRAM_BASE     0x00400   /* 00400-00xxx: Internal RAM */
#if defined(CONFIG_ARCH_CHIP_M30262F3) || defined(CONFIG_ARCH_CHIP_M30262F4)
#  define M16C_IRAM_END    0x007ff   /*              End+1 address of internal RAM */
                                     /* 00800-0efff: Reserved */
#elif defined(CONFIG_ARCH_CHIP_M30262F6) || defined(CONFIG_ARCH_CHIP_M30262F8)
#  define M16C_IRAM_END    0x00bff   /*              End+1 address of internal RAM */
                                     /* 00c00-0efff: Reserved */
#endif

/* Two banks of virtual EEPROM (all parts) */

#define M16C_VEEPROM1_BASE 0x0f000   /* 0f000-0f7fff: Virtual EEPPROM block 1 */
#define M16C_VEEPROM2_BASE 0x0f800   /* 0f800-0fffff: Virtual EEPPROM block 2 */

/* If there were external, "far" RAM, it would be begin at 0x10000.  However, these
 * specific chips do not support external RAM.
 */

/* Each part has a different amount on on-chip FLASH.  The ending FLASH address is
 * 0xfffff for all chips, but the starting address varies depening on the amount
 * of on-chip FLASH.
 */

#if defined(CONFIG_ARCH_CHIP_M30262F3)
                                     /* 10000-f9ffff: Reserved */
#  define M16C_FLASH_BASE    0xfa000 /* fa000-ffffff: Flash ROM (M30262F8) */
#elif defined(CONFIG_ARCH_CHIP_M30262F4)
                                     /* 10000-f7ffff: Reserved */
#  define M16C_FLASH_BASE    0xf8000 /* f8000-ffffff: Flash ROM (M30262F8) */
#elif defined(CONFIG_ARCH_CHIP_M30262F6)
                                     /* 10000-f3ffff: Reserved */
#  define M16C_FLASH_BASE    0xf4000 /* f4000-ffffff: Flash ROM (M30262F8) */
#elif defined(CONFIG_ARCH_CHIP_M30262F8)
                                     /* 10000-efffff: Reserved */
#  define M16C_FLASH_BASE    0xf0000 /* f0000-ffffff: Flash ROM (M30262F8) */
#endif

/* Special Function Register Addresses */

#define M16C_PM0           0x00004    /* Processor mode 0 */
#define M16C_PM1           0x00005    /* Processor mode 1 */
#define M16C_CM0           0x00006    /* System clock control 0 */
#define M16C_CM1           0x00007    /* System clock control 1 */
#define M16C_AIER          0x00009    /* Addrese match interrupt enable */
#define M16C_PRCR          0x0000a    /* Protect */
#define M16C_CM2           0x0000c    /* Oscillation stop detection */
#define M16C_WDTS          0x0000e    /* Watchdog timer start */
#define M16C_WDC           0x0000f    /* Watchdog timer control */
#define M16C_RMAD0         0x00010    /* Address match interrupt 0 */
#define M16C_RMAD1         0x00014    /* Address match interrupt 1 */
#define M16C_VCR1          0x00019    /* Power supply detection 1 */
#define M16C_VCR2          0x0001a    /* Power supply detection 2 */
#define M16C_PM2           0x0001e    /* Processor mode 2 */
#define M16C_D4INT         0x0001f    /* Power supply 4V detection */
#define M16C_SAR0          0x00020    /* DMA0 source pointer */
#define M16C_DAR0          0x00024    /* DMA0 destination pointer */
#define M16C_TCR0          0x00028    /* DMA0 transfer counter */
#define M16C_DM0CON        0x0002c    /* DMA0 control */
#define M16C_SAR1          0x00030    /* DMA1 source pointer */
#define M16C_DAR1          0x00034    /* DMA1 destination pointer */
#define M16C_TCR1          0x00038    /* DMA1 transfer counter */
#define M16C_DM1CON        0x0003c    /* DMA1 control */
#define M16C_INT3IC        0x00044    /* INT3 interrupt control */
#define M16C_INT5IC        0x00048    /* INT5 interrupt control */
#define M16C_INT4IC        0x00049    /* INT4 interrupt control */
#define M16C_BCNIC         0x0004a    /* Bus collision detection interrupt control */
#define M16C_DM0IC         0x0004b    /* DMA0 interrupt control */
#define M16C_DM1IC         0x0004c    /* DMA1 interrupt control */
#define M16C_KUPIC         0x0004d    /* Key input interrupt control */
#define M16C_ADIC          0x0004e    /* A-D conversion interrupt control */
#define M16C_S2TIC         0x0004f    /* UART2 transmit interrupt control */
#define M16C_S2RIC         0x00050    /* UART2 receive interrupt control */
#define M16C_S0TIC         0x00051    /* UART0 transmit interrupt control */
#define M16C_S0RIC         0x00052    /* UART0 receive interrupt control */
#define M16C_S1TIC         0x00053    /* UART1 transmit interrupt control */
#define M16C_S1RIC         0x00054    /* UART1 receive interrupt control */
#define M16C_TA0IC         0x00055    /* Timer A0 interrupt control */
#define M16C_TA1IC         0x00056    /* Timer A1 interrupt control */
#define M16C_TA2IC         0x00057    /* Timer A2 interrupt control */
#define M16C_TA3IC         0x00058    /* Timer A3 interrupt control */
#define M16C_TA4IC         0x00059    /* Timer A4 interrupt control */
#define M16C_TB0IC         0x0005a    /* Timer B0 interrupt control */
#define M16C_TB1IC         0x0005b    /* Timer B1 interrupt control */
#define M16C_TB2IC         0x0005c    /* Timer B2 interrupt control */
#define M16C_INT0IC        0x0005d    /* INT0 interrupt control */
#define M16C_INT1IC        0x0005e    /* INT1 interrupt control */
#define M16C_FMR4          0x001b3    /* Flash Control 4 */
#define M16C_FMR1          0x001b5    /* Flash Control 1 */
#define M16C_FMR0          0x001b7    /* Flash Control 0 */
#define M16C_PCLKR         0x0025e    /* Peripheral Clock Select */
#define M16C_TA11          0x00342    /* Timer A1-1 */
#define M16C_TA21          0x00344    /* Timer A2-1 */
#define M16C_TA41          0x00346    /* Timer A4-1 */
#define M16C_INVC0         0x00348    /* Three-phase PWM control 0 */
#define M16C_INVC1         0x00349    /* Three-phase PWM control 1 */
#define M16C_IDB0          0x0034a    /* Three-phase output buffer 0 */
#define M16C_IDB1          0x0034b    /* Three-phase output buffer 1 */
#define M16C_DTT           0x0034c    /* Dead time timer */
#define M16C_ICTB2         0x0034d    /* Timer B2 interrupt occurences frequency set counter */
#define M16C_IFSR          0x0035f    /* Interrupt request cause select */
#define M16C_U2SMR4        0x00374    /* UART2 special mode register4 */
#define M16C_U2SMR3        0x00375    /* UART2 special mode register3 */
#define M16C_U2SMR2        0x00376    /* UART2 special mode register2 */
#define M16C_U2SMR         0x00377    /* UART2 special mode */
#define M16C_U2MR          0x00378    /* UART2 transmit/receive mode */
#define M16C_U2BRG         0x00379    /* UART2 bit rate generator */
#define M16C_U2TB          0x0037a    /* UART2 transmit buffer */
#define M16C_U2C0          0x0037c    /* UART2 transmit/receive control 0 */
#define M16C_U2C1          0x0037d    /* UART2 transmit/receive control 1 */
#define M16C_U2RB          0x0037e    /* UART2 receive buffer */
#define M16C_TABSR         0x00380    /* Count start flag */
#define M16C_CPSRF         0x00381    /* Clock prescaler reset flag */
#define M16C_ONSF          0x00382    /* One-shot start flag */
#define M16C_TRGSR         0x00383    /* Trigger select */
#define M16C_UDF           0x00384    /* Up-down flag */
#define M16C_TA0           0x00386    /* Timer A0 */
#define M16C_TA1           0x00388    /* Timer A1 */
#define M16C_TA2           0x0038a    /* Timer A2 */
#define M16C_TA3           0x0038c    /* Timer A3 */
#define M16C_TA4           0x0038e    /* Timer A4 */
#define M16C_TB0           0x00390    /* Timer B0 */
#define M16C_TB1           0x00392    /* Timer B1 */
#define M16C_TB2           0x00394    /* Timer B2 */
#define M16C_TA0MR         0x00396    /* Timer A0 mode */
#define M16C_TA1MR         0x00397    /* Timer A1 mode */
#define M16C_TA2MR         0x00398    /* Timer A2 mode */
#define M16C_TA3MR         0x00399    /* Timer A3 mode */
#define M16C_TA4MR         0x0039a    /* Timer A4 mode */
#define M16C_TB0MR         0x0039b    /* Timer B0 mode */
#define M16C_TB1MR         0x0039c    /* Timer B1 mode */
#define M16C_TB2MR         0x0039d    /* Timer B2 mode */
#define M16C_TB2SC         0x0039e    /* Timer B2 special mode */
#define M16C_U0MR          0x003a0    /* UART0 transmit/receive mode */
#define M16C_U0BRG         0x003a1    /* UART0 bit rate generator */
#define M16C_U0TB          0x003a2    /* UART0 transmit buffer */
#define M16C_U0C0          0x003a4    /* UART0 transmit/receive control 0 */
#define M16C_U0C1          0x003a5    /* UART0 transmit/receive control 1 */
#define M16C_U0RB          0x003a6    /* UART0 receive buffer */
#define M16C_U1MR          0x003a8    /* UART1 transmit/receive mode */
#define M16C_U1BRG         0x003a9    /* UART1 bit rate generator */
#define M16C_U1TB          0x003aa    /* UART1 transmit buffer */
#define M16C_U1C0          0x003ac    /* UART1 transmit/receive control 0 */
#define M16C_U1C1          0x003ad    /* UART1 transmit/receive control 1 */
#define M16C_U1RB          0x003ae    /* UART1 receive buffer */
#define M16C_UCON          0x003b0    /* UART2 transmit/receive control 2 */
#define M16C_DM0SL         0x003b8    /* DMA0 cause select */
#define M16C_DM1SL         0x003ba    /* DMA1 cause select */
#define M16C_AD0           0x003c0    /* A-D 0 */
#define M16C_AD1           0x003c2    /* A-D 1 */
#define M16C_AD2           0x003c4    /* A-D 2 */
#define M16C_AD3           0x003c6    /* A-D 3 */
#define M16C_AD4           0x003c8    /* A-D 4 */
#define M16C_AD5           0x003ca    /* A-D 5 */
#define M16C_AD6           0x003cc    /* A-D 6 */
#define M16C_AD7           0x003ce    /* A-D 7 */
#define M16C_ADCON2        0x003d4    /* A-D control 2 */
#define M16C_ADCON0        0x003d6    /* A-D control 0 */
#define M16C_ADCON1        0x003d7    /* A-D control 1 */
#define M16C_P1            0x003e1    /* Port P1 */
#define M16C_PD1           0x003e3    /* Port P1 direction */
#define M16C_P6            0x003ec    /* Port P6 */
#define M16C_P7            0x003ed    /* Port P7 */
#define M16C_PD6           0x003ee    /* Port P6 direction */
#define M16C_PD7           0x003ef    /* Port P7 direction */
#define M16C_P8            0x003f0    /* Port P8 */
#define M16C_P9            0x003f1    /* Port P9 */
#define M16C_PD8           0x003f2    /* Port P8 direction */
#define M16C_PD9           0x003f3    /* Port P9 direction */
#define M16C_P10           0x003f4    /* Port P10 */
#define M16C_PD10          0x003f6    /* Port P10 direction */
#define M16C_PUR0          0x003fc    /* Pull-up control 0 */
#define M16C_PUR1          0x003fd    /* Pull-up control 1 */
#define M16C_PUR2          0x003fe    /* Pull-up control 2 */
#define M16C_PCR           0x003ff    /* Port control */

/************************************************************************************
 * Global Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

extern uint16_t g_snbss;		/* Start of near .bss */
extern uint16_t g_enbss;		/* End+1 of near .bss */
extern uint16_t g_sndata;		/* Start of near .data */
extern uint16_t g_endata;		/* End+1 of near .data */
extern uint32_t g_enronly;		/* Start of relocated read-only data in FLASH */
#ifdef CONFIG_M16C_HAVEFARRAM
  extern uint32_t g_sfbss;		/* Start of far .bss */
  extern uint32_t g_efbss;		/* End+1 of far .bss */
  extern uint32_t g_sfdata;		/* Start of far .data */
  extern uint32_t g_efdata;		/* End_1 of far .data */
  extern uint32_t g_efronly;	/* Start of relocated read-only data in FLASH */
#endif
extern uint32_t g_svarvect;		/* Start of variable vectors */
extern uint32_t g_heapbase;		/* Start of the heap */

/* Address of the saved user stack pointer */

#ifndef __ASSEMBLY__
#  if CONFIG_ARCH_INTERRUPTSTACK > 3
     extern uint16_t g_userstack;
#  endif
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_SH_SRC_M16C_CHIP_H */
