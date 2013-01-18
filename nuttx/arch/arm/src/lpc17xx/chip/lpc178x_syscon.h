/****************************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc178x_syscon.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Rommel Marcelo
 *            Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_SYSCON_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_SYSCON_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register offsets *********************************************************************************/
/* Flash accelerator module */

#define LPC17_SYSCON_FLASHCFG_OFFSET     0x0000 /* Flash Accelerator Configuration Register */

/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define LPC17_SYSCON_MEMMAP_OFFSET       0x0040 /* Memory Mapping Control register */

/* Clocking and power control - Phase locked loops */

#define LPC17_SYSCON_PLL0CON_OFFSET      0x0080 /* PLL0 Control Register */
#define LPC17_SYSCON_PLL0CFG_OFFSET      0x0084 /* PLL0 Configuration Register */
#define LPC17_SYSCON_PLL0STAT_OFFSET     0x0088 /* PLL0 Status Register */
#define LPC17_SYSCON_PLL0FEED_OFFSET     0x008c /* PLL0 Feed Register */

#define LPC17_SYSCON_PLL1CON_OFFSET      0x00a0 /* PLL1 Control Register */
#define LPC17_SYSCON_PLL1CFG_OFFSET      0x00a4 /* PLL1 Configuration Register */
#define LPC17_SYSCON_PLL1STAT_OFFSET     0x00a8 /* PLL1 Status Register */
#define LPC17_SYSCON_PLL1FEED_OFFSET     0x00ac /* PLL1 Feed Register */

/* Clocking and power control - Peripheral power control registers */

#define LPC17_SYSCON_PCON_OFFSET         0x00c0 /* Power Control Register */
#define LPC17_SYSCON_PCONP_OFFSET        0x00c4 /* Power Control for Peripherals Register */

/* Clocking and power control -- Clock dividers */

#define LPC17_SYSCON_EMCCLKCFG_OFFSET    0x0100 /* EMC Clock Configuration Register */
#define LPC17_SYSCON_CCLKCFG_OFFSET      0x0104 /* CPU Clock Configuration Register */
#define LPC17_SYSCON_USBCLKCFG_OFFSET    0x0108 /* USB Clock Configuration Register */

/* 0x400f c110 - 0x400f c114: CAN Wake and Sleep Registers */

/* Clocking and power control -- Clock source selection */

#define LPC17_SYSCON_CLKSRCSEL_OFFSET    0x010c /* Clock Source Select Register */
#define LPC17_SYSCON_CANSLEEPCLR_OFFSET  0x0110 /* CAN Channel Sleep State Register */
#define LPC17_SYSCON_CANWAKEFLAGS_OFFSET 0x0114 /* CAN Channel Wake-Up State Register */

/* System control registers -- External Interrupts */

#define LPC17_SYSCON_EXTINT_OFFSET       0x0140 /* External Interrupt Flag Register */
#define LPC17_SYSCON_EXTMODE_OFFSET      0x0148 /* External Interrupt Mode register */
#define LPC17_SYSCON_EXTPOLAR_OFFSET     0x014c /* External Interrupt Polarity Register */

/* System control registers -- Reset */

#define LPC17_SYSCON_RSID_OFFSET         0x0180 /* Reset Source Identification Register */

/* System control registers -- Syscon Miscellaneous Registers */

#define LPC17_SYSCON_MATRIXARB_OFFSET    0x0188 /* Matrix Arbitration Register */
#define LPC17_SYSCON_SCS_OFFSET          0x01a0 /* System Control and Status */
#define LPC17_SYSCON_PCLKSEL_OFFSET      0x01a8 /* Peripheral Clock Selection Register */
#define LPC17_SYSCON_PBOOST_OFFSET       0x01b0 /* Power Boost Register */
#define LPC17_SYSCON_SPIFICLKSEL_OFFSET  0x01b4 /* SPIFI Clock Selection Register */
#define LPC17_SYSCON_LCDCFG_OFFSET       0x01b8 /* LCD Clock Configuration Register */

/* Device Interrupt Registers (Might be a error in the User Manual, might be at 0x5000c1c0) */

#define LPC17_SYSCON_USBINTST_OFFSET     0x01c0 /* USB Interrupt Status  */

/* DMA Request Select Register */

#define LPC17_SYSCON_DMAREQSEL_OFFSET    0x01c4 /* Selects between UART and timer DMA requests */

/* More clocking and power control -- Utility */

#define LPC17_SYSCON_CLKOUTCFG_OFFSET    0x01c8 /* Clock Output Configuration Register */

/* Peripheral Reset Control */

#define LPC17_SYSCON_RSTCON0_OFFSET      0x01cc /* Individual Peripheral Reset Control Bits */
#define LPC17_SYSCON_RSTCON1_OFFSET      0x01d0 /* Individual Peripheral Reset Control Bits */

/* EMC Clock Control and Calibration */

#define LPC17_SYSCON_EMCDLYCTL_OFFSET    0x01dc /* Programmable Delays for SDRAM Operation */
#define LPC17_SYSCON_EMCCAL_OFFSET       0x01e0 /* Calibration Counter for EMCDLYCTL */


/* Register addresses *******************************************************************************/
/* Flash accelerator module */

#define LPC17_SYSCON_FLASHCFG            (LPC17_SYSCON_BASE+LPC17_SYSCON_FLASHCFG_OFFSET)

/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define LPC17_SYSCON_MEMMAP              (LPC17_SYSCON_BASE+LPC17_SYSCON_MEMMAP_OFFSET)

/* Clocking and power control - Phase locked loops */

#define LPC17_SYSCON_PLL0CON             (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0CON_OFFSET)
#define LPC17_SYSCON_PLL0CFG             (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0CFG_OFFSET)
#define LPC17_SYSCON_PLL0STAT            (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0STAT_OFFSET)
#define LPC17_SYSCON_PLL0FEED            (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0FEED_OFFSET)

#define LPC17_SYSCON_PLL1CON             (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1CON_OFFSET)
#define LPC17_SYSCON_PLL1CFG             (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1CFG_OFFSET)
#define LPC17_SYSCON_PLL1STAT            (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1STAT_OFFSET)
#define LPC17_SYSCON_PLL1FEED            (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1FEED_OFFSET)

/* Clocking and power control - Peripheral power control registers */

#define LPC17_SYSCON_PCON                (LPC17_SYSCON_BASE+LPC17_SYSCON_PCON_OFFSET)
#define LPC17_SYSCON_PCONP               (LPC17_SYSCON_BASE+LPC17_SYSCON_PCONP_OFFSET)

/* Clocking and power control -- Clock dividers */

#define LPC17_SYSCON_EMCCLKCFG           (LPC17_SYSCON_BASE+LPC17_SYSCON_EMCCLKCFG_OFFSET)
#define LPC17_SYSCON_CCLKCFG             (LPC17_SYSCON_BASE+LPC17_SYSCON_CCLKCFG_OFFSET)
#define LPC17_SYSCON_USBCLKCFG           (LPC17_SYSCON_BASE+LPC17_SYSCON_USBCLKCFG_OFFSET)

/* 0x400f c110 - 0x400f c114: CAN Wake and Sleep Registers */

/* Clocking and power control -- Clock source selection */

#define LPC17_SYSCON_CLKSRCSEL           (LPC17_SYSCON_BASE+LPC17_SYSCON_CLKSRCSEL_OFFSET)
#define LPC17_SYSCON_CANSLEEPCLR         (LPC17_SYSCON_BASE+LPC17_SYSCON_CANSLEEPCLR_OFFSET)
#define LPC17_SYSCON_CANWAKEFLAGS        (LPC17_SYSCON_BASE+LPC17_SYSCON_CANWAKEFLAGS_OFFSET)

/* System control registers -- External Interrupts */

#define LPC17_SYSCON_EXTINT              (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTINT_OFFSET)

#define LPC17_SYSCON_EXTMODE             (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTMODE_OFFSET)
#define LPC17_SYSCON_EXTPOLAR            (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTPOLAR_OFFSET)

/* System control registers -- Reset */

#define LPC17_SYSCON_RSID                (LPC17_SYSCON_BASE+LPC17_SYSCON_RSID_OFFSET)

/* System control registers -- Syscon Miscellaneous Registers */

#define LPC17_SYSCON_MATRIXARB           (LPC17_SYSCON_BASE+LPC17_SYSCON_MATRIXARB_OFFSET)
#define LPC17_SYSCON_SCS                 (LPC17_SYSCON_BASE+LPC17_SYSCON_SCS_OFFSET)
#define LPC17_SYSCON_PCLKSEL             (LPC17_SYSCON_BASE+LPC17_SYSCON_PCLKSEL_OFFSET)
#define LPC17_SYSCON_PBOOST              (LPC17_SYSCON_BASE+LPC17_SYSCON_PBOOST_OFFSET)
#define LPC17_SYSCON_SPIFICLKSEL         (LPC17_SYSCON_BASE+LPC17_SYSCON_SPIFICLKSEL_OFFSET)
#define LPC17_SYSCON_LCDCFG              (LPC17_SYSCON_BASE+LPC17_SYSCON_LCDCFG_OFFSET)

/* Device Interrupt Registers (Might be a error in the User Manual, might be at 0x5000c1c0) */

#define LPC17_SYSCON_USBINTST            (LPC17_SYSCON_BASE+LPC17_SYSCON_USBINTST_OFFSET)

/* DMA Request Select Register */

#define LPC17_SYSCON_DMAREQSEL           (LPC17_SYSCON_BASE+LPC17_SYSCON_DMAREQSEL_OFFSET)

/* More clocking and power control -- Utility */

#define LPC17_SYSCON_CLKOUTCFG           (LPC17_SYSCON_BASE+LPC17_SYSCON_CLKOUTCFG_OFFSET)


/* Peripheral Reset Control */

#define LPC17_SYSCON_RSTCON0             (LPC17_SYSCON_BASE+LPC17_SYSCON_RSTCON0_OFFSET)
#define LPC17_SYSCON_RSTCON1             (LPC17_SYSCON_BASE+LPC17_SYSCON_RSTCON1_OFFSET)

/* EMC Clock Control and Calibration */

#define LPC17_SYSCON_EMCDLYCTL           (LPC17_SYSCON_BASE+LPC17_SYSCON_EMCDLYCTL_OFFSET)
#define LPC17_SYSCON_EMCCAL              (LPC17_SYSCON_BASE+LPC17_SYSCON_EMCCAL_OFFSET)

/* Register bit definitions *************************************************************************/
/* Flash accelerator module */
                                                      /* Bits 0-11:  Reserved */
#define SYSCON_FLASHCFG_TIM_SHIFT        (12)         /* Bits 12-15: FLASHTIM Flash access time */
#define SYSCON_FLASHCFG_TIM_MASK         (15 << SYSCON_FLASHCFG_TIM_SHIFT)
#  define SYSCON_FLASHCFG_TIM_1          (0 << SYSCON_FLASHCFG_TIM_SHIFT) /* 1 CPU clock <= 20 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_2          (1 << SYSCON_FLASHCFG_TIM_SHIFT) /* 2 CPU clock <= 40 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_3          (2 << SYSCON_FLASHCFG_TIM_SHIFT) /* 3 CPU clock <= 60 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_4          (3 << SYSCON_FLASHCFG_TIM_SHIFT) /* 4 CPU clock <= 80 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_5          (4 << SYSCON_FLASHCFG_TIM_SHIFT) /* 5 CPU clock <= 100 MHz CPU clock
                                                                           * (Up to 120 Mhz for LPC1788x) */
#  define SYSCON_FLASHCFG_TIM_6          (5 << SYSCON_FLASHCFG_TIM_SHIFT) /* "safe" setting for any conditions */
                                                   /* Bits 16-31:  Reserved */
/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define SYSCON_MEMMAP_MAP                (1 << 0)  /* Bit 0:
                                                    * 0:Boot mode. A portion of the Boot ROM is mapped to address 0.
                                                    * 1:User mode. The on-chip Flash memory is mapped to address 0 */
                                                   /* Bits 1-31:  Reserved */
/* Clocking and power control -- Clock source selection */

#define SYSCON_CLKSRCSEL_SHIFT           (0)       /* Bits 0: Clock selection */
#define SYSCON_CLKSRCSEL_MASK            (1 << SYSCON_CLKSRCSEL_SHIFT)
#  define SYSCON_CLKSRCSEL_INTRC         (0 << SYSCON_CLKSRCSEL_SHIFT) /* PLL0 source = internal RC oscillator */
#  define SYSCON_CLKSRCSEL_MAIN          (1 << SYSCON_CLKSRCSEL_SHIFT) /* PLL0 source = main oscillator */
                                                   /* Bits 1-31:  Reserved */
/* Clocking and power control - Phase locked loops */
/* PLL0/1 Control register */

#define SYSCON_PLLCON_PLLE               (1 << 0)  /* Bit 0: PLL Enable */
                                                   /* Bits 1-31:  Reserved */
/* PLL0/1 Configuration register */

#define SYSCON_PLLCFG_MSEL_SHIFT         (0)       /* Bit 0-4: PLL Multiplier value */
#define SYSCON_PLLCFG_MSEL_MASK          (0x1f << SYSCON_PLL0CFG_MSEL_SHIFT)
#define SYSCON_PLLCFG_PSEL_SHIFT         (5)       /* Bit 5-6: PLL Pre-Divider value */
#define SYSCON_PLLCFG_PSEL_MASK          (3 << SYSCON_PLL0CFG_PSEL_SHIFT)

/* PLL0/1 Status register */

#define SYSCON_PLLSTAT_MSEL_SHIFT        (0)       /* Bit 0-4: PLLMultiplier value readback */
#define SYSCON_PLLSTAT_MSEL_MASK         (0x1f << SYSCON_PLLSTAT_MSEL_SHIFT)
#define SYSCON_PLLSTAT_PSEL_SHIFT        (5)       /* Bit 5-6: PLL Pre-Divider value readback */
#define SYSCON_PLLSTAT_PSEL_MASK         (3 << SYSCON_PLLSTAT_PSEL_SHIFT)
                                                   /* Bit 7:  Reserved */
#define SYSCON_PLLSTAT_PLLE              (1 << 8)  /* Bit 8: PLL enable readback */
#define SYSCON_PLLSTAT_PLLC              (1 << 9)  /* Bit 9: PLL connect readback */
#define SYSCON_PLLSTAT_PLOCK             (1 << 10) /* Bit 10: PLL lock status */
                                                   /* Bits 11-31:  Reserved */
/* PLL0/1 Feed register */

#define SYSCON_PLLFEED_SHIFT             (0)       /* Bit 0-7: PLL0/1 feed sequence */
#define SYSCON_PLLFEED_MASK              (0xff << SYSCON_PLLFEED_SHIFT)
                                                   /* Bits 8-31:  Reserved */
/* Clocking and power control -- Clock dividers */

/* EMC Clock Selection Register */

#define SYSCON_EMCDIV                    (1 << 0)  /* Bit 0: EMC Clock rate relative to CPU */
                                                   /* 0: EMC uses same clock as CPU */
                                                   /* 1: EMC uses half the rate of CPU */
                                                   /* Bits 1-31: Reserved
/* CPU Clock Configuration register */

#define SYSCON_CCLKCFG_CCLKDIV_SHIFT     (0)       /* 0-4: Divide value for CPU clock (CCLK) */
#define SYSCON_CCLKCFG_CCLKDIV_MASK      (0x1f << SYSCON_CCLKCFG_CCLKDIV_SHIFT)
#  define SYSCON_CCLKCFG_CCLKDIV(n)      ((n-1) << SYSCON_CCLKCFG_CCLKDIV_SHIFT) /* n = 2 - 31 */
                                                   /* Bits 5-7:  Reserved */
#define SYSCON_CCLKCFG_CCLKSEL           (1 << 8)  /* Bit 8: Select input clock to CPU clock divider */
                                                   /* 0: Sysclk used as input to CCLKDIV */
                                                   /* 1: Main PLL used as input to CCLKDIV */
                                                   /* Bits 9-31:  Reserved */
/* USB Clock Configuration register */

#define SYSCON_USBCLKCFG_USBDIV_SHIFT    (0)       /* Bits 0-4: PLL0/1 divide value USB clock */
#define SYSCON_USBCLKCFG_USBDIV_MASK     (0x1f << SYSCON_USBCLKCFG_USBDIV_SHIFT)
#  define SYSCON_USBCLKCFG_USBDIV_DIV1   (1 << SYSCON_USBCLKCFG_USBDIV_SHIFT) /* PLL0/1 output must be 48MHz */
#  define SYSCON_USBCLKCFG_USBDIV_DIV2   (2 << SYSCON_USBCLKCFG_USBDIV_SHIFT) /* PLL0/1 output must be 96MHz */
#  define SYSCON_USBCLKCFG_USBDIV_DIV3   (3 << SYSCON_USBCLKCFG_USBDIV_SHIFT) /* PLL0/1 output must be 144MHz */
                                                   /* Bits 5-7:  Reserved */
#define SYSCON_USBCLKCFG_USBSEL_SHIFT    (8)       /* Bits 8-9: Input clock to USBDIV */
#define SYSCON_USBCLKCFG_USBSEL_MASK     (3 << SYSCON_USBCLKCFG_USBSEL_SHIFT)
#define SYSCON_USBCLKCFG_USBSEL_MAINPLL  (1 << SYSCON_USBCLKCFG_USBSEL_SHIFT)  /* 01: PLL0 is used as input clock to USBDIV */
#define SYSCON_USBCLKCFG_USBSEL_ALTPLL   (2 << SYSCON_USBCLKCFG_USBSEL_SHIFT)  /* 10: PLL1 is used as input clock to USBDIV */
                                                                               /* 11: unused */
                                                   /* Bits 10-31:  Reserved */
/* CAN0/1 Sleep Clear Register */
                                                   /* Bit 0: Reserved */
#define SYSCON_CANSLEEPCLR_SHIFT         (1)       /* Bits 1-2: CAN0/1 Sleep Status and Control */
#define SYSCON_CANSLEEPCLR_MASK          (3 << SYSCON_CANSLEEPCLR_SHIFT) /*  */
#define SYSCON_CANSLEEPCLR_CAN1          (1 << SYSCON_CANSLEEPCLR_SHIFT) /* CAN1 Sleep Status */
#define SYSCON_CANSLEEPCLR_CAN2          (2 << SYSCON_CANSLEEPCLR_SHIFT) /* CAN2 Sleep Status */
                                                   /* Read 1: CAN channel in sleep mode */
                                                   /* Write 1: CAN channel clocks restored */
                                                   /* Bits 3-31: Reserved */
/* CAN0/1 WakeUp Flags Register */
                                                   /* Bit 0: Reserved */
#define SYSCON_CANWAKEFLAGS_SHIFT        (1)       /* Bits 1-2: CAN0/1 WakeUp Status */
#define SYSCON_CANWAKEFLAGS_MASK         (3 << SYSCON_CANWAKEFLAGS_SHIFT) /*  */
#define SYSCON_CANWAKEFLAGS_CAN1         (1 << SYSCON_CANWAKEFLAGS_SHIFT) /* CAN1 WakeUp Status */
#define SYSCON_CANWAKEFLAGS_CAN2         (2 << SYSCON_CANWAKEFLAGS_SHIFT) /* CAN2 WakeUp Status */
                                                   /* Read 1: CAN channel falling edge occur on receive line */
                                                   /* Write 1: CAN channel clears wakeup flag bit */
                                                   /* Bits 3-31: Reserved */
/* Peripheral Clock Selection register */
/* PCLK is common to all peripheral */

#define SYSCON_PCLKSEL_PCLKDIV_SHIFT     (0)       /* Bits 0-4: Clock divide value for all APB peripherals */
#define SYSCON_PCLKSEL_PCLKDIV_MASK      (0x1f << SYSCON_PCLKSEL_PCLKDIV_SHIFT)
#  define SYSCON_PCLKSEL_PCLKDIV(n)      ((n-1) << SYSCON_PCLKSEL_PCLKDIV_SHIFT) /* n = 2 - 31 */
                                                   /* Bits 5-31: Reserved */
/* Power Boost Control Register */

#define SYSCON_PBOOST_BOOST_SHIFT        (0)       /* Bits 0-1: Boost control bits */
#define SYSCON_PBOOST_BOOST_MASK         (3 << SYSCON_PBOOST_BOOST_SHIFT)
#define SYSCON_PBOOST_BOOST_OFF          (0)       /* Boost OFF, operation must be below 100MHz */
#define SYSCON_PBOOST_BOOST_ON           (3)       /* Boost ON, operation upto 120MHz allowed */
                                                   /* Bits 2-31: Reserved */
/* SPIFI Clock Selection Register */

#define SYSCON_SPIFICLKSEL_SPIFIDIV_SHIFT  (0)     /* Bits 0-4: divide value for SPIFI clock  */
#define SYSCON_SPIFICLKSEL_SPIFIDIV_MASK   (0x1f << SYSCON_SPIFICLKSEL_SPIFIDIV_SHIFT)
# define SYSCON_SPIFICLKSEL_SPIFIDIV(n)    ((n-1) << SYSCON_SPIFICLKSEL_SPIFIDIV_SHIFT) /* n = 2 - 31 */
                                                   /* Bits 5-7: Reserved */
#define SYSCON_SPIFICLKSEL_SPIFISEL_SHIFT  (8)     /* Bits 8-9: Selects input clock for SPIFI clock divider */
#define SYSCON_SPIFICLKSEL_SPIFISEL_MASK   (3 << SYSCON_SPIFICLKSEL_SPIFISEL_SHIFT)
#define SYSCON_SPIFICLKSEL_SPIFISEL_SYSCLK (0)     /* Sysclk used as input to SPIFIDIV  */
#define SYSCON_SPIFICLKSEL_SPIFISEL_PLL0   (1 << SYSCON_SPIFICLKSEL_SPIFISEL_SHIFT) /* Main PLL used as input to SPIFIDIV */
#define SYSCON_SPIFICLKSEL_SPIFISEL_PLL1   (2 << SYSCON_SPIFICLKSEL_SPIFISEL_SHIFT) /* Alt PLL used as input to SPIFIDIV */
                                                   /* Bits 10-31: Reserved */
/* LCD Configuration Register  */

#define SYSCON_LCDCFG_CLKDIV_SHIFT       (0) /* Bits 0-4: LCD Panel clock prescaler  */
#define SYSCON_LCDCFG_CLKDIV_MASK        (0x1f << SYSCON_LCDCFG_CLKDIV_SHIFT)
#define SYSCON_LCDCFG_CLKDIV(n)          ((n+1) << SYSCON_LCDCFG_CLKDIV_SHIFT) /* n = 0 - 31 */
                                             /* Bits 5-31: Reserved */
/* Clocking and power control - Peripheral power control registers */
/* Power Control Register */

#define SYSCON_PCON_PM0                  (1 << 0)  /* Bit 0:  Power mode control bit 0 */
#define SYSCON_PCON_PM1                  (1 << 1)  /* Bit 1:  Power mode control bit 1 */
#define SYSCON_PCON_BODRPM               (1 << 2)  /* Bit 2:  Brown-Out Reduced Power Mode */
#define SYSCON_PCON_BOGD                 (1 << 3)  /* Bit 3:  Brown-Out Global Disable */
#define SYSCON_PCON_BORD                 (1 << 4)  /* Bit 4:  Brown-Out Reset Disable */
                                                   /* Bits 5-7:  Reserved */
#define SYSCON_PCON_SMFLAG               (1 << 8)  /* Bit 8:  Sleep Mode entry flag */
#define SYSCON_PCON_DSFLAG               (1 << 9)  /* Bit 9:  Deep Sleep entry flag */
#define SYSCON_PCON_PDFLAG               (1 << 10) /* Bit 10: Power-down entry flag */
#define SYSCON_PCON_DPDFLAG              (1 << 11) /* Bit 11:  Deep Power-down entry flag */
                                                   /* Bits 12-31:  Reserved */
/* Power Control for Peripherals Register */

#define SYSCON_PCONP_PCLCD               (1 << 0)  /* Bit 0:  LCD power/clock control */
#define SYSCON_PCONP_PCTIM0              (1 << 1)  /* Bit 1:  Timer/Counter 0 power/clock control */
#define SYSCON_PCONP_PCTIM1              (1 << 2)  /* Bit 2:  Timer/Counter 1 power/clock control */
#define SYSCON_PCONP_PCUART0             (1 << 3)  /* Bit 3:  UART0 power/clock control */
#define SYSCON_PCONP_PCUART1             (1 << 4)  /* Bit 4:  UART1 power/clock control */
#define SYSCON_PCONP_PCPWM0              (1 << 5)  /* Bit 5:  PWM0 power/clock control */
#define SYSCON_PCONP_PCPWM1              (1 << 6)  /* Bit 6:  PWM1 power/clock control */
#define SYSCON_PCONP_PCI2C0              (1 << 7)  /* Bit 7:  I2C0 power/clock control */
#define SYSCON_PCONP_PCSPI               (1 << 8)  /* Bit 8:  SPI power/clock control */
#define SYSCON_PCONP_PCRTC               (1 << 9)  /* Bit 9:  RTC power/clock control */
#define SYSCON_PCONP_PCSSP1              (1 << 10) /* Bit 10: SSP 1 power/clock control */
#define SYSCON_PCONP_PCEMC               (1 << 11) /* Bit 11: External Memory */
#define SYSCON_PCONP_PCADC               (1 << 12) /* Bit 12: A/D converter (ADC) power/clock control */
#define SYSCON_PCONP_PCCAN1              (1 << 13) /* Bit 13: CAN Controller 1 power/clock control */
#define SYSCON_PCONP_PCCAN2              (1 << 14) /* Bit 14: CAN Controller 2 power/clock control */
#define SYSCON_PCONP_PCGPIO              (1 << 15) /* Bit 15: GPIOs power/clock enable */
#define SYSCON_PCONP_PCRIT               (1 << 16) /* Bit 16: Repetitive Interrupt Timer power/clock control */
#define SYSCON_PCONP_PCMCPWM             (1 << 17) /* Bit 17: Motor Control PWM */
#define SYSCON_PCONP_PCQEI               (1 << 18) /* Bit 18: Quadrature Encoder power/clock control */
#define SYSCON_PCONP_PCI2C1              (1 << 19) /* Bit 19: I2C1 power/clock control */
#define SYSCON_PCONP_PCSSP0              (1 << 20) /* Bit 20: SSP2 power/clock control */
#define SYSCON_PCONP_PCSSP0              (1 << 21) /* Bit 21: SSP0 power/clock control */
#define SYSCON_PCONP_PCTIM2              (1 << 22) /* Bit 22: Timer 2 power/clock control */
#define SYSCON_PCONP_PCTIM3              (1 << 23) /* Bit 23: Timer 3 power/clock control */
#define SYSCON_PCONP_PCUART2             (1 << 24) /* Bit 24: UART 2 power/clock control */
#define SYSCON_PCONP_PCUART3             (1 << 25) /* Bit 25: UART 3 power/clock control */
#define SYSCON_PCONP_PCI2C2              (1 << 26) /* Bit 26: I2C 2 power/clock control */
#define SYSCON_PCONP_PCI2S               (1 << 27) /* Bit 27: I2S power/clock control */
#define SYSCON_PCONP_PCSDC               (1 << 28) /* Bit 28: SD Card power/clock control */
#define SYSCON_PCONP_PCGPDMA             (1 << 29) /* Bit 29: GPDMA function power/clock control */
#define SYSCON_PCONP_PCENET              (1 << 30) /* Bit 30: Ethernet block power/clock control */
#define SYSCON_PCONP_PCUSB               (1 << 31) /* Bit 31: USB power/clock control */

/* More clocking and power control -- Utility */

#define SYSCON_CLKOUTCFG_SEL_SHIFT       (0)       /* Bits 0-3: Selects clock source for CLKOUT */
#define SYSCON_CLKOUTCFG_SEL_MASK        (15 << SYSCON_CLKOUTCFG_SEL_SHIFT)
#  define SYSCON_CLKOUTCFG_SEL_CPU       (0 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=CPU clock */
#  define SYSCON_CLKOUTCFG_SEL_MAIN      (1 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=main osc */
#  define SYSCON_CLKOUTCFG_SEL_INTRC     (2 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=internal RC osc */
#  define SYSCON_CLKOUTCFG_SEL_USB       (3 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=USB clock */
#  define SYSCON_CLKOUTCFG_SEL_RTC       (4 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=RTC osc */
#  define SYSCON_CLKOUTCFG_SEL_SPIFI     (5 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=SPIFI osc */
#define SYSCON_CLKOUTCFG_DIV_SHIFT       (4)       /* Bits 4-7:  CLKOUT divisor */
#define SYSCON_CLKOUTCFG_DIV_MASK        (15 << SYSCON_CLKOUTCFG_DIV_SHIFT)
#  define SYSCON_CLKOUTCFG_DIV(n)        ((n-1) << SYSCON_CLKOUTCFG_DIV_SHIFT) /* n=1..16 */
#define SYSCON_CLKOUTCFG_EN              (1 << 8)  /* Bit 8:  CLKOUT enable control */
#define SYSCON_CLKOUTCFG_ACT             (1 << 9)  /* Bit 9:  CLKOUT activity indication */
                                                   /* Bits 10-31: Reserved */
/* System control registers -- External Interrupts */
/* External Interrupt Flag register */

#define SYSCON_EXTINT_EINT0              (1 << 0)  /* Bit 0:  EINT0 */
#define SYSCON_EXTINT_EINT1              (1 << 1)  /* Bit 1:  EINT1 */
#define SYSCON_EXTINT_EINT2              (1 << 2)  /* Bit 2:  EINT2 */
#define SYSCON_EXTINT_EINT3              (1 << 3)  /* Bit 3:  EINT3 */
                                                   /* Bits 4-31: Reserved */
/* External Interrupt Mode register */

#define SYSCON_EXTMODE_EINT0             (1 << 0)  /* Bit 0:  1=EINT0 edge sensitive */
#define SYSCON_EXTMODE_EINT1             (1 << 1)  /* Bit 1:  1=EINT1 edge sensitive */
#define SYSCON_EXTMODE_EINT2             (1 << 2)  /* Bit 2:  1=EINT2 edge sensitive */
#define SYSCON_EXTMODE_EINT3             (1 << 3)  /* Bit 3:  1=EINT3 edge sensitive */
                                                   /* Bits 4-31: Reserved */
/* External Interrupt Polarity register */

#define SYSCON_EXTPOLAR_EINT0            (1 << 0)  /* Bit 0:  1=EINT0 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT1            (1 << 1)  /* Bit 1:  1=EINT1 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT2            (1 << 2)  /* Bit 2:  1=EINT2 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT3            (1 << 3)  /* Bit 3:  1=EINT3 high active/rising edge */
                                                   /* Bits 4-31: Reserved */
/* System control registers -- Reset */
/* Reset Source Identification Register */

#define SYSCON_RSID_POR                  (1 << 0)  /* Bit 0:  Power on reset */
#define SYSCON_RSID_EXTR                 (1 << 1)  /* Bit 1:  external RESET signal */
#define SYSCON_RSID_WDTR                 (1 << 2)  /* Bit 2:  Watchdog Timer time out w/WDTRESET */
#define SYSCON_RSID_BODR                 (1 << 3)  /* Bit 3:  Brown out detection */
#define SYSCON_RSID_SYSRESET             (1 << 4)  /* Bit 4:  System Reset */
#define SYSCON_RSID_LOCKUP               (1 << 5)  /* Bit 5:  Lockup Reset */
                                                   /* Bits 6-31: Reserved */
/* System control registers -- Matrix Arbitration Priorities */
/*TODO*/
#define SYSCON_MATRIXARB_PRI_ICODE       (3 << 0)  /* Bit 0-1:  I-Code bus priority (should be lower than D-Code */
#define SYSCON_MATRIXARB_PRI_DCODE       (3 << 2)  /* Bit 2-3:  D-Code bus priority */
#define SYSCON_MATRIXARB_PRI_SYS         (3 << 4)  /* Bit 4-5:  System bus priority */
#define SYSCON_MATRIXARB_PRI_GPDMA       (3 << 6)  /* Bit 6-7:  General Purpose DMA priority */
#define SYSCON_MATRIXARB_PRI_ETH         (3 << 8)  /* Bit 8-9:  Ethernet DMA priority */
#define SYSCON_MATRIXARB_PRI_LCD         (3 << 10) /* Bit 10-11:  LCD DMA priority */
#define SYSCON_MATRIXARB_PRI_USB         (3 << 12) /* Bit 12-13:  USB DMA priority */
                                                   /* Bits 14-15: Reserved */
#define SYSCON_MATRIXARB_ROM_LAT         (1 << 16) /* Bit 16:  ROM Latency select (should always be zero) */
                                                   /* Bits 17-31: Reserved */
/* System control registers -- Syscon Miscellaneous Registers */

#define SYSCON_SCS_EMCSC                 (1 << 0)  /* Bit 0:  EMC shift control */
#define SYSCON_SCS_EMCRD                 (1 << 1)  /* Bit 1:  EMC reset disable */
#define SYSCON_SCS_EMCBC                 (1 << 2)  /* Bit 2:  EMC burst control */
#define SYSCON_SCS_MCIPWRAL              (1 << 3)  /* Bit 3:  MCI power active level */
#define SYSCON_SCS_OSCRS                 (1 << 4)  /* Bit 4:  Main oscillator range select */
#define SYSCON_SCS_OSCEN                 (1 << 5)  /* Bit 5:  Main oscillator enable */
#define SYSCON_SCS_OSCSTAT               (1 << 6)  /* Bit 6:  Main oscillator status */
                                                   /* Bits 7-31: Reserved */
/* Device Interrupt Registers */
/* USB Interrupt Status register */

#define SYSCON_USBINTST_REQLP            (1 << 0)  /* Bit 0:  Low priority interrupt line status */
#define SYSCON_USBINTST_REQHP            (1 << 1)  /* Bit 1:  High priority interrupt line status */
#define SYSCON_USBINTST_REQDMA           (1 << 2)  /* Bit 2:  DMA interrupt line status */
#define SYSCON_USBINTST_HOSTINT          (1 << 3)  /* Bit 3:  USB host interrupt line status */
#define SYSCON_USBINTST_ATXINT           (1 << 4)  /* Bit 4:  External ATX interrupt line status */
#define SYSCON_USBINTST_OTGINT           (1 << 5)  /* Bit 5:  OTG interrupt line status */
#define SYSCON_USBINTST_I2CINT           (1 << 6)  /* Bit 6:  I2C module interrupt line status */
                                                   /* Bit 7:  Reserved */
#define SYSCON_USBINTST_NEEDCLK          (1 << 8)  /* Bit 8:  USB need clock indicator */
                                                   /* Bits 9-30: Reserved */
#define SYSCON_USBINTST_ENINTS           (1 << 31) /* Bit 31: Enable all USB interrupts */

/* DMA Request Select Register */

#define SYSCON_DMAREQSEL_INP0            (1 << 0)  /* Bit 0:  Input 0 0=unused 1=Timer 0 match 0 */
#define SYSCON_DMAREQSEL_INP1            (1 << 1)  /* Bit 1:  Input 1 0=SD 1=Timer 0 match 1 */
#define SYSCON_DMAREQSEL_INP2            (1 << 2)  /* Bit 2:  Input 2 0=SSP0 TX 1=Timer 1 match 0 */
#define SYSCON_DMAREQSEL_INP3            (1 << 3)  /* Bit 3:  Input 3 0=SSP0 RX 1=Timer 1 match 1 */
#define SYSCON_DMAREQSEL_INP4            (1 << 4)  /* Bit 4:  Input 4 0=SSP1 TX 1=Timer 2 match 0 */
#define SYSCON_DMAREQSEL_INP5            (1 << 5)  /* Bit 5:  Input 5 0=SSP1 RX 1=Timer 2 match 1 */
#define SYSCON_DMAREQSEL_INP6            (1 << 6)  /* Bit 6:  Input 6 0=SSP2 TX 1=I2S0 */
#define SYSCON_DMAREQSEL_INP7            (1 << 7)  /* Bit 7:  Input 7 0=SSP2 RX 1=I2S1 */
                                                   /* Bits 8-9: Reserved */
#define SYSCON_DMAREQSEL_INP10           (1 << 10) /* Bit 10:  Input 10 0=UART0 TX 1=UART3 TX */
#define SYSCON_DMAREQSEL_INP11           (1 << 11) /* Bit 11:  Input 11 0=UART0 RX 1=UART3 RX */
#define SYSCON_DMAREQSEL_INP12           (1 << 12) /* Bit 12:  Input 12 0=UART1 TX 1=UART4 TX */
#define SYSCON_DMAREQSEL_INP13           (1 << 13) /* Bit 13:  Input 13 0=UART1 RX 1=UART4 RX */
#define SYSCON_DMAREQSEL_INP14           (1 << 14) /* Bit 14:  Input 14 0=UART2 TX 1=Timer 3 match 0 */
#define SYSCON_DMAREQSEL_INP15           (1 << 15) /* Bit 15:  Input 15 0=UART2 RX 1=Timer 3 match 1 */
                                                   /* Bits 16-31: Reserved */
/* Reset Control Register 0 */

#define SYSCON_RSTCON0_RSTLCD            (1 << 0)  /* LCD controller reset control bit */
#define SYSCON_RSTCON0_RSTTIM0           (1 << 1)  /* Timer/Counter 0 reset control bit */
#define SYSCON_RSTCON0_RSTTIM1           (1 << 2)  /* Timer/Counter 1 reset control bit */
#define SYSCON_RSTCON0_RSTUART0          (1 << 3)  /* UART0 reset control bit */
#define SYSCON_RSTCON0_RSTUART1          (1 << 4)  /* UART1 reset control bit */
#define SYSCON_RSTCON0_RSTPWM0           (1 << 5)  /* PWM0 reset control bit */
#define SYSCON_RSTCON0_RSTPWM1           (1 << 6)  /* PWM1 reset control bit */
#define SYSCON_RSTCON0_RSTI2C0           (1 << 7)  /* The I2C0 interface reset control bit */
#define SYSCON_RSTCON0_RSTUART4          (1 << 8)  /* UART4 reset control bit */
#define SYSCON_RSTCON0_RSTRTC            (1 << 9)  /* RTC and Event Monitor/Recorder reset control bit. RTC reset is limited */
#define SYSCON_RSTCON0_RSTSSP1           (1 << 10) /* The SSP 1 interface reset control bit */
#define SYSCON_RSTCON0_RSTEMC            (1 << 11) /* External Memory Controller reset control bit */
#define SYSCON_RSTCON0_RSTADC            (1 << 12) /* A/D converter (ADC) reset control bit */
#define SYSCON_RSTCON0_RSTCAN1           (1 << 13) /* CAN Controller 1 reset control bit */
                                                   /* Note: The CAN acceptance filter may be reset by 0
                                                    * a separate bit in the RSTCON1 register. */
#define SYSCON_RSTCON0_RSTCAN2           (1 << 14) /* CAN Controller 2 reset control bit */
                                                   /* Note: The CAN acceptance filter may be reset by 0
                                                    * a separate bit in the RSTCON1 register */
#define SYSCON_RSTCON0_RSTGPIO           (1 << 15) /* Reset control bit for GPIO, and GPIO interrupts */
                                                   /* Note: IOCON may be reset by a 0
                                                    * separate bit in the RSTCON1 register */
#define SYSCON_RSTCON0_RSTSPIFI          (1 << 16) /* SPI Flash Interface reset control bit (LPC1773 only) */
#define SYSCON_RSTCON0_RSTMCPWM          (1 << 17) /* Motor Control PWM reset control bit */
#define SYSCON_RSTCON0_RSTQEI            (1 << 18) /* Quadrature Encoder Interface reset control bit */
#define SYSCON_RSTCON0_RSTI2C1           (1 << 19) /* The I2C1 interface reset control bit */
#define SYSCON_RSTCON0_RSTSSP2           (1 << 20) /* The SSP2 interface reset control bit */
#define SYSCON_RSTCON0_RSTSSP0           (1 << 21) /* The SSP0 interface reset control bit */
#define SYSCON_RSTCON0_RSTTIM2           (1 << 22) /* Timer 2 reset control bit */
#define SYSCON_RSTCON0_RSTTIM3           (1 << 23) /* Timer 3 reset control bit */
#define SYSCON_RSTCON0_RSTUART2          (1 << 24) /* UART 2 reset control bit */
#define SYSCON_RSTCON0_RSTUART3          (1 << 25) /* UART 3 reset control bit */
#define SYSCON_RSTCON0_RSTI2C2           (1 << 26) /* I2C2 interface reset control bit.*/
#define SYSCON_RSTCON0_RSTI2S            (1 << 27) /* I2S interface reset control bit */
#define SYSCON_RSTCON0_RSTSDC            (1 << 28) /* SD Card interface reset control bit */
#define SYSCON_RSTCON0_RSTGPDMA          (1 << 29) /* GPDMA function reset control bit */

#define SYSCON_RSTCON0_RSTENET           (1 << 30) /* Ethernet block reset control bit */
#define SYSCON_RSTCON0_RSTUSB            (1 << 31) /* USB interface reset control bit */

/* Reset Control Register 1 */

#define SYSCON_RSTCON1_RSTIOCON          (1 << 0) /* Reset control bit for the IOCON registers */
#define SYSCON_RSTCON1_RSTDAC            (1 << 1) /* D/A converter (DAC) reset control bit */
#define SYSCON_RSTCON1_RSTCANACC         (1 << 2) /* CAN acceptance filter reset control bit */
                                                  /* Bits 3-31: Reserved */
/* Delay Control Register - EMC */
                                                  /* Delay values multiplied by 250 picoseconds */
#define SYSCON_EMCDLYCTL_CMDDLY_SHIFT     (0)     /* Bits 0-4: Delay value for EMC outputs in command delayed mode */
#define SYSCON_EMCDLYCTL_CMDDLY_MASK      (0x1f << SYSCON_EMCDLYCTL_CMDDLY_SHIFT)
# define SYSCON_EMCDLYCTL_CMDDLY(n)       ((n+1) << SYSCON_EMCDLYCTL_CMDDLY_SHIFT) /* n = 2 - 31 */
                                                  /* Bits 5-7: Reserved */
#define SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT   (8)     /* Bits 8-12: Delay value for the feedback clock that controls input data sampling */
#define SYSCON_EMCDLYCTL_FBCLKDLY_MASK    (0x1f << SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT)
#define SYSCON_EMCDLYCTL_FBCLKDLY(n)      ((n+1)<< SYSCON_EMCDLYCTL_FBCLKDLY_SHIFT) /* n = 2 - 31 */
                                                  /* Bits 13-15: Reserved */
#define SYSCON_EMCDLYCTL_CLKOUT0DLY_SHIFT (16)    /* Bits 16-20: Delay value for the CLKOUT0 output */
#define SYSCON_EMCDLYCTL_CLKOUT0DLY_MASK  (0x1f << SYSCON_EMCDLYCTL_CLKOUT0DLY_SHIFT)
# define SYSCON_EMCDLYCTL_CLKOUT0DLY(n)   ((n+1) << SYSCON_EMCDLYCTL_CLKOUT0DLY_SHIFT) /* n = 2 - 31 */
                                                  /* Bits 21-23: Reserved */
#define SYSCON_EMCDLYCTL_CLKOUT1DLY_SHIFT (24)    /* Bits 24-28: Delay value for the CLKOUT1 output */
#define SYSCON_EMCDLYCTL_CLKOUT1DLY_MASK  (0x1f << SYSCON_EMCDLYCTL_CLKOUT1DLY_SHIFT)
# define SYSCON_EMCDLYCTL_CLKOUT1DLY(n)   ((n+1) << SYSCON_EMCDLYCTL_CLKOUT1DLY_SHIFT) /* n = 2 - 31 */
                                                  /* Bits 29-31: Reserved */
/* Calibration Register - EMC */

#define SYSCON_EMCCAL_CALVALUE_SHIFT     (0)      /* Bits 0-7: Ring oscillator count during 32 clocks of Internal RC */
#define SYSCON_EMCCAL_CALVALUE_MASK      (0xff << SYSCON_EMCCAL_CALVALUE_SHIFT)
//~ #define SYSCON_EMCCAL_CALVALUE
                                                  /* Bits 8-13: Reserved */
#define SYSCON_EMCCAL_START_SHIFT        (14)     /* Bit 14: Start control bit for EMC calibration counter */
#define SYSCON_EMCCAL_START_MASK         (1 << SYSCON_EMCCAL_START_SHIFT)
# define SYSCON_EMCCAL_START             (1)      /* Automatically cleared when measurement is done */
#define SYSCON_EMCCAL_DONE_SHIFT         (15)     /* Bit 15: Measurement completetion flag bit */
#define SYSCON_EMCCAL_DONE_MASK          (1 << SYSCON_EMCCAL_DONE_SHIFT)
                                                  /* Automatically cleared when START bit is set */
//~ # define SYSCON_EMCCAL_DONE
                                                  /* Bits 16-31: Reserved */
/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC178X_SYSCON_H */
