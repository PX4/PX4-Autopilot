/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc176x_syscon.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_SYSCON_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_SYSCON_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Flash accelerator module */

#define LPC17_SYSCON_FLASHCFG_OFFSET  0x0000 /* Flash Accelerator Configuration Register */

/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define LPC17_SYSCON_MEMMAP_OFFSET    0x0040 /* Memory Mapping Control register */

/* Clocking and power control - Phase locked loops */

#define LPC17_SYSCON_PLL0CON_OFFSET   0x0080 /* PLL0 Control Register */     
#define LPC17_SYSCON_PLL0CFG_OFFSET   0x0084 /* PLL0 Configuration Register */
#define LPC17_SYSCON_PLL0STAT_OFFSET  0x0088 /* PLL0 Status Register */
#define LPC17_SYSCON_PLL0FEED_OFFSET  0x008c /* PLL0 Feed Register */

#define LPC17_SYSCON_PLL1CON_OFFSET   0x00a0 /* PLL1 Control Register */     
#define LPC17_SYSCON_PLL1CFG_OFFSET   0x00a4 /* PLL1 Configuration Register */
#define LPC17_SYSCON_PLL1STAT_OFFSET  0x00a8 /* PLL1 Status Register */
#define LPC17_SYSCON_PLL1FEED_OFFSET  0x00ac /* PLL1 Feed Register */

/* Clocking and power control - Peripheral power control registers */

#define LPC17_SYSCON_PCON_OFFSET      0x00c0 /* Power Control Register */
#define LPC17_SYSCON_PCONP_OFFSET     0x00c4 /* Power Control for Peripherals Register */

/* Clocking and power control -- Clock dividers */

#define LPC17_SYSCON_CCLKCFG_OFFSET   0x0104 /* CPU Clock Configuration Register */
#define LPC17_SYSCON_USBCLKCFG_OFFSET 0x0108 /* USB Clock Configuration Register */

/* 0x400f c110 - 0x400f c114: CAN Wake and Sleep Registers */

/* Clocking and power control -- Clock source selection */

#define LPC17_SYSCON_CLKSRCSEL_OFFSET 0x010c      /* Clock Source Select Register */

/* System control registers -- External Interrupts */

#define LPC17_SYSCON_EXTINT_OFFSET    0x0140      /* External Interrupt Flag Register */

#define LPC17_SYSCON_EXTMODE_OFFSET   0x0148      /* External Interrupt Mode register */
#define LPC17_SYSCON_EXTPOLAR_OFFSET  0x014c      /* External Interrupt Polarity Register */

/* System control registers -- Reset */

#define LPC17_SYSCON_RSID_OFFSET      0x0180      /* Reset Source Identification Register */

/* System control registers -- Syscon Miscellaneous Registers */

#define LPC17_SYSCON_SCS_OFFSET       0x01a0      /* System Control and Status */

/* More clocking and power control -- Clock dividers */

#define LPC17_SYSCON_PCLKSEL0_OFFSET  0x01a8      /* Peripheral Clock Selection register 0 */
#define LPC17_SYSCON_PCLKSEL1_OFFSET  0x01ac      /* Peripheral Clock Selection register 1 */

/* Device Interrupt Registers (Might be a error in the User Manual, might be at 0x5000c1c0) */

#define LPC17_SYSCON_USBINTST_OFFSET  0x01c0      /* USB Interrupt Status  */

/* DMA Request Select Register */

#define LPC17_SYSCON_DMAREQSEL_OFFSET 0x01c4      /* Selects between UART and timer DMA requests */

/* More clocking and power control -- Utility */

#define LPC17_SYSCON_CLKOUTCFG_OFFSET 0x01c8 /* Clock Output Configuration Register */

/* Register addresses ***************************************************************/
/* Flash accelerator module */

#define LPC17_SYSCON_FLASHCFG         (LPC17_SYSCON_BASE+LPC17_SYSCON_FLASHCFG_OFFSET)

/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define LPC17_SYSCON_MEMMAP           (LPC17_SYSCON_BASE+LPC17_SYSCON_MEMMAP_OFFSET)

/* Clocking and power control - Phase locked loops */

#define LPC17_SYSCON_PLL0CON          (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0CON_OFFSET)
#define LPC17_SYSCON_PLL0CFG          (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0CFG_OFFSET)
#define LPC17_SYSCON_PLL0STAT         (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0STAT_OFFSET)
#define LPC17_SYSCON_PLL0FEED         (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL0FEED_OFFSET)

#define LPC17_SYSCON_PLL1CON          (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1CON_OFFSET)
#define LPC17_SYSCON_PLL1CFG          (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1CFG_OFFSET)
#define LPC17_SYSCON_PLL1STAT         (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1STAT_OFFSET)
#define LPC17_SYSCON_PLL1FEED         (LPC17_SYSCON_BASE+LPC17_SYSCON_PLL1FEED_OFFSET)

/* Clocking and power control - Peripheral power control registers */

#define LPC17_SYSCON_PCON             (LPC17_SYSCON_BASE+LPC17_SYSCON_PCON_OFFSET)
#define LPC17_SYSCON_PCONP            (LPC17_SYSCON_BASE+LPC17_SYSCON_PCONP_OFFSET)

/* Clocking and power control -- Clock dividers */

#define LPC17_SYSCON_CCLKCFG          (LPC17_SYSCON_BASE+LPC17_SYSCON_CCLKCFG_OFFSET)
#define LPC17_SYSCON_USBCLKCFG        (LPC17_SYSCON_BASE+LPC17_SYSCON_USBCLKCFG_OFFSET)

/* 0x400f c110 - 0x400f c114: CAN Wake and Sleep Registers */

/* Clocking and power control -- Clock source selection */

#define LPC17_SYSCON_CLKSRCSEL        (LPC17_SYSCON_BASE+LPC17_SYSCON_CLKSRCSEL_OFFSET)

/* System control registers -- External Interrupts */

#define LPC17_SYSCON_EXTINT           (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTINT_OFFSET)

#define LPC17_SYSCON_EXTMODE          (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTMODE_OFFSET)
#define LPC17_SYSCON_EXTPOLAR         (LPC17_SYSCON_BASE+LPC17_SYSCON_EXTPOLAR_OFFSET)

/* System control registers -- Reset */

#define LPC17_SYSCON_RSID             (LPC17_SYSCON_BASE+LPC17_SYSCON_RSID_OFFSET)

/* System control registers -- Syscon Miscellaneous Registers */

#define LPC17_SYSCON_SCS              (LPC17_SYSCON_BASE+LPC17_SYSCON_SCS_OFFSET)

/* More clocking and power control -- Clock dividers */

#define LPC17_SYSCON_PCLKSEL0         (LPC17_SYSCON_BASE+LPC17_SYSCON_PCLKSEL0_OFFSET)
#define LPC17_SYSCON_PCLKSEL1         (LPC17_SYSCON_BASE+LPC17_SYSCON_PCLKSEL1_OFFSET)

/* Device Interrupt Registers (Might be a error in the User Manual, might be at 0x5000c1c0) */

#define LPC17_SYSCON_USBINTST         (LPC17_SYSCON_BASE+LPC17_SYSCON_USBINTST_OFFSET)

/* DMA Request Select Register */

#define LPC17_SYSCON_DMAREQSEL        (LPC17_SYSCON_BASE+LPC17_SYSCON_DMAREQSEL_OFFSET)

/* More clocking and power control -- Utility */

#define LPC17_SYSCON_CLKOUTCFG        (LPC17_SYSCON_BASE+LPC17_SYSCON_CLKOUTCFG_OFFSET)

/* Register bit definitions *********************************************************/
/* Flash accelerator module */
                                                  /* Bits 0-11:  Reserved */
#define SYSCON_FLASHCFG_TIM_SHIFT     (12)        /* Bits 12-15: FLASHTIM Flash access time */
#define SYSCON_FLASHCFG_TIM_MASK      (15 << SYSCON_FLASHCFG_TIM_SHIFT)
#  define SYSCON_FLASHCFG_TIM_1       (0 << SYSCON_FLASHCFG_TIM_SHIFT) /* 1 CPU clock <= 20 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_2       (1 << SYSCON_FLASHCFG_TIM_SHIFT) /* 2 CPU clock <= 40 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_3       (2 << SYSCON_FLASHCFG_TIM_SHIFT) /* 3 CPU clock <= 60 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_4       (3 << SYSCON_FLASHCFG_TIM_SHIFT) /* 4 CPU clock <= 80 MHz CPU clock */
#  define SYSCON_FLASHCFG_TIM_5       (4 << SYSCON_FLASHCFG_TIM_SHIFT) /* 5 CPU clock <= 100 MHz CPU clock
                                                                        * (Up to 120 Mhz for LPC1759/69 only */
#  define SYSCON_FLASHCFG_TIM_6       (5 << SYSCON_FLASHCFG_TIM_SHIFT) /* "safe" setting for any conditions */
                                                  /* Bits 16-31:  Reserved */

/* Memory Mapping Control register (MEMMAP - 0x400F C040) */

#define SYSCON_MEMMAP_MAP             (1 << 0)   /* Bit 0:
                                                  * 0:Boot mode. A portion of the Boot ROM is mapped to address 0.
                                                  * 1:User mode. The on-chip Flash memory is mapped to address 0 */
                                                  /* Bits 1-31:  Reserved */

/* Clocking and power control -- Clock source selection */

#define SYSCON_CLKSRCSEL_SHIFT        (0)         /* Bits 0-1: Clock selection */
#define SYSCON_CLKSRCSEL_MASK         (3 << SYSCON_CLKSRCSEL_SHIFT)
#  define SYSCON_CLKSRCSEL_INTRC      (0 << SYSCON_CLKSRCSEL_SHIFT) /* PLL0 source = internal RC oscillator */
#  define SYSCON_CLKSRCSEL_MAIN       (1 << SYSCON_CLKSRCSEL_SHIFT) /* PLL0 source = main oscillator */
#  define SYSCON_CLKSRCSEL_RTC        (2 << SYSCON_CLKSRCSEL_SHIFT) /* PLL0 source = RTC oscillator */
                                                  /* Bits 2-31:  Reserved */

/* Clocking and power control - Phase locked loops */
/* PLL0/1 Control register */

#define SYSCON_PLLCON_PLLE            (1 << 0)    /* Bit 0: PLL0/1 Enable */
#define SYSCON_PLLCON_PLLC            (1 << 1)    /* Bit 1: PLL0/1 Connect */
                                                  /* Bits 2-31:  Reserved */
/* PLL0 Configuration register */

#define SYSCON_PLL0CFG_MSEL_SHIFT     (0)         /* Bit 0-14: PLL0 Multiplier value */
#define SYSCON_PLL0CFG_MSEL_MASK      (0x7fff << SYSCON_PLL0CFG_MSEL_SHIFT)
                                                  /* Bit 15:  Reserved */
#define SYSCON_PLL0CFG_NSEL_SHIFT     (16)        /* Bit 16-23: PLL0 Pre-Divider value */
#define SYSCON_PLL0CFG_NSEL_MASK      (0xff << SYSCON_PLL0CFG_NSEL_SHIFT)
                                                  /* Bits 24-31:  Reserved */
/* PLL1 Configuration register */

#define SYSCON_PLL1CFG_MSEL_SHIFT     (0)         /* Bit 0-4: PLL1 Multiplier value */
#define SYSCON_PLL1CFG_MSEL_MASK      (0x1f < SYSCON_PLL1CFG_MSEL_SHIFT)
#define SYSCON_PLL1CFG_NSEL_SHIFT     (5)         /* Bit 5-6: PLL1 Pre-Divider value */
#define SYSCON_PLL1CFG_NSEL_MASK      (3 << SYSCON_PLL1CFG_NSEL_SHIFT)
                                                  /* Bits 7-31:  Reserved */
/* PLL0 Status register */

#define SYSCON_PLL0STAT_MSEL_SHIFT    (0)         /* Bit 0-14: PLL0 Multiplier value readback */
#define SYSCON_PLL0STAT_MSEL_MASK     (0x7fff << SYSCON_PLL0STAT_MSEL_SHIFT)
                                                  /* Bit 15: Reserved */
#define SYSCON_PLL0STAT_NSEL_SHIFT    (16)        /* Bit 16-23: PLL0 Pre-Divider value readback */
#define SYSCON_PLL0STAT_NSEL_MASK     (0xff << SYSCON_PLL0STAT_NSEL_SHIFT)
#define SYSCON_PLL0STAT_PLLE          (1 << 24)   /* Bit 24: PLL0 enable readback */
#define SYSCON_PLL0STAT_PLLC          (1 << 25)   /* Bit 25: PLL0 connect readback */
#define SYSCON_PLL0STAT_PLOCK         (1 << 26)   /* Bit 26: PLL0 lock status */
                                                  /* Bits 27-31: Reserved */
/* PLL1 Status register */

#define SYSCON_PLL1STAT_MSEL_SHIFT    (0)         /* Bit 0-4: PLL1 Multiplier value readback */
#define SYSCON_PLL1STAT_MSEL_MASK     (0x1f << SYSCON_PLL1STAT_MSEL_SHIFT)
#define SYSCON_PLL1STAT_NSEL_SHIFT    (5)         /* Bit 5-6: PLL1 Pre-Divider value readback */
#define SYSCON_PLL1STAT_NSEL_MASK     (3 << SYSCON_PLL1STAT_NSEL_SHIFT)
                                                  /* Bit 7:  Reserved */
#define SYSCON_PLL1STAT_PLLE          (1 << 8)    /* Bit 8:  PLL1 enable readback */
#define SYSCON_PLL1STAT_PLLC          (1 << 9)    /* Bit 9:  PLL1 connect readback */
#define SYSCON_PLL1STAT_PLOCK         (1 << 10)   /* Bit 10: PLL1 lock status */
                                                  /* Bits 11-31: Reserved */
/* PLL0/1 Feed register */

#define SYSCON_PLLFEED_SHIFT          (0)         /* Bit 0-7: PLL0/1 feed sequence */
#define SYSCON_PLLFEED_MASK           (0xff << SYSCON_PLLFEED_SHIFT)
                                                  /* Bits 8-31: Reserved */
/* Clocking and power control -- Clock dividers */
/* CPU Clock Configuration register */

#define SYSCON_CCLKCFG_SHIFT          (0)         /* 0-7: Divide value for CPU clock (CCLK) */
#define SYSCON_CCLKCFG_MASK           (0xff << SYSCON_CCLKCFG_SHIFT)
#  define SYSCON_CCLKCFG_DIV(n)       ((n-1) << SYSCON_CCLKCFG_SHIFT) /* n=2,3,..255 */
                                                  /* Bits 8-31:  Reserved */
/* USB Clock Configuration register */

#define SYSCON_USBCLKCFG_SHIFT        (0)         /* Bits 0-3: PLL0 divide value USB clock */
#define SYSCON_USBCLKCFG_MASK         (15 << SYSCON_USBCLKCFG_SHIFT)
#  define SYSCON_USBCLKCFG_DIV6       (5 << SYSCON_USBCLKCFG_SHIFT) /* PLL0/6 for PLL0=288 MHz */
#  define SYSCON_USBCLKCFG_DIV8       (7 << SYSCON_USBCLKCFG_SHIFT) /* PLL0/8 for PLL0=384 MHz */
#  define SYSCON_USBCLKCFG_DIV10      (9 << SYSCON_USBCLKCFG_SHIFT) /* PLL0/10 for PLL0=480 MHz */
                                                  /* Bits 8-31:  Reserved */
/* Peripheral Clock Selection registers 0 and 1 */

#define SYSCON_PCLKSEL_CCLK4          (0)         /* PCLK_peripheral = CCLK/4 */
#define SYSCON_PCLKSEL_CCLK           (1)         /* PCLK_peripheral = CCLK */
#define SYSCON_PCLKSEL_CCLK2          (2)         /* PCLK_peripheral = CCLK/2 */
#define SYSCON_PCLKSEL_CCLK8          (3)         /* PCLK_peripheral = CCLK/8 (except CAN1, CAN2, and CAN) */
#define SYSCON_PCLKSEL_CCLK6          (3)         /* PCLK_peripheral = CCLK/6 (CAN1, CAN2, and CAN) */
#define SYSCON_PCLKSEL_MASK           (3)

#define SYSCON_PCLKSEL0_WDT_SHIFT     (0)         /* Bits 0-1: Peripheral clock WDT */
#define SYSCON_PCLKSEL0_WDT_MASK      (3 << SYSCON_PCLKSEL0_WDT_SHIFT)
#define SYSCON_PCLKSEL0_TMR0_SHIFT    (2)         /* Bits 2-3: Peripheral clock TIMER0 */
#define SYSCON_PCLKSEL0_TMR0_MASK     (3 << SYSCON_PCLKSEL0_TMR0_SHIFT)
#define SYSCON_PCLKSEL0_TMR1_SHIFT    (4)         /* Bits 4-5: Peripheral clock TIMER1 */
#define SYSCON_PCLKSEL0_TMR1_MASK     (3 << SYSCON_PCLKSEL0_TMR1_SHIFT)
#define SYSCON_PCLKSEL0_UART0_SHIFT   (6)         /* Bits 6-7: Peripheral clock UART0 */
#define SYSCON_PCLKSEL0_UART0_MASK    (3 << SYSCON_PCLKSEL0_UART0_SHIFT)
#define SYSCON_PCLKSEL0_UART1_SHIFT   (8)         /* Bits 8-9: Peripheral clock UART1 */
#define SYSCON_PCLKSEL0_UART1_MASK    (3 << SYSCON_PCLKSEL0_UART1_SHIFT)
                                                  /* Bits 10-11:  Reserved */
#define SYSCON_PCLKSEL0_PWM1_SHIFT    (12)        /* Bits 12-13: Peripheral clock PWM1 */
#define SYSCON_PCLKSEL0_PWM1_MASK     (3 << SYSCON_PCLKSEL0_PWM1_SHIFT)
#define SYSCON_PCLKSEL0_I2C0_SHIFT    (14)        /* Bits 14-15: Peripheral clock I2C0 */
#define SYSCON_PCLKSEL0_I2C0_MASK     (3 << SYSCON_PCLKSEL0_I2C0_SHIFT)
#define SYSCON_PCLKSEL0_SPI_SHIFT     (16)        /* Bits 16-17: Peripheral clock SPI */
#define SYSCON_PCLKSEL0_SPI_MASK      (3 << SYSCON_PCLKSEL0_SPI_SHIFT)
                                                  /* Bits 18-19:  Reserved */
#define SYSCON_PCLKSEL0_SSP1_SHIFT    (20)        /* Bits 20-21: Peripheral clock SSP1 */
#define SYSCON_PCLKSEL0_SSP1_MASK     (3 << SYSCON_PCLKSEL0_SSP1_SHIFT)
#define SYSCON_PCLKSEL0_DAC_SHIFT     (22)        /* Bits 22-23: Peripheral clock DAC */
#define SYSCON_PCLKSEL0_DAC_MASK      (3 << SYSCON_PCLKSEL0_DAC_SHIFT)
#define SYSCON_PCLKSEL0_ADC_SHIFT     (24)        /* Bits 24-25: Peripheral clock ADC */
#define SYSCON_PCLKSEL0_ADC_MASK      (3 << SYSCON_PCLKSEL0_ADC_SHIFT)
#define SYSCON_PCLKSEL0_CAN1_SHIFT    (26)        /* Bits 26-27: Peripheral clock CAN1 */
#define SYSCON_PCLKSEL0_CAN1_MASK     (3 << SYSCON_PCLKSEL0_CAN1_SHIFT)
#define SYSCON_PCLKSEL0_CAN2_SHIFT    (28)        /* Bits 28-29: Peripheral clock CAN2 */
#define SYSCON_PCLKSEL0_CAN2_MASK     (3 << SYSCON_PCLKSEL0_CAN2_SHIFT)
#define SYSCON_PCLKSEL0_ACF_SHIFT     (30)        /* Bits 30-31: Peripheral clock CAN AF */
#define SYSCON_PCLKSEL0_ACF_MASK      (3 << SYSCON_PCLKSEL0_ACF_SHIFT)

#define SYSCON_PCLKSEL1_QEI_SHIFT     (0)         /* Bits 0-1: Peripheral clock Quadrature Encoder */
#define SYSCON_PCLKSEL1_QEI_MASK      (3 << SYSCON_PCLKSEL1_QEI_SHIFT)
#define SYSCON_PCLKSEL1_GPIOINT_SHIFT (2)         /* Bits 2-3: Peripheral clock GPIO interrupts */
#define SYSCON_PCLKSEL1_GPIOINT_MASK  (3 << SYSCON_PCLKSEL1_GPIOINT_SHIFT)
#define SYSCON_PCLKSEL1_PCB_SHIFT     (4)         /* Bits 4-5: Peripheral clock the Pin Connect block */
#define SYSCON_PCLKSEL1_PCB_MASK      (3 << SYSCON_PCLKSEL1_PCB_SHIFT)
#define SYSCON_PCLKSEL1_I2C1_SHIFT    (6)         /* Bits 6-7: Peripheral clock I2C1 */
#define SYSCON_PCLKSEL1_I2C1_MASK     (3 << SYSCON_PCLKSEL1_I2C1_SHIFT)
                                                  /* Bits 8-9:  Reserved */
#define SYSCON_PCLKSEL1_SSP0_SHIFT    (10)        /* Bits 10-11: Peripheral clock SSP0 */
#define SYSCON_PCLKSEL1_SSP0_MASK     (3 << SYSCON_PCLKSEL1_SSP0_SHIFT)
#define SYSCON_PCLKSEL1_TMR2_SHIFT    (12)        /* Bits 12-13: Peripheral clock TIMER2 */
#define SYSCON_PCLKSEL1_TMR2_MASK     (3 << SYSCON_PCLKSEL1_TMR2_SHIFT)
#define SYSCON_PCLKSEL1_TMR3_SHIFT    (14)        /* Bits 14-15: Peripheral clock TIMER3 */
#define SYSCON_PCLKSEL1_TMR3_MASK     (3 << SYSCON_PCLKSEL1_TMR3_SHIFT)
#define SYSCON_PCLKSEL1_UART2_SHIFT   (16)        /* Bits 16-17: Peripheral clock UART2 */
#define SYSCON_PCLKSEL1_UART2_MASK    (3 << SYSCON_PCLKSEL1_UART2_SHIFT)
#define SYSCON_PCLKSEL1_UART3_SHIFT   (18)        /* Bits 18-19: Peripheral clock UART3 */
#define SYSCON_PCLKSEL1_UART3_MASK    (3 << SYSCON_PCLKSEL1_UART3_SHIFT)
#define SYSCON_PCLKSEL1_I2C2_SHIFT    (20)        /* Bits 20-21: Peripheral clock I2C2 */
#define SYSCON_PCLKSEL1_I2C2_MASK     (3 << SYSCON_PCLKSEL1_I2C2_SHIFT)
#define SYSCON_PCLKSEL1_I2S_SHIFT     (22)        /* Bits 22-23: Peripheral clock I2S */
#define SYSCON_PCLKSEL1_I2S_MASK      (3 << SYSCON_PCLKSEL1_I2S_SHIFT)
                                                  /* Bits 24-25:  Reserved */
#define SYSCON_PCLKSEL1_RIT_SHIFT     (26)        /* Bits 26-27: Peripheral clock Repetitive Interrupt Timer */
#define SYSCON_PCLKSEL1_RIT_MASK      (3 << SYSCON_PCLKSEL1_RIT_SHIFT)
#define SYSCON_PCLKSEL1_SYSCON_SHIFT  (28)        /* Bits 28-29: Peripheral clock the System Control block */
#define SYSCON_PCLKSEL1_SYSCON_MASK   (3 << SYSCON_PCLKSEL1_SYSCON_SHIFT)
#define SYSCON_PCLKSEL1_MC_SHIFT      (30)        /* Bits 30-31: Peripheral clock the Motor Control PWM */
#define SYSCON_PCLKSEL1_MC_MASK       (3 << SYSCON_PCLKSEL1_MC_SHIFT)

/* Clocking and power control - Peripheral power control registers */
/* Power Control Register */

#define SYSCON_PCON_PM0               (1 << 0)    /* Bit 0:  Power mode control bit 0 */
#define SYSCON_PCON_PM1               (1 << 1)    /* Bit 1:  Power mode control bit 1 */
#define SYSCON_PCON_BODRPM            (1 << 2)    /* Bit 2:  Brown-Out Reduced Power Mode */
#define SYSCON_PCON_BOGD              (1 << 3)    /* Bit 3:  Brown-Out Global Disable */
#define SYSCON_PCON_BORD              (1 << 4)    /* Bit 4:  Brown-Out Reset Disable */
                                                  /* Bits 5-7:  Reserved */
#define SYSCON_PCON_SMFLAG            (1 << 8)    /* Bit 8:  Sleep Mode entry flag */
#define SYSCON_PCON_DSFLAG            (1 << 9)    /* Bit 9:  Deep Sleep entry flag */
#define SYSCON_PCON_PDFLAG            (1 << 10)   /* Bit 10: Power-down entry flag */
#define SYSCON_PCON_DPDFLAG           (1 << 11)   /* Bit 11:  Deep Power-down entry flag */
                                                  /* Bits 12-31:  Reserved */
/* Power Control for Peripherals Register */

                                                  /* Bit 0:  Reserved */
#define SYSCON_PCONP_PCTIM0           (1 << 1)    /* Bit 1:  Timer/Counter 0 power/clock control */
#define SYSCON_PCONP_PCTIM1           (1 << 2)    /* Bit 2:  Timer/Counter 1 power/clock control */
#define SYSCON_PCONP_PCUART0          (1 << 3)    /* Bit 3:  UART0 power/clock control */
#define SYSCON_PCONP_PCUART1          (1 << 4)    /* Bit 4:  UART1 power/clock control */
                                                  /* Bit 5:  Reserved */
#define SYSCON_PCONP_PCPWM1           (1 << 6)    /* Bit 6:  PWM1 power/clock control */
#define SYSCON_PCONP_PCI2C0           (1 << 7)    /* Bit 7:  I2C0 power/clock control */
#define SYSCON_PCONP_PCSPI            (1 << 8)    /* Bit 8:  SPI power/clock control */
#define SYSCON_PCONP_PCRTC            (1 << 9)    /* Bit 9:  RTC power/clock control */
#define SYSCON_PCONP_PCSSP1           (1 << 10)   /* Bit 10: SSP 1 power/clock control */
                                                  /* Bit 11: Reserved */
#define SYSCON_PCONP_PCADC            (1 << 12)   /* Bit 12: A/D converter (ADC) power/clock control */
#define SYSCON_PCONP_PCCAN1           (1 << 13)   /* Bit 13: CAN Controller 1 power/clock control */
#define SYSCON_PCONP_PCCAN2           (1 << 14)   /* Bit 14: CAN Controller 2 power/clock control */
#define SYSCON_PCONP_PCGPIO           (1 << 15)   /* Bit 15: GPIOs power/clock enable */
#define SYSCON_PCONP_PCRIT            (1 << 16)   /* Bit 16: Repetitive Interrupt Timer power/clock control */
#define SYSCON_PCONP_PCMCPWM          (1 << 17)   /* Bit 17: Motor Control PWM */
#define SYSCON_PCONP_PCQEI            (1 << 18)   /* Bit 18: Quadrature Encoder power/clock control */
#define SYSCON_PCONP_PCI2C1           (1 << 19)   /* Bit 19: I2C1 power/clock control */
                                                  /* Bit 20: Reserved */
#define SYSCON_PCONP_PCSSP0           (1 << 21)   /* Bit 21: SSP0 power/clock control */
#define SYSCON_PCONP_PCTIM2           (1 << 22)   /* Bit 22: Timer 2 power/clock control */
#define SYSCON_PCONP_PCTIM3           (1 << 23)   /* Bit 23: Timer 3 power/clock control */
#define SYSCON_PCONP_PCUART2          (1 << 24)   /* Bit 24: UART 2 power/clock control */
#define SYSCON_PCONP_PCUART3          (1 << 25)   /* Bit 25: UART 3 power/clock control */
#define SYSCON_PCONP_PCI2C2           (1 << 26)   /* Bit 26: I2C 2 power/clock control */
#define SYSCON_PCONP_PCI2S            (1 << 27)   /* Bit 27: I2S power/clock control */
                                                  /* Bit 28: Reserved */
#define SYSCON_PCONP_PCGPDMA          (1 << 29)   /* Bit 29: GPDMA function power/clock control */
#define SYSCON_PCONP_PCENET           (1 << 30)   /* Bit 30: Ethernet block power/clock control */
#define SYSCON_PCONP_PCUSB            (1 << 31)   /* Bit 31: USB power/clock control */

/* More clocking and power control -- Utility */

#define SYSCON_CLKOUTCFG_SEL_SHIFT    (0)         /* Bits 0-3: Selects clock source for CLKOUT */
#define SYSCON_CLKOUTCFG_SEL_MASK     (15 << SYSCON_CLKOUTCFG_SEL_SHIFT)
#  define SYSCON_CLKOUTCFG_SEL_CPU    (0 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=CPU clock */
#  define SYSCON_CLKOUTCFG_SEL_MAIN   (1 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=main osc */
#  define SYSCON_CLKOUTCFG_SEL_INTRC  (2 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=internal RC osc */
#  define SYSCON_CLKOUTCFG_SEL_USB    (3 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=USB clock */
#  define SYSCON_CLKOUTCFG_SEL_RTC    (4 << SYSCON_CLKOUTCFG_SEL_SHIFT) /* CLKOUT source=RTC osc */
#define SYSCON_CLKOUTCFG_DIV_SHIFT    (4)         /* Bits 4-7:  CLKOUT divisor */
#define SYSCON_CLKOUTCFG_DIV_MASK     (15 << SYSCON_CLKOUTCFG_DIV_SHIFT)
#  define SYSCON_CLKOUTCFG_DIV(n)     ((n-1) << SYSCON_CLKOUTCFG_DIV_SHIFT) /* n=1..16 */
#define SYSCON_CLKOUTCFG_EN           (1 << 8)    /* Bit 8:  CLKOUT enable control */
#define SYSCON_CLKOUTCFG_ACT          (1 << 9)    /* Bit 9:  CLKOUT activity indication */
                                                  /* Bits 10-31: Reserved */
/* System control registers -- External Interrupts */
/* External Interrupt Flag register */

#define SYSCON_EXTINT_EINT0           (1 << 0)    /* Bit 0:  EINT0 */
#define SYSCON_EXTINT_EINT1           (1 << 1)    /* Bit 1:  EINT1 */
#define SYSCON_EXTINT_EINT2           (1 << 2)    /* Bit 2:  EINT2 */
#define SYSCON_EXTINT_EINT3           (1 << 3)    /* Bit 3:  EINT3 */
                                                  /* Bits 4-31: Reserved */
/* External Interrupt Mode register */

#define SYSCON_EXTMODE_EINT0          (1 << 0)    /* Bit 0:  1=EINT0 edge sensitive */
#define SYSCON_EXTMODE_EINT1          (1 << 1)    /* Bit 1:  1=EINT1 edge sensitive */
#define SYSCON_EXTMODE_EINT2          (1 << 2)    /* Bit 2:  1=EINT2 edge sensitive */
#define SYSCON_EXTMODE_EINT3          (1 << 3)    /* Bit 3:  1=EINT3 edge sensitive */
                                                  /* Bits 4-31: Reserved */
/* External Interrupt Polarity register */

#define SYSCON_EXTPOLAR_EINT0         (1 << 0)    /* Bit 0:  1=EINT0 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT1         (1 << 1)    /* Bit 1:  1=EINT1 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT2         (1 << 2)    /* Bit 2:  1=EINT2 high active/rising edge */
#define SYSCON_EXTPOLAR_EINT3         (1 << 3)    /* Bit 3:  1=EINT3 high active/rising edge */
                                                  /* Bits 4-31: Reserved */
/* System control registers -- Reset */
/* Reset Source Identification Register */

#define SYSCON_RSID_POR               (1 << 0)    /* Bit 0:  Power on reset */
#define SYSCON_RSID_EXTR              (1 << 1)    /* Bit 1:  external RESET signal */
#define SYSCON_RSID_WDTR              (1 << 2)    /* Bit 2:  Watchdog Timer time out w/WDTRESET */
#define SYSCON_RSID_BODR              (1 << 3)    /* Bit 3:  Brown out detection */
                                                  /* Bits 4-31: Reserved */
/* System control registers -- Syscon Miscellaneous Registers */

                                                  /* Bits 0-3: Reserved */
#define SYSCON_SCS_OSCRANGE           (1 << 4)    /* Bit 4:  Main oscillator range select */
#define SYSCON_SCS_OSCEN              (1 << 5)    /* Bit 5:  Main oscillator enable */
#define SYSCON_SCS_OSCSTAT            (1 << 6)    /* Bit 6:  Main oscillator status */
                                                  /* Bits 7-31: Reserved */
/* Device Interrupt Registers */
/* USB Interrupt Status register */

#define SYSCON_USBINTST_REQLP         (1 << 0)    /* Bit 0:  Low priority interrupt line status */
#define SYSCON_USBINTST_REQHP         (1 << 1)    /* Bit 1:  High priority interrupt line status */
#define SYSCON_USBINTST_REQDMA        (1 << 2)    /* Bit 2:  DMA interrupt line status */
#define SYSCON_USBINTST_HOSTINT       (1 << 3)    /* Bit 3:  USB host interrupt line status */
#define SYSCON_USBINTST_ATXINT        (1 << 4)    /* Bit 4:  External ATX interrupt line status */
#define SYSCON_USBINTST_OTGINT        (1 << 5)    /* Bit 5:  OTG interrupt line status */
#define SYSCON_USBINTST_I2CINT        (1 << 6)    /* Bit 6:  I2C module interrupt line status */
                                                  /* Bit 7:  Reserved */
#define SYSCON_USBINTST_NEEDCLK       (1 << 8)    /* Bit 8:  USB need clock indicator */
                                                  /* Bits 9-30: Reserved */
#define SYSCON_USBINTST_ENINTS        (1 << 31)   /* Bit 31: Enable all USB interrupts */

/* DMA Request Select Register */

#define SYSCON_DMAREQSEL_INP8         (1 << 0)    /* Bit 0:  Input 8 0=UART0 TX 1=Timer 0 match 0 */
#define SYSCON_DMAREQSEL_INP9         (1 << 1)    /* Bit 1:  Input 8 0=UART0 RX 1=Timer 0 match 1 */
#define SYSCON_DMAREQSEL_INP10        (1 << 2)    /* Bit 2:  Input 8 0=UART1 TX 1=Timer 1 match 0 */
#define SYSCON_DMAREQSEL_INP11        (1 << 3)    /* Bit 3:  Input 8 0=UART1 RX 1=Timer 1 match 1 */
#define SYSCON_DMAREQSEL_INP12        (1 << 4)    /* Bit 4:  Input 8 0=UART2 TX 1=Timer 2 match 0 */
#define SYSCON_DMAREQSEL_INP13        (1 << 5)    /* Bit 5:  Input 8 0=UART2 RX 1=Timer 2 match 1 */
#define SYSCON_DMAREQSEL_INP14        (1 << 6)    /* Bit 6:  Input 8 0=UART3 TX 1=Timer 3 match 0 */
#define SYSCON_DMAREQSEL_INP15        (1 << 7)    /* Bit 7:  Input 8 0=UART3 RX 1=Timer 3 match 1 */
                                                  /* Bits 8-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC176X_SYSCON_H */
