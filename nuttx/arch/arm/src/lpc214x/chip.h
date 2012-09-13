/****************************************************************************************************
 * arch/arm/src/lpc214x/chip.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __LPC214X_CHIP_H
#define __LPC214X_CHIP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Memory Map ***************************************************************************************/

#define LPC214X_FLASH_BASE              0x00000000
#define LPC214X_FIO_BASE                0x3fffc000
#define LPC214X_ONCHIP_RAM_BASE         0x40000000
#define LPC214X_USBDMA_RAM_BASE         0x7fd00000
#define LPC214X_BOOT_BLOCK              0x7fffd000
#define LPC214X_EXTMEM_BASE             0x80000000
#define LPC214X_APB_BASE                0xe0000000
#define LPC214X_AHB_BASE                0xf0000000

/* Peripheral Registers ****************************************************************************/

/* FIO Register block base addresses */

#define LPC214X_FIO0_BASE               0x3fffc000  /* Fast I/O 0 base address */
#define LPC214X_FIO1_BASE               0x3fffc020  /* Fast I/O 1 base address */

/* APB Register block base addresses */

#define LPC214X_WD_BASE                 0xe0000000  /* Watchdog base address */
#define LPC214X_TMR0_BASE               0xe0004000  /* Timer 0 base address*/
#define LPC214X_TMR1_BASE               0xe0008000  /* Timer 1 base address */
#define LPC214X_UART0_BASE              0xe000c000  /* UART0 base address */
#define LPC214X_UART1_BASE              0xe0010000  /* UART1 base address */
#define LPC214X_PWM_BASE                0xe0014000  /* Pulse width modulator (PWM) base address */
#define LPC214X_I2C0_BASE               0xe001c000  /* I2C0 base address */
#define LPC214X_SPI0_BASE               0xe0020000  /* Serial Peripheral Interface 0 (SPI0) base */
#define LPC214X_RTC_BASE                0xe0024000  /* Real Time Clock (RTC) base address */
#define LPC214X_GPIO0_BASE              0xe0028000  /* General Purpose I/O (GPIO) 0 base address */
#define LPC214X_GPIO1_BASE              0xe0028010  /* General Purpose I/O (GPIO) 0 base address */
#define LPC214X_PINSEL_BASE             0xe002c000  /* Pin function select registers */
#define LPC214X_AD0_BASE                0xe0034000  /* Analog to Digital Converter 0 base address*/
#define LPC214X_I2C1_BASE               0xe005c000  /* I2C1 base address */
#define LPC214X_AD1_BASE                0xe0060000  /* Analog to Digital Converter 1 base address */
#define LPC214X_SPI1_BASE               0xe0068000  /* Serial Peripheral Interface 1 (SPI1) base */
#define LPC214X_DAC_BASE                0xe0090000  /* DAC base address */
#define LPC214X_USB_BASE                0xe0090000  /* USB base address */

#define LPC214X_SCB_BASE                0xe01fc000  /* System Control Block (SBC) base address */
#define   LPC214X_MAM_BASE              0xe01fc000  /* Memory Accelerator Module (MAM) base address */
#define   LPC214X_SCS                   0xe01fc1a0  /* System Control and Status flags (SCS) */
#define   LPC214X_MEMMAP                0xe01fc040  /* Memory Mapping Control */
#define   LPC214X_PLL_BASE              0xe01fc080  /* Phase Locked Loop (PLL) base address */
#define   LPC214X_PCON_BASE             0xe01fc0c0  /* Power Control (PCON) base address */
#define   LPC214X_APBDIV                0xe01fc100  /* APBDIV Address */
#define   LPC214X_EXT_BASE              0xe01fc140  /* External Interrupt base address */       

/* AHB Register block base addresses */

#define LPC214X_EMC_BASE                0xffe00000  /* External Memory Controller (EMC) base address */
#define LPC214X_VIC_BASE                0xfffff000  /* Vectored Interrupt Controller (VIC) Base */

/* Watchdog Register Offsets */

#define LPC214X_WD_MOD_OFFSET           0x00 /* Watchdog Mode Register */
#define LPC214X_WD_TC_OFFSET            0x04 /* Watchdog Time Constant Register */
#define LPC214X_WD_FEED_OFFSET          0x08 /* Watchdog Feed Register */
#define LPC214X_WD_TV_OFFSET            0x0C /* Watchdog Time Value Register */

/* Timer 0/1 register offsets */

#define LPC214X_TMR_IR_OFFSET           0x00   /* RW:Interrupt Register */
#define LPC214X_TMR_TCR_OFFSET          0x04   /* RW: Timer Control Register */
#define LPC214X_TMR_TC_OFFSET           0x08   /* RW: Timer Counter */
#define LPC214X_TMR_PR_OFFSET           0x0c   /* RW: Prescale Register */
#define LPC214X_TMR_PC_OFFSET           0x10   /* RW: Prescale Counter Register */
#define LPC214X_TMR_MCR_OFFSET          0x14   /* RW: Match Control Register */
#define LPC214X_TMR_MR0_OFFSET          0x18   /* RW: Match Register 0 */
#define LPC214X_TMR_MR1_OFFSET          0x1c   /* RW: Match Register 1 */
#define LPC214X_TMR_MR2_OFFSET          0x20   /* RW: Match Register 2 */
#define LPC214X_TMR_MR3_OFFSET          0x24   /* RW: Match Register 3 */
#define LPC214X_TMR_CCR_OFFSET          0x28   /* RW: Capture Control Register */
#define LPC214X_TMR_CR0_OFFSET          0x2c   /* R: Capture Register 0 */
#define LPC214X_TMR_CR1_OFFSET          0x30   /* R: Capture Register 1 */
#define LPC214X_TMR_CR2_OFFSET          0x34   /* R: Capture Register 2 */
#define LPC214X_TMR_CR3_OFFSET          0x38   /* RW: Capture Register 3 */
#define LPC214X_TMR_EMR_OFFSET          0x3c   /* RW: External Match Register */

#define LPC214X_TMR_CTCR_OFFSET         0x70   /* RW: Count Control Register */

/* UART0/1 Register Offsets */

#define LPC214X_UART_RBR_OFFSET         0x00   /* R: Receive Buffer Register (DLAB=0) */
#define LPC214X_UART_THR_OFFSET         0x00   /* W: Transmit Holding Register (DLAB=0) */
#define LPC214X_UART_DLL_OFFSET         0x00   /* W: Divisor Latch Register (LSB, DLAB=1) */
#define LPC214X_UART_IER_OFFSET         0x04   /* W: Interrupt Enable Register (DLAB=0) */
#define LPC214X_UART_DLM_OFFSET         0x04   /* RW: Divisor Latch Register (MSB, DLAB=1) */
#define LPC214X_UART_IIR_OFFSET         0x08   /* R: Interrupt ID Register */
#define LPC214X_UART_FCR_OFFSET         0x08   /* W: FIFO Control Register */
#define LPC214X_UART_LCR_OFFSET         0x0c   /* RW: Line Control Register */
#define LPC214X_UART_MCR_OFFSET         0x10   /* RW: Modem Control REgister (2146/6/8 UART1 Only) */
#define LPC214X_UART_LSR_OFFSET         0x14   /* R: Scratch Pad Register */
#define LPC214X_UART_MSR_OFFSET         0x18   /* RW: MODEM Status Register (2146/6/8 UART1 Only) */
#define LPC214X_UART_SCR_OFFSET         0x1c   /* RW: Line Status Register */
#define LPC214X_UART_ACR_OFFSET         0x20   /* RW: Autobaud Control Register */
#define LPC214X_UART_FDR_OFFSET         0x28   /* RW: Fractional Divider Register */
#define LPC214X_UART_TER_OFFSET         0x30   /* RW: Transmit Enable Register */

/* PWM register offsets */

#define LPC214X_PWM_IR_OFFSET           0x00   /* Interrupt Register */
#define LPC214X_PWM_TCR_OFFSET          0x04   /* Timer Control Register */
#define LPC214X_PWM_TC_OFFSET           0x08   /* Timer Counter */
#define LPC214X_PWM_PR_OFFSET           0x0c   /* Prescale Register */
#define LPC214X_PWM_PC_OFFSET           0x10   /* Prescale Counter Register */
#define LPC214X_PWM_MCR_OFFSET          0x14   /* Match Control Register */
#define LPC214X_PWM_MR0_OFFSET          0x18   /* Match Register 0 */
#define LPC214X_PWM_MR1_OFFSET          0x1c   /* Match Register 1 */
#define LPC214X_PWM_MR2_OFFSET          0x20   /* Match Register 2 */
#define LPC214X_PWM_MR3_OFFSET          0x24   /* Match Register 3 */
#define LPC214X_PWM_MR4_OFFSET          0x40   /* Match Register 4 */
#define LPC214X_PWM_MR5_OFFSET          0x44   /* Match Register 5 */
#define LPC214X_PWM_MR6_OFFSET          0x48   /* Match Register 6 */
#define LPC214X_PWM_PCR_OFFSET          0x4c   /* Control Register */
#define LPC214X_PWM_LER_OFFSET          0x50   /* Latch Enable Register */

/* I2C register offsets */

#define LPC214X_I2C_CONSET_OFFSET       0x00   /* Control Set Register */
#define LPC214X_I2C_STAT_OFFSET         0x04   /* Status Register */
#define LPC214X_I2C_DAT_OFFSET          0x08   /* Data Register */
#define LPC214X_I2C_ADR_OFFSET          0x0c   /* Slave Address Register */
#define LPC214X_I2C_SCLH_OFFSET         0x10   /* SCL Duty Cycle Register (high half word) */
#define LPC214X_I2C_SCLL_OFFSET         0x14   /* SCL Duty Cycle Register (low half word) */
#define LPC214X_I2C_CONCLR_OFFSET       0x18   /* Control Clear Register */

/* Pin function select register offsets */

#define LPC214X_PINSEL0_OFFSET          0x00   /* Pin function select register 0 */
#define LPC214X_PINSEL1_OFFSET          0x04   /* Pin function select register 1 */
#define LPC214X_PINSEL2_OFFSET          0x14   /* Pin function select register 2 */

/* Analog to Digital (AD) Converter registger offsets */

#define LPC214X_AD_ADCR_OFFSET          0x00   /* A/D Control Register */
#define LPC214X_AD_ADGDR_OFFSET         0x04   /* A/D Global Data Register (only one common register!) */
#define LPC214X_AD_ADGSR_OFFSET         0x08   /* A/D Global Start Register */
#define LPC214X_AD_ADINTEN_OFFSET       0x0c   /* A/D Interrupt Enable Register */
#define LPC214X_AD_ADDR0_OFFSET         0x10   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR1_OFFSET         0x14   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR2_OFFSET         0x18   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR3_OFFSET         0x1c   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR4_OFFSET         0x20   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR5_OFFSET         0x24   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR6_OFFSET         0x28   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADDR7_OFFSET         0x2c   /* A/D Chanel 0 Data Register */
#define LPC214X_AD_ADSTAT_OFFSET        0x30   /* A/D Status Register */

/* SPI0 register offsets */

#define LPC214X_SPI0_CR_OFFSET          0x00   /* Control Register 0 */
#define LPC214X_SPI0_SR_OFFSET          0x04   /* Control Register 1 */
#define LPC214X_SPI0_DR_OFFSET          0x08   /* Data Register */
#define LPC214X_SPI0_CCR_OFFSET         0x0c   /* Status Register */
#define LPC214X_SPI0_INT_OFFSET         0x1c   /* Clock Pre-Scale Regisrer */

/* SPI1 register offsets */

#define LPC214X_SPI1_CR0_OFFSET         0x00   /* Control Register 0 */
#define LPC214X_SPI1_CR1_OFFSET         0x04   /* Control Register 1 */
#define LPC214X_SPI1_DR_OFFSET          0x08   /* Data Register */
#define LPC214X_SPI1_SR_OFFSET          0x0c   /* Status Register */
#define LPC214X_SPI1_CPSR_OFFSET        0x10   /* Clock Pre-Scale Regisrer */
#define LPC214X_SPI1_IMSC_OFFSET        0x14   /* Interrupt Mask Set and Clear Register */
#define LPC214X_SPI1_RIS_OFFSET         0x18   /* Raw Interrupt Status Register */
#define LPC214X_SPI1_MIS_OFFSET         0x1c   /* Masked Interrupt Status Register */
#define LPC214X_SPI1_ICR_OFFSET         0x20   /* Interrupt Clear Register */

/* RTC register offsets */

#define LPC214X_RTC_ILR_OFFSET          0x00   /* Interrupt Location Register */
#define LPC214X_RTC_CTC_OFFSET          0x04   /* Clock Tick Counter */
#define LPC214X_RTC_CCR_OFFSET          0x08   /* Clock Control Register */
#define LPC214X_RTC_CIIR_OFFSET         0x0c   /* Counter Increment Interrupt Register */
#define LPC214X_RTC_AMR_OFFSET          0x10   /* Alarm Mask Register */
#define LPC214X_RTC_CTIME0_OFFSET       0x14   /* Consolidated Time Register 0 */
#define LPC214X_RTC_CTIME1_OFFSET       0x18   /* Consolidated Time Register 1 */
#define LPC214X_RTC_CTIME2_OFFSET       0x1c   /* Consolidated Time Register 2 */
#define LPC214X_RTC_SEC_OFFSET          0x20   /* Seconds Register */
#define LPC214X_RTC_MIN_OFFSET          0x24   /* Minutes Register */
#define LPC214X_RTC_HOUR_OFFSET         0x28   /* Hours Register */
#define LPC214X_RTC_DOM_OFFSET          0x2c   /* Day Of Month Register */
#define LPC214X_RTC_DOW_OFFSET          0x30   /* Day Of Week Register */
#define LPC214X_RTC_DOY_OFFSET          0x34   /* Day Of Year Register */
#define LPC214X_RTC_MONTH_OFFSET        0x38   /* Months Register */
#define LPC214X_RTC_YEAR_OFFSET         0x3c   /* Years Register */

#define LPC214X_RTC_ALSEC_OFFSET        0x60   /* Alarm Seconds Register */
#define LPC214X_RTC_ALMIN_OFFSET        0x64   /* Alarm Minutes Register */
#define LPC214X_RTC_ALHOUR_OFFSET       0x68   /* Alarm Hours Register */
#define LPC214X_RTC_ALDOM_OFFSET        0x6c   /* Alarm Day Of Month Register */
#define LPC214X_RTC_ALDOW_OFFSET        0x70   /* Alarm Day Of Week Register */
#define LPC214X_RTC_ALDOY_OFFSET        0x74   /* Alarm Day Of Year Register */
#define LPC214X_RTC_ALMON_OFFSET        0x78   /* Alarm Months Register */
#define LPC214X_RTC_ALYEAR_OFFSET       0x7c   /* Alarm Years Register */
#define LPC214X_RTC_PREINT_OFFSET       0x80   /* Prescale Value Register (integer) */
#define LPC214X_RTC_PREFRAC_OFFSET      0x84   /* Prescale Value Register (fraction) */

/* GPIO register offsets */

#define LPC214X_GPIO_PIN_OFFSET         0x00   /* Pin Value Register */
#define LPC214X_GPIO_SET_OFFSET         0x04   /* Pin Output Set Register */
#define LPC214X_GPIO_DIR_OFFSET         0x08   /* Pin Direction Register */
#define LPC214X_GPIO_CLR_OFFSET         0x0c   /* Pin Output Clear Register */

/* FIO register offsets */

#define LPC214X_FIO_DIR_OFFSET          0x00   /* Fast GPIO Port Direction Register */
#define LPC214X_FIO_MASK_OFFSET         0x10   /* Fast GPIO Mask Register */
#define LPC214X_FIO_PIN_OFFSET          0x14   /* Fast GPIO Pin Value Register */
#define LPC214X_FIO_SET_OFFSET          0x18   /* Fast GPIO Port Output Set Register */
#define LPC214X_FIO_CLR_OFFSET          0x1c   /* Fast GPIO Port Output Clear Register */

/* Memory Accelerator Module (MAM) Regiser Offsets */

#define LPC214X_MAM_CR_OFFSET           0x00   /* MAM Control Offset*/
#define LPC214x_MAM_TIM_OFFSET          0x04   /* MAM Timing Offset */

/* Phase Locked Loop (PLL) Register Offsets */

#define LPC214X_PLL_CON_OFFSET          0x00   /* PLL Control Offset*/
#define LPC214X_PLL_CFG_OFFSET          0x04   /* PLL Configuration Offset */
#define LPC214X_PLL_STAT_OFFSET         0x08   /* PLL Status Offset */
#define LPC214X_PLL_FEED_OFFSET         0x0c   /* PLL Feed Offset */

/* Power Control register offsets */

#define LPC214X_PCON_OFFSET             0x00   /* Control Register */
#define LPC214X_PCONP_OFFSET            0x04   /* Peripherals Register */

/* External Interrupt register offsets */

#define LPC214X_EXT_INT_OFFSET          0x00   /* Flag Register */
#define LPC214X_EXT_WAKE_OFFSET         0x04   /* Wakeup Register */
#define LPC214X_EXT_MODE_OFFSET         0x08   /* Mode Register */
#define LPC214X_EXT_POLAR_OFFSET        0x0c   /* Polarity Register */

/* External Memory Controller (EMC) definitions */

#define LPC214X_BCFG0_OFFSET            0x00   /* BCFG0 Offset */
#define LPC214X_BCFG1_OFFSET            0x04   /* BCFG1 Offset */
#define LPC214X_BCFG2_OFFSET            0x08   /* BCFG2 Offset */
#define LPC214X_BCFG3_OFFSET            0x0c   /* BCFG3 Offset */

/* Vectored Interrupt Controller (VIC) register offsets */

#define LPC214X_VIC_IRQSTATUS_OFFSET    0x00   /* R: IRQ Status Register */
#define LPC214X_VIC_FIQSTATUS_OFFSET    0x04   /* R: FIQ Status Register */
#define LPC214X_VIC_RAWINTR_OFFSET      0x08   /* R: Raw Interrupt Status Register */
#define LPC214X_VIC_INTSELECT_OFFSET    0x0c   /* RW: Interrupt Select Register */
#define LPC214X_VIC_INTENABLE_OFFSET    0x10   /* RW: Interrupt Enable Register */
#define LPC214X_VIC_INTENCLEAR_OFFSET   0x14   /* W: Interrupt Enable Clear Register */
#define LPC214X_VIC_SOFTINT_OFFSET      0x18   /* RW: Software Interrupt Register */
#define LPC214X_VIC_SOFTINTCLEAR_OFFSET 0x1c   /* W: Software Interrupt Clear Register */
#define LPC214X_VIC_PROTECTION_OFFSET   0x20   /* Protection Enable Register */

#define LPC214X_VIC_VECTADDR_OFFSET     0x30   /* RW: Vector Address Register */
#define LPC214X_VIC_DEFVECTADDR_OFFSET  0x34   /* RW: Default Vector Address Register */

#define LPC214X_VIC_VECTADDR0_OFFSET    0x100  /* RW: Vector Address 0 Register */
#define LPC214X_VIC_VECTADDR1_OFFSET    0x104  /* RW: Vector Address 1 Register */
#define LPC214X_VIC_VECTADDR2_OFFSET    0x108  /* RW: Vector Address 2 Register */
#define LPC214X_VIC_VECTADDR3_OFFSET    0x10c  /* RW: Vector Address 3 Register */
#define LPC214X_VIC_VECTADDR4_OFFSET    0x110  /* RW: Vector Address 4 Register */
#define LPC214X_VIC_VECTADDR5_OFFSET    0x114  /* RW: Vector Address 5 Register */
#define LPC214X_VIC_VECTADDR6_OFFSET    0x118  /* RW: Vector Address 6 Register */
#define LPC214X_VIC_VECTADDR7_OFFSET    0x11c  /* RW: Vector Address 7 Register */
#define LPC214X_VIC_VECTADDR8_OFFSET    0x120  /* RW: Vector Address 8 Register */
#define LPC214X_VIC_VECTADDR9_OFFSET    0x124  /* RW: Vector Address 9 Register */
#define LPC214X_VIC_VECTADDR10_OFFSET   0x128  /* RW: Vector Address 10 Register */
#define LPC214X_VIC_VECTADDR11_OFFSET   0x12c  /* RW: Vector Address 11 Register */
#define LPC214X_VIC_VECTADDR12_OFFSET   0x130  /* RW: Vector Address 12 Register */
#define LPC214X_VIC_VECTADDR13_OFFSET   0x134  /* RW: Vector Address 13 Register */
#define LPC214X_VIC_VECTADDR14_OFFSET   0x138  /* RW: Vector Address 14 Register */
#define LPC214X_VIC_VECTADDR15_OFFSET   0x13c  /* RW: Vector Address 15 Register */

#define LPC214X_VIC_VECTCNTL0_OFFSET    0x200  /* RW: Vector Control 0 Register */
#define LPC214X_VIC_VECTCNTL1_OFFSET    0x204  /* RW: Vector Control 1 Register */
#define LPC214X_VIC_VECTCNTL2_OFFSET    0x208  /* RW: Vector Control 2 Register */
#define LPC214X_VIC_VECTCNTL3_OFFSET    0x20c  /* RW: Vector Control 3 Register */
#define LPC214X_VIC_VECTCNTL4_OFFSET    0x210  /* RW: Vector Control 4 Register */
#define LPC214X_VIC_VECTCNTL5_OFFSET    0x214  /* RW: Vector Control 5 Register */
#define LPC214X_VIC_VECTCNTL6_OFFSET    0x218  /* RW: Vector Control 6 Register */
#define LPC214X_VIC_VECTCNTL7_OFFSET    0x21c  /* RW: Vector Control 7 Register */
#define LPC214X_VIC_VECTCNTL8_OFFSET    0x220  /* RW: Vector Control 8 Register */
#define LPC214X_VIC_VECTCNTL9_OFFSET    0x224  /* RW: Vector Control 9 Register */
#define LPC214X_VIC_VECTCNTL10_OFFSET   0x228  /* RW: Vector Control 10 Register */
#define LPC214X_VIC_VECTCNTL11_OFFSET   0x22c  /* RW: Vector Control 11 Register */
#define LPC214X_VIC_VECTCNTL12_OFFSET   0x230  /* RW: Vector Control 12 Register */
#define LPC214X_VIC_VECTCNTL13_OFFSET   0x234  /* RW: Vector Control 13 Register */
#define LPC214X_VIC_VECTCNTL14_OFFSET   0x238  /* RW: Vector Control 14 Register */
#define LPC214X_VIC_VECTCNTL15_OFFSET   0x23c  /* RW: Vector Control 15 Register */

/****************************************************************************************************
 * Inline Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Global Function Prototypes
 ****************************************************************************************************/

#endif  /* __LPC214X_CHIP_H */
