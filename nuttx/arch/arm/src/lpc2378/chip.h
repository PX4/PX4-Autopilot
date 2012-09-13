/****************************************************************************************************
 * arch/arm/src/lpc2378/chip.h
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS and based on the lpc2148 port:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#ifndef _ARCH_ARM_SRC_LPC2378_CHIP_H
#define _ARCH_ARM_SRC_LPC2378_CHIP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/
 
#include <sys/types.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Memory Map ***************************************************************************************/

#define LPC23XX_FLASH_BASE			0x00000000
#define LPC23XX_FIO_BASE			0x3fffc000
#define LPC23XX_ONCHIP_RAM_BASE		0x40000000
#define LPC23XX_USBDMA_RAM_BASE		0x7fd00000
#define LPC23XX_ETHERNET_RAM_BASE	0x7fe00000 
#define LPC23XX_BOOT_BLOCK			0x7fffd000
#define LPC23XX_EXTMEM_BASE			0x80000000
#define LPC23XX_APB_BASE			0xe0000000
#define LPC23XX_AHB_BASE			0xf0000000

/* Peripheral Registers ****************************************************************************/

/* APB Register block base addresses */

#define LPC23XX_WD_BASE				0xe0000000  /* Watchdog base address */
#define LPC23XX_TMR0_BASE			0xE0004000  /* Timer 0 base address*/
#define LPC23XX_TMR1_BASE			0xe0008000  /* Timer 1 base address */
#define LPC23XX_UART0_BASE			0xe000c000  /* UART0 base address */
#define LPC23XX_UART1_BASE			0xe0010000  /* UART1 base address */
#define LPC23XX_PWM_BASE			0xe0018000  /* Pulse width modulator (PWM) base address */
#define LPC23XX_I2C0_BASE			0xe001c000  /* I2C0 base address */
#define LPC23XX_SPI0_BASE			0xe0020000  /* Serial Peripheral Interface 0 (SPI0) base */
#define LPC23XX_RTC_BASE			0xe0024000  /* Real Time Clock (RTC) base address */
#define LPC23XX_GPIO0_BASE			0xe0028000  /* General Purpose I/O (GPIO) 0 base address */
#define LPC23XX_GPIO1_BASE			0xe0028010  /* General Purpose I/O (GPIO) 1 base address */
#define LPC23XX_PINSEL_BASE			0xe002c000  /* Pin function select registers */
#define LPC23XX_SSP1_BASE			0xe0030000  /* Synchronous Serial Port 1 base address */
#define LPC23XX_AD0_BASE			0xe0034000  /* Analog to Digital Converter 0 base address*/
//~ #define LPC23XX_CAN_ACCEPT_RAM_BASE	 0xe0038000  /* CAN Acceptance Filter RAM base address*/
#define LPC23XX_CAN_ACCEPT_BASE		0xE003C000  /* CAN Acceptance Filter Register base address*/
#define LPC23XX_CAN_COMMON_BASE		0xe0040000  /* CAN Common Register base address*/
#define LPC23XX_CAN1_BASE			0xe0044000  /* CAN 1 Controller base address*/
#define LPC23XX_CAN2_BASE			0xe0048000  /* CAN 2 Controller base address*/
#define LPC23XX_I2C1_BASE			0xe005c000  /* I2C1 base address */
#define LPC23XX_SSP0_BASE			0xE0068000  /* Synchronous Serial Port 0 base address */
#define LPC23XX_DAC_BASE			0xE006C000  /* DAC base address */
#define LPC23XX_TMR2_BASE			0xE0070000  /* Timer 2 base address */
#define LPC23XX_TMR3_BASE			0xE0074000  /* Timer 3 base address */
#define LPC23XX_UART2_BASE			0xE0078000  /* UART2 base address */
#define LPC23XX_UART3_BASE			0xE007C000  /* UART3 base address */
#define LPC23XX_I2C2_BASE			0xE0080000  /* I2C2 base address */
#define LPC23XX_BATT_RAM_BASE		0xE0084000  /* Battery RAM base address */
#define LPC23XX_I2S_BASE			0xE0088000  /* I2S base address */
#define LPC23XX_MCI_BASE			0xE008C000  /* SD/MMC Card Interface base address */
//~ #define LPC23XX_SPI1_BASE		0xe0068000  /* Serial Peripheral Interface 1 (SPI1) base */
#define LPC23XX_EMAC_BASE			0xFFE00000  /* Ethernet MAC base address */
#define LPC23XX_USB_BASE			0xFFE0C200  /* USB base address */
#define LPC23XX_SCB_BASE			0xE01FC000  /* System Control Block (SBC) base address */
#define LPC23XX_EXT_BASE			0xe01fc140  /* External Interrupt base address */	   

/* AHB Register block base addresses */

#define LPC23XX_EMC_BASE			0xFFE08000  /* External Memory Controller (EMC) base address */
#define LPC23XX_VIC_BASE			0xFFFFF000  /* Vectored Interrupt Controller (VIC) Base */
#define LPC23XX_GPDMA_BASE			0xFFE04000  /* General Purpose DMA */


/* Watchdog Register Offsets */
#define WD_MOD_OFFSET		0x00 /* Watchdog Mode Register */
#define WD_TC_OFFSET		0x04 /* Watchdog Time Constant Register */
#define WD_FEED_OFFSET		0x08 /* Watchdog Feed Register */
#define WD_TV_OFFSET		0x0C /* Watchdog Time Value Register */

/* Timers Base Addresses */
#define TMR0_BASE_ADDR		0xE0004000
#define TMR1_BASE_ADDR		0xE0008000
#define TMR2_BASE_ADDR		0xE0070000
#define TMR3_BASE_ADDR		0xE0074000
/* Timer 0/1/2/3 register offsets */
#define TMR_IR_OFFSET		0x00   /* RW:Interrupt Register */
#define TMR_TCR_OFFSET		0x04   /* RW: Timer Control Register */
#define TMR_TC_OFFSET		0x08   /* RW: Timer Counter */
#define TMR_PR_OFFSET		0x0c   /* RW: Prescale Register */
#define TMR_PC_OFFSET		0x10   /* RW: Prescale Counter Register */
#define TMR_MCR_OFFSET		0x14   /* RW: Match Control Register */
#define TMR_MR0_OFFSET		0x18   /* RW: Match Register 0 */
#define TMR_MR1_OFFSET		0x1c   /* RW: Match Register 1 */
#define TMR_MR2_OFFSET		0x20   /* RW: Match Register 2 */
#define TMR_MR3_OFFSET		0x24   /* RW: Match Register 3 */
#define TMR_CCR_OFFSET		0x28   /* RW: Capture Control Register */
#define TMR_CR0_OFFSET		0x2c   /* R: Capture Register 0 */
#define TMR_CR1_OFFSET		0x30   /* R: Capture Register 1 */
#define TMR_CR2_OFFSET		0x34   /* R: Capture Register 2 */
#define TMR_CR3_OFFSET		0x38   /* RW: Capture Register 3 */
#define TMR_EMR_OFFSET		0x3c   /* RW: External Match Register */
#define TMR_CTCR_OFFSET		0x70   /* RW: Count Control Register */

/* Universal Asynchronous Receiver Transmitter Base Addresses */
#define UART0_BASE_ADDR		0xE000C000
#define UART1_BASE_ADDR		0xE0010000
#define UART2_BASE_ADDR		0xE0078000
#define UART3_BASE_ADDR		0xE007C000
/* UART 0/1/2/3 Register Offsets */
#define UART_RBR_OFFSET		 0x00   /* R: Receive Buffer Register (DLAB=0) */
#define UART_THR_OFFSET		 0x00   /* W: Transmit Holding Register (DLAB=0) */
#define UART_DLL_OFFSET		 0x00   /* W: Divisor Latch Register (LSB, DLAB=1) */
#define UART_IER_OFFSET		 0x04   /* W: Interrupt Enable Register (DLAB=0) */
#define UART_DLM_OFFSET		 0x04   /* RW: Divisor Latch Register (MSB, DLAB=1) */
#define UART_IIR_OFFSET		 0x08   /* R: Interrupt ID Register */
#define UART_FCR_OFFSET		 0x08   /* W: FIFO Control Register */
#define UART_LCR_OFFSET		 0x0c   /* RW: Line Control Register */
#define UART_MCR_OFFSET		 0x10   /* RW: Modem Control REgister (2146/6/8 UART1 Only) */
#define UART_LSR_OFFSET		 0x14   /* R: Scratch Pad Register */
#define UART_MSR_OFFSET		 0x18   /* RW: MODEM Status Register (2146/6/8 UART1 Only) */
#define UART_SCR_OFFSET		 0x1c   /* RW: Line Status Register */
#define UART_ACR_OFFSET		 0x20   /* RW: Autobaud Control Register */
#define UART_FDR_OFFSET		 0x28   /* RW: Fractional Divider Register */
#define UART_TER_OFFSET		 0x30   /* RW: Transmit Enable Register */


/* Pulse Width Modulation Base Address */
#define PWM1_BASE_ADDR		0xE0018000
/* PWM register offsets */
#define PWM_IR_OFFSET		0x00   /* Interrupt Register */
#define PWM_TCR_OFFSET		0x04   /* Timer Control Register */
#define PWM_TC_OFFSET		0x08   /* Timer Counter */
#define PWM_PR_OFFSET		0x0c   /* Prescale Register */
#define PWM_PC_OFFSET		0x10   /* Prescale Counter Register */
#define PWM_MCR_OFFSET		0x14   /* Match Control Register */
#define PWM_MR0_OFFSET		0x18   /* Match Register 0 */
#define PWM_MR1_OFFSET		0x1c   /* Match Register 1 */
#define PWM_MR2_OFFSET		0x20   /* Match Register 2 */
#define PWM_MR3_OFFSET		0x24   /* Match Register 3 */
#define PWM_CCR_OFFSET		0x28   /* Capture Control Register*/
#define PWM_CCR0_OFFSET		0x2C   /* Capture Register 0 */
#define PWM_CCR1_OFFSET		0x30   /* Capture Register 1 */
#define PWM_CCR2_OFFSET		0x34   /* Capture Register 2 */
#define PWM_CCR3_OFFSET		0x38   /* Capture Register 3 */
#define PWM_EMR_OFFSET		0x3C   /* */
#define PWM_MR4_OFFSET		0x40   /* Match Register 4 */
#define PWM_MR5_OFFSET		0x44   /* Match Register 5 */
#define PWM_MR6_OFFSET		0x48   /* Match Register 6 */
#define PWM_PCR_OFFSET		0x4c   /* Control Register */
#define PWM_LER_OFFSET		0x50   /* Latch Enable Register */
#define PWM_CTCR_OFFSET		0x70

/* I2C Base Addresses */
#define I2C0_BASE_ADDR		0xE001C000
#define I2C1_BASE_ADDR		0xE005C000
#define I2C2_BASE_ADDR		0xE0080000
/* I2C 0/1/2 register offsets */
#define I2C_CONSET_OFFSET	0x00   /* Control Set Register */
#define I2C_STAT_OFFSET		0x04   /* Status Register */
#define I2C_DAT_OFFSET		0x08   /* Data Register */
#define I2C_ADR_OFFSET		0x0c   /* Slave Address Register */
#define I2C_SCLH_OFFSET		0x10   /* SCL Duty Cycle Register (high half word) */
#define I2C_SCLL_OFFSET		0x14   /* SCL Duty Cycle Register (low half word) */
#define I2C_CONCLR_OFFSET	0x18   /* Control Clear Register */

/* Pin function select register offsets */

#define PINSEL0_OFFSET		0x00   /* Pin function select register 0 */
#define PINSEL1_OFFSET		0x04   /* Pin function select register 1 */
#define PINSEL2_OFFSET		0x08   /* Pin function select register 2 */
#define PINSEL3_OFFSET		0x0C   /* Pin function select register 3 */
#define PINSEL4_OFFSET		0x10   /* Pin function select register 4 */
#define PINSEL5_OFFSET		0x14   /* Pin function select register 5 */
#define PINSEL6_OFFSET		0x18   /* Pin function select register 6 */
#define PINSEL7_OFFSET		0x1C   /* Pin function select register 7 */
#define PINSEL8_OFFSET		0x20   /* Pin function select register 8 */
#define PINSEL9_OFFSET		0x24   /* Pin function select register 9 */
#define PINSEL10_OFFSET		0x28   /* Pin function select register 10 */


/* Analog to Digital (AD) Base Address */

#define	ADC0_BASE_ADDR		0xE0034000

/* Analog to Digital (AD) Converter registger offsets */

#define AD_ADCR_OFFSET		0x00   /* A/D Control Register */
#define AD_ADGDR_OFFSET		 0x04   /* A/D Global Data Register (only one common register!) */
//~ #define AD_ADGSR_OFFSET		 0x08   /* A/D Global Start Register */
#define AD_ADINTEN_OFFSET	0x0c   /* A/D Interrupt Enable Register */
#define AD_ADDR0_OFFSET		0x10   /* A/D Chanel 0 Data Register */
#define AD_ADDR1_OFFSET		0x14   /* A/D Chanel 0 Data Register */
#define AD_ADDR2_OFFSET		0x18   /* A/D Chanel 0 Data Register */
#define AD_ADDR3_OFFSET		0x1c   /* A/D Chanel 0 Data Register */
#define AD_ADDR4_OFFSET		0x20   /* A/D Chanel 0 Data Register */
#define AD_ADDR5_OFFSET		0x24   /* A/D Chanel 0 Data Register */
#define AD_ADDR6_OFFSET		0x28   /* A/D Chanel 0 Data Register */
#define AD_ADDR7_OFFSET		0x2c   /* A/D Chanel 0 Data Register */
#define AD_ADSTAT_OFFSET	0x30   /* A/D Status Register */

/* Digital to Analog (DAC) Base Address */

#define DAC_BASE_ADDR		0xE006C000	

/* Digital to Analog  (DAC) reister offset */
//#define DACR_OFFSET			0x00

/* SPI0 register offsets */

#define SPI0_BASE_ADDR		0xE0020000

#define SPI0_CR_OFFSET		0x00   /* Control Register */
#define SPI0_SR_OFFSET		0x04   /* Status Register */
#define SPI0_DR_OFFSET		0x08   /* Data Register */
#define SPI0_CCR_OFFSET		0x0c   /* Clock Counter Register */
#define SPI0_INTF_OFFSET	0x1c   /* Interrupt Flag Register */

/* SPI1 register offsets */

//~ #define SPI1_CR0_OFFSET		0x00   /* Control Register 0 */
//~ #define SPI1_CR1_OFFSET		0x04   /* Control Register 1 */
//~ #define SPI1_DR_OFFSET		0x08   /* Data Register */
//~ #define SPI1_SR_OFFSET		0x0c   /* Status Register */
//~ #define SPI1_CPSR_OFFSET	0x10   /* Clock Pre-Scale Regisrer */
//~ #define SPI1_IMSC_OFFSET	0x14   /* Interrupt Mask Set and Clear Register */
//~ #define SPI1_RIS_OFFSET		0x18   /* Raw Interrupt Status Register */
//~ #define SPI1_MIS_OFFSET		0x1c   /* Masked Interrupt Status Register */
//~ #define SPI1_ICR_OFFSET		0x20   /* Interrupt Clear Register */

/* SSP Base Addresses */
#define SSP0_BASE_ADDR		0xE0068000
#define SSP1_BASE_ADDR		0xE0030000
/* SSP 0/1 register offsets */
#define SSP_CR0_OFFSET		0x00	/* Control Register 0 */
#define SSP_CR1_OFFSET		0x04	/* Control Register 1 */
#define SSP_DR_OFFSET		0x08	/* Data Register */
#define SSP_SR_OFFSET		0x0C	/* Status Register*/
#define SSP_CPSR_OFFSET		0x10	/* Clock Prescale Register */
#define SSP_IMSC_OFFSET		0x14	/* Interrupt Mask Set/Clear Register */
#define SSP_RIS_OFFSET		0x18	/* Raw Interrupt Register */
#define SSP_MIS_OFFSET		0x1C	/* Masked Interrupt Status Register */
#define SSP_ICR_OFFSET		0x20	/* Interrupt Clear Register */
#define SSP_DMACR_OFFSET	0x24	/* DMA Control Register */

/* Real Time Clock Base Address */
#define RTC_BASE_ADDR		0xE0024000
/* RTC register offsets */
#define RTC_ILR_OFFSET		0x00   /* Interrupt Location Register */
#define RTC_CTC_OFFSET		0x04   /* Clock Tick Counter */
#define RTC_CCR_OFFSET		0x08   /* Clock Control Register */
#define RTC_CIIR_OFFSET		0x0c   /* Counter Increment Interrupt Register */
#define RTC_AMR_OFFSET		0x10   /* Alarm Mask Register */
#define RTC_CTIME0_OFFSET	0x14   /* Consolidated Time Register 0 */
#define RTC_CTIME1_OFFSET	0x18   /* Consolidated Time Register 1 */
#define RTC_CTIME2_OFFSET	0x1c   /* Consolidated Time Register 2 */
#define RTC_SEC_OFFSET		0x20   /* Seconds Register */
#define RTC_MIN_OFFSET		0x24   /* Minutes Register */
#define RTC_HOUR_OFFSET		0x28   /* Hours Register */
#define RTC_DOM_OFFSET		0x2c   /* Day Of Month Register */
#define RTC_DOW_OFFSET		0x30   /* Day Of Week Register */
#define RTC_DOY_OFFSET		0x34   /* Day Of Year Register */
#define RTC_MONTH_OFFSET	0x38   /* Months Register */
#define RTC_YEAR_OFFSET		0x3c   /* Years Register */

#define RTC_ALSEC_OFFSET	0x60   /* Alarm Seconds Register */
#define RTC_ALMIN_OFFSET	0x64   /* Alarm Minutes Register */
#define RTC_ALHOUR_OFFSET	0x68   /* Alarm Hours Register */
#define RTC_ALDOM_OFFSET	0x6c   /* Alarm Day Of Month Register */
#define RTC_ALDOW_OFFSET	0x70   /* Alarm Day Of Week Register */
#define RTC_ALDOY_OFFSET	0x74   /* Alarm Day Of Year Register */
#define RTC_ALMON_OFFSET	0x78   /* Alarm Months Register */
#define RTC_ALYEAR_OFFSET	0x7c   /* Alarm Years Register */
#define RTC_PREINT_OFFSET	0x80   /* Prescale Value Register (integer) */
#define RTC_PREFRAC_OFFSET	0x84   /* Prescale Value Register (fraction) */


/* Watchdog */
//~ WDG_BASE_ADDR		0xE0000000
#define WDMOD_OFFSET		0x00
#define WDTC_OFFSET			0x04
#define WDFEED_OFFSET		0x08
#define WDTV_OFFSET			0x0C
#define WDCLKSEL_OFFSET		0x10

/* CAN CONTROLLERS AND ACCEPTANCE FILTER */
//~ CAN_ACCEPT_BASE_ADDR		0xE003C000
#define CAN_AFMR_OFFSET			0x00
#define CAN_SFF_SA_OFFSET 		0x04
#define CAN_SFF_GRP_SA_OFFSET 	0x08
#define CAN_EFF_SA_OFFSET 		0x0C
#define CAN_EFF_GRP_SA_OFFSET 	0x10
#define CAN_EOT_OFFSET 			0x14
#define CAN_LUT_ERR_ADR_OFFSET 	0x18
#define CAN_LUT_ERR_OFFSET 		0x1C

//~ CAN_COMMON_BASE_ADDR	0xE0040000  	
#define CAN_TX_SR_OFFSET 	 	0x00
#define CAN_RX_SR_OFFSET 	 	0x04
#define CAN_MSR_OFFSET 		 	0x08

//~ CAN1_BASE_ADDR		0xE0044000
#define CAN1MOD_OFFSET 			0x00
#define CAN1CMR_OFFSET 			0x04
#define CAN1GSR_OFFSET 			0x08
#define CAN1ICR_OFFSET 			0x0C
#define CAN1IER_OFFSET 			0x10
#define CAN1BTR_OFFSET 			0x14
#define CAN1EWL_OFFSET 			0x18
#define CAN1SR_OFFSET 			0x1C
#define CAN1RFS_OFFSET 			0x20
#define CAN1RID_OFFSET 			0x24
#define CAN1RDA_OFFSET 			0x28
#define CAN1RDB_OFFSET 			0x2C
  	
#define CAN1TFI1_OFFSET 		0x30
#define CAN1TID1_OFFSET 		0x34
#define CAN1TDA1_OFFSET 		0x38
#define CAN1TDB1_OFFSET 		0x3C
#define CAN1TFI2_OFFSET 		0x40
#define CAN1TID2_OFFSET 		0x44
#define CAN1TDA2_OFFSET 		0x48
#define CAN1TDB2_OFFSET 		0x4C
#define CAN1TFI3_OFFSET 		0x50
#define CAN1TID3_OFFSET 		0x54
#define CAN1TDA3_OFFSET 		0x58
#define CAN1TDB3_OFFSET 		0x5C

//~ CAN2_BASE_ADDR		0xE0048000
#define CAN2MOD_OFFSET			0x00
#define CAN2CMR_OFFSET			0x04
#define CAN2GSR_OFFSET			0x08
#define CAN2ICR_OFFSET			0x0C
#define CAN2IER_OFFSET			0x10
#define CAN2BTR_OFFSET			0x14
#define CAN2EWL_OFFSET			0x18
#define CAN2SR_OFFSET			0x1C
#define CAN2RFS_OFFSET			0x20
#define CAN2RID_OFFSET			0x24
#define CAN2RDA_OFFSET			0x28
#define CAN2RDB_OFFSET			0x2C
  	
#define CAN2TFI1_OFFSET			0x30
#define CAN2TID1_OFFSET			0x34
#define CAN2TDA1_OFFSET			0x38
#define CAN2TDB1_OFFSET			0x3C
#define CAN2TFI2_OFFSET			0x40
#define CAN2TID2_OFFSET			0x44
#define CAN2TDA2_OFFSET			0x48
#define CAN2TDB2_OFFSET			0x4C
#define CAN2TFI3_OFFSET			0x50
#define CAN2TID3_OFFSET			0x54
#define CAN2TDA3_OFFSET			0x58
#define CAN2TDB3_OFFSET			0x5C


/* MultiMedia Card Interface(MCI) Controller */
//~ MCI_BASE_ADDR		0xE008C000
#define MCI_POWER_OFFSET		0x00
#define MCI_CLOCK_OFFSET		0x04
#define MCI_ARGUMENT_OFFSET		0x08
#define MCI_COMMAND_OFFSET	 	0x0C
#define MCI_RESP_CMD_OFFSET		0x10
#define MCI_RESP0_OFFSET		0x14
#define MCI_RESP1_OFFSET		0x18
#define MCI_RESP2_OFFSET		0x1C
#define MCI_RESP3_OFFSET		0x20
#define MCI_DATA_TMR_OFFSET		0x24
#define MCI_DATA_LEN_OFFSET		0x28
#define MCI_DATA_CTRL_OFFSET	0x2C
#define MCI_DATA_CNT_OFFSET		0x30
#define MCI_STATUS_OFFSET	  	0x34
#define MCI_CLEAR_OFFSET		0x38
#define MCI_MASK0_OFFSET		0x3C
#define MCI_MASK1_OFFSET		0x40
#define MCI_FIFO_CNT_OFFSET		0x48
#define MCI_FIFO_OFFSET			0x80


/* I2S Interface Controller (I2S) */
//~ I2S_BASE_ADDR		0xE0088000
#define I2S_DAO_OFFSET			0x00
#define I2S_DAI_OFFSET			0x04
#define I2S_TX_FIFO_OFFSET		0x08
#define I2S_RX_FIFO_OFFSET		0x0C
#define I2S_STATE_OFFSET	  	0x10
#define I2S_DMA1_OFFSET			0x14
#define I2S_DMA2_OFFSET			0x18
#define I2S_IRQ_OFFSET			0x1C
#define I2S_TXRATE_OFFSET		0x20
#define I2S_RXRATE_OFFSET		0x24

/* General-purpose DMA Controller */
/* DMA_BASE_ADDR	0xFFE04000 */
#define GPDMA_INT_STAT_OFFSET			0x4000
#define GPDMA_INT_TCSTAT_OFFSET			0x4004
#define GPDMA_INT_TCCLR_OFFSET			0x4008
#define GPDMA_INT_ERR_STAT_OFFSET		0x400C
#define GPDMA_INT_ERR_CLR_OFFSET		0x4010
#define GPDMA_RAW_INT_TCSTAT_OFFSET		0x4014
#define GPDMA_RAW_INT_ERR_STAT_OFFSET	0x4018
#define GPDMA_ENABLED_CHNS_OFFSET		0x401C
#define GPDMA_SOFT_BREQ_OFFSET			0x4020
#define GPDMA_SOFT_SREQ_OFFSET			0x4024
#define GPDMA_SOFT_LBREQ_OFFSET			0x4028
#define GPDMA_SOFT_LSREQ_OFFSET			0x402C
#define GPDMA_CONFIG_OFFSET				0x4030
#define GPDMA_SYNC_OFFSET		0x4034
/* DMA channel 0 registers */
#define GPDMA_CH0_SRC_OFFSET	0x4100
#define GPDMA_CH0_DEST_OFFSET	0x4104
#define GPDMA_CH0_LLI_OFFSET	0x4108
#define GPDMA_CH0_CTRL_OFFSET	0x410C
#define GPDMA_CH0_CFG_OFFSET	0x4110
/* DMA channel 1 registers */
#define GPDMA_CH1_SRC_OFFSET	0x4120
#define GPDMA_CH1_DEST_OFFSET	0x4124
#define GPDMA_CH1_LLI_OFFSET	0x4128
#define GPDMA_CH1_CTRL_OFFSET	0x412C
#define GPDMA_CH1_CFG_OFFSET	0x4130


/* USB Controller */
#define USB_INT_BASE_ADDR	0xE01FC1C0
#define USB_BASE_ADDR		0xFFE0C200		/* USB Base Address */

/* USB Device Interrupt Registers */
#define USB_INT_STAT_OFFSET		0x00
#define USB_INT_EN_OFFSET	  	0x04
#define USB_INT_CLR_OFFSET	 	0x08
#define USB_INT_SET_OFFSET	 	0x0C
#define USB_INT_PRIO_OFFSET		0x2C

/* USB Device Endpoint Interrupt Registers */
#define USB_EP_INT_STAT_OFFSET	 	0x30
#define USB_EP_INT_EN_OFFSET	   	0x34
#define USB_EP_INT_CLR_OFFSET	  	0x38
#define USB_EP_INT_SET_OFFSET	  	0x3C
#define USB_EP_INT_PRIO_OFFSET	 	0x40

/* USB Device Endpoint Realization Registers */
#define USB_REALIZE_EP_OFFSET	  	0x44
#define USB_EP_INDEX_OFFSET			0x48
#define USB_MAXPACKET_SIZE_OFFSET  	0x4C

/* USB Device Command Reagisters */
#define USB_CMD_CODE_OFFSET			0x10
#define USB_CMD_DATA_OFFSET			0x14

/* USB Device Data Transfer Registers */
#define USB_RX_DATA_OFFSET		 	0x18
#define USB_TX_DATA_OFFSET		 	0x1C
#define USB_RX_PLENGTH_OFFSET	  	0x20
#define USB_TX_PLENGTH_OFFSET	  	0x24
#define USB_USB_CTRL_OFFSET			0x28

/* USB Device DMA Registers */
#define USB_DMA_REQ_STAT_OFFSET		0x50
#define USB_DMA_REQ_CLR_OFFSET	 	0x54
#define USB_DMA_REQ_SET_OFFSET	 	0x58
#define USB_UDCA_HEAD_OFFSET		0x80
#define USB_EP_DMA_STAT_OFFSET		0x84
#define USB_EP_DMA_EN_OFFSET		0x88
#define USB_EP_DMA_DIS_OFFSET		0x8C
#define USB_DMA_INT_STAT_OFFSET		0x90
#define USB_DMA_INT_EN_OFFSET		0x94
#define USB_EOT_INT_STAT_OFFSET		0xA0
#define USB_EOT_INT_CLR_OFFSET		0xA4
#define USB_EOT_INT_SET_OFFSET		0xA8
#define USB_NDD_REQ_INT_STAT_OFFSET	0xAC
#define USB_NDD_REQ_INT_CLR_OFFSET	0xB0
#define USB_NDD_REQ_INT_SET_OFFSET	0xB4
#define USB_SYS_ERR_INT_STAT_OFFSET	0xB8
#define USB_SYS_ERR_INT_CLR_OFFSET	0xBC
#define USB_SYS_ERR_INT_SET_OFFSET	0xC0

/* System Control Block(SCB) modules include Memory Accelerator Module,
Phase Locked Loop, VPB divider, Power Control, External Interrupt, 
Reset, and Code Security/Debugging */

#define SCB_BASE_ADDR	0xE01FC000
/* Memory Accelerator Module (MAM) Regiser */
#define SCB_MAMCR			(*(volatile uint32_t*)(0xE01FC000))
#define SCB_MAMTIM			(*(volatile uint32_t*)(0xE01FC004))
#define SCB_MEMMAP			(*(volatile uint32_t*)(0xE01FC040))
/* Phase Locked Loop (PLL) Register */
#define SCB_PLLCON			(*(volatile uint32_t*)(0xE01FC080))
#define SCB_PLLCFG			(*(volatile uint32_t*)(0xE01FC084))
#define SCB_PLLSTAT			(*(volatile uint32_t*)(0xE01FC088))
#define SCB_PLLFEED			(*(volatile uint32_t*)(0xE01FC08C))
/* Power Control register */
#define SCB_PCON			(*(volatile uint32_t*)(0xE01FC0C0))
#define SCB_PCONP			(*(volatile uint32_t*)(0xE01FC0C4))
#define SCB_PCONP_OFFSET	0x0C4
/* Clock Divider Register */
#define SCB_CCLKCFG			(*(volatile uint32_t*)(0xE01FC104))
#define SCB_USBCLKCFG		(*(volatile uint32_t*)(0xE01FC108))
#define SCB_CLKSRCSEL		(*(volatile uint32_t*)(0xE01FC10C))
#define SCB_PCLKSEL0		(*(volatile uint32_t*)(0xE01FC1A8))
#define SCB_PCLKSEL1		(*(volatile uint32_t*)(0xE01FC1AC))
#define SCB_PCLKSEL0_OFFSET	(0x1A8)
#define SCB_PCLKSEL1_OFFSET	(0x1AC)
/* External Interrupt register */
#define SCB_EXTINT			(*(volatile uint32_t*)(0xE01FC140))
#define SCB_INTWAKE			(*(volatile uint32_t*)(0xE01FC144))
#define SCB_EXTMODE			(*(volatile uint32_t*)(0xE01FC148))
#define SCB_EXTPOLAR		(*(volatile uint32_t*)(0xE01FC14C))
/* Reser Source Indentification register */
#define SCB_RSIR			(*(volatile uint32_t*)(0xE01FC180))
/* RSID, code security protection */
#define SCB_CSPR			(*(volatile uint32_t*)(0xE01FC184))

#define SCB_AHBCFG1			(*(volatile uint32_t*)(0xE01FC188))
#define SCB_AHBCFG2			(*(volatile uint32_t*)(0xE01FC18C))
/* System Controls and Status Register */
#define SCB_SCS				(*(volatile uint32_t*)(0xE01FC1A0))

//~ /* External Memory Controller (EMC) definitions */


/* MPMC(EMC) registers, note: all the external memory controller(EMC) registers 
are for LPC24xx only. */
#define STATIC_MEM0_BASE	0x80000000
#define STATIC_MEM1_BASE	0x81000000
#define STATIC_MEM2_BASE	0x82000000
#define STATIC_MEM3_BASE	0x83000000

#define DYNAMIC_MEM0_BASE	0xA0000000
#define DYNAMIC_MEM1_BASE	0xB0000000
#define DYNAMIC_MEM2_BASE	0xC0000000
#define DYNAMIC_MEM3_BASE	0xD0000000

/* External Memory Controller (EMC) */
//~ #define EMC_BASE_ADDR		0xFFE08000
#define EMC_CTRL_OFFSET		0x000
#define EMC_STAT_OFFSET		0x004
#define EMC_CONFIG_OFFSET		0x008

/* Dynamic RAM access registers */
#define EMC_DYN_CTRL_OFFSET		0x020
#define EMC_DYN_RFSH_OFFSET		0x024
#define EMC_DYN_RD_CFG_OFFSET	0x028
#define EMC_DYN_RP_OFFSET		0x030
#define EMC_DYN_RAS_OFFSET		0x034
#define EMC_DYN_SREX_OFFSET		0x038
#define EMC_DYN_APR_OFFSET		0x03C
#define EMC_DYN_DAL_OFFSET		0x040
#define EMC_DYN_WR_OFFSET		0x044
#define EMC_DYN_RC_OFFSET		0x048
#define EMC_DYN_RFC_OFFSET		0x04C
#define EMC_DYN_XSR_OFFSET		0x050
#define EMC_DYN_RRD_OFFSET		0x054
#define EMC_DYN_MRD_OFFSET		0x058

#define EMC_DYN_CFG0_OFFSET		 0x100
#define EMC_DYN_RASCAS0_OFFSET	 0x104
#define EMC_DYN_CFG1_OFFSET		 0x140
#define EMC_DYN_RASCAS1_OFFSET	 0x144
#define EMC_DYN_CFG2_OFFSET		 0x160
#define EMC_DYN_RASCAS2_OFFSET	 0x164
#define EMC_DYN_CFG3_OFFSET		 0x180
#define EMC_DYN_RASCAS3_OFFSET	 0x184

/* static RAM access registers */
#define EMC_STA_CFG0_OFFSET		 0x200
#define EMC_STA_WAITWEN0_OFFSET  0x204
#define EMC_STA_WAITOEN0_OFFSET  0x208
#define EMC_STA_WAITRD0_OFFSET   0x20C
#define EMC_STA_WAITPAGE0_OFFSET 0x210
#define EMC_STA_WAITWR0_OFFSET   0x214
#define EMC_STA_WAITTURN0_OFFSET 0x218

#define EMC_STA_CFG1_OFFSET		 0x220
#define EMC_STA_WAITWEN1_OFFSET  0x224
#define EMC_STA_WAITOEN1_OFFSET  0x228
#define EMC_STA_WAITRD1_OFFSET   0x22C
#define EMC_STA_WAITPAGE1_OFFSET 0x230
#define EMC_STA_WAITWR1_OFFSET   0x234
#define EMC_STA_WAITTURN1_OFFSET 0x238

#define EMC_STA_CFG2_OFFSET		 0x240
#define EMC_STA_WAITWEN2_OFFSET  0x244
#define EMC_STA_WAITOEN2_OFFSET  0x248
#define EMC_STA_WAITRD2_OFFSET   0x24C
#define EMC_STA_WAITPAGE2_OFFSET 0x250
#define EMC_STA_WAITWR2_OFFSET   0x254
#define EMC_STA_WAITTURN2_OFFSET 0x258

#define EMC_STA_CFG3	  0x260
#define EMC_STA_WAITWEN3_OFFSET  0x264
#define EMC_STA_WAITOEN3_OFFSET  0x268
#define EMC_STA_WAITRD3_OFFSET   0x26C
#define EMC_STA_WAITPAGE3_OFFSET 0x270
#define EMC_STA_WAITWR3_OFFSET   0x274
#define EMC_STA_WAITTURN3_OFFSET 0x278

#define EMC_STA_EXT_WAIT_OFFSET  0x880

/* GPIO register offsets WORD access */
#define GPIO0_PIN_OFFSET		0x00   /* Pin Value Register */
#define GPIO0_SET_OFFSET		0x04   /* Pin Output Set Register */
#define GPIO0_DIR_OFFSET		0x08   /* Pin Direction Register */
#define GPIO0_CLR_OFFSET		0x0c   /* Pin Output Clear Register */
#define GPIO1_PIN_OFFSET		0x10   /* Pin Value Register */
#define GPIO1_SET_OFFSET		0x14   /* Pin Output Set Register */
#define GPIO1_DIR_OFFSET		0x18   /* Pin Direction Register */
#define GPIO1_CLR_OFFSET		0x1c   /* Pin Output Clear Register */

/* Fast I0 Base Address */
#define FIO_BASE_ADDR		0x3FFFC000
/* FIO register offsets WORD access */
#define FIO0_DIR_OFFSET		 	0x00   /* Fast GPIO Port Direction Register */
#define FIO0_MASK_OFFSET		0x10   /* Fast GPIO Mask Register */
#define FIO0_PIN_OFFSET			0x14   /* Fast GPIO Pin Value Register */
#define FIO0_SET_OFFSET			0x18   /* Fast GPIO Port Output Set Register */
#define FIO0_CLR_OFFSET			0x1c   /* Fast GPIO Port Output Clear Register */

#define FIO1_DIR_OFFSET			0x20   /* Fast GPIO Port Direction Register */
#define FIO1_MASK_OFFSET		0x30   /* Fast GPIO Mask Register */
#define FIO1_PIN_OFFSET			0x34   /* Fast GPIO Pin Value Register */
#define FIO1_SET_OFFSET			0x38   /* Fast GPIO Port Output Set Register */
#define FIO1_CLR_OFFSET			0x3c   /* Fast GPIO Port Output Clear Register */

#define FIO2_DIR_OFFSET			0x40   /* Fast GPIO Port Direction Register */
#define FIO2_MASK_OFFSET		0x50   /* Fast GPIO Mask Register */
#define FIO2_PIN_OFFSET			0x54   /* Fast GPIO Pin Value Register */
#define FIO2_SET_OFFSET			0x58   /* Fast GPIO Port Output Set Register */
#define FIO2_CLR_OFFSET			0x5c   /* Fast GPIO Port Output Clear Register */

#define FIO3_DIR_OFFSET			0x60   /* Fast GPIO Port Direction Register */
#define FIO3_MASK_OFFSET		0x70   /* Fast GPIO Mask Register */
#define FIO3_PIN_OFFSET			0x74   /* Fast GPIO Pin Value Register */
#define FIO3_SET_OFFSET			0x78   /* Fast GPIO Port Output Set Register */
#define FIO3_CLR_OFFSET			0x7c   /* Fast GPIO Port Output Clear Register */

#define FIO4_DIR_OFFSET			0x80   /* Fast GPIO Port Direction Register */
#define FIO4_MASK_OFFSET		0x90   /* Fast GPIO Mask Register */
#define FIO4_PIN_OFFSET			0x94   /* Fast GPIO Pin Value Register */
#define FIO4_SET_OFFSET			0x98   /* Fast GPIO Port Output Set Register */
#define FIO4_CLR_OFFSET			0x9c   /* Fast GPIO Port Output Clear Register */


/* FIO register offsets HALF-WORD access */

#define FIO0MASKL_OFFSET	0x10 /* Fast IO Mask Lower HALF-WORD */
#define FIO1MASKL_OFFSET	0x30
#define FIO2MASKL_OFFSET	0x50
#define FIO3MASKL_OFFSET	0x70
#define FIO4MASKL_OFFSET	0x90

#define FIO0MASKU_OFFSET	0x12 /* Fast IO Mask Upper HALF-WORD */
#define FIO1MASKU_OFFSET	0x32
#define FIO2MASKU_OFFSET	0x52
#define FIO3MASKU_OFFSET	0x72
#define FIO4MASKU_OFFSET	0x92

#define FIO0PINL_OFFSET		0x14 /* Fast IOPIN Lower HALF-WORD */
#define FIO1PINL_OFFSET		0x34
#define FIO2PINL_OFFSET		0x54
#define FIO3PINL_OFFSET		0x74
#define FIO4PINL_OFFSET		0x94

#define FIO0PINU_OFFSET		0x16 /* Fast IOPIN Upper HALF-WORD */
#define FIO1PINU_OFFSET		0x36
#define FIO2PINU_OFFSET		0x56
#define FIO3PINU_OFFSET		0x76
#define FIO4PINU_OFFSET		0x96

#define FIO0SETL_OFFSET		0x18 /* Fast IOSET Lower HALF-WORD */
#define FIO1SETL_OFFSET		0x38
#define FIO2SETL_OFFSET		0x58
#define FIO3SETL_OFFSET		0x78
#define FIO4SETL_OFFSET		0x98

#define FIO0SETU_OFFSET		0x1A /* Fast IOSET Upper HALF-WORD */
#define FIO1SETU_OFFSET		0x3A
#define FIO2SETU_OFFSET		0x5A
#define FIO3SETU_OFFSET		0x7A
#define FIO4SETU_OFFSET		0x9A

#define FIO0CLRL_OFFSET		0x1C /* Fast IOCLR Lower HALF-WORD */
#define FIO1CLRL_OFFSET		0x3C
#define FIO2CLRL_OFFSET		0x5C
#define FIO3CLRL_OFFSET		0x7C
#define FIO4CLRL_OFFSET		0x9C

#define FIO0CLRU_OFFSET		0x1E /* Fast IOCLR Upper HALF-WORD */
#define FIO1CLRU_OFFSET		0x3E
#define FIO2CLRU_OFFSET		0x5E
#define FIO3CLRU_OFFSET		0x7E
#define FIO4CLRU_OFFSET		0x9E

#define FIO0DIRL_OFFSET		0x00 /* Fast IODIR Lower HALF-WORD */
#define FIO1DIRL_OFFSET		0x20
#define FIO2DIRL_OFFSET		0x40
#define FIO3DIRL_OFFSET		0x60
#define FIO4DIRL_OFFSET		0x80

#define FIO0DIRU_OFFSET		0x02 /* Fast IODIR Upper HALF-WORD */
#define FIO1DIRU_OFFSET		0x22
#define FIO2DIRU_OFFSET		0x42
#define FIO3DIRU_OFFSET		0x62
#define FIO4DIRU_OFFSET		0x82


/* FIO register offsets BYTE access */

#define FIO0DIR0_OFFSET		0x00
#define FIO1DIR0_OFFSET		0x20
#define FIO2DIR0_OFFSET		0x40
#define FIO3DIR0_OFFSET		0x60
#define FIO4DIR0_OFFSET		0x80

#define FIO0DIR1_OFFSET		0x01
#define FIO1DIR1_OFFSET		0x21
#define FIO2DIR1_OFFSET		0x41
#define FIO3DIR1_OFFSET		0x61
#define FIO4DIR1_OFFSET		0x81

#define FIO0DIR2_OFFSET		0x02
#define FIO1DIR2_OFFSET		0x22
#define FIO2DIR2_OFFSET		0x42
#define FIO3DIR2_OFFSET		0x62
#define FIO4DIR2_OFFSET		0x82

#define FIO0DIR3_OFFSET		0x03
#define FIO1DIR3_OFFSET		0x23
#define FIO2DIR3_OFFSET		0x43
#define FIO3DIR3_OFFSET		0x63
#define FIO4DIR3_OFFSET		0x83

#define FIO0MASK0_OFFSET	0x10
#define FIO1MASK0_OFFSET	0x30
#define FIO2MASK0_OFFSET	0x50
#define FIO3MASK0_OFFSET	0x70
#define FIO4MASK0_OFFSET	0x90

#define FIO0MASK1_OFFSET	0x11
#define FIO1MASK1_OFFSET	0x21
#define FIO2MASK1_OFFSET	0x51
#define FIO3MASK1_OFFSET	0x71
#define FIO4MASK1_OFFSET	0x91

#define FIO0MASK2_OFFSET	0x12
#define FIO1MASK2_OFFSET	0x32
#define FIO2MASK2_OFFSET	0x52
#define FIO3MASK2_OFFSET	0x72
#define FIO4MASK2_OFFSET	0x92

#define FIO0MASK3_OFFSET	0x13
#define FIO1MASK3_OFFSET	0x33
#define FIO2MASK3_OFFSET	0x53
#define FIO3MASK3_OFFSET	0x73
#define FIO4MASK3_OFFSET	0x93

#define FIO0PIN0_OFFSET		0x14
#define FIO1PIN0_OFFSET		0x34
#define FIO2PIN0_OFFSET		0x54
#define FIO3PIN0_OFFSET		0x74
#define FIO4PIN0_OFFSET		0x94

#define FIO0PIN1_OFFSET		0x15
#define FIO1PIN1_OFFSET		0x25
#define FIO2PIN1_OFFSET		0x55
#define FIO3PIN1_OFFSET		0x75
#define FIO4PIN1_OFFSET		0x95

#define FIO0PIN2_OFFSET		0x16
#define FIO1PIN2_OFFSET		0x36
#define FIO2PIN2_OFFSET		0x56
#define FIO3PIN2_OFFSET		0x76
#define FIO4PIN2_OFFSET		0x96

#define FIO0PIN3_OFFSET		0x17
#define FIO1PIN3_OFFSET		0x37
#define FIO2PIN3_OFFSET		0x57
#define FIO3PIN3_OFFSET		0x77
#define FIO4PIN3_OFFSET		0x97

#define FIO0SET0_OFFSET		0x18
#define FIO1SET0_OFFSET		0x38
#define FIO2SET0_OFFSET		0x58
#define FIO3SET0_OFFSET		0x78
#define FIO4SET0_OFFSET		0x98

#define FIO0SET1_OFFSET		0x19
#define FIO1SET1_OFFSET		0x29
#define FIO2SET1_OFFSET		0x59
#define FIO3SET1_OFFSET		0x79
#define FIO4SET1_OFFSET		0x99

#define FIO0SET2_OFFSET		0x1A
#define FIO1SET2_OFFSET		0x3A
#define FIO2SET2_OFFSET		0x5A
#define FIO3SET2_OFFSET		0x7A
#define FIO4SET2_OFFSET		0x9A

#define FIO0SET3_OFFSET		0x1B
#define FIO1SET3_OFFSET		0x3B
#define FIO2SET3_OFFSET		0x5B
#define FIO3SET3_OFFSET		0x7B
#define FIO4SET3_OFFSET		0x9B

#define FIO0CLR0_OFFSET		0x1C
#define FIO1CLR0_OFFSET		0x3C
#define FIO2CLR0_OFFSET		0x5C
#define FIO3CLR0_OFFSET		0x7C
#define FIO4CLR0_OFFSET		0x9C

#define FIO0CLR1_OFFSET		0x1D
#define FIO1CLR1_OFFSET		0x2D
#define FIO2CLR1_OFFSET		0x5D
#define FIO3CLR1_OFFSET		0x7D
#define FIO4CLR1_OFFSET		0x9D

#define FIO0CLR2_OFFSET		0x1E
#define FIO1CLR2_OFFSET		0x3E
#define FIO2CLR2_OFFSET		0x5E
#define FIO3CLR2_OFFSET		0x7E
#define FIO4CLR2_OFFSET		0x9E

#define FIO0CLR3_OFFSET		0x1F
#define FIO1CLR3_OFFSET		0x3F
#define FIO2CLR3_OFFSET		0x5F
#define FIO3CLR3_OFFSET		0x7F
#define FIO4CLR3_OFFSET		0x9F

/* Vectored Interrupt Controller (VIC) register offsets */

#define VIC_IRQSTATUS_OFFSET	0x000   /* R: IRQ Status Register */
#define VIC_FIQSTATUS_OFFSET	0x004   /* R: FIQ Status Register */
#define VIC_RAWINTR_OFFSET		0x008   /* R: Raw Interrupt Status Register */
#define VIC_INTSELECT_OFFSET	0x00c   /* RW: Interrupt Select Register */
#define VIC_INTENABLE_OFFSET	0x010   /* RW: Interrupt Enable Register */
#define VIC_INTENCLEAR_OFFSET	0x014   /* W: Interrupt Enable Clear Register */
#define VIC_SOFTINT_OFFSET		0x018   /* RW: Software Interrupt Register */
#define VIC_SOFTINTCLEAR_OFFSET 0x01c   /* W: Software Interrupt Clear Register */
#define VIC_PROTECTION_OFFSET	0x020   /* Protection Enable Register */
#define VIC_PRIORITY_MASK_OFFSET 0x024   /* Priority Mask Register */


//~ #define LPC23XX_VIC_BASE			0xfffff000  /* Vectored Interrupt Controller (VIC) Base */
#define VIC_ADDRESS_OFFSET		0xF00   /* RW: Vector Address Register */

#define VIC_VECTADDR0_OFFSET	0x100  /* RW: Vector Address 0 Register */
#define VIC_VECTADDR1_OFFSET	0x104  /* RW: Vector Address 1 Register */
#define VIC_VECTADDR2_OFFSET	0x108  /* RW: Vector Address 2 Register */
#define VIC_VECTADDR3_OFFSET	0x10c  /* RW: Vector Address 3 Register */
#define VIC_VECTADDR4_OFFSET	0x110  /* RW: Vector Address 4 Register */
#define VIC_VECTADDR5_OFFSET	0x114  /* RW: Vector Address 5 Register */
#define VIC_VECTADDR6_OFFSET	0x118  /* RW: Vector Address 6 Register */
#define VIC_VECTADDR7_OFFSET	0x11c  /* RW: Vector Address 7 Register */
#define VIC_VECTADDR8_OFFSET	0x120  /* RW: Vector Address 8 Register */
#define VIC_VECTADDR9_OFFSET	0x124  /* RW: Vector Address 9 Register */
#define VIC_VECTADDR10_OFFSET   0x128  /* RW: Vector Address 10 Register */
#define VIC_VECTADDR11_OFFSET   0x12c  /* RW: Vector Address 11 Register */
#define VIC_VECTADDR12_OFFSET   0x130  /* RW: Vector Address 12 Register */
#define VIC_VECTADDR13_OFFSET   0x134  /* RW: Vector Address 13 Register */
#define VIC_VECTADDR14_OFFSET   0x138  /* RW: Vector Address 14 Register */
#define VIC_VECTADDR15_OFFSET   0x13c  /* RW: Vector Address 15 Register */
#define VIC_VECTADDR16_OFFSET   0x140  /* RW: Vector Address 16 Register */
#define VIC_VECTADDR17_OFFSET   0x144  /* RW: Vector Address 17 Register */
#define VIC_VECTADDR18_OFFSET   0x148  /* RW: Vector Address 18 Register */
#define VIC_VECTADDR19_OFFSET   0x14c  /* RW: Vector Address 19 Register */
#define VIC_VECTADDR20_OFFSET   0x150  /* RW: Vector Address 20 Register */
#define VIC_VECTADDR21_OFFSET   0x154  /* RW: Vector Address 21 Register */
#define VIC_VECTADDR22_OFFSET   0x158  /* RW: Vector Address 22 Register */
#define VIC_VECTADDR23_OFFSET   0x15c  /* RW: Vector Address 23 Register */
#define VIC_VECTADDR24_OFFSET   0x160  /* RW: Vector Address 24 Register */
#define VIC_VECTADDR25_OFFSET   0x164  /* RW: Vector Address 25 Register */
#define VIC_VECTADDR26_OFFSET   0x168  /* RW: Vector Address 26 Register */
#define VIC_VECTADDR27_OFFSET   0x16c  /* RW: Vector Address 27 Register */
#define VIC_VECTADDR28_OFFSET   0x170  /* RW: Vector Address 28 Register */
#define VIC_VECTADDR29_OFFSET   0x174  /* RW: Vector Address 29 Register */
#define VIC_VECTADDR30_OFFSET   0x178  /* RW: Vector Address 30 Register */
#define VIC_VECTADDR31_OFFSET   0x17c  /* RW: Vector Address 31 Register */


/*VICVectPriority */
#define VIC_VECTPRIORITY0_OFFSET	0x200  /* RW: Vector Control 0 Register */
#define VIC_VECTPRIORITY1_OFFSET	0x204  /* RW: Vector Control 1 Register */
#define VIC_VECTPRIORITY2_OFFSET	0x208  /* RW: Vector Control 2 Register */
#define VIC_VECTPRIORITY3_OFFSET	0x20c  /* RW: Vector Control 3 Register */
#define VIC_VECTPRIORITY4_OFFSET	0x210  /* RW: Vector Control 4 Register */
#define VIC_VECTPRIORITY5_OFFSET	0x214  /* RW: Vector Control 5 Register */
#define VIC_VECTPRIORITY6_OFFSET	0x218  /* RW: Vector Control 6 Register */
#define VIC_VECTPRIORITY7_OFFSET	0x21c  /* RW: Vector Control 7 Register */
#define VIC_VECTPRIORITY8_OFFSET	0x220  /* RW: Vector Control 8 Register */
#define VIC_VECTPRIORITY9_OFFSET	0x224  /* RW: Vector Control 9 Register */
#define VIC_VECTPRIORITY10_OFFSET   0x228  /* RW: Vector Control 10 Register */
#define VIC_VECTPRIORITY11_OFFSET   0x22c  /* RW: Vector Control 11 Register */
#define VIC_VECTPRIORITY12_OFFSET   0x230  /* RW: Vector Control 12 Register */
#define VIC_VECTPRIORITY13_OFFSET   0x234  /* RW: Vector Control 13 Register */
#define VIC_VECTPRIORITY14_OFFSET   0x238  /* RW: Vector Control 14 Register */
#define VIC_VECTPRIORITY15_OFFSET   0x23c  /* RW: Vector Control 15 Register */
#define VIC_VECTPRIORITY16_OFFSET   0x240  /* RW: Vector Control 16 Register */
#define VIC_VECTPRIORITY17_OFFSET   0x244  /* RW: Vector Control 17 Register */
#define VIC_VECTPRIORITY18_OFFSET   0x248  /* RW: Vector Control 18 Register */
#define VIC_VECTPRIORITY19_OFFSET   0x24c  /* RW: Vector Control 19 Register */
#define VIC_VECTPRIORITY20_OFFSET   0x250  /* RW: Vector Control 20 Register */
#define VIC_VECTPRIORITY21_OFFSET   0x254  /* RW: Vector Control 21 Register */
#define VIC_VECTPRIORITY22_OFFSET   0x258  /* RW: Vector Control 22 Register */
#define VIC_VECTPRIORITY23_OFFSET   0x25c  /* RW: Vector Control 23 Register */
#define VIC_VECTPRIORITY24_OFFSET   0x260  /* RW: Vector Control 24 Register */
#define VIC_VECTPRIORITY25_OFFSET   0x264  /* RW: Vector Control 25 Register */
#define VIC_VECTPRIORITY26_OFFSET   0x268  /* RW: Vector Control 26 Register */
#define VIC_VECTPRIORITY27_OFFSET   0x26c  /* RW: Vector Control 27 Register */
#define VIC_VECTPRIORITY28_OFFSET   0x270  /* RW: Vector Control 28 Register */
#define VIC_VECTPRIORITY29_OFFSET   0x274  /* RW: Vector Control 29 Register */
#define VIC_VECTPRIORITY30_OFFSET   0x278  /* RW: Vector Control 30 Register */
#define VIC_VECTPRIORITY31_OFFSET   0x27c  /* RW: Vector Control 31 Register */

/****************************************************************************************************
 * Inline Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Global Function Prototypes
 ****************************************************************************************************/

#endif  /* _ARCH_ARM_SRC_LPC2378_CHIP_H */
