/****************************************************************************
 * include/nuttx/wireless/cc1101.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

/** \file
 *  \author Uros Platise
 *  \brief Chipcon CC1101 Device Driver
 **/ 

#ifndef __INCLUDE_NUTTX_WIRELESS_CC1101_H
#define __INCLUDE_NUTTX_WIRELESS_CC1101_H

#include <nuttx/config.h>
#include <nuttx/spi.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Present maximum packet length */

#define CC1101_PACKET_MAXTOTALLEN    63
#define CC1101_PACKET_MAXDATALEN     61

/*
 * General Purpose, Test Output Pin Options
 */
 
/* CC1101 General Purpose Pins */

#define CC1101_PIN_GDO0                         2
#define CC1101_PIN_GDO1                         1
#define CC1101_PIN_GDO2                         0
  
/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or above 
 * the RX FIFO threshold. De-asserts when RX FIFO is drained below the 
 * same threshold. */
#define CC1101_GDO_RXFIFO_THR                   0x00

/* Associated to the RX FIFO: Asserts when RX FIFO is filled at or above 
 * the RX FIFO threshold or the end of packet is reached. De-asserts when 
 * the RX FIFO is empty. */
#define CC1101_GDO_RXFIFO_THREND                0x01   

/* Associated to the TX FIFO: Asserts when the TX FIFO is filled at or 
 * above the TX FIFO threshold. De-asserts when the TX FIFO is below the 
 * same threshold. */
#define CC1101_GDO_TXFIFO_THR                   0x02

/* Associated to the TX FIFO: Asserts when TX FIFO is full. De-asserts
 * when the TX FIFO is drained below theTX FIFO threshold. */
#define CC1101_GDO_TXFIFO_FULL                  0x03

/* Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has 
 * been flushed. */
#define CC1101_GDO_RXFIFO_OVR                   0x04

/* Asserts when the TX FIFO has underflowed. De-asserts when the FIFO is 
 * flushed. */
#define CC1101_GDO_TXFIFO_UNR                   0x05

/* Asserts when sync word has been sent / received, and de-asserts at the
 * end of the packet. In RX, the pin will de-assert when the optional 
 * address check fails or the RX FIFO overflows. In TX the pin will 
 * de-assert if the TX FIFO underflows. */
#define CC1101_GDO_SYNC                         0x06

/* Asserts when a packet has been received with CRC OK. De-asserts when 
 * the first byte is read from the RX FIFO. */
#define CC1101_GDO_PKTRCV_CRCOK                 0x07

/* Preamble Quality Reached. Asserts when the PQI is above the programmed 
 * PQT value. */
#define CC1101_GDO_PREAMBLE                     0x08

/* Clear channel assessment. High when RSSI level is below threshold 
 * (dependent on the current CCA_MODE setting). */
#define CC1101_GDO_CHCLEAR                      0x09

/* Lock detector output. The PLL is in lock if the lock detector output 
 * has a positive transition or is constantly logic high. To check for 
 * PLL lock the lock detector output should be used as an interrupt for 
 * the MCU. */
#define CC1101_GDO_LOCK                         0x0A

/* Serial Clock. Synchronous to the data in synchronous serial mode.
 * In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0.
 * In TX mode, data is sampled by CC1101 on the rising edge of the serial 
 * clock when GDOx_INV=0. */
#define CC1101_GDO_SSCLK                        0x0B

/* Serial Synchronous Data Output. Used for synchronous serial mode. */
#define CC1101_GDO_SSDO                         0x0C

/* Serial Data Output. Used for asynchronous serial mode. */
#define CC1101_GDO_ASDO                         0x0D

/* Carrier sense. High if RSSI level is above threshold. */
#define CC1101_GDO_CARRIER                      0x0E

/* CRC_OK. The last CRC comparison matched. Cleared when entering or 
 * restarting RX mode. */
#define CC1101_GDO_CRCOK                        0x0F

/* RX_HARD_DATA[1]. Can be used together with RX_SYMBOL_TICK for 
 * alternative serial RX output. */
#define CC1101_GDO_RXOUT1                       0x16
 
/* RX_HARD_DATA[0]. Can be used together with RX_SYMBOL_TICK for 
 * alternative serial RX output. */
#define CC1101_GDO_RXOUT0                       0x17

/* PA_PD. Note: PA_PD will have the same signal level in SLEEP and TX 
 * states. To control an external PA or RX/TX switch in applications 
 * where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F 
 * instead. */
#define CC1101_GDO_PA_PD                        0x1B

/* LNA_PD. Note: LNA_PD will have the same signal level in SLEEP and RX 
 * states. To control an external LNA or RX/TX switch in applications 
 * where the SLEEP state is used it is recommended to use GDOx_CFGx=0x2F 
 * instead. */
#define CC1101_GDO_LNA_PD                       0x1C

/* RX_SYMBOL_TICK. Can be used together with RX_HARD_DATA for alternative 
 * serial RX output. */
#define CC1101_GDO_RXSYMTICK                    0x1D

#define CC1101_GDO_WOR_EVNT0                    0x24
#define CC1101_GDO_WOR_EVNT1                    0x25
#define CC1101_GDO_CLK32K                       0x27
#define CC1101_GDO_CHIP_RDYn                    0x29
#define CC1101_GDO_XOSC_STABLE                  0x2B

/* GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input 
 * (for serial TX data). */
#define CC1101_GDO_GDO0_Z_EN_N                  0x2D

/* High impedance (3-state). */
#define CC1101_GDO_HIZ                          0x2E

/* HW to 0 (HW1 achieved by setting GDOx_INV=1). Can be used to control 
 * an external LNA/PA or RX/TX switch. */
#define CC1101_GDO_HW                           0x2F

/* There are 3 GDO pins, but only one CLK_XOSC/n can be selected as an 
 * output at any time. If CLK_XOSC/n is to be monitored on one of the 
 * GDO pins, the other two GDO pins must be configured to values less 
 * than 0x30. The GDO0 default value is CLK_XOSC/192. To optimize RF 
 * performance, these signals should not be used while the radio is 
 * in RX or TX mode. */
#define CC1101_GDO_CLK_XOSC1                    0x30
#define CC1101_GDO_CLK_XOSC1_5                  0x31
#define CC1101_GDO_CLK_XOSC2                    0x32
#define CC1101_GDO_CLK_XOSC3                    0x33
#define CC1101_GDO_CLK_XOSC4                    0x34
#define CC1101_GDO_CLK_XOSC6                    0x35
#define CC1101_GDO_CLK_XOSC8                    0x36
#define CC1101_GDO_CLK_XOSC12                   0x37
#define CC1101_GDO_CLK_XOSC16                   0x38
#define CC1101_GDO_CLK_XOSC24                   0x39
#define CC1101_GDO_CLK_XOSC32                   0x3A
#define CC1101_GDO_CLK_XOSC48                   0x3B
#define CC1101_GDO_CLK_XOSC64                   0x3C
#define CC1101_GDO_CLK_XOSC96                   0x3D
#define CC1101_GDO_CLK_XOSC128                  0x3E
#define CC1101_GDO_CLK_XOSC192                  0x3F


#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

struct cc1101_dev_s;

/** The RF Settings includes only those fields required to configure
 *  the RF radio. Other configuration fields depended on this driver
 *  are configured by the cc1101_init().
 */
struct c1101_rfsettings_s {
	uint8_t FSCTRL1;    /* Frequency synthesizer control. */
	uint8_t FSCTRL0;    /* Frequency synthesizer control. */
    
	uint8_t FREQ2;      /* Frequency control word, high byte. */
	uint8_t FREQ1;      /* Frequency control word, middle byte. */
	uint8_t FREQ0;      /* Frequency control word, low byte. */
    
	uint8_t MDMCFG4;    /* Modem configuration. */
	uint8_t MDMCFG3;    /* Modem configuration. */
	uint8_t MDMCFG2;    /* Modem configuration. */
	uint8_t MDMCFG1;    /* Modem configuration. */
	uint8_t MDMCFG0;    /* Modem configuration. */
    
	uint8_t DEVIATN;    /* Modem deviation setting (when FSK modulation is enabled). */
    
    /* GAP */

	uint8_t FOCCFG;     /* Frequency Offset Compensation Configuration. */

	uint8_t BSCFG;      /* Bit synchronization Configuration. */

	uint8_t AGCCTRL2;   /* AGC control. */
	uint8_t AGCCTRL1;   /* AGC control. */
	uint8_t AGCCTRL0;   /* AGC control. */
    
    /* GAP */
    
	uint8_t FREND1;     /* Front end RX configuration. */
	uint8_t FREND0;     /* Front end RX configuration. */
    
	uint8_t FSCAL3;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL2;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL1;     /* Frequency synthesizer calibration. */
	uint8_t FSCAL0;     /* Frequency synthesizer calibration. */
    
    /* REGULATORY LIMITS */
    
    uint8_t CHMIN;      /* Channel Range defintion MIN .. */
    uint8_t CHMAX;      /* .. and MAX */
    uint8_t PAMAX;      /* at given maximum output power */
    
    /* Power Table, for ramp-up/down and ASK modulation defined for
     * output power values as:
     *   PA = {-30, -20, -15, -10, -5, 0, 5, 10} [dBm]
     */
    uint8_t PA[8];
};


/****************************************************************************
 * RF Configuration Database
 ****************************************************************************/

EXTERN const struct c1101_rfsettings_s

// \todo Recalculate ERP in maximum power level

/* 868 MHz, GFSK, 100 kbps, ISM Region 1 (Europe only)
 *  
 * ISM Region 1 (Europe) only, Band 868–870 MHz
 * 
 * Frequency bands for non-specific short range devices in Europe:
 *
 * Frequency            ERP         Duty Cycle  Bandwidth   Remarks
 * 868 – 868.6 MHz      +14 dBm     < 1%        No limits
 * 868.7 – 869.2 MHz    +14 dBm     < 0.1%      No limits
 * 869.3 – 869.4 MHz    +10 dBm     No limits   < 25 kHz    Appropriate access protocol required
 * 869.4 – 869.65 MHz   +27 dBm     < 10%       < 25 kHz    Channels may be combined to one high speed channel
 * 869.7 -870 MHz       +7 dBm      No limits   No limits
 * 
 * Frequency Band For License-Free Specific Applications in Europe
 * 
 * Frequency            Application     ERP     Duty Cycle  Bandwidth
 * 868.6 – 868.7 MHz    Alarms          +10 dBm  < 0.1%     25 kHz(1)
 * 869.2 – 869.25 MHz   Social Alarms   +10 dBm  < 0.1%     25 kHz
 * 869.25 – 869.3 MHz   Alarms          +10 dBm  < 0.1%     25 kHz
 * 869.65 -869.7 MHz    Alarms          +14 dBm  < 10%      25 kHz
 * 863 – 865 MHz        Radio Microphones +10 dBm No limits 200 kHz
 * 863 -865 MHz Wireless Audio Applications +10 dBm No limits 300 kHz
 * 
 * Duty Cycle Limit     Total On Time       Maximum On Time of      Minimum Off Time of 
 *                      Within One Hour     One Transmission        Two Transmission
 * < 0.1%               3.6 seconds         0.72 seconds            0.72 seconds
 * < 1%                 36 seconds          3.6 seconds             1.8 seconds
 * < 10%                360 seconds         36 seconds              3.6 seconds
 * 
 * Reference: TI Application Report: swra048.pdf, May 2005
 *   ISM-Band and Short Range Device Regulatory Compliance Overview
 */
    cc1101_rfsettings_ISM1_868MHzGFSK100kbps,


/* 905 MHz, GFSK, 250 kbps, ISM Region 2 (America only)
 *  
 * ISM Region 2 (America) only, Band 902–928 MHz
 * 
 * Cordless phones          1 W
 * Microwave ovens        750 W
 * Industrial heaters     100 kW
 * Military radar        1000 kW
 */
    cc1101_rfsettings_ISM2_905MHzGFSK250kbps;


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/** Initialize Chipcon CC1101 Chip.
 *  After initialization CC1101 is ready to listen, receive and transmit
 *  messages on the default channel 0 at given RF configuration.
 * 
 * \param spi SPI Device Structure
 * \param isrpin Select the CC1101_PIN_GDOx used to signal interrupts
 * \param rfsettings Pointer to default RF Settings loaded at boot time.
 * \return Pointer to newly allocated CC1101 structure or NULL on error with errno set.
 * 
 * Possible errno as set by this function on error:
 *  - ENODEV: When device addressed is not compatible or it is not a CC1101
 *  - EFAULT: When there is no device
 *  - ENOMEM: Out of kernel memory to allocate the device
 *  - EBUSY: When device is already addressed by other device driver (not yet supported by low-level driver)
 **/
EXTERN struct cc1101_dev_s * cc1101_init(struct spi_dev_s * spi, uint8_t isrpin,
    uint32_t pinset, const struct c1101_rfsettings_s * rfsettings);


/** Deinitialize Chipcon CC1101 Chip
 * 
 * \param dev Device to CC1101 device structure, as returned by the cc1101_init()
 * \return OK On success
 * 
 **/
EXTERN int cc1101_deinit(struct cc1101_dev_s * dev);


/** Power up device, start conversion. \return Zero on success. */
EXTERN int cc1101_powerup(struct cc1101_dev_s * dev);

/** Power down device, stop conversion. \return Zero on success. */
EXTERN int cc1101_powerdown(struct cc1101_dev_s * dev);

/** Set Multi Purpose Output Function. \return Zero on success. */
EXTERN int cc1101_setgdo(struct cc1101_dev_s * dev, uint8_t pin, uint8_t function);

/** Set RF settings. Use one from the database above. */
EXTERN int cc1101_setrf(struct cc1101_dev_s * dev, const struct c1101_rfsettings_s *settings);

/** Set Channel. 
 *  Note that regulatory check is made and sending may be prohibited.
 * 
 * \retval 0 On success, sending and receiving is allowed.
 * \retval 1 Only receive mode is allowed.
 * \retval <0 On error.
 */
EXTERN int cc1101_setchannel(struct cc1101_dev_s * dev, uint8_t channel);


/** Set Output Power
 * 
 * \param power Value from 0 - 8, where 0 means power off, and values
 *  from 1 .. 8 denote the following output power in dBm:
 *   {-30, -20, -15, -10, -5, 0, 5, 10} [dBm]
 * 
 * If power is above the regulatory limit (defined by the RF settings)
 * it is limited.
 * 
 * \return Actual output power in range from 0..8.
 */
EXTERN uint8_t cc1101_setpower(struct cc1101_dev_s * dev, uint8_t power);


/** Convert RSSI as obtained from CC1101 to [dBm] */
EXTERN int cc1101_calcRSSIdBm(int rssi);


/** Enter receive mode and wait for a packet. 
 *  If transmission is in progress, receive mode is entered upon its
 *  completion. As long cc1101_idle() is not called, each transmission
 *  returns to receive mode.
 * 
 * \param dev Device to CC1101 structure
 * \return Zero on success. 
 */
EXTERN int cc1101_receive(struct cc1101_dev_s * dev);


/** Read received packet 
 * 
 * If size of buffer is too small then the remaining part of data can
 * be discarded by the driver.
 * 
 * Packet contains raw data, including the two bytes:
 *  - RSSI and 
 *  - LQI 
 * appended at the end of the message.
 * 
 * To inquery about the data pending size you the following:
 *  - pass buf=NULL and size > 0, returns pending data packet size
 *  - pass buf=NULL and size = 0, returns maximum data packet size
 * 
 * NOTE: messages length are typically defined by the MAC, transmit/
 *   receive windows at some rate.
 */
EXTERN int cc1101_read(struct cc1101_dev_s * dev, uint8_t * buf, size_t size);


/** Write data to be send, using the cc1101_send()
 * 
 * \param dev Device to CC1101 structure
 * \param buf Pointer to data.
 * \param size Size must be within limits, otherwise data is truncated.
 *    Present driver limitation supports a single cc1101_write() 
 *    prioer cc1101_send() is called.
 */
EXTERN int cc1101_write(struct cc1101_dev_s * dev, const uint8_t * buf, size_t size);


/** Send data previously writtenusing cc1101_write()
 * 
 * \param dev Device to CC1101 structure
 * \return Zero on success. 
 */
EXTERN int cc1101_send(struct cc1101_dev_s * dev);


/** Enter idle state (after reception and transmission completes). 
 * 
 * \return Zero on success.
 */
EXTERN int cc1101_idle(struct cc1101_dev_s * dev);


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WIRELESS_CC1101_H */
