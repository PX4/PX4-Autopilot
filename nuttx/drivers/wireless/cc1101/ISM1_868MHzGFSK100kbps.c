/****************************************************************************
 * drivers/wireless/cc1101/ISM1_868MHzGFSK100kbps.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2011 Ales Verbic.  All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Ales Verbic <ales.verbic@isotel.eu>
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
 
#include <nuttx/wireless/cc1101.h>

/** Settings for 868 MHz, GFSK at 100kbps
 * 
 *  ISM Region 1 (Europe) only, Band 868–870 MHz
 *
 * Frequency            ERP         Duty Cycle  Bandwidth   Remarks
 * 868 – 868.6 MHz      +14 dBm     < 1%        No limits
 * 868.7 – 869.2 MHz    +14 dBm     < 0.1%      No limits
 * 869.3 – 869.4 MHz    +10 dBm     No limits   < 25 kHz    Appropriate access protocol required
 * 869.4 – 869.65 MHz   +27 dBm     < 10%       < 25 kHz    Channels may be combined to one high speed channel
 * 869.7 -870 MHz       +7 dBm      No limits   No limits
 * 
 *   Deviation = 46.142578 
 *   Base frequency = 867.999985 
 *   Carrier frequency = 867.999985 
 *   Channel number = 0 
 *   Carrier frequency = 867.999985 
 *   Modulated = true 
 *   Modulation format = GFSK 
 *   Manchester enable = false 
 *   Sync word qualifier mode = 30/32 sync word bits detected 
 *   Preamble count = 4 
 *   Channel spacing = 199.813843 
 *   Carrier frequency = 867.999985 
 *   Data rate = 99.9069 
 *   RX filter BW = 210.937500 
 *   Data format = Normal mode 
 *   Length config = Fixed packet length mode. Length configured in PKTLEN register 
 *   CRC enable = true 
 *   Packet length = 62 
 *   Device address = 00 
 *   Address config = NO Address check, no broadcast 
 *   CRC autoflush = true 
 *   PA ramping = false 
 *   TX power = 0 
 */
const struct c1101_rfsettings_s cc1101_rfsettings_ISM1_868MHzGFSK100kbps = {
    .FSCTRL1 = 0x08,    // FSCTRL1       Frequency Synthesizer Control
    .FSCTRL0 = 0x00,    // FSCTRL0       Frequency Synthesizer Control
    
    .FREQ2   = 0x20,    // FREQ2         Frequency Control Word, High Byte
    .FREQ1   = 0x25,    // FREQ1         Frequency Control Word, Middle Byte
    .FREQ0   = 0xED,    // FREQ0         Frequency Control Word, Low Byte
    
    .MDMCFG4 = 0x8B,    // MDMCFG4       Modem Configuration
    .MDMCFG3 = 0xE5,    // MDMCFG3       Modem Configuration
    .MDMCFG2 = 0x13,    // MDMCFG2       Modem Configuration
    .MDMCFG1 = 0x22,    // MDMCFG1       Modem Configuration
    .MDMCFG0 = 0xE5,    // MDMCFG0       Modem Configuration
    
    .DEVIATN = 0x46,    // DEVIATN       Modem Deviation Setting

    .FOCCFG  = 0x1D,    // FOCCFG        Frequency Offset Compensation Configuration

    .BSCFG   = 0x1C,    // BSCFG         Bit Synchronization Configuration
    
    .AGCCTRL2= 0xC7,    // AGCCTRL2      AGC Control
    .AGCCTRL1= 0x00,    // AGCCTRL1      AGC Control
    .AGCCTRL0= 0xB2,    // AGCCTRL0      AGC Control

    .FREND1  = 0xB6,    // FREND1        Front End RX Configuration
    .FREND0  = 0x10,    // FREND0        Front End TX Configuration

    .FSCAL3  = 0xEA,    // FSCAL3        Frequency Synthesizer Calibration
    .FSCAL2  = 0x2A,    // FSCAL2        Frequency Synthesizer Calibration
    .FSCAL1  = 0x00,    // FSCAL1        Frequency Synthesizer Calibration
    .FSCAL0  = 0x1F,    // FSCAL0        Frequency Synthesizer Calibration
    
    .CHMIN   = 0,       // Fix at 9th channel: 869.80 MHz +- 100 kHz RF Bandwidth
    .CHMAX   = 9,       // single channel
    
    .PAMAX   = 8,       // 0 means power OFF, 8 represents PA[7]
    .PA      = {0x03, 0x0F, 0x1E, 0x27, 0x67, 0x50, 0x81, 0xC2}
};
