/****************************************************************************
 * drivers/net/e1000.h
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_NET_E1000_H
#define __DRIVERS_NET_E1000_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/************** PCI ID ***************/

#define INTEL_VENDERID   0x8086
#define E1000_82573L     0x109a
#define E1000_82540EM    0x100e
#define E1000_82574L     0x10d3
#define E1000_82567LM    0x10f5
#define E1000_82541PI    0x107c

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum e1000_registers {
	E1000_CTRL	= 0x0000,	// Device Control
	E1000_STATUS	= 0x0008,	// Device Status
	E1000_CTRL_EXT	= 0x0018,	// Device Control Extension
	E1000_FCAL      = 0x0028,       // Flow Control Address Low
	E1000_FCAH      = 0x002C,       // Flow Control Address High
	E1000_FCT       = 0x0030,       // Flow Control Type
	E1000_ICR	= 0x00C0,	// Interrupt Cause Read
	E1000_ICS	= 0x00C8,	// Interrupt Cause Set
	E1000_IMS	= 0x00D0,	// Interrupt Mask Set
	E1000_IMC	= 0x00D8,	// Interrupt Mask Clear
	E1000_RCTL	= 0x0100,	// Receive Control
	E1000_FCTTV     = 0x0170,       // Flow Control Transmit Timer Value
	E1000_TCTL	= 0x0400,	// Transmit Control
	E1000_PBA       = 0x1000,       // Packet Buffer Allocation
	E1000_FCRTL     = 0x2160,       // Flow Control Receive Threshold Low
	E1000_FCRTH     = 0x2168,       // Flow Control Receive Threshold High
	E1000_RDBAL	= 0x2800,	// Rx Descriptor Base Address Low	
	E1000_RDBAH	= 0x2804,	// Rx Descriptor Base Address High
	E1000_RDLEN	= 0x2808,	// Rx Descriptor Length
	E1000_RDH	= 0x2810,	// Rx Descriptor Head	
	E1000_RDT	= 0x2818,	// Rx Descriptor Tail	
	E1000_RXDCTL	= 0x2828,	// Rx Descriptor Control	
	E1000_TDBAL	= 0x3800,	// Tx Descriptor Base Address Low	
	E1000_TDBAH	= 0x3804,	// Tx Descriptor Base Address High
	E1000_TDLEN	= 0x3808,	// Tx Descriptor Length
	E1000_TDH	= 0x3810,	// Tx Descriptor Head	
	E1000_TDT	= 0x3818,	// Tx Descriptor Tail	
	E1000_TXDCTL	= 0x3828,	// Tx Descriptor Control	
	E1000_TPR	= 0x40D0,	// Total Packets Received
	E1000_TPT	= 0x40D4,	// Total Packets Transmitted
	E1000_RA	= 0x5400,	// Receive-filter Array
};

/***************** e1000 device structure *****************/

struct tx_desc {
    uint64_t base_address;
    uint16_t packet_length;
    uint8_t  cksum_offset;
    uint8_t  desc_command;
    uint8_t  desc_status;
    uint8_t  cksum_origin;
    uint16_t special_info;
};

struct rx_desc {
    uint64_t base_address;
    uint16_t packet_length;
    uint16_t packet_cksum;
    uint8_t  desc_status;
    uint8_t  desc_errors;
    uint16_t vlan_tag;
};

#endif
