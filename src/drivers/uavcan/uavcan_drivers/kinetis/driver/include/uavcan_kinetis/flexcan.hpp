/*
 * Copyright (C) 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 * Bit definitions were copied from NuttX KINETIS CAN driver.
 */

#pragma once

#include <uavcan_kinetis/build_config.hpp>

#include <uavcan/uavcan.hpp>
#include <stdint.h>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
// #undef'ed at the end of this file
# define constexpr const
#endif

namespace uavcan_kinetis
{
namespace flexcan
{
enum { HWMaxMB = 16 };

union MBcsType {
	volatile uint32_t w;
	struct {
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 1;
		volatile uint32_t code : 4;
		volatile uint32_t res2 : 4;
	};
};

union FIFOcsType {
	volatile uint32_t cs;
	struct {
		volatile uint32_t time_stamp : 16;
		volatile uint32_t dlc : 4;
		volatile uint32_t rtr : 1;
		volatile uint32_t ide : 1;
		volatile uint32_t srr : 1;
		volatile uint32_t res : 9;
	};
};

union IDType {
	volatile uint32_t w;
	struct {
		volatile uint32_t ext : 29;
		volatile uint32_t resex : 3;
	};
	struct {
		volatile uint32_t res : 18;
		volatile uint32_t std : 11;
		volatile uint32_t resstd : 3;
	};
};

union FilterType {
	volatile uint32_t w;
	struct {
		volatile uint32_t res : 1; // Bit 0 - Reserved
		volatile uint32_t ext : 29; // Bits 1 - 29 EID
	};
	struct {
		volatile uint32_t ress : 19; // Bits 0, 1-18 Reserved
		volatile uint32_t std : 11; // StD ID
	};
	struct {
		volatile uint32_t resc : 30; // Bits 0 - 29  Reserved
		volatile uint32_t ide : 1; // Bit 30 - EID
		volatile uint32_t rtr : 1; // Bit 31 Remote
	};
};

union DataType {
	volatile uint32_t l;
	volatile uint32_t h;
	struct {
		volatile uint32_t b3 : 8;
		volatile uint32_t b2 : 8;
		volatile uint32_t b1 : 8;
		volatile uint32_t b0 : 8;
		volatile uint32_t b7 : 8;
		volatile uint32_t b6 : 8;
		volatile uint32_t b5 : 8;
		volatile uint32_t b4 : 8;
	};
};


struct MessageBufferType {
	MBcsType CS;
	IDType ID;
	DataType data;
};

enum mb_code_tx {
	TxMbInactive = 0x8,     /*!< MB is not active.*/
	TxMbAbort = 0x9,        /*!< MB is aborted.*/
	TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or */
	/*!< MB is a TX Remote Request Frame (when MB RTR = 1).*/
	TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from */
	/*!  an incoming Remote Request Frame.*/
	TxMbNotUsed = 0xF,      /*!< Not used.*/
};

struct RxFiFoType {
	FIFOcsType CS;
	IDType ID;
	DataType data;
};

enum mb_code_rx {
	kRxMbInactive = 0x0, /*!< MB is not active.*/
	kRxMbFull = 0x2,     /*!< MB is full.*/
	kRxMbEmpty = 0x4,    /*!< MB is active and empty.*/
	kRxMbOverrun = 0x6,  /*!< MB is overwritten into a full buffer.*/
	kRxMbBusy = 0x8,     /*!< FlexCAN is updating the contents of the MB.*/
	/*!  The CPU must not access the MB.*/
	kRxMbRanswer = 0xA,  /*!< A frame was configured to recognize a Remote Request Frame */
	/*!  and transmit a Response Frame in return.*/
	kRxMbNotUsed = 0xF,  /*!< Not used.*/
};

struct CanType {
	volatile uint32_t MCR;                  /*!< Module Configuration Register,       Address offset: 0x0000        */
	volatile uint32_t CTRL1;                /*!< Control 1 Register,                  Address offset: 0x0004        */
	volatile uint32_t TIMER;                /*!< Free Running Timer,                  Address offset: 0x0008        */
	uint32_t Reserved0;                     /*!< Reserved                             Address offset: 0x000c        */
	volatile uint32_t RXMGMASK;             /*!< Rx Mailboxes Global Mask Register,   Address offset: 0x0010        */
	volatile uint32_t RX14MASK;             /*!< Rx 14 Mask Register,                 Address offset: 0x0014        */
	volatile uint32_t RX15MASK;             /*!< Rx 15 Mask Register,                 Address offset: 0x0018        */
	volatile uint32_t ECR;                  /*!< Error Counter,                       Address offset: 0x001c        */
	volatile uint32_t ESR1;                 /*!< Error and Status 1 Register,         Address offset: 0x0020        */
	uint32_t Reserved1;                     /*!< Reserved                             Address offset: 0x0024        */
	volatile uint32_t IMASK1;               /*!< Interrupt Masks 1 Register,          Address offset: 0x0028        */
	uint32_t Reserved2;                     /*!< Reserved                             Address offset: 0x002C        */
	volatile uint32_t IFLAG1;               /*!< Interrupt Flags 1 Register,          Address offset: 0x0030        */
	volatile uint32_t CTRL2;                /*!< Control 2 Register,                  Address offset: 0x0034        */
	volatile uint32_t ESR2;                 /*!< Error and Status 2 Register,         Address offset: 0x0038        */
	uint32_t Reserved3;                     /*!< Reserved                             Address offset: 0x003c        */
	uint32_t Reserved4;                     /*!< Reserved                             Address offset: 0x0040        */
	volatile uint32_t CRCR;                 /*!< CRC Register,                        Address offset: 0x0044        */
	volatile uint32_t RXFGMASK;             /*!< Rx FIFO Global Mask Register,        Address offset: 0x0048        */
	volatile uint32_t RXFIR;                /*!< Rx FIFO Information Register,        Address offset: 0x004c        */
	uint32_t RESERVED5[12];                 /*!< Reserved                             Address offset: 0x0050        */
	union {
		RxFiFoType fifo;
		MessageBufferType mb;
	} MB[HWMaxMB];
	uint32_t RESERVED6[448];                /*!< Reserved                             Address offset: 0x0180        */
	volatile FilterType RXIMR[HWMaxMB];         /*!< R0 Individual Mask Registers,        Address offset: 0x0880
                                                          */
};

/* Layout of Fifo, filters and Message buffers  */

enum { FiFo = 0 };
/* 0                       */
/* 1                       */
/* 2                       */
/* 3         Fifo          */
/* 4                       */
/* 5                       */
enum { FirstFilter = 6 };
/* 6                       */
/* 7                       */
/* 8         Filters       */
/* 9                       */
enum { NumHWFilters = 16 };
enum { NumMBinFiFoAndFilters = 10 };
/* 10                      */
/* 11                      */
/* 12                      */
/* 13 Tx Message Buffers   */
/* 14                      */
/* 15                      */
/*-- ----------------------*/

enum { TXMBMask = (0b111111 << NumMBinFiFoAndFilters)  };

/**
 * CANx register sets
 */

/*   Address of the CAN registers */

CanType *const Can[UAVCAN_KINETIS_NUM_IFACES] = {
	reinterpret_cast<CanType *>(0x40024000) // CAN0
#if UAVCAN_KINETIS_NUM_IFACES > 1
	,
	reinterpret_cast<CanType *>(0x400A4000) // CAN1
#endif
};

/* Module Configuration Register */

constexpr unsigned long MCR_MAXMB_SHIFT = (0);            /* Bits 0-6: Number of the Last Message Buffer */
constexpr unsigned long MCR_MAXMB_MASK = (0x7fU << MCR_MAXMB_SHIFT);
/* Bit 7:  Reserved */
constexpr unsigned long MCR_IDAM_SHIFT = (8);             /* Bits 8-9: ID Acceptance Mode */
constexpr unsigned long MCR_IDAM_MASK = (3U << MCR_IDAM_SHIFT);
constexpr unsigned long MCR_IDAM_FMTA(0U << MCR_IDAM_SHIFT);          /* Format A: One full ID  */
constexpr unsigned long CAN_MCR_IDAM_FMTB(1U << MCR_IDAM_SHIFT);      /* Format B: Two full (or partial) IDs */
constexpr unsigned long MCR_IDAM_FMTC(2U << MCR_IDAM_SHIFT);          /* Format C: Four partial IDs */
constexpr unsigned long MCR_IDAM_FMTD(3U << MCR_IDAM_SHIFT);          /* Format D: All frames rejected */
/* Bits 10-11: Reserved */
constexpr unsigned long MCR_AEN = (1U << 12);             /* Bit 12: Abort Enable */
constexpr unsigned long MCR_LPRIOEN = (1U << 13);         /* Bit 13: Local Priority Enable */
/* Bits 14-15: Reserved */
constexpr unsigned long MCR_IRMQ = (1U << 16);            /* Bit 16: Individual Rx Masking and Queue Enable */
constexpr unsigned long MCR_SRXDIS = (1U << 17);          /* Bit 17: Self Reception Disable */
/* Bit 18: Reserved */
constexpr unsigned long MCR_WAKSRC = (1U << 19);          /* Bit 19: Wake up Source */
constexpr unsigned long MCR_LPMACK = (1U << 20);          /* Bit 20: Low Power Mode Acknowledge */
constexpr unsigned long MCR_WRNEN = (1U << 21);           /* Bit 21: Warning Interrupt Enable */
constexpr unsigned long MCR_SLFWAK = (1U << 22);          /* Bit 22: Self Wake Up */
constexpr unsigned long MCR_SUPV = (1U << 23);            /* Bit 23: Supervisor Mode */
constexpr unsigned long MCR_FRZACK = (1U << 24);          /* Bit 24: Freeze Mode Acknowledge */
constexpr unsigned long MCR_SOFTRST = (1U << 25);         /* Bit 25: Soft Reset */
constexpr unsigned long MCR_WAKMSK = (1U << 26);          /* Bit 26: Wake Up Interrupt Mask */
constexpr unsigned long MCR_NOTRDY = (1U << 27);          /* Bit 27: FlexCAN Not Ready */
constexpr unsigned long MCR_HALT = (1U << 28);            /* Bit 28: Halt FlexCAN */
constexpr unsigned long MCR_RFEN = (1U << 29);            /* Bit 29: Rx FIFO Enable */
constexpr unsigned long MCR_FRZ = (1U << 30);             /* Bit 30: Freeze Enable */
constexpr unsigned long MCR_MDIS = (1U << 31);            /* Bit 31: Module Disable */

/* Control 1 Register */

constexpr unsigned long CTRL1_ROPSEG_SHIFT = (0);        /* Bits 0-2: Propagation Segment */
constexpr unsigned long CTRL1_ROPSEG_MASK = (7U << CTRL1_ROPSEG_SHIFT);
constexpr unsigned long CTRL1_LOM = (1U << 3);            /* Bit 3:  Listen-Only Mode */
constexpr unsigned long CTRL1_LBUF = (1U << 4);           /* Bit 4:  Lowest Buffer Transmitted First */
constexpr unsigned long CTRL1_TSYN = (1U << 5);           /* Bit 5:  Timer Sync */
constexpr unsigned long CTRL1_BOFFREC = (1U << 6);        /* Bit 6:  Bus Off Recovery */
constexpr unsigned long CTRL1_SMP = (1U << 7);            /* Bit 7:  CAN Bit Sampling */
/* Bits 8-9: Reserved */
constexpr unsigned long CTRL1_RWRNMSK = (1U << 10);       /* Bit 10: Rx Warning Interrupt Mask */
constexpr unsigned long CTRL1_TWRNMSK = (1U << 11);       /* Bit 11: Tx Warning Interrupt Mask */
constexpr unsigned long CTRL1_LPB = (1U << 12);           /* Bit 12: Loop Back Mode */
constexpr unsigned long CTRL1_CLKSRC = (1U << 13);        /* Bit 13: CAN Engine Clock Source */
constexpr unsigned long CTRL1_ERRMSK = (1U << 14);        /* Bit 14: Error Mask */
constexpr unsigned long CTRL1_BOFFMSK = (1U << 15);       /* Bit 15: Bus Off Mask */
constexpr unsigned long CTRL1_PSEG2_SHIFT = (16);         /* Bits 16-18: Phase Segment 2 */
constexpr unsigned long CTRL1_PSEG2_MASK = (7U << CTRL1_PSEG2_SHIFT);
constexpr unsigned long CTRL1_PSEG1_SHIFT = (19);         /* Bits 19-21: Phase Segment 1 */
constexpr unsigned long CTRL1_PSEG1_MASK = (7U << CTRL1_PSEG1_SHIFT);
constexpr unsigned long CTRL1_RJW_SHIFT = (22);           /* Bits 22-23: Resync Jump Width */
constexpr unsigned long CTRL1_RJW_MASK = (3U << CTRL1_RJW_SHIFT);
constexpr unsigned long CTRL1_PRESDIV_SHIFT = (24);       /* Bits 24-31: Prescaler Division Factor */
constexpr unsigned long CTRL1_PRESDIV_MASK = (0xff << CTRL1_PRESDIV_SHIFT);

/* Free Running Timer */

constexpr unsigned long TIMER_SHIFT = (0U);               /* Bits 0-15: Timer value */
constexpr unsigned long TIMER_MASK = (0xffffU << TIMER_SHIFT);
/* Bits 16-31: Reserved */
/* Rx Mailboxes Global Mask Register (32 Rx Mailboxes Global Mask Bits) */

constexpr unsigned long RXMGMASK0 = (1U << 0);             /* Bit 0: Rx Mailbox 0 Global Mask Bit */
constexpr unsigned long RXMGMASK1 = (1U << 1);             /* Bit 1: Rx Mailbox 1 Global Mask Bit */
constexpr unsigned long RXMGMASK2 = (1U << 2);             /* Bit 2: Rx Mailbox 2 Global Mask Bit */
constexpr unsigned long RXMGMASK3 = (1U << 3);             /* Bit 3: Rx Mailbox 3 Global Mask Bit */
constexpr unsigned long RXMGMASK4 = (1U << 4);             /* Bit 4: Rx Mailbox 4 Global Mask Bit */
constexpr unsigned long RXMGMASK5 = (1U << 5);             /* Bit 5: Rx Mailbox 5 Global Mask Bit */
constexpr unsigned long RXMGMASK6 = (1U << 6);             /* Bit 6: Rx Mailbox 6 Global Mask Bit */
constexpr unsigned long RXMGMASK7 = (1U << 7);             /* Bit 7: Rx Mailbox 7 Global Mask Bit */
constexpr unsigned long RXMGMASK8 = (1U << 8);             /* Bit 8: Rx Mailbox 8 Global Mask Bit */
constexpr unsigned long RXMGMASK9 = (1U << 9);             /* Bit 9: Rx Mailbox 9 Global Mask Bit */
constexpr unsigned long RXMGMASK10 = (1U << 10);            /* Bit 10: Rx Mailbox 10 Global Mask Bit */
constexpr unsigned long RXMGMASK11 = (1U << 11);            /* Bit 11: Rx Mailbox 11 Global Mask Bit */
constexpr unsigned long RXMGMASK12 = (1U << 12);            /* Bit 12: Rx Mailbox 12 Global Mask Bit */
constexpr unsigned long RXMGMASK13 = (1U << 13);            /* Bit 13: Rx Mailbox 13 Global Mask Bit */
constexpr unsigned long RXMGMASK14 = (1U << 14);            /* Bit 14: Rx Mailbox 14 Global Mask Bit */
constexpr unsigned long RXMGMASK15 = (1U << 15);            /* Bit 15: Rx Mailbox 15 Global Mask Bit */
constexpr unsigned long RXMGMASK16 = (1U << 16);            /* Bit 16: Rx Mailbox 16 Global Mask Bit */
constexpr unsigned long RXMGMASK17 = (1U << 17);            /* Bit 17: Rx Mailbox 17 Global Mask Bit */
constexpr unsigned long RXMGMASK18 = (1U << 18);            /* Bit 18: Rx Mailbox 18 Global Mask Bit */
constexpr unsigned long RXMGMASK19 = (1U << 19);            /* Bit 19: Rx Mailbox 19 Global Mask Bit */
constexpr unsigned long RXMGMASK20 = (1U << 20);            /* Bit 20: Rx Mailbox 20 Global Mask Bit */
constexpr unsigned long RXMGMASK21 = (1U << 21);            /* Bit 21: Rx Mailbox 21 Global Mask Bit */
constexpr unsigned long RXMGMASK22 = (1U << 22);            /* Bit 22: Rx Mailbox 22 Global Mask Bit */
constexpr unsigned long RXMGMASK23 = (1U << 23);            /* Bit 23: Rx Mailbox 23 Global Mask Bit */
constexpr unsigned long RXMGMASK24 = (1U << 24);            /* Bit 24: Rx Mailbox 24 Global Mask Bit */
constexpr unsigned long RXMGMASK25 = (1U << 25);            /* Bit 25: Rx Mailbox 25 Global Mask Bit */
constexpr unsigned long RXMGMASK26 = (1U << 26);            /* Bit 26: Rx Mailbox 26 Global Mask Bit */
constexpr unsigned long RXMGMASK27 = (1U << 27);            /* Bit 27: Rx Mailbox 27 Global Mask Bit */
constexpr unsigned long RXMGMASK28 = (1U << 28);            /* Bit 28: Rx Mailbox 28 Global Mask Bit */
constexpr unsigned long RXMGMASK29 = (1U << 29);            /* Bit 29: Rx Mailbox 29 Global Mask Bit */
constexpr unsigned long RXMGMASK30 = (1U << 30);            /* Bit 30: Rx Mailbox 30 Global Mask Bit */
constexpr unsigned long RXMGMASK31 = (1U << 31);            /* Bit 31: Rx Mailbox 31 Global Mask Bit */

/* Rx 14 Mask Register */

constexpr unsigned long RXM14MASK0 = (1U << 0);             /* Bit 0: Rx Buffer 14  Mask Bit 0 */
constexpr unsigned long RXM14MASK1 = (1U << 1);             /* Bit 1: Rx Buffer 14  Mask Bit 1 */
constexpr unsigned long RXM14MASK2 = (1U << 2);             /* Bit 2: Rx Buffer 14  Mask Bit 2 */
constexpr unsigned long RXM14MASK3 = (1U << 3);             /* Bit 3: Rx Buffer 14  Mask Bit 3 */
constexpr unsigned long RXM14MASK4 = (1U << 4);             /* Bit 4: Rx Buffer 14  Mask Bit 4 */
constexpr unsigned long RXM14MASK5 = (1U << 5);             /* Bit 5: Rx Buffer 14  Mask Bit 5 */
constexpr unsigned long RXM14MASK6 = (1U << 6);             /* Bit 6: Rx Buffer 14  Mask Bit 6 */
constexpr unsigned long RXM14MASK7 = (1U << 7);             /* Bit 7: Rx Buffer 14  Mask Bit 7 */
constexpr unsigned long RXM14MASK8 = (1U << 8);             /* Bit 8: Rx Buffer 14  Mask Bit 8 */
constexpr unsigned long RXM14MASK9 = (1U << 9);             /* Bit 9: Rx Buffer 14  Mask Bit 9 */
constexpr unsigned long RXM14MASK10 = (1U << 10);            /* Bit 10: Rx Buffer 14  Mask Bit 10 */
constexpr unsigned long RXM14MASK11 = (1U << 11);            /* Bit 11: Rx Buffer 14  Mask Bit 11 */
constexpr unsigned long RXM14MASK12 = (1U << 12);            /* Bit 12: Rx Buffer 14  Mask Bit 12 */
constexpr unsigned long RXM14MASK13 = (1U << 13);            /* Bit 13: Rx Buffer 14  Mask Bit 13 */
constexpr unsigned long RXM14MASK14 = (1U << 14);            /* Bit 14: Rx Buffer 14  Mask Bit 14 */
constexpr unsigned long RXM14MASK15 = (1U << 15);            /* Bit 15: Rx Buffer 14  Mask Bit 15 */
constexpr unsigned long RXM14MASK16 = (1U << 16);            /* Bit 16: Rx Buffer 14  Mask Bit 16 */
constexpr unsigned long RXM14MASK17 = (1U << 17);            /* Bit 17: Rx Buffer 14  Mask Bit 17 */
constexpr unsigned long RXM14MASK18 = (1U << 18);            /* Bit 18: Rx Buffer 14  Mask Bit 18 */
constexpr unsigned long RXM14MASK19 = (1U << 19);            /* Bit 19: Rx Buffer 14  Mask Bit 19 */
constexpr unsigned long RXM14MASK20 = (1U << 20);            /* Bit 20: Rx Buffer 14  Mask Bit 20 */
constexpr unsigned long RXM14MASK21 = (1U << 21);            /* Bit 21: Rx Buffer 14  Mask Bit 21 */
constexpr unsigned long RXM14MASK22 = (1U << 22);            /* Bit 22: Rx Buffer 14  Mask Bit 22 */
constexpr unsigned long RXM14MASK23 = (1U << 23);            /* Bit 23: Rx Buffer 14  Mask Bit 23 */
constexpr unsigned long RXM14MASK24 = (1U << 24);            /* Bit 24: Rx Buffer 14  Mask Bit 24 */
constexpr unsigned long RXM14MASK25 = (1U << 25);            /* Bit 25: Rx Buffer 14  Mask Bit 25 */
constexpr unsigned long RXM14MASK26 = (1U << 26);            /* Bit 26: Rx Buffer 14  Mask Bit 26 */
constexpr unsigned long RXM14MASK27 = (1U << 27);            /* Bit 27: Rx Buffer 14  Mask Bit 27 */
constexpr unsigned long RXM14MASK28 = (1U << 28);            /* Bit 28: Rx Buffer 14  Mask Bit 28 */
constexpr unsigned long RXM14MASK29 = (1U << 29);            /* Bit 29: Rx Buffer 14  Mask Bit 29 */
constexpr unsigned long RXM14MASK30 = (1U << 30);            /* Bit 30: Rx Buffer 14  Mask Bit 30 */
constexpr unsigned long RXM14MASK31 = (1U << 31);            /* Bit 31: Rx Buffer 14  Mask Bit 31 */

/* Rx 15 Mask Register */

constexpr unsigned long RXM15MASK0 = (1U << 0);             /* Bit 0: Rx Buffer 15  Mask Bit 0 */
constexpr unsigned long RXM15MASK1 = (1U << 1);             /* Bit 1: Rx Buffer 15  Mask Bit 1 */
constexpr unsigned long RXM15MASK2 = (1U << 2);             /* Bit 2: Rx Buffer 15  Mask Bit 2 */
constexpr unsigned long RXM15MASK3 = (1U << 3);             /* Bit 3: Rx Buffer 15  Mask Bit 3 */
constexpr unsigned long RXM15MASK4 = (1U << 4);             /* Bit 4: Rx Buffer 15  Mask Bit 4 */
constexpr unsigned long RXM15MASK5 = (1U << 5);             /* Bit 5: Rx Buffer 15  Mask Bit 5 */
constexpr unsigned long RXM15MASK6 = (1U << 6);             /* Bit 6: Rx Buffer 15  Mask Bit 6 */
constexpr unsigned long RXM15MASK7 = (1U << 7);             /* Bit 7: Rx Buffer 15  Mask Bit 7 */
constexpr unsigned long RXM15MASK8 = (1U << 8);             /* Bit 8: Rx Buffer 15  Mask Bit 8 */
constexpr unsigned long RXM15MASK9 = (1U << 9);             /* Bit 9: Rx Buffer 15  Mask Bit 9 */
constexpr unsigned long RXM15MASK10 = (1U << 10);            /* Bit 10: Rx Buffer 15  Mask Bit 10 */
constexpr unsigned long RXM15MASK11 = (1U << 11);            /* Bit 11: Rx Buffer 15  Mask Bit 11 */
constexpr unsigned long RXM15MASK12 = (1U << 12);            /* Bit 12: Rx Buffer 15  Mask Bit 12 */
constexpr unsigned long RXM15MASK13 = (1U << 13);            /* Bit 13: Rx Buffer 15  Mask Bit 13 */
constexpr unsigned long RXM15MASK14 = (1U << 14);            /* Bit 14: Rx Buffer 15  Mask Bit 14 */
constexpr unsigned long RXM15MASK15 = (1U << 15);            /* Bit 15: Rx Buffer 15  Mask Bit 15 */
constexpr unsigned long RXM15MASK16 = (1U << 16);            /* Bit 16: Rx Buffer 15  Mask Bit 16 */
constexpr unsigned long RXM15MASK17 = (1U << 17);            /* Bit 17: Rx Buffer 15  Mask Bit 17 */
constexpr unsigned long RXM15MASK18 = (1U << 18);            /* Bit 18: Rx Buffer 15  Mask Bit 18 */
constexpr unsigned long RXM15MASK19 = (1U << 19);            /* Bit 19: Rx Buffer 15  Mask Bit 19 */
constexpr unsigned long RXM15MASK20 = (1U << 20);            /* Bit 20: Rx Buffer 15  Mask Bit 20 */
constexpr unsigned long RXM15MASK21 = (1U << 21);            /* Bit 21: Rx Buffer 15  Mask Bit 21 */
constexpr unsigned long RXM15MASK22 = (1U << 22);            /* Bit 22: Rx Buffer 15  Mask Bit 22 */
constexpr unsigned long RXM15MASK23 = (1U << 23);            /* Bit 23: Rx Buffer 15  Mask Bit 23 */
constexpr unsigned long RXM15MASK24 = (1U << 24);            /* Bit 24: Rx Buffer 15  Mask Bit 24 */
constexpr unsigned long RXM15MASK25 = (1U << 25);            /* Bit 25: Rx Buffer 15  Mask Bit 25 */
constexpr unsigned long RXM15MASK26 = (1U << 26);            /* Bit 26: Rx Buffer 15  Mask Bit 26 */
constexpr unsigned long RXM15MASK27 = (1U << 27);            /* Bit 27: Rx Buffer 15  Mask Bit 27 */
constexpr unsigned long RXM15MASK28 = (1U << 28);            /* Bit 28: Rx Buffer 15  Mask Bit 28 */
constexpr unsigned long RXM15MASK29 = (1U << 29);            /* Bit 29: Rx Buffer 15  Mask Bit 29 */
constexpr unsigned long RXM15MASK30 = (1U << 30);            /* Bit 30: Rx Buffer 15  Mask Bit 30 */
constexpr unsigned long RXM15MASK31 = (1U << 31);            /* Bit 31: Rx Buffer 15  Mask Bit 31 */

/* Error Counter */

constexpr unsigned long ECR_TXERRCNT_SHIFT = (0U);     /* Bits 0-7: Transmit Error Counter */
constexpr unsigned long ECR_TXERRCNT_MASK = (0xff << ECR_TXERRCNT_SHIFT);
constexpr unsigned long ECR_RXERRCNT_SHIFT = (8);     /* Bits 8-15: Receive Error Counter */
constexpr unsigned long ECR_RXERRCNT_MASK = (0xff << ECR_RXERRCNT_SHIFT);
/* Bits 16-31: Reserved */

/* Error and Status 1 Register */

constexpr unsigned long ESR1_WAKINT = (1U << 0);            /* Bit 0:  Wake-Up Interrupt */
constexpr unsigned long ESR1_ERRINT = (1U << 1);            /* Bit 1:  Error Interrupt */
constexpr unsigned long ESR1_BOFFINT = (1U << 2);           /* Bit 2:  'Bus Off' Interrupt */
constexpr unsigned long ESR1_RX = (1U << 3);                /* Bit 3:  FlexCAN in Reception */
constexpr unsigned long ESR1_FLTCONF_SHIFT = (4U);          /* Bits 4-5: Fault Confinement State */
constexpr unsigned long ESR1_FLTCONF_MASK = (3U << ESR1_FLTCONF_SHIFT);
constexpr unsigned long ESR1_FLTCONF_ACTV = (0U << ESR1_FLTCONF_SHIFT);      /* Error Active */
constexpr unsigned long ESR1_FLTCONF_PASV = (1U << ESR1_FLTCONF_SHIFT);      /* Error Passive */
constexpr unsigned long ESR1_FLTCONF_OFF = (2U << ESR1_FLTCONF_SHIFT);       /* Bus Off */
constexpr unsigned long ESR1_TX = (1U << 6);                /* Bit 6:  FlexCAN in Transmission */
constexpr unsigned long ESR1_IDLE = (1U << 7);              /* Bit 7:  CAN bus is in IDLE state */
constexpr unsigned long ESR1_RXWRN = (1U << 8);             /* Bit 8:  Rx Error Warning */
constexpr unsigned long ESR1_TXWRN = (1U << 9);             /* Bit 9:  TX Error Warning */
constexpr unsigned long ESR1_STFERR = (1U << 10);            /* Bit 10: Stuffing Error */
constexpr unsigned long ESR1_FRMERR = (1U << 11);            /* Bit 11: Form Error */
constexpr unsigned long ESR1_CRCERR = (1U << 12);            /* Bit 12: Cyclic Redundancy Check Error */
constexpr unsigned long ESR1_ACKERR = (1U << 13);            /* Bit 13: Acknowledge Error */
constexpr unsigned long ESR1_BIT0ERR = (1U << 14);           /* Bit 14: Bit0 Error */
constexpr unsigned long ESR1_BIT1ERR = (1U << 15);           /* Bit 15: Bit1 Error */
constexpr unsigned long ESR1_RWRNINT = (1U << 16);           /* Bit 16: Rx Warning Interrupt Flag */
constexpr unsigned long ESR1_TWRNINT = (1U << 17);           /* Bit 17: Tx Warning Interrupt Flag */
constexpr unsigned long ESR1_SYNCH = (1U << 18);             /* Bit 18: CAN Synchronization Status */
/* Bits 19-31: Reserved */
/* Interrupt Masks 2 Register */

constexpr unsigned long CAN_IMASK2_0 = (1U << 0);            /* Bit 0: Buffer MB0 Mask */
constexpr unsigned long CAN_IMASK2_1 = (1U << 1);            /* Bit 1: Buffer MB1 Mask */
constexpr unsigned long CAN_IMASK2_2 = (1U << 2);            /* Bit 2: Buffer MB2 Mask */
constexpr unsigned long CAN_IMASK2_3 = (1U << 3);            /* Bit 3: Buffer MB3 Mask */
constexpr unsigned long CAN_IMASK2_4 = (1U << 4);            /* Bit 4: Buffer MB4 Mask */
constexpr unsigned long CAN_IMASK2_5 = (1U << 5);            /* Bit 5: Buffer MB5 Mask */
constexpr unsigned long CAN_IMASK2_6 = (1U << 6);            /* Bit 6: Buffer MB6 Mask */
constexpr unsigned long CAN_IMASK2_7 = (1U << 7);            /* Bit 7: Buffer MB7 Mask */
constexpr unsigned long CAN_IMASK2_8 = (1U << 8);            /* Bit 8: Buffer MB8 Mask */
constexpr unsigned long CAN_IMASK2_9 = (1U << 9);            /* Bit 9: Buffer MB9 Mask */
constexpr unsigned long CAN_IMASK2_10 = (1U << 10);            /* Bit 10: Buffer MB10 Mask */
constexpr unsigned long CAN_IMASK2_11 = (1U << 11);            /* Bit 11: Buffer MB11 Mask */
constexpr unsigned long CAN_IMASK2_12 = (1U << 12);            /* Bit 12: Buffer MB12 Mask */
constexpr unsigned long CAN_IMASK2_13 = (1U << 13);            /* Bit 13: Buffer MB13 Mask */
constexpr unsigned long CAN_IMASK2_14 = (1U << 14);            /* Bit 14: Buffer MB14 Mask */
constexpr unsigned long CAN_IMASK2_15 = (1U << 15);            /* Bit 15: Buffer MB15 Mask */
constexpr unsigned long CAN_IMASK2_16 = (1U << 16);            /* Bit 16: Buffer MB16 Mask */
constexpr unsigned long CAN_IMASK2_17 = (1U << 17);            /* Bit 17: Buffer MB17 Mask */
constexpr unsigned long CAN_IMASK2_18 = (1U << 18);            /* Bit 18: Buffer MB18 Mask */
constexpr unsigned long CAN_IMASK2_19 = (1U << 19);            /* Bit 19: Buffer MB19 Mask */
constexpr unsigned long CAN_IMASK2_20 = (1U << 20);            /* Bit 20: Buffer MB20 Mask */
constexpr unsigned long CAN_IMASK2_21 = (1U << 21);            /* Bit 21: Buffer MB21 Mask */
constexpr unsigned long CAN_IMASK2_22 = (1U << 22);            /* Bit 22: Buffer MB22 Mask */
constexpr unsigned long CAN_IMASK2_23 = (1U << 23);            /* Bit 23: Buffer MB23 Mask */
constexpr unsigned long CAN_IMASK2_24 = (1U << 24);            /* Bit 24: Buffer MB24 Mask */
constexpr unsigned long CAN_IMASK2_25 = (1U << 25);            /* Bit 25: Buffer MB25 Mask */
constexpr unsigned long CAN_IMASK2_26 = (1U << 26);            /* Bit 26: Buffer MB26 Mask */
constexpr unsigned long CAN_IMASK2_27 = (1U << 27);            /* Bit 27: Buffer MB27 Mask */
constexpr unsigned long CAN_IMASK2_28 = (1U << 28);            /* Bit 28: Buffer MB28 Mask */
constexpr unsigned long CAN_IMASK2_29 = (1U << 29);            /* Bit 29: Buffer MB29 Mask */
constexpr unsigned long CAN_IMASK2_30 = (1U << 30);            /* Bit 30: Buffer MB30 Mask */
constexpr unsigned long CAN_IMASK2_31 = (1U << 31);            /* Bit 31: Buffer MB31 Mask */

/* Interrupt Masks 1 Register */

constexpr unsigned long CAN_IMASK1_0 = (1U << 0);             /* Bit 0: Buffer MB0 Mask */
constexpr unsigned long CAN_IMASK1_1 = (1U << 1);             /* Bit 1: Buffer MB1 Mask */
constexpr unsigned long CAN_IMASK1_2 = (1U << 2);             /* Bit 2: Buffer MB2 Mask */
constexpr unsigned long CAN_IMASK1_3 = (1U << 3);             /* Bit 3: Buffer MB3 Mask */
constexpr unsigned long CAN_IMASK1_4 = (1U << 4);             /* Bit 4: Buffer MB4 Mask */
constexpr unsigned long CAN_IMASK1_5 = (1U << 5);             /* Bit 5: Buffer MB5 Mask */
constexpr unsigned long CAN_IMASK1_6 = (1U << 6);             /* Bit 6: Buffer MB6 Mask */
constexpr unsigned long CAN_IMASK1_7 = (1U << 7);             /* Bit 7: Buffer MB7 Mask */
constexpr unsigned long CAN_IMASK1_8 = (1U << 8);             /* Bit 8: Buffer MB8 Mask */
constexpr unsigned long CAN_IMASK1_9 = (1U << 9);             /* Bit 9: Buffer MB9 Mask */
constexpr unsigned long CAN_IMASK1_10 = (1U << 10);            /* Bit 10: Buffer MB10 Mask */
constexpr unsigned long CAN_IMASK1_11 = (1U << 11);            /* Bit 11: Buffer MB11 Mask */
constexpr unsigned long CAN_IMASK1_12 = (1U << 12);            /* Bit 12: Buffer MB12 Mask */
constexpr unsigned long CAN_IMASK1_13 = (1U << 13);            /* Bit 13: Buffer MB13 Mask */
constexpr unsigned long CAN_IMASK1_14 = (1U << 14);            /* Bit 14: Buffer MB14 Mask */
constexpr unsigned long CAN_IMASK1_15 = (1U << 15);            /* Bit 15: Buffer MB15 Mask */
constexpr unsigned long CAN_IMASK1_16 = (1U << 16);            /* Bit 16: Buffer MB16 Mask */
constexpr unsigned long CAN_IMASK1_17 = (1U << 17);            /* Bit 17: Buffer MB17 Mask */
constexpr unsigned long CAN_IMASK1_18 = (1U << 18);            /* Bit 18: Buffer MB18 Mask */
constexpr unsigned long CAN_IMASK1_19 = (1U << 19);            /* Bit 19: Buffer MB19 Mask */
constexpr unsigned long CAN_IMASK1_20 = (1U << 20);            /* Bit 20: Buffer MB20 Mask */
constexpr unsigned long CAN_IMASK1_21 = (1U << 21);            /* Bit 21: Buffer MB21 Mask */
constexpr unsigned long CAN_IMASK1_22 = (1U << 22);            /* Bit 22: Buffer MB22 Mask */
constexpr unsigned long CAN_IMASK1_23 = (1U << 23);            /* Bit 23: Buffer MB23 Mask */
constexpr unsigned long CAN_IMASK1_24 = (1U << 24);            /* Bit 24: Buffer MB24 Mask */
constexpr unsigned long CAN_IMASK1_25 = (1U << 25);            /* Bit 25: Buffer MB25 Mask */
constexpr unsigned long CAN_IMASK1_26 = (1U << 26);            /* Bit 26: Buffer MB26 Mask */
constexpr unsigned long CAN_IMASK1_27 = (1U << 27);            /* Bit 27: Buffer MB27 Mask */
constexpr unsigned long CAN_IMASK1_28 = (1U << 28);            /* Bit 28: Buffer MB28 Mask */
constexpr unsigned long CAN_IMASK1_29 = (1U << 29);            /* Bit 29: Buffer MB29 Mask */
constexpr unsigned long CAN_IMASK1_30 = (1U << 30);            /* Bit 30: Buffer MB30 Mask */
constexpr unsigned long CAN_IMASK1_31 = (1U << 31);            /* Bit 31: Buffer MB31 Mask */

/* Interrupt Flags 2 Register */

constexpr unsigned long CAN_IFLAG2_0 = (1U << 0);             /* Bit 0: Buffer MB0 Interrupt */
constexpr unsigned long CAN_IFLAG2_1 = (1U << 1);             /* Bit 1: Buffer MB1 Interrupt */
constexpr unsigned long CAN_IFLAG2_2 = (1U << 2);             /* Bit 2: Buffer MB2 Interrupt */
constexpr unsigned long CAN_IFLAG2_3 = (1U << 3);             /* Bit 3: Buffer MB3 Interrupt */
constexpr unsigned long CAN_IFLAG2_4 = (1U << 4);             /* Bit 4: Buffer MB4 Interrupt */
constexpr unsigned long CAN_IFLAG2_5 = (1U << 5);             /* Bit 5: Buffer MB5 Interrupt */
constexpr unsigned long CAN_IFLAG2_6 = (1U << 6);             /* Bit 6: Buffer MB6 Interrupt */
constexpr unsigned long CAN_IFLAG2_7 = (1U << 7);             /* Bit 7: Buffer MB7 Interrupt */
constexpr unsigned long CAN_IFLAG2_8 = (1U << 8);             /* Bit 8: Buffer MB8 Interrupt */
constexpr unsigned long CAN_IFLAG2_9 = (1U << 9);             /* Bit 9: Buffer MB9 Interrupt */
constexpr unsigned long CAN_IFLAG2_10 = (1U << 10);            /* Bit 10: Buffer MB10 Interrupt */
constexpr unsigned long CAN_IFLAG2_11 = (1U << 11);            /* Bit 11: Buffer MB11 Interrupt */
constexpr unsigned long CAN_IFLAG2_12 = (1U << 12);            /* Bit 12: Buffer MB12 Interrupt */
constexpr unsigned long CAN_IFLAG2_13 = (1U << 13);            /* Bit 13: Buffer MB13 Interrupt */
constexpr unsigned long CAN_IFLAG2_14 = (1U << 14);            /* Bit 14: Buffer MB14 Interrupt */
constexpr unsigned long CAN_IFLAG2_15 = (1U << 15);            /* Bit 15: Buffer MB15 Interrupt */
constexpr unsigned long CAN_IFLAG2_16 = (1U << 16);            /* Bit 16: Buffer MB16 Interrupt */
constexpr unsigned long CAN_IFLAG2_17 = (1U << 17);            /* Bit 17: Buffer MB17 Interrupt */
constexpr unsigned long CAN_IFLAG2_18 = (1U << 18);            /* Bit 18: Buffer MB18 Interrupt */
constexpr unsigned long CAN_IFLAG2_19 = (1U << 19);            /* Bit 19: Buffer MB19 Interrupt */
constexpr unsigned long CAN_IFLAG2_20 = (1U << 20);            /* Bit 20: Buffer MB20 Interrupt */
constexpr unsigned long CAN_IFLAG2_21 = (1U << 21);            /* Bit 21: Buffer MB21 Interrupt */
constexpr unsigned long CAN_IFLAG2_22 = (1U << 22);            /* Bit 22: Buffer MB22 Interrupt */
constexpr unsigned long CAN_IFLAG2_23 = (1U << 23);            /* Bit 23: Buffer MB23 Interrupt */
constexpr unsigned long CAN_IFLAG2_24 = (1U << 24);            /* Bit 24: Buffer MB24 Interrupt */
constexpr unsigned long CAN_IFLAG2_25 = (1U << 25);            /* Bit 25: Buffer MB25 Interrupt */
constexpr unsigned long CAN_IFLAG2_26 = (1U << 26);            /* Bit 26: Buffer MB26 Interrupt */
constexpr unsigned long CAN_IFLAG2_27 = (1U << 27);            /* Bit 27: Buffer MB27 Interrupt */
constexpr unsigned long CAN_IFLAG2_28 = (1U << 28);            /* Bit 28: Buffer MB28 Interrupt */
constexpr unsigned long CAN_IFLAG2_29 = (1U << 29);            /* Bit 29: Buffer MB29 Interrupt */
constexpr unsigned long CAN_IFLAG2_30 = (1U << 30);            /* Bit 30: Buffer MB30 Interrupt */
constexpr unsigned long CAN_IFLAG2_31 = (1U << 31);            /* Bit 31: Buffer MB31 Interrupt */

/* Interrupt Flags 1 Register */

constexpr unsigned long CAN_IFLAG1_0 = (1U << 0);             /* Bit 0: Buffer MB0 Interrupt */
constexpr unsigned long CAN_IFLAG1_1 = (1U << 1);             /* Bit 1: Buffer MB1 Interrupt */
constexpr unsigned long CAN_IFLAG1_2 = (1U << 2);             /* Bit 2: Buffer MB2 Interrupt */
constexpr unsigned long CAN_IFLAG1_3 = (1U << 3);             /* Bit 3: Buffer MB3 Interrupt */
constexpr unsigned long CAN_IFLAG1_4 = (1U << 4);             /* Bit 4: Buffer MB4 Interrupt */
constexpr unsigned long CAN_IFLAG1_5 = (1U << 5);             /* Bit 5: Buffer MB5 Interrupt */
constexpr unsigned long CAN_FIFO_NE =              CAN_IFLAG1_5; /* Bit 5: Fifo almost Not empty Interrupt */
constexpr unsigned long CAN_IFLAG1_6 = (1U << 6);             /* Bit 6: Buffer MB6 Interrupt */
constexpr unsigned long CAN_FIFO_WARN =            CAN_IFLAG1_6; /* Bit 6: Fifo almost full Interrupt */
constexpr unsigned long CAN_IFLAG1_7 = (1U << 7);             /* Bit 7: Buffer MB7 Interrupt */
constexpr unsigned long CAN_FIFO_OV =              CAN_IFLAG1_7; /* Bit 7: Fifo Overflowed Interrupt */
constexpr unsigned long CAN_IFLAG1_8 = (1U << 8);             /* Bit 8: Buffer MB8 Interrupt */
constexpr unsigned long CAN_IFLAG1_9 = (1U << 9);             /* Bit 9: Buffer MB9 Interrupt */
constexpr unsigned long CAN_IFLAG1_10 = (1U << 10);            /* Bit 10: Buffer MB10 Interrupt */
constexpr unsigned long CAN_IFLAG1_11 = (1U << 11);            /* Bit 11: Buffer MB11 Interrupt */
constexpr unsigned long CAN_IFLAG1_12 = (1U << 12);            /* Bit 12: Buffer MB12 Interrupt */
constexpr unsigned long CAN_IFLAG1_13 = (1U << 13);            /* Bit 13: Buffer MB13 Interrupt */
constexpr unsigned long CAN_IFLAG1_14 = (1U << 14);            /* Bit 14: Buffer MB14 Interrupt */
constexpr unsigned long CAN_IFLAG1_15 = (1U << 15);            /* Bit 15: Buffer MB15 Interrupt */
constexpr unsigned long CAN_IFLAG1_16 = (1U << 16);            /* Bit 16: Buffer MB16 Interrupt */
constexpr unsigned long CAN_IFLAG1_17 = (1U << 17);            /* Bit 17: Buffer MB17 Interrupt */
constexpr unsigned long CAN_IFLAG1_18 = (1U << 18);            /* Bit 18: Buffer MB18 Interrupt */
constexpr unsigned long CAN_IFLAG1_19 = (1U << 19);            /* Bit 19: Buffer MB19 Interrupt */
constexpr unsigned long CAN_IFLAG1_20 = (1U << 20);            /* Bit 20: Buffer MB20 Interrupt */
constexpr unsigned long CAN_IFLAG1_21 = (1U << 21);            /* Bit 21: Buffer MB21 Interrupt */
constexpr unsigned long CAN_IFLAG1_22 = (1U << 22);            /* Bit 22: Buffer MB22 Interrupt */
constexpr unsigned long CAN_IFLAG1_23 = (1U << 23);            /* Bit 23: Buffer MB23 Interrupt */
constexpr unsigned long CAN_IFLAG1_24 = (1U << 24);            /* Bit 24: Buffer MB24 Interrupt */
constexpr unsigned long CAN_IFLAG1_25 = (1U << 25);            /* Bit 25: Buffer MB25 Interrupt */
constexpr unsigned long CAN_IFLAG1_26 = (1U << 26);            /* Bit 26: Buffer MB26 Interrupt */
constexpr unsigned long CAN_IFLAG1_27 = (1U << 27);            /* Bit 27: Buffer MB27 Interrupt */
constexpr unsigned long CAN_IFLAG1_28 = (1U << 28);            /* Bit 28: Buffer MB28 Interrupt */
constexpr unsigned long CAN_IFLAG1_29 = (1U << 29);            /* Bit 29: Buffer MB29 Interrupt */
constexpr unsigned long CAN_IFLAG1_30 = (1U << 30);            /* Bit 30: Buffer MB30 Interrupt */
constexpr unsigned long CAN_IFLAG1_31 = (1U << 31);            /* Bit 31: Buffer MB31 Interrupt */

/* Control 2 Register */
/* Bits 0-15: Reserved */
constexpr unsigned long CTRL2_EACEN = (1U << 16); /* Bit 16:  Entire Frame Arbitration Field Comparison
                                                                  Enable (Rx) */
constexpr unsigned long CTRL2_RRS = (1U << 17);                /* Bit 17:  Remote Request Storing */
constexpr unsigned long CTRL2_MRP = (1U << 18);                /* Bit 18:  Mailboxes Reception Priority */
constexpr unsigned long CTRL2_TASD_SHIFT = (19);               /* Bits 19-23: Tx Arbitration Start Delay */
constexpr unsigned long CTRL2_TASD_MASK = (31U << CTRL2_TASD_SHIFT);
constexpr unsigned long CTRL2_RFFN_SHIFT = (24);               /* Bits 24-27: Number of Rx FIFO Filters */
constexpr unsigned long CTRL2_RFFN_MASK(15U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_8MB(0U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_16MB(1U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_24MB(2U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_32MB(3U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_40MB(4U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_48MB(5U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_56MB(6U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_64MB(7U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_72MB(8U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_80MB(9U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_88MB(10U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_96MB(11U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_104MB(12U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_112MB(13U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_120MB(14U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_RFFN_128MB(15U << CTRL2_RFFN_SHIFT);
constexpr unsigned long CTRL2_WRMFRZ = (1U << 28U);             /* Bit 28: Write-Access to Memory in Freeze mode */
/* Bits 29-31: Reserved */

/* Error and Status 2 Register */
/* Bits 0-12: Reserved */
constexpr unsigned long ESR2_IMB = (1U << 13);                  /* Bit 13: Inactive Mailbox */
constexpr unsigned long ESR2_VPS = (1U << 14);                  /* Bit 14: Valid Priority Status */
/* Bit 15: Reserved */
constexpr unsigned long ESR2_LPTM_SHIFT = (16);                 /* Bits 16-22: Lowest Priority Tx Mailbox */
constexpr unsigned long ESR2_LPTM_MASK = (0x7fU << ESR2_LPTM_SHIFT);
/* Bits 23-31: Reserved */
/* CRC Register */

constexpr unsigned long CRCR_TXCRC_SHIFT = (0U);         /* Bits 0-14: CRC Transmitted */
constexpr unsigned long CRCR_TXCRC_MASK = (0x7fffU << CRCR_TXCRC_SHIFT); /* Rx FIFO Global Mask Register (32 Rx
                                                                                     FIFO Global Mask Bits) */
/* Bits 23-31: Reserved */
constexpr unsigned long CRCR_MBCRC_SHIFT = (16);                 /* Bits 16-22: CRC Mailbox */
constexpr unsigned long CRCR_MBCRC_MASK = (0x7fU << CRCR_MBCRC_SHIFT);
/* Bit 15: Reserved */

/* Rx FIFO Information Register */
/* Bits 9-31: Reserved */
constexpr unsigned long RXFIR_IDHIT_SHIFT = (0);         /* Bits 0-8: Identifier Acceptance Filter Hit Indicator
                                                                  */
constexpr unsigned long RXFIR_IDHIT_MASK = (0x1ffU << RXFIR_IDHIT_SHIFT);

/* Rn Individual Mask Registers */

constexpr unsigned long RXIMR0 = (1U << 0);             /* Bit 0: Individual Mask Bits */
constexpr unsigned long RXIMR1 = (1U << 1);             /* Bit 1: Individual Mask Bits */
constexpr unsigned long RXIMR2 = (1U << 2);             /* Bit 2: Individual Mask Bits */
constexpr unsigned long RXIMR3 = (1U << 3);             /* Bit 3: Individual Mask Bits */
constexpr unsigned long RXIMR4 = (1U << 4);             /* Bit 4: Individual Mask Bits */
constexpr unsigned long RXIMR5 = (1U << 5);             /* Bit 5: Individual Mask Bits */
constexpr unsigned long RXIMR6 = (1U << 6);             /* Bit 6: Individual Mask Bits */
constexpr unsigned long RXIMR7 = (1U << 7);             /* Bit 7: Individual Mask Bits */
constexpr unsigned long RXIMR8 = (1U << 8);             /* Bit 8: Individual Mask Bits */
constexpr unsigned long RXIMR9 = (1U << 9);             /* Bit 9: Individual Mask Bits */
constexpr unsigned long RXIMR10 = (1U << 10);            /* Bit 10: Individual Mask Bits */
constexpr unsigned long RXIMR11 = (1U << 11);            /* Bit 11: Individual Mask Bits */
constexpr unsigned long RXIMR12 = (1U << 12);            /* Bit 12: Individual Mask Bits */
constexpr unsigned long RXIMR13 = (1U << 13);            /* Bit 13: Individual Mask Bits */
constexpr unsigned long RXIMR14 = (1U << 14);            /* Bit 14: Individual Mask Bits */
constexpr unsigned long RXIMR15 = (1U << 15);            /* Bit 15: Individual Mask Bits */
constexpr unsigned long RXIMR16 = (1U << 16);            /* Bit 16: Individual Mask Bits */
constexpr unsigned long RXIMR17 = (1U << 17);            /* Bit 17: Individual Mask Bits */
constexpr unsigned long RXIMR18 = (1U << 18);            /* Bit 18: Individual Mask Bits */
constexpr unsigned long RXIMR19 = (1U << 19);            /* Bit 19: Individual Mask Bits */
constexpr unsigned long RXIMR20 = (1U << 20);            /* Bit 20: Individual Mask Bits */
constexpr unsigned long RXIMR21 = (1U << 21);            /* Bit 21: Individual Mask Bits */
constexpr unsigned long RXIMR22 = (1U << 22);            /* Bit 22: Individual Mask Bits */
constexpr unsigned long RXIMR23 = (1U << 23);            /* Bit 23: Individual Mask Bits */
constexpr unsigned long RXIMR24 = (1U << 24);            /* Bit 24: Individual Mask Bits */
constexpr unsigned long RXIMR25 = (1U << 25);            /* Bit 25: Individual Mask Bits */
constexpr unsigned long RXIMR26 = (1U << 26);            /* Bit 26: Individual Mask Bits */
constexpr unsigned long RXIMR27 = (1U << 27);            /* Bit 27: Individual Mask Bits */
constexpr unsigned long RXIMR28 = (1U << 28);            /* Bit 28: Individual Mask Bits */
constexpr unsigned long RXIMR29 = (1U << 29);            /* Bit 29: Individual Mask Bits */
constexpr unsigned long RXIMR30 = (1U << 30);            /* Bit 30: Individual Mask Bits */
constexpr unsigned long RXIMR31 = (1U << 31);            /* Bit 31: Individual Mask Bits */
}
}

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
# undef constexpr
#endif
