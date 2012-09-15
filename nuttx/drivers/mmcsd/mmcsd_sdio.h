/********************************************************************************************
 * drivers/mmcsd/mmcsd_sdio.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __DRIVERS_MMCSD_MMCSD_SDIO_H
#define __DRIVERS_MMCSD_MMCSD_SDIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/* CMD8 Argument:
 *    [31:12]: Reserved (shall be set to '0')
 *    [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
 *    [7:0]: Check Pattern (recommended 0xaa)
 * CMD8 Response: R7
 */

#define MMCSD_CMD8VOLTAGE_SHIFT     (8)                    /* Bits 8-11: Supply voltage */
#define MMCSD_CMD8VOLTAGE_MASK      ((uint32_t)0x0f << MMCSD_CMD8VOLTAGE_SHIFT)
#  define MMCSD_CMD8VOLTAGE_27      ((uint32_t)0x01 << MMCSD_CMD8VOLTAGE_SHIFT) /* 2.7-3.6V */
#define MMCSD_CMD8ECHO_SHIFT        (0)                    /* Bits 0-7: Check pattern */
#define MMCSD_CMD8ECHO_MASK         ((uint32_t)0xff << MMCSD_CMD8ECHO_SHIFT)
#  define MMCSD_CMD8CHECKPATTERN    ((uint32_t)0xaa << MMCSD_CMD8ECHO_SHIFT)

/* ACMD6 argument */

#define MMCSD_ACMD6_BUSWIDTH_1      ((uint32_t)0)          /* Bus width = 1-bit */
#define MMCSD_ACMD6_BUSWIDTH_4      ((uint32_t)2)          /* Bus width = 4-bit */

/* ACMD41 argument */

#define MMCSD_ACMD41_VOLTAGEWINDOW  ((uint32_t)0x80100000)
#define MMCSD_ACMD41_HIGHCAPACITY   ((uint32_t)1 << 30)
#define MMCSD_ACMD41_STDCAPACITY    ((uint32_t)0)

/* ACMD42 argument */

#define MMCSD_ACMD42_CD_DISCONNECT  ((uint32_t)0)          /* Disconnect card detection logic */
#define MMCSD_ACMD42_CD_CONNECT     ((uint32_t)1)          /* Connect card detection logic */

/* R1 Card Status bit definitions */

#define MMCSD_R1_OUTOFRANGE         ((uint32_t)1 << 31)    /* Bad argument */
#define MMCSD_R1_ADDRESSERROR       ((uint32_t)1 << 30)    /* Bad address */
#define MMCSD_R1_BLOCKLENERROR      ((uint32_t)1 << 29)    /* Bad block length */
#define MMCSD_R1_ERASESEQERROR      ((uint32_t)1 << 28)    /* Erase cmd error */
#define MMCSD_R1_ERASEPARAM         ((uint32_t)1 << 27)    /* Bad write blocks */
#define MMCSD_R1_WPVIOLATION        ((uint32_t)1 << 26)    /* Erase access failure */
#define MMCSD_R1_CARDISLOCKED       ((uint32_t)1 << 25)    /* Card is locked */
#define MMCSD_R1_LOCKUNLOCKFAILED   ((uint32_t)1 << 24)    /* Password error */
#define MMCSD_R1_COMCRCERROR        ((uint32_t)1 << 23)    /* CRC error */
#define MMCSD_R1_ILLEGALCOMMAND     ((uint32_t)1 << 22)    /* Bad command */
#define MMCSD_R1_CARDECCFAILED      ((uint32_t)1 << 21)    /* Failed to correct data */
#define MMCSD_R1_CCERROR            ((uint32_t)1 << 20)    /* Card controller error */
#define MMCSD_R1_ERROR              ((uint32_t)1 << 19)    /* General error */
#define MMCSD_R1_UNDERRUN           ((uint32_t)1 << 18)    /* Underrun (MMC only) */
#define MMCSD_R1_OVERRRUN           ((uint32_t)1 << 17)    /* Overrun (MMC only) */
#define MMCSD_R1_CIDCSDOVERWRITE    ((uint32_t)1 << 16)    /* CID/CSD error */
#define MMCSD_R1_WPERASESKIP        ((uint32_t)1 << 15)    /* Not all erased */
#define MMCSD_R1_CARDECCDISABLED    ((uint32_t)1 << 14)    /* Internal ECC not used */
#define MMCSD_R1_ERASERESET         ((uint32_t)1 << 13)    /* Reset sequence cleared */
#define MMCSD_R1_STATE_SHIFT        (9)                    /* Current card state */
#define MMCSD_R1_STATE_MASK         ((uint32_t)15 << MMCSD_R1_STATE_SHIFT)
                                                           /* Card identification mode states */
#  define MMCSD_R1_STATE_IDLE       ((uint32_t)0 << MMCSD_R1_STATE_SHIFT) /* 0=Idle state */
#  define MMCSD_R1_STATE_READY      ((uint32_t)1 << MMCSD_R1_STATE_SHIFT) /* 1=Ready state */
#  define MMCSD_R1_STATE_IDENT      ((uint32_t)2 << MMCSD_R1_STATE_SHIFT) /* 2=Identification state */
                                                           /* Data transfer states */
#  define MMCSD_R1_STATE_STBY       ((uint32_t)3 << MMCSD_R1_STATE_SHIFT) /* 3=Standby state */
#  define MMCSD_R1_STATE_TRAN       ((uint32_t)4 << MMCSD_R1_STATE_SHIFT) /* 4=Transfer state */
#  define MMCSD_R1_STATE_DATA       ((uint32_t)5 << MMCSD_R1_STATE_SHIFT) /* 5=Sending data state */
#  define MMCSD_R1_STATE_RCV        ((uint32_t)6 << MMCSD_R1_STATE_SHIFT) /* 6=Receiving data state */
#  define MMCSD_R1_STATE_PRG        ((uint32_t)7 << MMCSD_R1_STATE_SHIFT) /* 7=Programming state */
#  define MMCSD_R1_STATE_DIS        ((uint32_t)8 << MMCSD_R1_STATE_SHIFT) /* 8=Disconnect state */
#define MMCSD_R1_READYFORDATA       ((uint32_t)1 << 8)     /* Buffer empty */
#define MMCSD_R1_APPCMD             ((uint32_t)1 << 5)     /* Next CMD is ACMD */
#define MMCSD_R1_AKESEQERROR        ((uint32_t)1 << 3)     /* Authentication error */
#define MMCSD_R1_ERRORMASK          ((uint32_t)0xfdffe008) /* Error mask */

#define IS_STATE(v,s)               ((((uint32_t)v)&MMCSD_R1_STATE_MASK)==(s))

/* R3 (OCR) */

#define MMC_VDD_20_36               ((uint32_t)0x00ffff00) /* VDD voltage 2.0-3.6 */

#define MMCSD_VDD_145_150           ((uint32_t)1 << 0)     /* VDD voltage 1.45 - 1.50 */
#define MMCSD_VDD_150_155           ((uint32_t)1 << 1)     /* VDD voltage 1.50 - 1.55 */
#define MMCSD_VDD_155_160           ((uint32_t)1 << 2)     /* VDD voltage 1.55 - 1.60 */
#define MMCSD_VDD_160_165           ((uint32_t)1 << 3)     /* VDD voltage 1.60 - 1.65 */
#define MMCSD_VDD_165_170           ((uint32_t)1 << 4)     /* VDD voltage 1.65 - 1.70 */
#define MMCSD_VDD_17_18             ((uint32_t)1 << 5)     /* VDD voltage 1.7 - 1.8 */
#define MMCSD_VDD_18_19             ((uint32_t)1 << 6)     /* VDD voltage 1.8 - 1.9 */
#define MMCSD_VDD_19_20             ((uint32_t)1 << 7)     /* VDD voltage 1.9 - 2.0 */
#define MMCSD_VDD_20_21             ((uint32_t)1 << 8)     /* VDD voltage 2.0-2.1 */
#define MMCSD_VDD_21_22             ((uint32_t)1 << 9)     /* VDD voltage 2.1-2.2 */
#define MMCSD_VDD_22_23             ((uint32_t)1 << 10)    /* VDD voltage 2.2-2.3 */
#define MMCSD_VDD_23_24             ((uint32_t)1 << 11)    /* VDD voltage 2.3-2.4 */
#define MMCSD_VDD_24_25             ((uint32_t)1 << 12)    /* VDD voltage 2.4-2.5 */
#define MMCSD_VDD_25_26             ((uint32_t)1 << 13)    /* VDD voltage 2.5-2.6 */
#define MMCSD_VDD_26_27             ((uint32_t)1 << 14)    /* VDD voltage 2.6-2.7 */
#define MMCSD_VDD_27_28             ((uint32_t)1 << 15)    /* VDD voltage 2.7-2.8 */
#define MMCSD_VDD_28_29             ((uint32_t)1 << 16)    /* VDD voltage 2.8-2.9 */
#define MMCSD_VDD_29_30             ((uint32_t)1 << 17)    /* VDD voltage 2.9-3.0 */
#define MMCSD_VDD_30_31             ((uint32_t)1 << 18)    /* VDD voltage 3.0-3.1 */
#define MMCSD_VDD_31_32             ((uint32_t)1 << 19)    /* VDD voltage 3.1-3.2 */
#define MMCSD_VDD_32_33             ((uint32_t)1 << 20)    /* VDD voltage 3.2-3.3 */
#define MMCSD_VDD_33_34             ((uint32_t)1 << 21)    /* VDD voltage 3.3-3.4 */
#define MMCSD_VDD_34_35             ((uint32_t)1 << 22)    /* VDD voltage 3.4-3.5 */
#define MMCSD_VDD_35_36             ((uint32_t)1 << 23)    /* VDD voltage 3.5-3.6 */
#define MMCSD_R3_HIGHCAPACITY       ((uint32_t)1 << 30)    /* true: Card supports block addressing */
#define MMCSD_CARD_BUSY             ((uint32_t)1 << 31)    /* Card power-up busy bit */

/* R6 Card Status bit definitions */

#define MMCSD_R6_RCA_SHIFT          (16)                   /* New published RCA */
#define MMCSD_R6_RCA_MASK           ((uint32_t)0xffff << MMCSD_R6_RCA_SHIFT)
#define MMCSD_R6_COMCRCERROR        ((uint32_t)1 << 15)    /* CRC error */
#define MMCSD_R6_ILLEGALCOMMAND     ((uint32_t)1 << 14)    /* Bad command */
#define MMCSD_R6_ERROR              ((uint32_t)1 << 13)    /* General error */
#define MMCSD_R6_STATE_SHIFT        (9)                    /* Current card state */
#define MMCSD_R6_STATE_MASK         ((uint32_t)15 << MMCSD_R6_STATE_SHIFT)
                                                           /* Card identification mode states */
#  define MMCSD_R6_STATE_IDLE       ((uint32_t)0 << MMCSD_R6_STATE_SHIFT) /* 0=Idle state */
#  define MMCSD_R6_STATE_READY      ((uint32_t)1 << MMCSD_R6_STATE_SHIFT) /* 1=Ready state */
#  define MMCSD_R6_STATE_IDENT      ((uint32_t)2 << MMCSD_R6_STATE_SHIFT) /* 2=Identification state */
                                                           /* Data transfer states */
#  define MMCSD_R6_STATE_STBY       ((uint32_t)3 << MMCSD_R6_STATE_SHIFT) /* 3=Standby state */
#  define MMCSD_R6_STATE_TRAN       ((uint32_t)4 << MMCSD_R6_STATE_SHIFT) /* 4=Transfer state */
#  define MMCSD_R6_STATE_DATA       (5(uint32_t) << MMCSD_R6_STATE_SHIFT) /* 5=Sending data state */
#  define MMCSD_R6_STATE_RCV        ((uint32_t)6 << MMCSD_R6_STATE_SHIFT) /* 6=Receiving data state */
#  define MMCSD_R6_STATE_PRG        ((uint32_t)7 << MMCSD_R6_STATE_SHIFT) /* 7=Programming state */
#  define MMCSD_R6_STATE_DIS        ((uint32_t) << MMCSD_R6_STATE_SHIFT) /* 8=Disconnect state */
#define MMCSD_R6_ERRORMASK          ((uint32_t)0x0000e000)  /* Error mask */

/* SD Configuration Register (SCR) encoding */

#define MMCSD_SCR_BUSWIDTH_1BIT     (1)
#define MMCSD_SCR_BUSWIDTH_2BIT     (2)
#define MMCSD_SCR_BUSWIDTH_4BIT     (4)
#define MMCSD_SCR_BUSWIDTH_8BIT     (8)

/* Last 4 bytes of the 48-bit R7 response */

#define MMCSD_R7VERSION_SHIFT       (28)                   /* Bits 28-31: Command version number */
#define MMCSD_R7VERSION_MASK        ((uint32_t)0x0f << MMCSD_R7VERSION_SHIFT)
#define MMCSD_R7VOLTAGE_SHIFT       (8)                    /* Bits 8-11: Voltage accepted */
#define MMCSD_R7VOLTAGE_MASK        ((uint32_t)0x0f << MMCSD_R7VOLTAGE_SHIFT)
#  define MMCSD_R7VOLTAGE_27        ((uint32_t)0x01 << MMCSD_R7VOLTAGE_SHIFT) /* 2.7-3.6V */
#define MMCSD_R7ECHO_SHIFT          (0)                    /* Bits 0-7: Echoed check pattern */
#define MMCSD_R7ECHO_MASK           ((uint32_t)0xff << MMCSD_R7ECHO_SHIFT)
#  define MMCSD_R7CHECKPATTERN      ((uint32_t)0xaa << MMCSD_R7ECHO_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Decoded Card Identification (CID) register */

struct mmcsd_cid_s
{
  uint8_t  mid;     /* 127:120  8-bit Manufacturer ID */
  uint16_t oid;     /* 119:104 16-bit OEM/Application ID (ascii) */
  uint8_t  pnm[6];  /* 103:64  40-bit Product Name (ascii) + null terminator */
  uint8_t  prv;     /*  63:56   8-bit Product revision */
  uint32_t psn;     /*  55:24  32-bit Product serial number */
                    /*  23:20   4-bit (reserved) */
  uint16_t mdt;     /*  19:8   12-bit Manufacturing date */
  uint8_t  crc;     /*   7:1    7-bit CRC7 */
                    /*   0:0    1-bit (not used) */
};

/* Decoded Card Specific Data (CSD) register */

struct mmcsd_csd_s
{
  uint8_t csdstructure;        /* 127:126 CSD structure */
  uint8_t mmcspecvers;         /* 125:122 MMC Spec version (MMC only) */

  struct
  {
    uint8_t timeunit;          /*   2:0   Time exponent */
    uint8_t timevalue;         /*   6:3   Time mantissa */
  } taac;                      /* 119:112 Data read access-time-1 */

  uint8_t nsac;                /* 111:104 Data read access-time-2 in CLK cycle(NSAC*100) */

  struct
  {
    uint8_t transferrateunit;  /*   2:0   Rate exponent */
    uint8_t timevalue;         /*   6:3   Rate mantissa */
  } transpeed;                 /* 103:96  Max. data transfer rate */

  uint16_t ccc;                /*  95:84  Card command classes */
  uint8_t readbllen;           /*  83:80  Max. read data block length */
  uint8_t readblpartial;       /*  79:79  Partial blocks for read allowed */
  uint8_t writeblkmisalign;    /*  78:78  Write block misalignment */
  uint8_t readblkmisalign;     /*  77:77  Read block misalignment */
  uint8_t dsrimp;              /*  76:76  DSR implemented */

  union
  {
#ifdef CONFIG_MMCSD_MMCSUPPORT
    struct
    {
      uint16_t csize;          /*  73:62  Device size */
      uint8_t vddrcurrmin;     /*  61:59  Max. read current at Vdd min */
      uint8_t vddrcurrmax;     /*  58:56  Max. read current at Vdd max */
      uint8_t vddwcurrmin;     /*  55:53  Max. write current at Vdd min */
      uint8_t vddwcurrmax;     /*  52:50  Max. write current at Vdd max */
      uint8_t csizemult;       /*  49:47  Device size multiplier */

      union
      {
        struct                 /* MMC system specification version 3.1 */
        {
          uint8_t ergrpsize;   /*  46:42  Erase group size (MMC 3.1) */
          uint8_t ergrpmult;   /*  41:37  Erase group multiplier (MMC 3.1) */
        } mmc31;
        struct                 /* MMC system specification version 2.2 */
        {
          uint8_t sectorsize;  /*  46:42  Erase sector size (MMC 2.2) */
          uint8_t ergrpsize;   /*  41:37  Erase group size (MMC 2.2) */
        } mmc22;
      } er;

      uint8_t mmcwpgrpsize;    /*  36:32  Write protect group size (MMC) */
    } mmc;
#endif
    struct
    {
      uint16_t csize;          /*  73:62  Device size */
      uint8_t vddrcurrmin;     /*  61:59  Max. read current at Vdd min */
      uint8_t vddrcurrmax;     /*  58:56  Max. read current at Vdd max */
      uint8_t vddwcurrmin;     /*  55:53  Max. write current at Vdd min */
      uint8_t vddwcurrmax;     /*  52:50  Max. write current at Vdd max */
      uint8_t csizemult;       /*  49:47  Device size multiplier */
      uint8_t sderblen;        /*  46:46  Erase single block enable (SD) */
      uint8_t sdsectorsize;    /*  45:39  Erase sector size (SD) */
      uint8_t sdwpgrpsize;     /*  38:32  Write protect group size (SD) */
    } sdbyte;

    struct
    {
                               /*  73:70  (reserved) */
      uint32_t csize;          /*  69:48  Device size */
                               /*  47:47  (reserved) */
      uint8_t sderblen;        /*  46:46  Erase single block enable (SD) */
      uint8_t sdsectorsize;    /*  45:39  Erase sector size (SD) */
      uint8_t sdwpgrpsize;     /*  38:32  Write protect group size (SD) */
    } sdblock;
  } u;

  uint8_t wpgrpen;             /*  31:31  Write protect group enable */
  uint8_t mmcdfltecc;          /*  30:29  Manufacturer default ECC (MMC) */
  uint8_t r2wfactor;           /*  28:26  Write speed factor */
  uint8_t writebllen;          /*  25:22  Max. write data block length */
  uint8_t writeblpartial;      /*  21:21  Partial blocks for write allowed */
  uint8_t fileformatgrp;       /*  15:15  File format group */
  uint8_t copy;                /*  14:14  Copy flag (OTP) */
  uint8_t permwriteprotect;    /*  13:13  Permanent write protection */
  uint8_t tmpwriteprotect;     /*  12:12  Temporary write protection */
  uint8_t fileformat;          /*  10:11  File format */
  uint8_t mmcecc;              /*   9:8   ECC (MMC) */
  uint8_t crc;                 /*   7:1   CRC */
                               /*   0:0   Not used */
};

struct mmcsd_scr_s
{
  uint8_t  scrversion;         /* 63:60 Version of SCR structure */
  uint8_t  sdversion;          /* 59:56 SD memory card physical layer version */
  uint8_t  erasestate;         /* 55:55 Data state after erase (1 or 0) */
  uint8_t  security;           /* 54:52 SD security support */
  uint8_t  buswidth;           /* 51:48 DAT bus widthes supported */
                               /* 47:32 SD reserved space */
  uint32_t mfgdata;            /* 31:0  Reserved for manufacturing data */
};

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_SDIO_H */
