/****************************************************************************
 * drivers/usbhost/rtl8187.h
 *
 * This file is part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Rafael Noronha. All rights reserved.
 *   Authors: Gregoyr Nutt <gnutt@nuttx.org>
 *            Rafael Noronha <rafael@pdsolucoes.com.br>
 *
 * Portions of the logic in this file derives from the KisMAC RTL8187x driver
 *
 *    Created by pr0gg3d on 02/24/08.
 *
 * Which, in turn, came frm the SourceForge rt2x00 project:
 *
 *   Copyright (C) 2004 - 2006 rt2x00 SourceForge Project
 *   <http://rt2x00.serialmonkey.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * There are probably also pieces from the Linux RTL8187x driver
 * 
 *   Copyright 2007 Michael Wu <flamingice@sourmilk.net>
 *   Copyright 2007 Andrea Merello <andreamrl@tiscali.it>
 *
 *   Based on the r8187 driver, which is:
 *   Copyright 2004-2005 Andrea Merello <andreamrl@tiscali.it>, et al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 ****************************************************************************/

#ifndef __DRIVERS_NET_RTL8187X_H
#define __DRIVERS_NET_RTL8187X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CSR Bit Field Definitions ************************************************/

/* Refers to "cmd" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CMD_TXENABLE           (1 << 2)
#define RTL8187X_CMD_RXENABLE           (1 << 3)
#define RTL8187X_CMD_RESET              (1 << 4)

/* Refers to "status" field of "rtl8187x_csr_s" struct */

#define RTL8187X_INT_RXOK               (1 << 0)
#define RTL8187X_INT_RXERR              (1 << 1)
#define RTL8187X_INT_TXLOK              (1 << 2)
#define RTL8187X_INT_TXLERR             (1 << 3)
#define RTL8187X_INT_RXDU               (1 << 4)
#define RTL8187X_INT_RXFO               (1 << 5)
#define RTL8187X_INT_TXNOK              (1 << 6)
#define RTL8187X_INT_TXNERR             (1 << 7)
#define RTL8187X_INT_TXHOK              (1 << 8)
#define RTL8187X_INT_TXHERR             (1 << 9)
#define RTL8187X_INT_TXBOK              (1 << 10)
#define RTL8187X_INT_TXBERR             (1 << 11)
#define RTL8187X_INT_ATIM               (1 << 12)
#define RTL8187X_INT_BEACON             (1 << 13)
#define RTL8187X_INT_TIMEOUT            (1 << 14)
#define RTL8187X_INT_TXFO               (1 << 15)

/* Refers to "tx_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_TXCONF_LOOPBACKMAC     (1 << 17)
#define RTL8187X_TXCONF_LOOPBACKCONT    (3 << 17)
#define RTL8187X_TXCONF_NOICV           (1 << 19)
#define RTL8187X_TXCONF_DISCW           (1 << 20)
#define RTL8187X_TXCONF_SATHWPLCP       (1 << 24)
#define RTL8187X_TXCONF_R8180ABCD       (2 << 25)
#define RTL8187X_TXCONF_R8180F          (3 << 25)
#define RTL8187X_TXCONF_R8185ABC        (4 << 25)
#define RTL8187X_TXCONF_R8185D          (5 << 25)
#define RTL8187X_TXCONF_R8187VD         (5 << 25)
#define RTL8187X_TXCONF_R8187VDB        (6 << 25)
#define RTL8187X_TXCONF_HWVERMASK       (7 << 25)
#define RTL8187X_TXCONF_DISREQQSIZE     (1 << 28)
#define RTL8187X_TXCONF_PROBEDTS        (1 << 29)
#define RTL8187X_TXCONF_HWSEQNUM        (1 << 30)
#define RTL8187X_TXCONF_CWMIN           (1 << 31)

/* Refers to "rx_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_RXCONF_MONITOR         (1 << 0)
#define RTL8187X_RXCONF_NICMAC          (1 << 1)
#define RTL8187X_RXCONF_MULTICAST       (1 << 2)
#define RTL8187X_RXCONF_BROADCAST       (1 << 3)
#define RTL8187X_RXCONF_FCS             (1 << 5)
#define RTL8187X_RXCONF_DATA            (1 << 18)
#define RTL8187X_RXCONF_CTRL            (1 << 19)
#define RTL8187X_RXCONF_MGMT            (1 << 20)
#define RTL8187X_RXCONF_ADDR3           (1 << 21)
#define RTL8187X_RXCONF_PM              (1 << 22)
#define RTL8187X_RXCONF_BSSID           (1 << 23)
#define RTL8187X_RXCONF_RXAUTORESETPHY  (1 << 28)
#define RTL8187X_RXCONF_CSDM1           (1 << 29)
#define RTL8187X_RXCONF_CSDM2           (1 << 30)
#define RTL8187X_RXCONF_ONLYERLPKT      (1 << 31)

/* Refers to "eeprom_cmd" field of "rtl8187x_csr_s" struct */

#define RTL8187X_EEPROMCMD_READ         (1 << 0)
#define RTL8187X_EEPROMCMD_WRITE        (1 << 1)
#define RTL8187X_EEPROMCMD_CK           (1 << 2)
#define RTL8187X_EEPROMCMD_CS           (1 << 3)
#define RTL8187X_EEPROMCMD_NORMAL       (0 << 6)
#define RTL8187X_EEPROMCMD_LOAD         (1 << 6)
#define RTL8187X_EEPROMCMD_PROGRAM      (2 << 6)
#define RTL8187X_EEPROMCMD_CONFIG       (3 << 6)

/* Refers to "config2" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG2_ANTENNADIV     (1 << 6)

/* Refers to "msr" field of "rtl8187x_csr_s" struct */

#define RTL8187X_MSR_NOLINK             (0 << 2)
#define RTL8187X_MSR_ADHOC              (1 << 2)
#define RTL8187X_MSR_INFRA              (2 << 2)
#define RTL8187X_MSR_MASTER             (3 << 2)
#define RTL8187X_MSR_ENEDCA             (4 << 2)

/* Refers to "config3" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG3_ANAPARAMWRITE  (1 << 6)
#define RTL8187X_CONFIG3_GNTSELECT      (1 << 7)

/* Refers to "config4" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CONFIG4_POWEROFF       (1 << 6)
#define RTL8187X_CONFIG4_VCOOFF         (1 << 7)

/* Refers to "tx_agc_ctl" field of "rtl8187x_csr_s" struct */

#define RTL8187X_TXAGCCTL_PERPACKETGAINSHIFT    (1 << 0)
#define RTL8187X_TXAGCCTL_PERPACKETANTSELSHIFT  (1 << 1)
#define RTL8187X_TXAGCCTL_FEEDBACKANT           (1 << 2)

/* Refers to "cw_conf" field of "rtl8187x_csr_s" struct */

#define RTL8187X_CWCONF_PERPACKETCWSHIFT      (1 << 0)
#define RTL8187X_CWCONF_PERPACKETRETRYSHIFT   (1 << 1)

/* Refers to "rate_fallback" field of "rtl8187x_csr_s" struct */

#define RTL8187X_RATEFALLBACK_ENABLE    (1 << 7)

/* TX/RX Descriptor Bit Field Definitions ***********************************/
/* Tx/Rx flags are common between RTL818X chips */

/* Refers to "flags" field of "rtl8187x_txdesc_s" struct */

#define RTL8187X_TXDESC_FLAG_NOENC      (1 << 15) /* Disable hardware based encryption */
#define RTL8187X_TXDESC_FLAG_TXOK       (1 << 15) /* TX frame was ACKed */
#define RTL8187X_TXDESC_FLAG_SPLCP      (1 << 16) /* Use short preamble */
#define RTL8187X_TXDESC_FLAG_RXUNDER    (1 << 16) 
#define RTL8187X_TXDESC_FLAG_MOREFRAG   (1 << 17) /* More fragments follow */
#define RTL8187X_TXDESC_FLAG_CTS        (1 << 18) /* Use CTS-to-self protection */
#define RTL8187X_TXDESC_FLAG_RTS        (1 << 23) /* Use RTS/CTS protection */
#define RTL8187X_TXDESC_FLAG_LS         (1 << 28) /* Last segment of the frame */
#define RTL8187X_TXDESC_FLAG_FS         (1 << 29) /* First segment of the frame */
#define RTL8187X_TXDESC_FLAG_DMA        (1 << 30)
#define RTL8187X_TXDESC_FLAG_OWN        (1 << 31)

/* Refers to "flags" field of "rtl8187x_rxdesc_s" struct */

#define RTL8187X_RXDESC_FLAG_ICVERR     (1 << 12)
#define RTL8187X_RXDESC_FLAG_CRC32ERR   (1 << 13)
#define RTL8187X_RXDESC_FLAG_PM         (1 << 14)
#define RTL8187X_RXDESC_FLAG_RXERR      (1 << 15)
#define RTL8187X_RXDESC_FLAG_BCAST      (1 << 16)
#define RTL8187X_RXDESC_FLAG_PAM        (1 << 17)
#define RTL8187X_RXDESC_FLAG_MCAST      (1 << 18)
#define RTL8187X_RXDESC_FLAG_QOS        (1 << 19) /* RTL8187(B) only */
#define RTL8187X_RXDESC_FLAG_TRSW       (1 << 24) /* RTL8187(B) only */
#define RTL8187X_RXDESC_FLAG_SPLCP      (1 << 25)
#define RTL8187X_RXDESC_FLAG_FOF        (1 << 26)
#define RTL8187X_RXDESC_FLAG_DMAFAIL    (1 << 27)
#define RTL8187X_RXDESC_FLAG_LS         (1 << 28)
#define RTL8187X_RXDESC_FLAG_FS         (1 << 29)
#define RTL8187X_RXDESC_FLAG_EOR        (1 << 30)
#define RTL8187X_RXDESC_FLAG_OWN        (1 << 31)

/* TX descriptor rate values */

#define RTL8187X_RATE_1                 0
#define RTL8187X_RATE_2                 1
#define RTL8187X_RATE_5p5               2
#define RTL8187X_RATE_11                3
#define RTL8187X_RATE_6                 4
#define RTL8187X_RATE_9                 5
#define RTL8187X_RATE_12                6
#define RTL8187X_RATE_18                7
#define RTL8187X_RATE_24                8
#define RTL8187X_RATE_36                9
#define RTL8187X_RATE_48                10
#define RTL8187X_RATE_54                11

/* Other RTL8187x Definitions **********************************************/

/* Number of IEEE 802.11 Channels */

#define RTL8187X_NCHANNELS              14

/* Vendor-Specific Requests */

#define RTL8187X_REQT_READ              0xc0 /* DIR=IN  TYPE=VENDOR RECIPIENT=DEVICE */
#define RTL8187X_REQT_WRITE             0x40 /* DIR=OUT TYPE=VENDOR RECIPIENT=DEVICE */
#define RTL8187X_REQ_GETREG             0x05
#define RTL8187X_REQ_SETREG             0x05

/* EEPROM Definitions */

#define PCI_EEPROM_WIDTH_93C46          6
#define PCI_EEPROM_WIDTH_93C56          8
#define PCI_EEPROM_WIDTH_93C66          8
#define PCI_EEPROM_WIDTH_OPCODE         3
#define PCI_EEPROM_WRITE_OPCODE         0x05
#define PCI_EEPROM_READ_OPCODE          0x06
#define PCI_EEPROM_EWDS_OPCODE          0x10
#define PCI_EEPROM_EWEN_OPCODE          0x13

#define RTL8187X_EEPROM_TXPWRBASE       0x05
#define RTL8187X_EEPROM_MACADDR         0x07
#define RTL8187X_EEPROM_TXPWRCHAN1      0x16  /* 3 channels */
#define RTL8187X_EEPROM_TXPWRCHAN6      0x1b  /* 2 channels */
#define RTL8187X_EEPROM_TXPWRCHAN4      0x3d  /* 2 channels */

/* RT8187x Register Addresses ***********************************************/

#define RTL8187X_ADDR_MAR0              0xff08
#define RTL8187X_ADDR_MAR1              0xff0c
#define RTL8187X_ADDR_BRSR              0xff2c
#define RTL8187X_ADDR_RESPRATE          0xff34
#define RTL8187X_ADDR_CMD               0xff37
#define RTL8187X_ADDR_INTMASK           0xff3c
#define RTL8187X_ADDR_TXCONF            0xff40
#define RTL8187X_ADDR_RXCONF            0xff44
#define RTL8187X_ADDR_INTTIMEOUT        0xff48
#define RTL8187X_ADDR_EEPROMCMD         0xff50
#define RTL8187X_ADDR_CONFIG1           0xff52
#define RTL8187X_ADDR_ANAPARAM          0xff54
#define RTL8187X_ADDR_CONFIG3           0xff59
#define RTL8187X_ADDR_CONFIG4           0xff5a
#define RTL8187X_ADDR_TESTR             0xff5b
#define RTL8187X_ADDR_PGSELECT          0xff5e
#define RTL8187X_ADDR_ANAPARAM2         0xff60
#define RTL8187X_ADDR_PHY0              0xff7c
#define RTL8187X_ADDR_PHY1              0xff7d
#define RTL8187X_ADDR_PHY2              0xff7e
#define RTL8187X_ADDR_PHY3              0xff7f
#define RTL8187X_ADDR_RFPINSOUTPUT      0xff80
#define RTL8187X_ADDR_RFPINSENABLE      0xff82
#define RTL8187X_ADDR_RFPINSSELECT      0xff84
#define RTL8187X_ADDR_RFPINSINPUT       0xff86
#define RTL8187X_ADDR_RFPARA            0xff88
#define RTL8187X_ADDR_RFTIMING          0xff8c
#define RTL8187X_ADDR_GPENABLE          0xff90
#define RTL8187X_ADDR_GPIO              0xff91
#define RTL8187X_ADDR_TXAGCCTL          0xff9c
#define RTL8187X_ADDR_TXGAINCCK         0xff9d
#define RTL8187X_ADDR_TXGAINOFDM        0xff9e
#define RTL8187X_ADDR_TXANTENNA         0xff9f
#define RTL8187X_ADDR_WPACONF           0xffb0
#define RTL8187X_ADDR_CWCONF            0xffbc
#define RTL8187X_ADDR_CWVAL             0xffbd
#define RTL8187X_ADDR_RATEFALLBACK      0xffbe
#define RTL8187X_ADDR_ANAPARAM3         0xffee
#define RTL8187X_ADDR_TALLYSEL          0xfffc

/* Other RTL8187x Register Values ******************************************/

#define RTL8187X_RTL8225_ANAPARAM_ON    0xa0000a59
#define RTL8187X_RTL8225_ANAPARAM2_ON   0x860c7312
#define RTL8187X_RTL8225_ANAPARAM_OFF   0xa00beb59
#define RTL8187X_RTL8225_ANAPARAM2_OFF  0x840dec11

#define RTL8187B_RTL8225_ANAPARAM_ON    0x45090658
#define RTL8187B_RTL8225_ANAPARAM2_ON   0x727f3f52
#define RTL8187B_RTL8225_ANAPARAM3_ON   0x00
#define RTL8187B_RTL8225_ANAPARAM_OFF   0x55480658
#define RTL8187B_RTL8225_ANAPARAM2_OFF  0x72003f50
#define RTL8187B_RTL8225_ANAPARAM3_OFF  0x00

/* Standard Helper Macros ***************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef NULL
#  define NULL ((void*)0)
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

 /* Linux RTL-818x mapping struct.  This structure is not used in this driver
  * and will, eventually, be removed. It is retained here now for reference.
  * See the RTL8187x_ADDR_* definitions above.
  */

struct rtl8187x_csr_s 
{
  uint8_t   mac[6];                /*                            0xff00-0xff05 */
  uint8_t   reserved_0[2];         /*                            0xff06-0xff07 */
  uint32_t  mar[2];                /* RTL8187X_ADDR_MARn         0xff08-0xff0f */
  uint8_t   rx_fifo_count;         /*                            0xff10        */
  uint8_t   reserved_1;            /*                            0xff11        */
  uint8_t   tx_fifo_count;         /*                            0xff12        */
  uint8_t   bqreq;                 /*                            0xff13        */
  uint8_t   reserved_2[4];         /*                            0xff14-0xff17 */
  uint32_t  tsft[2];               /*                            0xff18-0xff1f */
  uint32_t  tlpda;                 /*                            0xff20        */
  uint32_t  tnpda;                 /*                            0xff24        */
  uint32_t  thpda;                 /*                            0xff28        */
  uint16_t  brsr;                  /* RTL8187X_ADDR_BRSR         0xff2c        */
  uint8_t   bssid[6];              /*                            0xff2e-0xff33 */
  uint8_t   resp_rate;             /* RTL8187X_ADDR_RESPRATE     0xff34        */
  uint8_t   eifs;                  /*                            0xff35        */
  uint8_t   reserved_3[1];         /*                            0xff36        */
  uint8_t   cmd;                   /* RTL8187X_ADDR_CMD          0xff37        */
  uint8_t   reserved_4[4];         /*                            0xff38-0xff3b */
  uint16_t  int_mask;              /* RTL8187X_ADDR_INTMASK      0xff3c        */
  uint16_t  int_status;            /*                            0xff3e        */
  uint32_t  tx_conf;               /* RTL8187X_ADDR_TXCONF       0xff40        */
  uint32_t  rx_conf;               /* RTL8187X_ADDR_RXCONF       0xff44        */
  uint32_t  int_timeout;           /* RTL8187X_ADDR_INTTIMEOUT   0xff48        */
  uint32_t  tbda;                  /*                            0xff4c        */
  uint8_t   eeprom_cmd;            /* RTL8187X_ADDR_EEPROMCMD    0xff50        */
  uint8_t   config0;               /*                            0xff51        */
  uint8_t   config1;               /* RTL8187X_ADDR_CONFIG1      0xff52        */
  uint8_t   config2;               /*                            0xff53        */
  uint32_t  anaparam;              /* RTL8187X_ADDR_ANAPARAM     0xff54        */
  uint8_t   msr;                   /*                            0xff58        */
  uint8_t   config3;               /* RTL8187X_ADDR_CONFIG3      0xff59        */
  uint8_t   config4;               /* RTL8187X_ADDR_CONFIG4      0xff5a        */
  uint8_t   testr;                 /* RTL8187X_ADDR_TESTR        0xff5b        */
  uint8_t   reserved_9[2];         /*                            0xff5c-0xff5d */
  uint8_t   pgselect;              /* RTL8187X_ADDR_PGSELECT     0xff5e        */
  uint8_t   security;              /*                            0xff5f        */
  uint32_t  anaparam2;             /* RTL8187X_ADDR_ANAPARAM2    0xff60        */
  uint8_t   reserved_10[12];       /*                            0xff64-0xff6f */
  uint16_t  beacon_interval;       /*                            0xff70        */
  uint16_t  atim_wnd;              /*                            0xff72        */
  uint16_t  beacon_interval_time;  /*                            0xff74        */
  uint16_t  atimtr_interval;       /*                            0xff76        */
  uint8_t   phy_delay;             /*                            0xff78        */
  uint8_t   carrier_sense_counter; /*                            0xff79        */
  uint8_t   reserved_11[2];        /*                            0xff7a-0xff7b */
  uint8_t   phy[4];                /* RTL8187X_ADDR_PHYn         0xff7c-0xff7f */
  uint16_t  rfpinsoutput;          /* RTL8187X_ADDR_RFPINSOUTPUT 0xff80        */
  uint16_t  rfpinsenable;          /* RTL8187X_ADDR_RFPINSENABLE 0xff82        */
  uint16_t  rfpinsselect;          /* RTL8187X_ADDR_RFPINSSELECT 0xff84        */
  uint16_t  rfpinsinput;           /* RTL8187X_ADDR_RFPINSINPUT  0xff86        */
  uint32_t  rf_para;               /* RTL8187X_ADDR_RFPARA       0xff88        */
  uint32_t  rf_timing;             /* RTL8187X_ADDR_RFTIMING     0xff8c        */
  uint8_t   gp_enable;             /* RTL8187X_ADDR_GPENABLE     0xff90        */
  uint8_t   gpio0;                 /* RTL8187X_ADDR_GPIO         0xff91        */
  uint8_t   gpio1;                 /*                            0xff92        */
  uint8_t   reserved_12;           /*                            0xff93        */
  uint32_t  hssi_para;             /*                            0xff94        */
  uint8_t   reserved_13[4];        /*                            0xff98-0xff9d */
  uint8_t   tx_agc_ctl;            /* RTL8187X_ADDR_TXAGCCTL     0xff9c        */
  uint8_t   tx_gain_cck;           /* RTL8187X_ADDR_TXGAINCCK    0xff9d        */
  uint8_t   tx_gain_ofdm;          /* RTL8187X_ADDR_TXGAINOFDM   0xff9e        */
  uint8_t   tx_antenna;            /* RTL8187X_ADDR_TXANTENNA    0xff9f        */
  uint8_t   reserved_14[16];       /*                            0xffa0-0xffaf */
  uint8_t   wpa_conf;              /* RTL8187X_ADDR_WPACONF      0xffb0        */
  uint8_t   reserved_15[3];        /*                            0xffb1-0xffb3 */
  uint8_t   sifs;                  /*                            0xffb4        */
  uint8_t   difs;                  /*                            0xffb5        */
  uint8_t   slot;                  /*                            0xffb6        */
  uint8_t   reserved_16[5];        /*                            0xffb7-0xffbb */
  uint8_t   cw_conf;               /* RTL8187X_ADDR_CWCONF       0xffbc        */
  uint8_t   cw_val;                /* RTL8187X_ADDR_CWVAL        0xffbd        */
  uint8_t   rate_fallback;         /* RTL8187X_ADDR_RATEFALLBACK 0xffbe        */
  uint8_t   acm_control;           /*                            0xffbf        */
  uint8_t   reserved_17[24];       /*                            0xffc0-ffd7   */
  uint8_t   config5;               /*                            0xffd8        */
  uint8_t   tx_dma_polling;        /*                            0xffd9        */
  uint8_t   reserved_18[2];        /*                            0xffda-0xffdb */
  uint16_t  cwr;                   /*                            0xffdc        */
  uint8_t   retry_ctr;             /*                            0xffde        */
  uint8_t   reserved_19[3];        /*                            0xffdf-0xffe1 */
  uint16_t  int_mig;               /*                            0xffe2        */
  uint32_t  rdsar;                 /*                            0xffe4        */
  uint16_t  tid_ac_map;            /*                            0xffe8        */
  uint8_t   reserved_20[4];        /*                            0xffea-0xffed */
  uint8_t   anaparam3;             /* RTL8187X_ADDR_ANAPARAM3    0xffee        */
  uint8_t   reserved_21[5];        /*                            0xffef-0xfff3 */
  uint16_t  femr;                  /*                            0xfff4        */
  uint8_t   reserved_22[4];        /*                            0xfff6-0xfff9 */
  uint16_t  tally_cnt;             /*                            0xfffa        */
  uint8_t   tally_sel;             /* RTL8187X_ADDR_TALLYSEL     0xfffc        */
} __attribute__ ((packed));

/* RX and TX descriptors */

struct rtl8187x_rxdesc_s
{
  uint32_t  flags;
  uint8_t   noise;
  uint8_t   signal;
  uint8_t   agc;
  uint8_t   reserved;
  uint64_t  mactime;
} __attribute__((packed));

#define SIZEOF_RXDESC 16

#ifdef CONFIG_RTL8187B
struct rtl8187x_txdesc_s
{
  uint32_t flags;
  uint16_t rtsduration;
  uint16_t len;
  uint32_t unused1;
  uint16_t unused2;
  uint16_t txduration;
  uint32_t unused3;
  uint32_t retry;
  uint32_t unused4[2];
} __attribute__((packed));

#define SIZEOF_TXDESC 32

#else
struct rtl8187x_txdesc_s
{
  uint32_t  flags;
  uint16_t  rtsduration;
  uint16_t  len;
  uint32_t  retry;
} __attribute__((packed));

#define SIZEOF_TXDESC 12
#endif

#endif /* __DRIVERS_NET_RTL8187X_H */

