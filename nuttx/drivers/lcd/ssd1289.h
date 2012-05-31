/**************************************************************************************
 * drivers/lcd/ssd1289.h
 * Definitions for the Solomon Systech SSD1289 LCD controller
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: SSD1289, Rev 1.3, Apr 2007, Solomon Systech Limited
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
 **************************************************************************************/

#ifndef __DRIVERS_LCD_SSD1289_H
#define __DRIVERS_LCD_SSD1289_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_SSD1289

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* SSD1289 Register Addresses (All with DC=1) */

#define SSD1289_OSCSTART                0x00  /* Oscillation Start (write) */
#define SSD1289_DEVCODE                 0x00  /* Oscillation Start (read) */
#define SSD1289_OUTCTRL                 0x01  /* Driver output control */
#define SSD1289_ACCTRL                  0x02  /* LCD drive AC control */
#define SSD1289_PWRCTRL1                0x03  /* Power control 1 */
#define SSD1289_CMP1                    0x05  /* Compare register 1 */ 
#define SSD1289_CMP2                    0x06  /* Compare register 2 */
#define SSD1289_DSPCTRL                 0x07  /* Display control */
#define SSD1289_FCYCCTRL                0x0b  /* Frame cycle control */
#define SSD1289_PWRCTRL2                0x0c  /* Power control 2 */
#define SSD1289_PWRCTRL3                0x0d  /* Power control 3 */
#define SSD1289_PWRCTRL4                0x0e  /* Power control 4 */
#define SSD1289_GSTART                  0x0f  /* Gate scan start position */
#define SSD1289_SLEEP                   0x10  /* Sleep mode */
#define SSD1289_ENTRY                   0x11  /* Entry mode */
#define SSD1289_OPT3                    0x12  /* Optimize Access Speed 3 */
#define SSD1289_GIFCTRL                 0x15  /* Generic Interface Control */
#define SSD1289_HPORCH                  0x16  /* Horizontal Porch */
#define SSD1289_VPORCH                  0x17  /* Vertical Porch */
#define SSD1289_PWRCTRL5                0x1e  /* Power control 5 */
#define SSD1289_DATA                    0x22  /* RAM data/write data */
#define SSD1289_WRMASK1                 0x23  /* RAM write data mask 1 */
#define SSD1289_WRMASK2                 0x24  /* RAM write data mask 2 */
#define SSD1289_FFREQ                   0x25  /* Frame Frequency */
#define SSD1289_VCOMOTP1                0x28  /* VCOM OTP */
#define SSD1289_OPT1                    0x28  /* Optimize Access Speed 1 */
#define SSD1289_VCOMOTP2                0x29  /* VCOM OTP */
#define SSD1289_OPT2                    0x2f  /* Optimize Access Speed  2 */
#define SSD1289_GAMMA1                  0x30  /* Gamma control 1 */
#define SSD1289_GAMMA2                  0x31  /* Gamma control 2 */
#define SSD1289_GAMMA3                  0x32  /* Gamma control 3 */
#define SSD1289_GAMMA4                  0x33  /* Gamma control 4 */
#define SSD1289_GAMMA5                  0x34  /* Gamma control 5 */
#define SSD1289_GAMMA6                  0x35  /* Gamma control 6 */
#define SSD1289_GAMMA7                  0x36  /* Gamma control 7 */
#define SSD1289_GAMMA8                  0x37  /* Gamma control 8 */
#define SSD1289_GAMMA9                  0x3a  /* Gamma control 9 */
#define SSD1289_GAMMA10                 0x3b  /* Gamma control 10 */
#define SSD1289_VSCROLL1                0x41  /* Vertical scroll control 1 */
#define SSD1289_VSCROLL2                0x42  /* Vertical scroll control 2 */
#define SSD1289_HADDR                   0x44  /* Horizontal RAM address position */
#define SSD1289_VSTART                  0x45  /* Vertical RAM address start position */
#define SSD1289_VEND                    0x46  /* Vertical RAM address end position */
#define SSD1289_W1START                 0x48  /* First window start */
#define SSD1289_W1END                   0x49  /* First window end */
#define SSD1289_W2START                 0x4a  /* Second window start */
#define SSD1289_W2END                   0x4b  /* Second window end */
#define SSD1289_XADDR                   0x4e  /* Set GDDRAM X address counter */
#define SSD1289_YADDR                   0x4f  /* Set GDDRAM Y address counter */

/* SSD1289 Register Bit definitions */

/* Index register (DC=0) */

#define SSD1289_INDEX_MASK              0xff

/* Device code (read) */

#define SSD1289_DEVCODE_VALUE           0x8989

/* Oscillation Start (write) */

#define SSD1289_OSCSTART_OSCEN          (1 << 0)  /* Enable oscillator */

/* Driver output control */

#define SSD1289_OUTCTRL_MUX_SHIFT       (0)       /* Number of lines for the LCD driver */
#define SSD1289_OUTCTRL_MUX_MASK        (0x1ff << SSD1289_OUTCTRL_MUX_SHIFT)
#  define SSD1289_OUTCTRL_MUX(n)        ((n) << SSD1289_OUTCTRL_MUX_SHIFT)
#define SSD1289_OUTCTRL_TB              (1 << 9)  /* Selects the output shift direction of the gate driver */
#define SSD1289_OUTCTRL_SM              (1 << 10) /* Scanning order of gate driver */
#define SSD1289_OUTCTRL_BGR             (1 << 11) /* Order from RGB to BGR in 18-bit GDDRAM data */
#define SSD1289_OUTCTRL_CAD             (1 << 12) /* Retention capacitor configuration of the TFT panel */
#define SSD1289_OUTCTRL_REV             (1 << 13) /* Reversed display */
#define SSD1289_OUTCTRL_RL              (1 << 14) /* RL pin state */

/* LCD drive AC control */

#define SSD1289_ACCTRL_NW_SHIFT         (0)       /* Number of lines to alternate in N-line inversion */
#define SSD1289_ACCTRL_NW_MASK          (0xff << SSD1289_ACCTRL_NW_SHIFT)
#define SSD1289_ACCTRL_WSMD             (1 << 8)  /* Waveform of WSYNC output */
#define SSD1289_ACCTRL_EOR              (1 << 9)  /* EOR signals */
#define SSD1289_ACCTRL_BC               (1 << 10) /* Select the liquid crystal drive waveform */
#define SSD1289_ACCTRL_ENWS             (1 << 11) /* Enables WSYNC output pin */
#define SSD1289_ACCTRL_FLD              (1 << 12) /* Set display in interlace drive mode */

/* Power control 1 */

#define SSD1289_PWRCTRL1_AP_SHIFT       (1)       /* Current from internal operational amplifier */
#define SSD1289_PWRCTRL1_AP_MASK        (7 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_LEAST     (0 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_SMALL     (1 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_SMMED     (2 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_MEDIUM    (3 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_MEDLG     (4 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_LARGE     (5 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_LGMX      (6 << SSD1289_PWRCTRL1_AP_SHIFT)
#  define SSD1289_PWRCTRL1_AP_MAX       (7 << SSD1289_PWRCTRL1_AP_SHIFT)
#define SSD1289_PWRCTRL1_DC_SHIFT       (4)       /* Set the step-up cycle of the step-up circuit for 262k-color mode */
#define SSD1289_PWRCTRL1_DC_MASK        (15 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx24  (0 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx16  (1 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx12  (2 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx8   (3 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx6   (4 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx5   (5 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx4   (6 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx3   (7 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx2   (8 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FLINEx1   (9 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd4     (10 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd6     (11 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd8     (12 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd10    (13 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd12    (14 << SSD1289_PWRCTRL1_DC_SHIFT)
#  define SSD1289_PWRCTRL1_DC_FOSd16    (15 << SSD1289_PWRCTRL1_DC_SHIFT)
#define SSD1289_PWRCTRL1_BT_SHIFT       (9)       /* Control the step-up factor of the step-up circuit */
#define SSD1289_PWRCTRL1_BT_MASK        (7 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p6m5      (0 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p6m4      (1 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p6m6      (2 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p5m5      (3 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p5m4      (4 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p5m3      (5 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p4m4      (6 << SSD1289_PWRCTRL1_BT_SHIFT)
#  define SSD1289_PWRCTRL1_BT_p4m3      (7 << SSD1289_PWRCTRL1_BT_SHIFT)
#define SSD1289_PWRCTRL1_DCT_SHIFT      (12)     /* Step-up cycle of the step-up circuit for 8-color mode */
#define SSD1289_PWRCTRL1_DCT_MASK       (15 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx24 (0 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx16 (1 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx12 (2 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx8  (3 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx6  (4 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx5  (5 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx4  (6 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx3  (7 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx2  (8 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FLINEx1  (9 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd4    (10 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd6    (11 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd8    (12 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd10   (13 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd12   (14 << SSD1289_PWRCTRL1_DCT_SHIFT)
#  define SSD1289_PWRCTRL1_DCT_FOSd16   (15 << SSD1289_PWRCTRL1_DCT_SHIFT)

/* Compare register 1 and 2 */ 

#define SSD1289_CMP1_CPG_SHIFT          (2)
#define SSD1289_CMP1_CPG_MASK           (0x3f << SSD1289_CMP1_CPG_SHIFT)
#define SSD1289_CMP1_CPR_SHIFT          (10)
#define SSD1289_CMP1_CPR_MASK           (0x3f << SSD1289_CMP1_CPR_SHIFT)

#define SSD1289_CMP2_CPB_SHIFT          (2)
#define SSD1289_CMP2_CPB_MASK           (0x3f << SSD1289_CMP2_CPB_SHIFT)

/* Display control */

#define SSD1289_DSPCTRL_D_SHIFT         (0)       /* Display control */
#define SSD1289_DSPCTRL_D_MASK          (3 << SSD1289_DSPCTRL_D_SHIFT)
#  define SSD1289_DSPCTRL_OFF           (0 << SSD1289_DSPCTRL_D_SHIFT)
#  define SSD1289_DSPCTRL_INTERNAL      (1 << SSD1289_DSPCTRL_D_SHIFT)
#  define SSD1289_DSPCTRL_ON            (3 << SSD1289_DSPCTRL_D_SHIFT)
#define SSD1289_DSPCTRL_CM              (1 << 3)  /* 8-color mode setting */
#define SSD1289_DSPCTRL_DTE             (1 << 4)  /* Selected gate level */
#define SSD1289_DSPCTRL_GON             (1 << 5)  /* Gate off level */
#define SSD1289_DSPCTRL_SPT             (1 << 8)  /* 2-division LCD drive */
#define SSD1289_DSPCTRL_VLE_SHIFT       (9)       /* Vertical scroll control */
#define SSD1289_DSPCTRL_VLE_MASK        (3 << SSD1289_DSPCTRL_VLE_SHIFT)
#  define SSD1289_DSPCTRL_VLE(n)        ((n) << SSD1289_DSPCTRL_VLE_SHIFT)
#define SSD1289_DSPCTRL_PT_SHIFT        (11)      /* Normalize the source outputs */
#define SSD1289_DSPCTRL_PT_MASK         (3 << SSD1289_DSPCTRL_PT_SHIFT)
#  define SSD1289_DSPCTRL_PT(n)         ((n) << SSD1289_DSPCTRL_PT_SHIFT)

/* Frame cycle control */

#define SSD1289_FCYCCTRL_RTN_SHIFT      (0)       /* Number of clocks in each line */
#define SSD1289_FCYCCTRL_RTN_MASK       (3 << SSD1289_FCYCCTRL_RTN_SHIFT)
#  define SSD1289_FCYCCTRL_RTN(n)       (((n)-16) << SSD1289_FCYCCTRL_RTN_SHIFT)
#define SSD1289_FCYCCTRL_SRTN           (1 << 4)  /* When SRTN =1, RTN3-0 value will be count */
#define SSD1289_FCYCCTRL_SDIV           (1 << 5)  /* When SDIV = 1, DIV1-0 value will be count */
#define SSD1289_FCYCCTRL_DIV_SHIFT      (6)       /* Set the division ratio of clocks */
#define SSD1289_FCYCCTRL_DIV_MASK       (3 << SSD1289_FCYCCTRL_DIV_SHIFT)
#  define SSD1289_FCYCCTRL_DIV1         (0 << SSD1289_FCYCCTRL_DIV_SHIFT)
#  define SSD1289_FCYCCTRL_DIV2         (1 << SSD1289_FCYCCTRL_DIV_SHIFT)
#  define SSD1289_FCYCCTRL_DIV4         (2 << SSD1289_FCYCCTRL_DIV_SHIFT)
#  define SSD1289_FCYCCTRL_DIV8         (3 << SSD1289_FCYCCTRL_DIV_SHIFT)
#define SSD1289_FCYCCTRL_EQ_SHIFT       (8)       /* Sets the equalizing period */
#define SSD1289_FCYCCTRL_EQ_MASK        (3 << SSD1289_FCYCCTRL_EQ_SHIFT)
#  define SSD1289_FCYCCTRL_EQ(n)        (((n)-1) << SSD1289_FCYCCTRL_EQ_SHIFT) /* n = 2-8 clocks */
#define SSD1289_FCYCCTRL_SDT_SHIFT      (12)      /* Set delay amount from the gate output */
#define SSD1289_FCYCCTRL_SDT_MASK       (3 << SSD1289_FCYCCTRL_SDT_SHIFT)
#  define SSD1289_FCYCCTRL_SDT(n)       ((n) << SSD1289_FCYCCTRL_SDT_SHIFT) /* n = 1-3 clocks */
#define SSD1289_FCYCCTRL_NO_SHIFT       (14)      /* Sets amount of non-overlap of the gate output */
#define SSD1289_FCYCCTRL_NO_MASK        (3 << SSD1289_FCYCCTRL_NO_SHIFT)
#  define SSD1289_FCYCCTRL_NO(n)        ((n) << SSD1289_FCYCCTRL_NO_SHIFT) /* n = 1-3 clocks */

/* Power control 2 */

#define SSD1289_PWRCTRL2_VRC_SHIFT      (0)       /* Adjust VCIX2 output voltage */
#define SSD1289_PWRCTRL2_VRC_MASK       (7 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p1V     (0 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p2V     (1 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p3V     (2 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p4V     (3 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p5V     (4 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p6V     (5 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p7V     (6 << SSD1289_PWRCTRL2_VRC_SHIFT)
#  define SSD1289_PWRCTRL2_VRC_5p8V     (7 << SSD1289_PWRCTRL2_VRC_SHIFT)

/* Power control 3 */

#define SSD1289_PWRCTRL3_VRH_SHIFT      (0)       /* Set amplitude magnification of VLCD63 */
#define SSD1289_PWRCTRL3_VRH_MASK       (15 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p540   (0 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p620   (1 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p700   (2 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p780   (3 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p850   (4 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x1p930   (5 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p020   (6 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p090   (7 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p165   (8 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p245   (9 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p335   (10 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p400   (11 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p500   (12 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p570   (13 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p645   (14 << SSD1289_PWRCTRL3_VRH_SHIFT)
#  define SSD1289_PWRCTRL3_VRH_x2p725   (15 << SSD1289_PWRCTRL3_VRH_SHIFT)

/* Power control 4 */

#define SSD1289_PWRCTRL4_VDV_SHIFT      (8)       /* Set amplitude magnification of VLCD63 */
#define SSD1289_PWRCTRL4_VDV_MASK       (32 << SSD1289_PWRCTRL4_VDV_SHIFT)
#  define SSD1289_PWRCTRL4_VDV(n)       ((n) << SSD1289_PWRCTRL4_VDV_SHIFT)
#define SSD1289_PWRCTRL4_VCOMG          (1 << 13) /* VcomL variable */

/* Gate scan start position */

#define SSD1289_GSTART_MASK             0x1ff

/* Sleep mode */

#define SSD1289_SLEEP_ON                (1 << 0)

/* Entry mode */

#define SSD1289_ENTRY_LG_SHIFT          (0)       /* Write after comparing */
#define SSD1289_ENTRY_LG_MASK           (7 << SSD1289_ENTRY_LG_SHIFT)
#define SSD1289_ENTRY_AM                (1 << 3)  /* Address counter direction */
#define SSD1289_ENTRY_ID_SHIFT          (4)       /* Address increment mode */
#define SSD1289_ENTRY_ID_MASK           (3 << SSD1289_ENTRY_ID_SHIFT)
#  define SSD1289_ENTRY_ID_HDECVDEC     (0 << SSD1289_ENTRY_ID_SHIFT)
#  define SSD1289_ENTRY_ID_HINCVDEC     (1 << SSD1289_ENTRY_ID_SHIFT)
#  define SSD1289_ENTRY_ID_HDECVINC     (2 << SSD1289_ENTRY_ID_SHIFT)
#  define SSD1289_ENTRY_ID_HINCVINC     (3 << SSD1289_ENTRY_ID_SHIFT)
#define SSD1289_ENTRY_TY_SHIFT          (6)       /* RAM data write method */
#define SSD1289_ENTRY_TY_MASK           (3 << SSD1289_ENTRY_TY_SHIFT)
#  define SSD1289_ENTRY_TY_A            (0 << SSD1289_ENTRY_TY_SHIFT)
#  define SSD1289_ENTRY_TY_B            (1 << SSD1289_ENTRY_TY_SHIFT)
#  define SSD1289_ENTRY_TY_C            (2 << SSD1289_ENTRY_TY_SHIFT)
#define SSD1289_ENTRY_DMODE_SHIFT       (8)       /* Data display mode */
#define SSD1289_ENTRY_DMODE_MASK        (3 << SSD1289_ENTRY_DMODE_SHIFT)
#  define SSD1289_ENTRY_DMODE_RAM       (0 << SSD1289_ENTRY_DMODE_SHIFT)
#  define SSD1289_ENTRY_DMODE_GENERIC   (1 << SSD1289_ENTRY_DMODE_SHIFT)
#  define SSD1289_ENTRY_DMODE_RAMGEN    (2 << SSD1289_ENTRY_DMODE_SHIFT)
#  define SSD1289_ENTRY_DMODE_GENRAM    (3 << SSD1289_ENTRY_DMODE_SHIFT)
#define SSD1289_ENTRY_WMODE             (1 << 10) /* Select source of data in RAM */
#define SSD1289_ENTRY_OEDEF             (1 << 11) /* Define display window */
#define SSD1289_ENTRY_TRANS             (1 << 12) /* Transparent display */
#define SSD1289_ENTRY_DFM_SHIFT         (13)      /* Color display mode */
#define SSD1289_ENTRY_DFM_MASK          (3 << SSD1289_ENTRY_DFM_SHIFT)
#  define SSD1289_ENTRY_DFM_262K        (2 << SSD1289_ENTRY_DFM_SHIFT)
#  define SSD1289_ENTRY_DFM_65K         (3 << SSD1289_ENTRY_DFM_SHIFT)
#define SSD1289_ENTRY_VSMODE            (1 << 15) /* Frame frequency depends on VSYNC */

/* Generic Interface Control */

#define SSD1289_GIFCTRL_INVVS           (1 << 0)  /* Sets the signal polarity of DOTCLK pin */
#define SSD1289_GIFCTRL_INVHS           (1 << 1)  /* Sets the signal polarity of DEN pin */
#define SSD1289_GIFCTRL_NVDEN           (1 << 2)  /* Sets the signal polarity of HSYNC pin */
#define SSD1289_GIFCTRL_INVDOT          (1 << 3)  /* Sets the signal polarity of VSYNC pin */

/* Horizontal Porch */

#define SSD1289_HPORCH_HBP_SHIFT        (0)       /* Set delay from falling edge of HSYNC signal to data */
#define SSD1289_HPORCH_HBP_MASK         (0xff << SSD1289_HPORCH_HBP_SHIFT)
#define SSD1289_HPORCH_XL_SHIFT         (8)       /* number of valid pixel per line */
#define SSD1289_HPORCH_XL_MASK          (0xff << SSD1289_HPORCH_XL_SHIFT)

/* Vertical Porch */

#define SSD1289_VPORCH_VBP_SHIFT        (0)       /* Set delay from falling edge of VSYNC signal to line */
#define SSD1289_VPORCH_VBP_MASK         (0xff << SSD1289_VPORCH_VBP_SHIFT)
#define SSD1289_VPORCH_XFP_SHIFT        (8)       /* Delay from last line to falling edge of VSYNC of next frame */
#define SSD1289_VPORCH_XFP_MASK         (0xff << SSD1289_VPORCH_XFP_SHIFT)
#define SSD1289_VPORCH_

/* Power control 5 */

#define SSD1289_PWRCTRL5_VCM_SHIFT      (0)       /* Set the VcomH voltage */
#define SSD1289_PWRCTRL5_VCM_MASK       (0x3f << SSD1289_PWRCTRL5_VCM_SHIFT)
#  define SSD1289_PWRCTRL5_VCM(n)       ((n) << SSD1289_PWRCTRL5_VCM_SHIFT)
#define SSD1289_PWRCTRL5_NOTP           (1 << 7)  /* 1=VCM valid */

/* RAM write data mask 1 */

#define SSD1289_WRMASK1_WMG_SHIFT       (2)
#define SSD1289_WRMASK1_WMG_MASK        (0x3f << SSD1289_WRMASK1_WMG_SHIFT)
#define SSD1289_WRMASK1_WMR_SHIFT       (10)
#define SSD1289_WRMASK1_WMR_MASK        (0x3f << SSD1289_WRMASK1_WMR_SHIFT)

#define SSD1289_WRMASK2_WMB_SHIFT       (2)
#define SSD1289_WRMASK2_WMB_MASK        (0x3f << SSD1289_WRMASK2_WMB_SHIFT)

/* Frame Frequency */

#define SSD1289_FFREQ_OSC_SHIFT         (12)        /* Set the frame frequency */
#define SSD1289_FFREQ_OSC_MASK          (15 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF50        (0 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF55        (2 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF60        (5 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF65        (8 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF70        (10 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF75        (12 << SSD1289_FFREQ_OSC_SHIFT)
#  define SSD1289_FFREQ_OSC_FF80        (14 << SSD1289_FFREQ_OSC_SHIFT)

/* VCOM OTP */

#define SSD1289_VCOMOTP1_ACTIVATE       0x0006
#define SSD1289_VCOMOTP1_FIRE           0x000a
#define SSD1289_VCOMOTP2_ACTIVATE       0x80c0

/* Optimize Access Speed 1, 2, 3 (omitted) */

/* Gamma control 1-10.  Magic values.  I won't try to represent the fields. */

/* Vertical scroll control 1 and 2 */

#define SSD1289_VSCROLL_MASK            0x1ff /* Scroll length */

/* Horizontal RAM address position */

#define SSD1289_HADDR_HSA_SHIFT         (0)    /* Window horizontal start address */
#define SSD1289_HADDR_HSA_MASK          (0xff << SSD1289_HADDR_HSA_SHIFT)
#define SSD1289_HADDR_HEA_SHIFT         (8)    /* Window horizontal end address */
#define SSD1289_HADDR_HEA_MASK          (0xff << SSD1289_HADDR_HEA_SHIFT)

/* Vertical RAM address start/end position */

#define SSD1289_VSTART_MASK             0x1ff  /* Window Vertical start address */
#define SSD1289_VEND_MASK               0x1ff  /* Window Vertical end address */

/* First window start/end */

#define SSD1289_W1START_MASK            0x1ff  /* Start line for first screen */
#define SSD1289_W1END_MASK              0x1ff  /* End line for first screen */

/* Second window start/end */

#define SSD1289_W2START_MASK            0x1ff  /* Start line for second screen */
#define SSD1289_W2END_MASK              0x1ff  /* End line for second screen */

/* Set GDDRAM X/Y address counter */

#define SSD1289_XADDR_MASK              0xff   /* GDDRAM X address in the address counter */
#define SSD1289_YADDR_MASK              0x1ff  /* GDDRAM Y address in the address counter */

#endif /* CONFIG_LCD_SSD1289 */
#endif /* __DRIVERS_LCD_SSD1289_H */
