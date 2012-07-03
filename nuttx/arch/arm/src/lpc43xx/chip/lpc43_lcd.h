/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_lcd.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_LCD_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_LCD_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_LCD_TIMH_OFFSET         0x000 /* Horizontal Timing Control register */
#define LPC43_LCD_TIMV_OFFSET         0x004 /* Vertical Timing Control register */
#define LPC43_LCD_POL_OFFSET          0x008 /* Clock and Signal Polarity Control register */
#define LPC43_LCD_LE_OFFSET           0x00c /* Line End Control register */
#define LPC43_LCD_UPBASE_OFFSET       0x010 /* Upper Panel Frame Base Address register */
#define LPC43_LCD_LPBASE_OFFSET       0x014 /* Lower Panel Frame Base Address register */
#define LPC43_LCD_CTRL_OFFSET         0x018 /* LCD Control register */
#define LPC43_LCD_INTMSK_OFFSET       0x01c /* Interrupt Mask register */
#define LPC43_LCD_INTRAW_OFFSET       0x020 /* Raw Interrupt Status register */
#define LPC43_LCD_INTSTAT_OFFSET      0x024 /* Masked Interrupt Status register */
#define LPC43_LCD_INTCLR_OFFSET       0x028 /* Interrupt Clear register */
#define LPC43_LCD_UPCURR_OFFSET       0x02c /* Upper Panel Current Address Value register */
#define LPC43_LCD_LPCURR_OFFSET       0x030 /* Lower Panel Current Address Value register */

/*  0x200 to 0x3fc 256x16-bit Color Palette registers */

#define LPC43_LCD_PAL_OFFSET(n)       (0x200 + ((n) << 2)) /* n=0..128, two colors per word */

/* 0x800 to 0xbfc Cursor Image registers */

#define LPC43_LCD_CRSR_IMG_OFFSET(n)  (0x800 + ((n) << 2)) /* n = 0..256 */

#define LPC43_LCD_CRSR_CTRL_OFFSET    0xc00 /* Cursor Control register */
#define LPC43_LCD_CRSR_CFG_OFFSET     0xc04 /* Cursor Configuration register */
#define LPC43_LCD_CRSR_PAL0_OFFSET    0xc08 /* Cursor Palette register 0 */
#define LPC43_LCD_CRSR_PAL1_OFFSET    0xc0c /* Cursor Palette register 1 */
#define LPC43_LCD_CRSR_XY_OFFSET      0xc10 /* Cursor XY Position register */
#define LPC43_LCD_CRSR_CLIP_OFFSET    0xc14 /* Cursor Clip Position register */
#define LPC43_LCD_CRSR_INTMSK_OFFSET  0xc20 /* Cursor Interrupt Mask register */
#define LPC43_LCD_CRSR_INTCLR_OFFSET  0xc24 /* Cursor Interrupt Clear register */
#define LPC43_LCD_CRSR_INTRAW_OFFSET  0xc28 /* Cursor Raw Interrupt Status register */
#define LPC43_LCD_CRSR_INTSTAT_OFFSET 0xc2c /* Cursor Masked Interrupt Status register */

/* Register Addresses *******************************************************************************/

#define LPC43_LCD_TIMH                (LPC43_LCD_BASE+LPC43_LCD_TIMH_OFFSET)
#define LPC43_LCD_TIMV                (LPC43_LCD_BASE+LPC43_LCD_TIMV_OFFSET)
#define LPC43_LCD_POL                 (LPC43_LCD_BASE+LPC43_LCD_POL_OFFSET)
#define LPC43_LCD_LE                  (LPC43_LCD_BASE+LPC43_LCD_LE_OFFSET)
#define LPC43_LCD_UPBASE              (LPC43_LCD_BASE+LPC43_LCD_UPBASE_OFFSET)
#define LPC43_LCD_LPBASE              (LPC43_LCD_BASE+LPC43_LCD_LPBASE_OFFSET)
#define LPC43_LCD_CTRL                (LPC43_LCD_BASE+LPC43_LCD_CTRL_OFFSET)
#define LPC43_LCD_INTMSK              (LPC43_LCD_BASE+LPC43_LCD_INTMSK_OFFSET)
#define LPC43_LCD_INTRAW              (LPC43_LCD_BASE+LPC43_LCD_INTRAW_OFFSET)
#define LPC43_LCD_INTSTAT             (LPC43_LCD_BASE+LPC43_LCD_INTSTAT_OFFSET)
#define LPC43_LCD_INTCLR              (LPC43_LCD_BASE+LPC43_LCD_INTCLR_OFFSET)
#define LPC43_LCD_UPCURR              (LPC43_LCD_BASE+LPC43_LCD_UPCURR_OFFSET)
#define LPC43_LCD_LPCURR              (LPC43_LCD_BASE+LPC43_LCD_LPCURR_OFFSET)

/*  0x200 to 0x3fc 256x16-bit Color Palette registers */

#define LPC43_LCD_PAL(n)              (LPC43_LCD_BASE+LPC43_LCD_PAL_OFFSET(n))

/* 0x800 to 0xbfc Cursor Image registers */

#define LPC43_LCD_CRSR_IMG(n)         (LPC43_LCD_BASE+LPC43_LCD_CRSR_IMG_OFFSET(n))

#define LPC43_LCD_CRSR_CTRL           (LPC43_LCD_BASE+LPC43_LCD_CRSR_CTRL_OFFSET)
#define LPC43_LCD_CRSR_CFG            (LPC43_LCD_BASE+LPC43_LCD_CRSR_CFG_OFFSET)
#define LPC43_LCD_CRSR_PAL0           (LPC43_LCD_BASE+LPC43_LCD_CRSR_PAL0_OFFSET)
#define LPC43_LCD_CRSR_PAL1           (LPC43_LCD_BASE+LPC43_LCD_CRSR_PAL1_OFFSET)
#define LPC43_LCD_CRSR_XY             (LPC43_LCD_BASE+LPC43_LCD_CRSR_XY_OFFSET)
#define LPC43_LCD_CRSR_CLIP           (LPC43_LCD_BASE+LPC43_LCD_CRSR_CLIP_OFFSET)
#define LPC43_LCD_CRSR_INTMSK         (LPC43_LCD_BASE+LPC43_LCD_CRSR_INTMSK_OFFSET)
#define LPC43_LCD_CRSR_INTCLR         (LPC43_LCD_BASE+LPC43_LCD_CRSR_INTCLR_OFFSET)
#define LPC43_LCD_CRSR_INTRAW         (LPC43_LCD_BASE+LPC43_LCD_CRSR_INTRAW_OFFSET)
#define LPC43_LCD_CRSR_INTSTAT        (LPC43_LCD_BASE+LPC43_LCD_CRSR_INTSTAT_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Horizontal Timing Control register */

                                                /* Bits 0-1: Reserved */
#define LCD_TIMH_PPL_SHIFT            (2)       /* Bits 2-7: Pixels-per-line */
#define LCD_TIMH_PPL_MASK             (0x3f << LCD_TIMH_PPL_SHIFT)
#define LCD_TIMH_HSW_SHIFT            (8)       /* Bits 8-15: Horizontal synchronization pulse width */
#define LCD_TIMH_HSW_MASK             (0xff << LCD_TIMH_HSW_SHIFT)
#define LCD_TIMH_HFP_SHIFT            (16)      /* Bits 16-23:  Horizontal front porch */
#define LCD_TIMH_HFP_MASK             (0xff << LCD_TIMH_HFP_SHIFT)
#define LCD_TIMH_HBP_SHIFT            (24)      /* Bits 24-31:  Horizontal back porch */
#define LCD_TIMH_HBP_MASK             (0xff << LCD_TIMH_HBP_SHIFT)
                                                /* Bit nn: Reserved */
/* Vertical Timing Control register */

#define LCD_TIMV_LPP_SHIFT            (0)       /* Bits 0-9: Lines per panel */
#define LCD_TIMV_LPP_MASK             (0x3ff << LCD_TIMV_LPP_SHIFT)
#define LCD_TIMV_VSW_SHIFT            (10)      /* Bits 10-15: Vertical synchronization pulse width */
#define LCD_TIMV_VSW_MASK             (0x3f << LCD_TIMV_VSW_SHIFT)
#define LCD_TIMV_VFP_SHIFT            (16)      /* Bits 16-23: Vertical front porch */
#define LCD_TIMV_VFP_MASK             (0xff << LCD_TIMV_VFP_SHIFT)
#define LCD_TIMV_VBP_SHIFT            (24)      /* Bits 24-31: Vertical back porch */
#define LCD_TIMV_VBP_MASK             (0xff << LCD_TIMV_VBP_SHIFT)

/* Clock and Signal Polarity Control register */

#define LCD_POL_PCDLO_SHIFT           (0)       /* Bits 0-4: Lower five bits of panel clock divisor */
#define LCD_POL_PCDLO_MASK            (31 << LCD_POL_PCDLO_SHIFT)
#define LCD_POL_CLKSEL                (1 << 5)  /* Bit 5:  Clock Select */
#define LCD_POL_ACB_SHIFT             (6)       /* Bits 6-10: AC bias pin frequency */
#define LCD_POL_ACB_MASK              (31 << LCD_POL_ACB_SHIFT)
#define LCD_POL_IVS                   (1 << 11) /* Bit 11: Invert vertical synchronization */
#define LCD_POL_IHS                   (1 << 12) /* Bit 12: Invert horizontal synchronization */
#define LCD_POL_IPC                   (1 << 13) /* Bit 13: Invert panel clock */
#define LCD_POL_IOE                   (1 << 14) /* Bit 14: Invert output enable */
                                                /* Bit 15: Reserved */
#define LCD_POL_CPL_SHIFT             (16)      /* Bits 16-25: Clocks per line */
#define LCD_POL_CPL_MASK              (0x3ff << LCD_POL_CPL_SHIFT)
#define LCD_POL_BCD                   (1 << 26) /* Bit 26: Bypass pixel clock divider */
#define LCD_POL_PCDHI_SHIFT           (27)       /* Bits 27-31: Upper five bits of panel clock divisor */
#define LCD_POL_PCDHI_MASK            (31 << LCD_POL_PCDHI_SHIFT)

/* Line End Control register */

#define LCD_LE_DELAY_SHIFT            (0)       /* Bits 0-6: Line-end delay */
#define LCD_LE_DELAY_MASK             (0x7f << LCD_LE_DELAY_SHIFT)
                                                /* Bits 7-15: Reserved */
#define LCD_LE_ENA                    (1 << 16) /* Bit 16: LCD Line end enable */
                                                /* Bits 17-31: Reserved */

/* Upper Panel Frame Base Address register */
                                                /* Bits 0-2: Reserved */
#define LCD_UPBASE_SHIFT              (3)       /* Bits 3-31: Upper panel base address */
#define LCD_UPBASE_MASK               (0xfffffff8)

/* Lower Panel Frame Base Address register */
                                                /* Bits 0-2: Reserved */
#define LCD_LPBASE_SHIFT              (3)       /* Bits 3-31: Lower panel base address */
#define LCD_LPBASE_MASK               (0xfffffff8)

/* LCD Control register */

#define LCD_CTRL_LCDEN                (1 << 0)  /* Bit 0:  LCD enable control bit */
#define LCD_CTRL_LCDBPP_SHIFT         (1)       /* Bits 1-3: LCD bits per pixel */
#define LCD_CTRL_LCDBPP_MASK          (7 << LCD_CTRL_LCDBPP_SHIFT)
#  define LCD_CTRL_LCDBPP_1BPP        (0 << LCD_CTRL_LCDBPP_SHIFT) /* 1 bpp */
#  define LCD_CTRL_LCDBPP_2BPP        (1 << LCD_CTRL_LCDBPP_SHIFT) /* 2 bpp */
#  define LCD_CTRL_LCDBPP_4BPP        (2 << LCD_CTRL_LCDBPP_SHIFT) /* 4 bpp */
#  define LCD_CTRL_LCDBPP_8BPP        (3 << LCD_CTRL_LCDBPP_SHIFT) /* 8 bpp */
#  define LCD_CTRL_LCDBPP_16BPP       (4 << LCD_CTRL_LCDBPP_SHIFT) /* 16 bpp */
#  define LCD_CTRL_LCDBPP_24BPP       (5 << LCD_CTRL_LCDBPP_SHIFT) /* 24 bpp (TFT panel only) */
#  define LCD_CTRL_LCDBPP_RGB565      (6 << LCD_CTRL_LCDBPP_SHIFT) /* 16 bpp, 5:6:5 mode */
#  define LCD_CTRL_LCDBPP_RGB444      (7 << LCD_CTRL_LCDBPP_SHIFT) /* 12 bpp, 4:4:4 mode */
#define LCD_CTRL_LCDBW                (1 << 4)  /* Bit 4:  STN LCD monochrome/color selection */
#define LCD_CTRL_LCDTFT               (1 << 5)  /* Bit 5:  LCD panel TFT type selection */
#define LCD_CTRL_LCDMONO8             (1 << 6)  /* Bit 6:  Monochrome LCD interface width */
#define LCD_CTRL_LCDDUAL              (1 << 7)  /* Bit 7:  Single or Dual LCD panel selection */
#define LCD_CTRL_BGR                  (1 << 8)  /* Bit 8:  Color format selection */
#define LCD_CTRL_BEBO                 (1 << 9)  /* Bit 9:  Big-endian Byte Order */
#define LCD_CTRL_BEPO                 (1 << 10) /* Bit 10: Big-Endian Pixel Ordering */
#define LCD_CTRL_LCDPWR               (1 << 11) /* Bit 11: LCD power enable */
#define LCD_CTRL_LCDVCOMP_SHIFT       (12)      /* Bits 12-13: LCD vertical compare interrupt */
#define LCD_CTRL_LCDVCOMP_MASK        (3 << LCD_CTRL_LCDVCOMP_SHIFT)
#  define LCD_CTRL_LCDVCOMP_START     (0 << LCD_CTRL_LCDVCOMP_SHIFT) /* Start of vertical synchronization */
#  define LCD_CTRL_LCDVCOMP_BACK      (1 << LCD_CTRL_LCDVCOMP_SHIFT) /* Start of back porch */
#  define LCD_CTRL_LCDVCOMP_ACTIVE    (2 << LCD_CTRL_LCDVCOMP_SHIFT) /* Start of active video */
#  define LCD_CTRL_LCDVCOMP_FRONT     (3 << LCD_CTRL_LCDVCOMP_SHIFT) /* Start of front porch */
                                                /* Bits 14-15: Reserved */
#define LCD_INTMSK_WATERMARK          (1 << 16) /* Bit 16: LCD DMA FIFO watermark level */
                                                /* Bits 17-31: Reserved */
/* Interrupt Mask register */
/* Raw Interrupt Status register */
/* Masked Interrupt Status register */
/* Interrupt Clear register */


                                                /* Bit 0: Reserved */
#define LCD_INT_FUFI                  (1 << 1)  /* Bit 1:  FIFO underflow interrupt */
#define LCD_INT_LNBUI                 (1 << 2)  /* Bit 2:  LCD next base address update interrupt enable */
#define LCD_INT_VCOMPI                (1 << 3)  /* Bit 3:  Vertical compare interrupt enable */
#define LCD_INT_BERI                  (1 << 4)  /* Bit 4:  AHB master error interrupt enable */
                                                /* Bits 5-31: Reserved */
/* Upper Panel Current Address Value register (32-bit address) */
/* Lower Panel Current Address Value register (32-bit address) */

/*  256x16-bit Color Palette registers */

#define LCD_PAL_R0_SHIFT              (0)       /* Bits 0-4: Red palette data */
#define LCD_PAL_R0_MASK               (31 << LCD_PAL_R0_SHIFT)
#define LCD_PAL_G0_SHIFT              (5)       /* Bits 5-9: Green palette data */
#define LCD_PAL_G0_MASK               (31 << LCD_PAL_G0_SHIFT)
#define LCD_PAL_B0_SHIFT              (10)      /* Bits 10-14: Blue palette data */
#define LCD_PAL_B0_MASK               (31 << LCD_PAL_B0_SHIFT)
#define LCD_PAL_I0                    (1 << 16) /* Bit 15: Intensity / unused bit */
#define LCD_PAL_R1_SHIFT              (16)      /* Bits 16-20: Red palette data */
#define LCD_PAL_R1_MASK               (31 << LCD_PAL_R1_SHIFT)
#define LCD_PAL_G1_SHIFT              (21)      /* Bits 21-25: Green palette data */
#define LCD_PAL_G1_MASK               (31 << LCD_PAL_G1_SHIFT)
#define LCD_PAL_B1_SHIFT              (26)      /* Bits 26-30: Blue palette data */
#define LCD_PAL_B1_MASK               (31 << LCD_PAL_B1_SHIFT)
#define LCD_PAL_I1                    (1 << 31) /* Bit 31: Intensity / unused bit */

/* Cursor Image registers (32-bit image data) */

/* Cursor Control register */

#define LCD_CRSR_CTRL_ON              (1 << 0)  /* Bit 0:  Cursor enable */
                                                /* Bits 1-3: Reserved */
#define LCD_CRSR_CTRL_NUM_SHIFT       (4)       /* Bits 4-5: Cursor image number */
#define LCD_CRSR_CTRL_NUM_MASK        (3 << LCD_CRSR_CTRL_NUM_SHIFT)
#  define LCD_CRSR_CTRL_NUM_0         (0 << LCD_CRSR_CTRL_NUM_SHIFT)
#  define LCD_CRSR_CTRL_NUM_1         (1 << LCD_CRSR_CTRL_NUM_SHIFT)
#  define LCD_CRSR_CTRL_NUM_2         (2 << LCD_CRSR_CTRL_NUM_SHIFT)
#  define LCD_CRSR_CTRL_NUM_3         (3 << LCD_CRSR_CTRL_NUM_SHIFT)
                                                /* Bits 6-31: Reserved */
/* Cursor Configuration register */

#define LCD_CRSR_CFG_CRSRSIZE         (1 << 0)  /* Bit 0:  Cursor size selection */
#define LCD_CRSR_CFG_FRAMESYNC        (1 << 1)  /* Bit 1:  Cursor frame synchronization type */
                                                /* Bits 2-31: Reserved */
/* Cursor Palette register 0/1 */

#define LCD_CRSR_PAL_RED_SHIFT        (0)       /* Bits 0-7: Red color component */
#define LCD_CRSR_PAL_RED_MASK         (0xff << LCD_CRSR_PAL_RED_SHIFT)
#define LCD_CRSR_PAL_GREEN_SHIFT      (8)       /* Bits 8-15: Green color component */
#define LCD_CRSR_PAL_GREEN_MASK       (0xff << LCD_CRSR_PAL_GREEN_SHIFT)
#define LCD_CRSR_PAL_BLUE_SHIFT       (16)       /* Bits 16-23: Blue color component */
#define LCD_CRSR_PAL_BLUE_MASK        (0xff << LCD_CRSR_PAL_BLUE_SHIFT)
                                                /* Bits 24-31: Reserved */
/* Cursor XY Position register */

#define LCD_CRSRX_SHIFT               (0)       /* Bits 0-9: X ordinate of the cursor origin measured in pixels */
#define LCD_CRSRX_MASK                (0x3ff << LCD_CRSRX_SHIFT)
                                                /* Bits 10-15: Reserved */
#define LCD_CRSRY_SHIFT               (16)      /* Bits 16-25: Y ordinate of the cursor origin measured in pixels */
#define LCD_CRSRY_MASK                (0x3ff << LCD_CRSRY_SHIFT)
                                                /* Bits 26-31: Reserved */
/* Cursor Clip Position register */

#define LCD_CRSR_CLIPX_SHIFT          (0)       /* Bits 0-5: Cursor clip position for X direction */
#define LCD_CRSR_CLIPX_MASK           (0x3f << LCD_CRSR_CLIPX_SHIFT)
                                                /* Bits 6-7: Reserved */
#define LCD_CRSR_CLIPY_SHIFT          (8)       /* Bits 8-13: Cursor clip position for Y direction */
#define LCD_CRSR_CLIPY_MASK           (0x3f << LCD_CRSR_CLIPY_SHIFT)
                                                /* Bits 14-31: Reserved */
/* Cursor Interrupt Mask register */
/* Cursor Interrupt Clear register */
/* Cursor Raw Interrupt Status register */
/* Cursor Masked Interrupt Status register */

#define LCD_CRSR_INT                  (1 << 0)  /* CRSRIM Cursor interrupt */
                                                /* Bits 1-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_LCD_H */
