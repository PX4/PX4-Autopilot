/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_sgpio.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SGPIO_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SGPIO_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_SGPIO_OUT_MUXCFG_OFFSET(n)   (0x0000 + ((n) << 2)
#define LPC43_SGPIO_OUT_MUXCFG0_OFFSET     0x0000 /* Pin multiplexer configuration register 0 */
#define LPC43_SGPIO_OUT_MUXCFG1_OFFSET     0x0004 /* Pin multiplexer configuration register 1 */
#define LPC43_SGPIO_OUT_MUXCFG2_OFFSET     0x0008 /* Pin multiplexer configuration register 2 */
#define LPC43_SGPIO_OUT_MUXCFG3_OFFSET     0x000c /* Pin multiplexer configuration register 3 */
#define LPC43_SGPIO_OUT_MUXCFG4_OFFSET     0x0010 /* Pin multiplexer configuration register 4 */
#define LPC43_SGPIO_OUT_MUXCFG5_OFFSET     0x0014 /* Pin multiplexer configuration register 5 */
#define LPC43_SGPIO_OUT_MUXCFG6_OFFSET     0x0018 /* Pin multiplexer configuration register 6 */
#define LPC43_SGPIO_OUT_MUXCFG7_OFFSET     0x001c /* Pin multiplexer configuration register 7 */
#define LPC43_SGPIO_OUT_MUXCFG8_OFFSET     0x0020 /* Pin multiplexer configuration register 8 */
#define LPC43_SGPIO_OUT_MUXCFG9_OFFSET     0x0024 /* Pin multiplexer configuration register 9 */
#define LPC43_SGPIO_OUT_MUXCFG10_OFFSET    0x0028 /* Pin multiplexer configuration register 10 */
#define LPC43_SGPIO_OUT_MUXCFG11_OFFSET    0x002c /* Pin multiplexer configuration register 11 */
#define LPC43_SGPIO_OUT_MUXCFG12_OFFSET    0x0030 /* Pin multiplexer configuration register 12 */
#define LPC43_SGPIO_OUT_MUXCFG13_OFFSET    0x0034 /* Pin multiplexer configuration register 13 */
#define LPC43_SGPIO_OUT_MUXCFG14_OFFSET    0x0038 /* Pin multiplexer configuration register 14 */
#define LPC43_SGPIO_OUT_MUXCFG15_OFFSET    0x003c /* Pin multiplexer configuration register 15 */

#define LPC43_SGPIO_MUXCFG_OFFSET(n)       (0x0040 + ((n) << 2)
#define LPC43_SGPIO_MUXCFG0_OFFSET         0x0040 /* SGPIO multiplexer configuration register 0 */
#define LPC43_SGPIO_MUXCFG1_OFFSET         0x0044 /* SGPIO multiplexer configuration register 1 */
#define LPC43_SGPIO_MUXCFG2_OFFSET         0x0048 /* SGPIO multiplexer configuration register 2 */
#define LPC43_SGPIO_MUXCFG3_OFFSET         0x004c /* SGPIO multiplexer configuration register 3 */
#define LPC43_SGPIO_MUXCFG4_OFFSET         0x0050 /* SGPIO multiplexer configuration register 4 */
#define LPC43_SGPIO_MUXCFG5_OFFSET         0x0054 /* SGPIO multiplexer configuration register 5 */
#define LPC43_SGPIO_MUXCFG6_OFFSET         0x0058 /* SGPIO multiplexer configuration register 6 */
#define LPC43_SGPIO_MUXCFG7_OFFSET         0x005c /* SGPIO multiplexer configuration register 7 */
#define LPC43_SGPIO_MUXCFG8_OFFSET         0x0060 /* SGPIO multiplexer configuration register 8 */
#define LPC43_SGPIO_MUXCFG9_OFFSET         0x0064 /* SGPIO multiplexer configuration register 9 */
#define LPC43_SGPIO_MUXCFG10_OFFSET        0x0068 /* SGPIO multiplexer configuration register 10 */
#define LPC43_SGPIO_MUXCFG11_OFFSET        0x006c /* SGPIO multiplexer configuration register 11 */
#define LPC43_SGPIO_MUXCFG12_OFFSET        0x0070 /* SGPIO multiplexer configuration register 12 */
#define LPC43_SGPIO_MUXCFG13_OFFSET        0x0074 /* SGPIO multiplexer configuration register 13 */
#define LPC43_SGPIO_MUXCFG14_OFFSET        0x0078 /* SGPIO multiplexer configuration register 14 */
#define LPC43_SGPIO_MUXCFG15_OFFSET        0x007c /* SGPIO multiplexer configuration register 15 */

#define LPC43_SGPIO_SLICE_MUXCFG_OFFSET(n) (0x0080 + ((n) << 2)
#define LPC43_SGPIO_SLICE_MUXCFG0_OFFSET   0x0080 /* Slice multiplexer configuration register 0 */
#define LPC43_SGPIO_SLICE_MUXCFG1_OFFSET   0x0084 /* Slice multiplexer configuration register 1 */
#define LPC43_SGPIO_SLICE_MUXCFG2_OFFSET   0x0088 /* Slice multiplexer configuration register 2 */
#define LPC43_SGPIO_SLICE_MUXCFG3_OFFSET   0x008c /* Slice multiplexer configuration register 3 */
#define LPC43_SGPIO_SLICE_MUXCFG4_OFFSET   0x0090 /* Slice multiplexer configuration register 4 */
#define LPC43_SGPIO_SLICE_MUXCFG5_OFFSET   0x0094 /* Slice multiplexer configuration register 5 */
#define LPC43_SGPIO_SLICE_MUXCFG6_OFFSET   0x0098 /* Slice multiplexer configuration register 6 */
#define LPC43_SGPIO_SLICE_MUXCFG7_OFFSET   0x009c /* Slice multiplexer configuration register 7 */
#define LPC43_SGPIO_SLICE_MUXCFG8_OFFSET   0x00a0 /* Slice multiplexer configuration register 8 */
#define LPC43_SGPIO_SLICE_MUXCFG9_OFFSET   0x00a4 /* Slice multiplexer configuration register 9 */
#define LPC43_SGPIO_SLICE_MUXCFG10_OFFSET  0x00a8 /* Slice multiplexer configuration register 10 */
#define LPC43_SGPIO_SLICE_MUXCFG11_OFFSET  0x00ac /* Slice multiplexer configuration register 11 */
#define LPC43_SGPIO_SLICE_MUXCFG12_OFFSET  0x00b0 /* Slice multiplexer configuration register 12 */
#define LPC43_SGPIO_SLICE_MUXCFG13_OFFSET  0x00b4 /* Slice multiplexer configuration register 13 */
#define LPC43_SGPIO_SLICE_MUXCFG14_OFFSET  0x00b8 /* Slice multiplexer configuration register 14 */
#define LPC43_SGPIO_SLICE_MUXCFG15_OFFSET  0x00bc /* Slice multiplexer configuration register 15 */

#define LPC43_SGPIO_REG_OFFSET(n)          (0x00c0 + ((n) << 2)
#define LPC43_SGPIO_REG0_OFFSET            0x00c0 /* Slice data register 0 */
#define LPC43_SGPIO_REG1_OFFSET            0x00c4 /* Slice data register 1 */
#define LPC43_SGPIO_REG2_OFFSET            0x00c8 /* Slice data register 2 */
#define LPC43_SGPIO_REG3_OFFSET            0x00cc /* Slice data register 3 */
#define LPC43_SGPIO_REG4_OFFSET            0x00d0 /* Slice data register 4 */
#define LPC43_SGPIO_REG5_OFFSET            0x00d4 /* Slice data register 5 */
#define LPC43_SGPIO_REG6_OFFSET            0x00d8 /* Slice data register 6 */
#define LPC43_SGPIO_REG7_OFFSET            0x00dc /* Slice data register 7 */
#define LPC43_SGPIO_REG8_OFFSET            0x00e0 /* Slice data register 8 */
#define LPC43_SGPIO_REG9_OFFSET            0x00e4 /* Slice data register 9 */
#define LPC43_SGPIO_REG10_OFFSET           0x00e8 /* Slice data register 10 */
#define LPC43_SGPIO_REG11_OFFSET           0x00ec /* Slice data register 11 */
#define LPC43_SGPIO_REG12_OFFSET           0x00f0 /* Slice data register 12 */
#define LPC43_SGPIO_REG13_OFFSET           0x00f4 /* Slice data register 13 */
#define LPC43_SGPIO_REG14_OFFSET           0x00f8 /* Slice data register 14 */
#define LPC43_SGPIO_REG15_OFFSET           0x00fc /* Slice data register 15 */

#define LPC43_SGPIO_REG_SS_OFFSET(n)       (0x0100 + ((n) << 2)
#define LPC43_SGPIO_REG_SS0_OFFSET         0x0100 /* Slice data shadow register 0 */
#define LPC43_SGPIO_REG_SS1_OFFSET         0x0104 /* Slice data shadow register 1 */
#define LPC43_SGPIO_REG_SS2_OFFSET         0x0108 /* Slice data shadow register 2 */
#define LPC43_SGPIO_REG_SS3_OFFSET         0x010c /* Slice data shadow register 3 */
#define LPC43_SGPIO_REG_SS4_OFFSET         0x0110 /* Slice data shadow register 4 */
#define LPC43_SGPIO_REG_SS5_OFFSET         0x0114 /* Slice data shadow register 5 */
#define LPC43_SGPIO_REG_SS6_OFFSET         0x0118 /* Slice data shadow register 6 */
#define LPC43_SGPIO_REG_SS7_OFFSET         0x011c /* Slice data shadow register 7 */
#define LPC43_SGPIO_REG_SS8_OFFSET         0x0120 /* Slice data shadow register 8 */
#define LPC43_SGPIO_REG_SS9_OFFSET         0x0124 /* Slice data shadow register 9 */
#define LPC43_SGPIO_REG_SS10_OFFSET        0x0128 /* Slice data shadow register 10 */
#define LPC43_SGPIO_REG_SS11_OFFSET        0x012c /* Slice data shadow register 11 */
#define LPC43_SGPIO_REG_SS12_OFFSET        0x0130 /* Slice data shadow register 12 */
#define LPC43_SGPIO_REG_SS13_OFFSET        0x0134 /* Slice data shadow register 13 */
#define LPC43_SGPIO_REG_SS14_OFFSET        0x0138 /* Slice data shadow register 14 */
#define LPC43_SGPIO_REG_SS15_OFFSET        0x013c /* Slice data shadow register 15 */

#define LPC43_SGPIO_PRESET_OFFSET(n)       (0x0140 + ((n) << 2)
#define LPC43_SGPIO_PRESET0_OFFSET         0x0140 /* COUNT0 reload value */
#define LPC43_SGPIO_PRESET1_OFFSET         0x0144 /* COUNT1 reload value */
#define LPC43_SGPIO_PRESET2_OFFSET         0x0148 /* COUNT2 reload value */
#define LPC43_SGPIO_PRESET3_OFFSET         0x014c /* COUNT3 reload value */
#define LPC43_SGPIO_PRESET4_OFFSET         0x0150 /* COUNT4 reload value */
#define LPC43_SGPIO_PRESET5_OFFSET         0x0154 /* COUNT5 reload value */
#define LPC43_SGPIO_PRESET6_OFFSET         0x0158 /* COUNT6 reload value */
#define LPC43_SGPIO_PRESET7_OFFSET         0x015c /* COUNT7 reload value */
#define LPC43_SGPIO_PRESET8_OFFSET         0x0160 /* COUNT8 reload value */
#define LPC43_SGPIO_PRESET9_OFFSET         0x0164 /* COUNT9 reload value */
#define LPC43_SGPIO_PRESET10_OFFSET        0x0168 /* COUNT10 reload value */
#define LPC43_SGPIO_PRESET11_OFFSET        0x016c /* COUNT11 reload value */
#define LPC43_SGPIO_PRESET12_OFFSET        0x0170 /* COUNT12 reload value */
#define LPC43_SGPIO_PRESET13_OFFSET        0x0174 /* COUNT13 reload value */
#define LPC43_SGPIO_PRESET14_OFFSET        0x0178 /* COUNT14 reload value */
#define LPC43_SGPIO_PRESET15_OFFSET        0x017c /* COUNT15 reload value */

#define LPC43_SGPIO_COUNT_OFFSET(n)        (0x0180 + ((n) << 2)
#define LPC43_SGPIO_COUNT0_OFFSET          0x0180 /* Down counter 0 */
#define LPC43_SGPIO_COUNT1_OFFSET          0x0184 /* Down counter 1 */
#define LPC43_SGPIO_COUNT2_OFFSET          0x0188 /* Down counter 2 */
#define LPC43_SGPIO_COUNT3_OFFSET          0x018c /* Down counter 3 */
#define LPC43_SGPIO_COUNT4_OFFSET          0x0190 /* Down counter 4 */
#define LPC43_SGPIO_COUNT5_OFFSET          0x0194 /* Down counter 5 */
#define LPC43_SGPIO_COUNT6_OFFSET          0x0198 /* Down counter 6 */
#define LPC43_SGPIO_COUNT7_OFFSET          0x019c /* Down counter 7 */
#define LPC43_SGPIO_COUNT8_OFFSET          0x01a0 /* Down counter 8 */
#define LPC43_SGPIO_COUNT9_OFFSET          0x01a4 /* Down counter 9 */
#define LPC43_SGPIO_COUNT10_OFFSET         0x01a8 /* Down counter 10 */
#define LPC43_SGPIO_COUNT11_OFFSET         0x01ac /* Down counter 11 */
#define LPC43_SGPIO_COUNT12_OFFSET         0x01b0 /* Down counter 12 */
#define LPC43_SGPIO_COUNT13_OFFSET         0x01b4 /* Down counter 13 */
#define LPC43_SGPIO_COUNT14_OFFSET         0x01b8 /* Down counter 14 */
#define LPC43_SGPIO_COUNT15_OFFSET         0x01bc /* Down counter 15 */

#define LPC43_SGPIO_POS_OFFSET(n)          (0x01c0 + ((n) << 2)
#define LPC43_SGPIO_POS0_OFFSET            0x01c0 /* Position register 0 */
#define LPC43_SGPIO_POS1_OFFSET            0x01c4 /* Position register 1 */
#define LPC43_SGPIO_POS2_OFFSET            0x01c8 /* Position register 2 */
#define LPC43_SGPIO_POS3_OFFSET            0x01cc /* Position register 3 */
#define LPC43_SGPIO_POS4_OFFSET            0x01d0 /* Position register 4 */
#define LPC43_SGPIO_POS5_OFFSET            0x01d4 /* Position register 5 */
#define LPC43_SGPIO_POS6_OFFSET            0x01d8 /* Position register 6 */
#define LPC43_SGPIO_POS7_OFFSET            0x01dc /* Position register 7 */
#define LPC43_SGPIO_POS8_OFFSET            0x01e0 /* Position register 8 */
#define LPC43_SGPIO_POS9_OFFSET            0x01e4 /* Position register 9 */
#define LPC43_SGPIO_POS10_OFFSET           0x01e8 /* Position register 0 */
#define LPC43_SGPIO_POS11_OFFSET           0x01ec /* Position register 1 */
#define LPC43_SGPIO_POS12_OFFSET           0x01f0 /* Position register 2 */
#define LPC43_SGPIO_POS13_OFFSET           0x01f4 /* Position register 3 */
#define LPC43_SGPIO_POS14_OFFSET           0x01f8 /* Position register 4 */
#define LPC43_SGPIO_POS15_OFFSET           0x01fc /* Position register 5 */

#define LPC43_SGPIO_MASKA_OFFSET           0x0200 /* Mask for pattern match function of slice A */
#define LPC43_SGPIO_MASKH_OFFSET           0x0204 /* Mask for pattern match function of slice H */
#define LPC43_SGPIO_MASKI_OFFSET           0x0208 /* Mask for pattern match function of slice I */
#define LPC43_SGPIO_MASKP_OFFSET           0x020c /* Mask for pattern match function of slice P */
#define LPC43_SGPIO_GPIO_INREG_OFFSET      0x0210 /* GPIO input status register */
#define LPC43_SGPIO_GPIO_OUTREG_OFFSET     0x0214 /* GPIO output control register */
#define LPC43_SGPIO_GPIO_OENREG_OFFSET     0x0218 /* GPIO OE control register */
#define LPC43_SGPIO_CTRL_ENABLE_OFFSET     0x021c /* Enables the slice COUNT counter */
#define LPC43_SGPIO_CTRL_DISABLE_OFFSET    0x0220 /* Disables the slice POS counter */

#define LPC43_SGPIO_INT_OFFSET(n)          (0xf00 + ((n) << 5))
#define LPC43_SGPIO_CLREN_INTOFFSET        0x0000 /* Interrupt clear mask */
#define LPC43_SGPIO_SETEN_INTOFFSET        0x0004 /* Interrupt set mask */
#define LPC43_SGPIO_ENABLE_INTOFFSET       0x0008 /* Interrupt enable */
#define LPC43_SGPIO_STATUS_INTOFFSET       0x000c /* Interrupt status */
#define LPC43_SGPIO_CLRSTAT_INTOFFSET      0x0010 /* Interrupt clear status */
#define LPC43_SGPIO_SETSTAT_INTOFFSET      0x0014 /* Interrupt set status */

#define LPC43_SGPIO_CLREN_OFFSET(n)        (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_CLREN_INTOFFSET)
#define LPC43_SGPIO_SETEN_OFFSET(n)        (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_SETEN_INTOFFSET)
#define LPC43_SGPIO_ENABLE_OFFSET(n)       (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_ENABLE_INTOFFSET)
#define LPC43_SGPIO_STATUS_OFFSET(n)       (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_STATUS_INTOFFSET)
#define LPC43_SGPIO_CLRSTAT_OFFSET(n)      (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_CLRSTAT_INTOFFSET)
#define LPC43_SGPIO_SETSTAT_OFFSET(n)      (LPC43_SGPIO_INT_OFFSET(n)+LPC43_SGPIO_SETSTAT_INTOFFSET)

#define LPC43_SGPIO_CLREN0_OFFSET          0x0f00 /* Shift clock interrupt clear mask */
#define LPC43_SGPIO_SETEN0_OFFSET          0x0f04 /* Shift clock interrupt set mask */
#define LPC43_SGPIO_ENABLE0_OFFSET         0x0f08 /* Shift clock interrupt enable */
#define LPC43_SGPIO_STATUS0_OFFSET         0x0f0c /* Shift clock interrupt status */
#define LPC43_SGPIO_CLRSTAT0_OFFSET        0x0f10 /* Shift clock interrupt clear status */
#define LPC43_SGPIO_SETSTAT0_OFFSET        0x0f14 /* Shift clock interrupt set status */

#define LPC43_SGPIO_CLREN1_OFFSET          0x0f20 /* Exchange clock interrupt clear mask */
#define LPC43_SGPIO_SETEN1_OFFSET          0x0f24 /* Exchange clock interrupt set mask */
#define LPC43_SGPIO_ENABLE1_OFFSET         0x0f28 /* Exchange clock interrupt enable */
#define LPC43_SGPIO_STATUS1_OFFSET         0x0f2c /* Exchange clock interrupt status */
#define LPC43_SGPIO_CLRSTAT1_OFFSET        0x0f30 /* Exchange clock interrupt clear status */
#define LPC43_SGPIO_SETSTAT1_OFFSET        0x0f34 /* Exchange clock interrupt set status */

#define LPC43_SGPIO_CLREN2_OFFSET          0x0f40 /* Pattern match interrupt clear mask */
#define LPC43_SGPIO_SETEN2_OFFSET          0x0f44 /* Pattern match interrupt set mask */
#define LPC43_SGPIO_ENABLE2_OFFSET         0x0f48 /* Pattern match interrupt enable */
#define LPC43_SGPIO_STATUS2_OFFSET         0x0f4c /* Pattern match interrupt status */
#define LPC43_SGPIO_CLRSTAT2_OFFSET        0x0f50 /* Pattern match interrupt clear status */
#define LPC43_SGPIO_SETSTAT2_OFFSET        0x0f54 /* Pattern match interrupt set status */

#define LPC43_SGPIO_CLREN3_OFFSET          0x0f60 /* Input interrupt clear mask */
#define LPC43_SGPIO_SETEN3_OFFSET          0x0f64 /* Input bit match interrupt set mask */
#define LPC43_SGPIO_ENABLE3_OFFSET         0x0f68 /* Input bit match interrupt enable */
#define LPC43_SGPIO_STATUS3_OFFSET         0x0f6c /* Input bit match interrupt status */
#define LPC43_SGPIO_CLRSTAT3_OFFSET        0x0f70 /* Input bit match interrupt clear status */
#define LPC43_SGPIO_SETSTAT3_OFFSET        0x0f74 /* Input bit match interrupt set status */

/* Register Addresses *******************************************************************************/

#define LPC43_SGPIO_OUT_MUXCFG(n)          (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG_OFFSET(n))
#define LPC43_SGPIO_OUT_MUXCFG0            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG0_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG1            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG1_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG2            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG2_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG3            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG3_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG4            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG4_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG5            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG5_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG6            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG6_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG7            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG7_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG8            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG8_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG9            (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG9_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG10           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG10_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG11           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG11_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG12           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG12_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG13           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG13_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG14           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG14_OFFSET)
#define LPC43_SGPIO_OUT_MUXCFG15           (LPC43_SGPIO_BASE+LPC43_SGPIO_OUT_MUXCFG15_OFFSET)

#define LPC43_SGPIO_MUXCFG(n)              (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG_OFFSET(n))
#define LPC43_SGPIO_MUXCFG0                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG0_OFFSET)
#define LPC43_SGPIO_MUXCFG1                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG1_OFFSET)
#define LPC43_SGPIO_MUXCFG2                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG2_OFFSET)
#define LPC43_SGPIO_MUXCFG3                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG3_OFFSET)
#define LPC43_SGPIO_MUXCFG4                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG4_OFFSET)
#define LPC43_SGPIO_MUXCFG5                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG5_OFFSET)
#define LPC43_SGPIO_MUXCFG6                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG6_OFFSET)
#define LPC43_SGPIO_MUXCFG7                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG7_OFFSET)
#define LPC43_SGPIO_MUXCFG8                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG8_OFFSET)
#define LPC43_SGPIO_MUXCFG9                (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG9_OFFSET)
#define LPC43_SGPIO_MUXCFG10               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG10_OFFSET)
#define LPC43_SGPIO_MUXCFG11               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG11_OFFSET)
#define LPC43_SGPIO_MUXCFG12               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG12_OFFSET)
#define LPC43_SGPIO_MUXCFG13               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG13_OFFSET)
#define LPC43_SGPIO_MUXCFG14               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG14_OFFSET)
#define LPC43_SGPIO_MUXCFG15               (LPC43_SGPIO_BASE+LPC43_SGPIO_MUXCFG15_OFFSET)

#define LPC43_SGPIO_SLICE_MUXCFG(n)        (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG_OFFSET(n))
#define LPC43_SGPIO_SLICE_MUXCFG0          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG0_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG1          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG1_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG2          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG2_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG3          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG3_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG4          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG4_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG5          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG5_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG6          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG6_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG7          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG7_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG8          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG8_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG9          (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG9_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG10         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG10_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG11         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG11_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG12         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG12_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG13         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG13_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG14         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG14_OFFSET)
#define LPC43_SGPIO_SLICE_MUXCFG15         (LPC43_SGPIO_BASE+LPC43_SGPIO_SLICE_MUXCFG15_OFFSET)

#define LPC43_SGPIO_REG(n)                 (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_OFFSET(n))
#define LPC43_SGPIO_REG0                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG0_OFFSET)
#define LPC43_SGPIO_REG1                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG1_OFFSET)
#define LPC43_SGPIO_REG2                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG2_OFFSET)
#define LPC43_SGPIO_REG3                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG3_OFFSET)
#define LPC43_SGPIO_REG4                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG4_OFFSET)
#define LPC43_SGPIO_REG5                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG5_OFFSET)
#define LPC43_SGPIO_REG6                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG6_OFFSET)
#define LPC43_SGPIO_REG7                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG7_OFFSET)
#define LPC43_SGPIO_REG8                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG8_OFFSET)
#define LPC43_SGPIO_REG9                   (LPC43_SGPIO_BASE+LPC43_SGPIO_REG9_OFFSET)
#define LPC43_SGPIO_REG10                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG10_OFFSET)
#define LPC43_SGPIO_REG11                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG11_OFFSET)
#define LPC43_SGPIO_REG12                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG12_OFFSET)
#define LPC43_SGPIO_REG13                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG13_OFFSET)
#define LPC43_SGPIO_REG14                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG14_OFFSET)
#define LPC43_SGPIO_REG15                  (LPC43_SGPIO_BASE+LPC43_SGPIO_REG15_OFFSET)

#define LPC43_SGPIO_REG_SS(n)              (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS_OFFSET(n))
#define LPC43_SGPIO_REG_SS0                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS0_OFFSET)
#define LPC43_SGPIO_REG_SS1                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS1_OFFSET)
#define LPC43_SGPIO_REG_SS2                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS2_OFFSET)
#define LPC43_SGPIO_REG_SS3                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS3_OFFSET)
#define LPC43_SGPIO_REG_SS4                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS4_OFFSET)
#define LPC43_SGPIO_REG_SS5                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS5_OFFSET)
#define LPC43_SGPIO_REG_SS6                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS6_OFFSET)
#define LPC43_SGPIO_REG_SS7                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS7_OFFSET)
#define LPC43_SGPIO_REG_SS8                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS8_OFFSET)
#define LPC43_SGPIO_REG_SS9                (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS9_OFFSET)
#define LPC43_SGPIO_REG_SS10               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS10_OFFSET)
#define LPC43_SGPIO_REG_SS11               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS11_OFFSET)
#define LPC43_SGPIO_REG_SS12               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS12_OFFSET)
#define LPC43_SGPIO_REG_SS13               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS13_OFFSET)
#define LPC43_SGPIO_REG_SS14               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS14_OFFSET)
#define LPC43_SGPIO_REG_SS15               (LPC43_SGPIO_BASE+LPC43_SGPIO_REG_SS15_OFFSET)

#define LPC43_SGPIO_PRESET(n)              (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET_OFFSET(n))
#define LPC43_SGPIO_PRESET0                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET0_OFFSET)
#define LPC43_SGPIO_PRESET1                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET1_OFFSET)
#define LPC43_SGPIO_PRESET2                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET2_OFFSET)
#define LPC43_SGPIO_PRESET3                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET3_OFFSET)
#define LPC43_SGPIO_PRESET4                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET4_OFFSET)
#define LPC43_SGPIO_PRESET5                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET5_OFFSET)
#define LPC43_SGPIO_PRESET6                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET6_OFFSET)
#define LPC43_SGPIO_PRESET7                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET7_OFFSET)
#define LPC43_SGPIO_PRESET8                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET8_OFFSET)
#define LPC43_SGPIO_PRESET9                (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET9_OFFSET)
#define LPC43_SGPIO_PRESET10               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET10_OFFSET)
#define LPC43_SGPIO_PRESET11               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET11_OFFSET)
#define LPC43_SGPIO_PRESET12               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET12_OFFSET)
#define LPC43_SGPIO_PRESET13               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET13_OFFSET)
#define LPC43_SGPIO_PRESET14               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET14_OFFSET)
#define LPC43_SGPIO_PRESET15               (LPC43_SGPIO_BASE+LPC43_SGPIO_PRESET15_OFFSET)

#define LPC43_SGPIO_COUNT(n)               (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT_OFFSET(n))
#define LPC43_SGPIO_COUNT0                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT0_OFFSET)
#define LPC43_SGPIO_COUNT1                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT1_OFFSET)
#define LPC43_SGPIO_COUNT2                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT2_OFFSET)
#define LPC43_SGPIO_COUNT3                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT3_OFFSET)
#define LPC43_SGPIO_COUNT4                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT4_OFFSET)
#define LPC43_SGPIO_COUNT5                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT5_OFFSET)
#define LPC43_SGPIO_COUNT6                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT6_OFFSET)
#define LPC43_SGPIO_COUNT7                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT7_OFFSET)
#define LPC43_SGPIO_COUNT8                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT8_OFFSET)
#define LPC43_SGPIO_COUNT9                 (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT9_OFFSET)
#define LPC43_SGPIO_COUNT10                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT10_OFFSET)
#define LPC43_SGPIO_COUNT11                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT11_OFFSET)
#define LPC43_SGPIO_COUNT12                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT12_OFFSET)
#define LPC43_SGPIO_COUNT13                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT13_OFFSET)
#define LPC43_SGPIO_COUNT14                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT14_OFFSET)
#define LPC43_SGPIO_COUNT15                (LPC43_SGPIO_BASE+LPC43_SGPIO_COUNT15_OFFSET)

#define LPC43_SGPIO_POS(n)                 (LPC43_SGPIO_BASE+LPC43_SGPIO_POS_OFFSET(n))
#define LPC43_SGPIO_POS0                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS0_OFFSET)
#define LPC43_SGPIO_POS1                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS1_OFFSET)
#define LPC43_SGPIO_POS2                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS2_OFFSET)
#define LPC43_SGPIO_POS3                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS3_OFFSET)
#define LPC43_SGPIO_POS4                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS4_OFFSET)
#define LPC43_SGPIO_POS5                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS5_OFFSET)
#define LPC43_SGPIO_POS6                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS6_OFFSET)
#define LPC43_SGPIO_POS7                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS7_OFFSET)
#define LPC43_SGPIO_POS8                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS8_OFFSET)
#define LPC43_SGPIO_POS9                   (LPC43_SGPIO_BASE+LPC43_SGPIO_POS9_OFFSET)
#define LPC43_SGPIO_POS10                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS10_OFFSET)
#define LPC43_SGPIO_POS11                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS11_OFFSET)
#define LPC43_SGPIO_POS12                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS12_OFFSET)
#define LPC43_SGPIO_POS13                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS13_OFFSET)
#define LPC43_SGPIO_POS14                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS14_OFFSET)
#define LPC43_SGPIO_POS15                  (LPC43_SGPIO_BASE+LPC43_SGPIO_POS15_OFFSET)

#define LPC43_SGPIO_MASKA                  (LPC43_SGPIO_BASE+LPC43_SGPIO_MASKA_OFFSET)
#define LPC43_SGPIO_MASKH                  (LPC43_SGPIO_BASE+LPC43_SGPIO_MASKH_OFFSET)
#define LPC43_SGPIO_MASKI                  (LPC43_SGPIO_BASE+LPC43_SGPIO_MASKI_OFFSET)
#define LPC43_SGPIO_MASKP                  (LPC43_SGPIO_BASE+LPC43_SGPIO_MASKP_OFFSET)
#define LPC43_SGPIO_GPIO_INREG             (LPC43_SGPIO_BASE+LPC43_SGPIO_GPIO_INREG_OFFSET)
#define LPC43_SGPIO_GPIO_OUTREG            (LPC43_SGPIO_BASE+LPC43_SGPIO_GPIO_OUTREG_OFFSET)
#define LPC43_SGPIO_GPIO_OENREG            (LPC43_SGPIO_BASE+LPC43_SGPIO_GPIO_OENREG_OFFSET)
#define LPC43_SGPIO_CTRL_ENABLE            (LPC43_SGPIO_BASE+LPC43_SGPIO_CTRL_ENABLE_OFFSET)
#define LPC43_SGPIO_CTRL_DISABLE           (LPC43_SGPIO_BASE+LPC43_SGPIO_CTRL_DISABLE_OFFSET)

#define LPC43_SGPIO_INT(n)                 (LPC43_SGPIO_BASE+LPC43_SGPIO_INT_OFFSET(n))
#define LPC43_SGPIO_CLREN(n)               (LPC43_SGPIO_BASE+LPC43_SGPIO_CLREN_OFFSET(n))
#define LPC43_SGPIO_SETEN(n)               (LPC43_SGPIO_BASE+LPC43_SGPIO_SETEN_OFFSET(n))
#define LPC43_SGPIO_ENABLE(n)              (LPC43_SGPIO_BASE+LPC43_SGPIO_ENABLE_OFFSET(n))
#define LPC43_SGPIO_STATUS(n)              (LPC43_SGPIO_BASE+LPC43_SGPIO_STATUS_OFFSET(n))
#define LPC43_SGPIO_CLRSTAT(n)             (LPC43_SGPIO_BASE+LPC43_SGPIO_CLRSTAT_OFFSET(n))
#define LPC43_SGPIO_SETSTAT(n)             (LPC43_SGPIO_BASE+LPC43_SGPIO_SETSTAT_OFFSET(n))

#define LPC43_SGPIO_CLREN0                 (LPC43_SGPIO_BASE+LPC43_SGPIO_CLREN0_OFFSET)
#define LPC43_SGPIO_SETEN0                 (LPC43_SGPIO_BASE+LPC43_SGPIO_SETEN0_OFFSET)
#define LPC43_SGPIO_ENABLE0                (LPC43_SGPIO_BASE+LPC43_SGPIO_ENABLE0_OFFSET)
#define LPC43_SGPIO_STATUS0                (LPC43_SGPIO_BASE+LPC43_SGPIO_STATUS0_OFFSET)
#define LPC43_SGPIO_CLRSTAT0               (LPC43_SGPIO_BASE+LPC43_SGPIO_CLRSTAT0_OFFSET)
#define LPC43_SGPIO_SETSTAT0               (LPC43_SGPIO_BASE+LPC43_SGPIO_SETSTAT0_OFFSET)

#define LPC43_SGPIO_CLREN1                 (LPC43_SGPIO_BASE+LPC43_SGPIO_CLREN1_OFFSET)
#define LPC43_SGPIO_SETEN1                 (LPC43_SGPIO_BASE+LPC43_SGPIO_SETEN1_OFFSET)
#define LPC43_SGPIO_ENABLE1                (LPC43_SGPIO_BASE+LPC43_SGPIO_ENABLE1_OFFSET)
#define LPC43_SGPIO_STATUS1                (LPC43_SGPIO_BASE+LPC43_SGPIO_STATUS1_OFFSET)
#define LPC43_SGPIO_CLRSTAT1               (LPC43_SGPIO_BASE+LPC43_SGPIO_CLRSTAT1_OFFSET)
#define LPC43_SGPIO_SETSTAT1               (LPC43_SGPIO_BASE+LPC43_SGPIO_SETSTAT1_OFFSET)

#define LPC43_SGPIO_CLREN2                 (LPC43_SGPIO_BASE+LPC43_SGPIO_CLREN2_OFFSET)
#define LPC43_SGPIO_SETEN2                 (LPC43_SGPIO_BASE+LPC43_SGPIO_SETEN2_OFFSET)
#define LPC43_SGPIO_ENABLE2                (LPC43_SGPIO_BASE+LPC43_SGPIO_ENABLE2_OFFSET)
#define LPC43_SGPIO_STATUS2                (LPC43_SGPIO_BASE+LPC43_SGPIO_STATUS2_OFFSET)
#define LPC43_SGPIO_CLRSTAT2               (LPC43_SGPIO_BASE+LPC43_SGPIO_CLRSTAT2_OFFSET)
#define LPC43_SGPIO_SETSTAT2               (LPC43_SGPIO_BASE+LPC43_SGPIO_SETSTAT2_OFFSET)

#define LPC43_SGPIO_CLREN3                 (LPC43_SGPIO_BASE+LPC43_SGPIO_CLREN3_OFFSET)
#define LPC43_SGPIO_SETEN3                 (LPC43_SGPIO_BASE+LPC43_SGPIO_SETEN3_OFFSET)
#define LPC43_SGPIO_ENABLE3                (LPC43_SGPIO_BASE+LPC43_SGPIO_ENABLE3_OFFSET)
#define LPC43_SGPIO_STATUS3                (LPC43_SGPIO_BASE+LPC43_SGPIO_STATUS3_OFFSET)
#define LPC43_SGPIO_CLRSTAT3               (LPC43_SGPIO_BASE+LPC43_SGPIO_CLRSTAT3_OFFSET)
#define LPC43_SGPIO_SETSTAT3               (LPC43_SGPIO_BASE+LPC43_SGPIO_SETSTAT3_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Pin multiplexer configuration registers */

#define SGPIO_OUT_MUXCFG_OUTCFG_SHIFT      (0)       /* Bits 0-3: P_OUT_CFG Output control SGPIOn */
#define SGPIO_OUT_MUXCFG_OUTCFG_MASK       (15 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM1   (0 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm1 (1-bit mode) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_ DOUTM2A (1 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm2a (2-bit mode 2a) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM2B  (2 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm2b (2-bit mode 2b) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM2C  (3 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm2c (2-bit mode 2c) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_GPIOOUT  (4 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* gpio_out (level set by GPIO_OUTREG) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM4A  (5 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm4a (4-bit mode 4a) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM4B  (6 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm4b (4-bit mode 4b) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM4C  (7 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm4c (4-bit mode 4c) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_CLKOUT   (8 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* clk_out */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM8A  (9 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT)  /* dout_doutm8a (8-bit mode 8a) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM8B  (10 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT) /* dout_doutm8b (8-bit mode 8b) */
#  define SGPIO_OUT_MUXCFG_OUTCFG_DOUTM8C  (11 << SGPIO_OUT_MUXCFG_OUTCFG_SHIFT) /* dout_doutm8c (8-bit mode 8c) */
#define SGPIO_OUT_MUXCFG_OECFG_SHIFT       (4)       /* Bits 4-6: P_OE_CFG Output enable source */
#define SGPIO_OUT_MUXCFG_OECFG_MASK        (7 << SGPIO_OUT_MUXCFG_OECFG_SHIFT)
#  define SGPIO_OUT_MUXCFG_OECFG_GPIOOE    (0 << SGPIO_OUT_MUXCFG_OECFG_SHIFT) /* gpio_oe (state set by GPIO_OEREG) */
#  define SGPIO_OUT_MUXCFG_OECFG_OEM1      (4 << SGPIO_OUT_MUXCFG_OECFG_SHIFT) /* dout_oem1 (1-bit mode) */
#  define SGPIO_OUT_MUXCFG_OECFG_OEM2      (5 << SGPIO_OUT_MUXCFG_OECFG_SHIFT) /* dout_oem2 (2-bit mode) */
#  define SGPIO_OUT_MUXCFG_OECFG_OEM4      (6 << SGPIO_OUT_MUXCFG_OECFG_SHIFT) /* dout_oem4 (4-bit mode) */
#  define SGPIO_OUT_MUXCFG_OECFG_OEM8      (7 << SGPIO_OUT_MUXCFG_OECFG_SHIFT) /* dout_oem8 (8-bit mode) */
                                                     /* Bits 7-31:  Reserved */
/* SGPIO multiplexer configuration registers */

#define SGPIO_MUXCFG_EXTCLK                (1 << 9)  /* Bit 9:  Select clock signal */
#define SGPIO_MUXCFG_CS_PMODE_SHIFT        (1)       /* Bits 1-2: Select source clock pin */
#define SGPIO_MUXCFG_CS_PMODE_MASK         (3 << SGPIO_MUXCFG_CS_PMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_PMODE_SGPIO8     (0 << SGPIO_MUXCFG_CS_PMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_PMODE_SGPIO9     (1 << SGPIO_MUXCFG_CS_PMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_PMODE_SGPIO10    (2 << SGPIO_MUXCFG_CS_PMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_PMODE_SGPIO11    (3 << SGPIO_MUXCFG_CS_PMODE_SHIFT)
#define SGPIO_MUXCFG_CS_SMODE_SHIFT        (3)       /* Bits 3-4: CLK_SOURCE_SLICE_MODE Select clock source slice */
#define SGPIO_MUXCFG_CS_SMODE_MASK         (3 << SGPIO_MUXCFG_CS_SMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_SMODE_SLICED     (0 << SGPIO_MUXCFG_CS_SMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_SMODE_SLICEH     (1 << SGPIO_MUXCFG_CS_SMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_SMODE_SLICEO     (2 << SGPIO_MUXCFG_CS_SMODE_SHIFT)
#  define SGPIO_MUXCFG_CS_SMODE_SLICEP     (3 << SGPIO_MUXCFG_CS_SMODE_SHIFT)
#define SGPIO_MUXCFG_QUAL_MODE_SHIFT       (5)       /* Bits 5-6: Select qualifier mode */
#define SGPIO_MUXCFG_QUAL_MODE_MASK        (3 << SGPIO_MUXCFG_QUAL_MODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_MODE_ENABLE    (0 << SGPIO_MUXCFG_QUAL_MODE_SHIFT) /* Enable */
#  define SGPIO_MUXCFG_QUAL_MODE_DISABLE   (1 << SGPIO_MUXCFG_QUAL_MODE_SHIFT) /* Disable */
#  define SGPIO_MUXCFG_QUAL_MODE_SLICE     (2 << SGPIO_MUXCFG_QUAL_MODE_SHIFT) /* Slice */
#  define SGPIO_MUXCFG_QUAL_MODE_SGPIO     (3 << SGPIO_MUXCFG_QUAL_MODE_SHIFT) /* External SGPIO pin (8, 9, 10, or 11) */
#define SGPIO_MUXCFG_QUAL_PMODE_SHIFT      (7)       /* Bits 7-8: Select qualifier pin */
#define SGPIO_MUXCFG_QUAL_PMODE_MASK       (3 << SGPIO_MUXCFG_QUAL_PMODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_PMODE_SGPIO8   (0 << SGPIO_MUXCFG_QUAL_PMODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_PMODE_SGPIO9   (1 << SGPIO_MUXCFG_QUAL_PMODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_PMODE_SGPIO10  (2 << SGPIO_MUXCFG_QUAL_PMODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_PMODE_SGPIO11  (3 << SGPIO_MUXCFG_QUAL_PMODE_SHIFT)
#define SGPIO_MUXCFG_QUAL_SMODE_SHIFT      (9)       /* Bits 9-10: Select qualifier slice */
#define SGPIO_MUXCFG_QUAL_SMODE_MASK       (3 << SGPIO_MUXCFG_QUAL_SMODE_SHIFT)
#  define SGPIO_MUXCFG_QUAL_SMODE_SLICEA   (0 << SGPIO_MUXCFG_QUAL_SMODE_SHIFT) /* Slice A, but for slice A slice D is used */
#  define SGPIO_MUXCFG_QUAL_SMODE_SLICEH   (1 << SGPIO_MUXCFG_QUAL_SMODE_SHIFT) /* Slice H, but for slice H slice O is used */
#  define SGPIO_MUXCFG_QUAL_SMODE_SLICEI   (2 << SGPIO_MUXCFG_QUAL_SMODE_SHIFT) /* Slice I, but for slice I slice D is used */
#  define SGPIO_MUXCFG_QUAL_SMODE_SLICEP   (3 << SGPIO_MUXCFG_QUAL_SMODE_SHIFT) /* Slice P, but for slice P slice O is used */
#define SGPIO_MUXCFG_CONCAT                (1 << 11) /* Bit 11: Enable concatenation */
#define SGPIO_MUXCFG_CONCAT_ORDER_SHIFT    (12)      /* Bits 12-13: CONCAT_ORDER Select concatenation order */
#define SGPIO_MUXCFG_CONCAT_ORDER_MASK     (3 << SGPIO_MUXCFG_CONCAT_ORDER_SHIFT)
#  define SGPIO_MUXCFG_CONCAT_ORDER_SELT   (0 << SGPIO_MUXCFG_CONCAT_ORDER_SHIFT) /* Self-loop */
#  define SGPIO_MUXCFG_CONCAT_ORDER_S2     (1 << SGPIO_MUXCFG_CONCAT_ORDER_SHIFT) /* 2 slices */
#  define SGPIO_MUXCFG_CONCAT_ORDER_S4     (2 << SGPIO_MUXCFG_CONCAT_ORDER_SHIFT) /* 4 slices */
#  define SGPIO_MUXCFG_CONCAT_ORDER_S8     (3 << SGPIO_MUXCFG_CONCAT_ORDER_SHIFT) /* 8 slices */
                                                     /* Bits 14-31:  Reserved */
/* Slice multiplexer configuration register 0 */

#define SGPIO_SLICE_MUXCFG_MATCH           (1 << 0)  /* Bit 0:  Match mode */
#define SGPIO_SLICE_MUXCFG_CAPTURE         (1 << 1)  /* Bit 1:  Capture clock mode */
#define SGPIO_SLICE_MUXCFG_CLKGEN          (1 << 2)  /* Bit 2:  Clock generation mode */
#define SGPIO_SLICE_MUXCFG_INTOUTCLK       (1 << 3)  /* Bit 3:  Invert output clock */
#define SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT   (4)       /* Bits 4-5: Condition for input bit match interrupt */
#define SGPIO_SLICE_MUXCFG_CAPMODE_MASK    (3 << SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT)
#  define SGPIO_SLICE_MUXCFG_CAPMODE_RISING  (0 << SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT) /* Detect rising edge */
#  define SGPIO_SLICE_MUXCFG_CAPMODE_FALLING (1 << SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT) /* Detect falling edge */
#  define SGPIO_SLICE_MUXCFG_CAPMODE_LOW     (2 << SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT) /* Detect LOW level */
#  define SGPIO_SLICE_MUXCFG_CAPMODE_HIGH    (3 << SGPIO_SLICE_MUXCFG_CAPMODE_SHIFT) /* Detect HIGH level */
#define SGPIO_SLICE_MUXCFG_PARMODE_SHIFT   (6)       /* Bits 6-7: Parallel mode */
#define SGPIO_SLICE_MUXCFG_PARMODE_MASK    (3 << SGPIO_SLICE_MUXCFG_PARMODE_SHIFT)
#  define SGPIO_SLICE_MUXCFG_PARMODE_SHIFT1  (0 << SGPIO_SLICE_MUXCFG_PARMODE_SHIFT) /* Shift 1 bit per clock */
#  define SGPIO_SLICE_MUXCFG_PARMODE_SHIFT2  (1 << SGPIO_SLICE_MUXCFG_PARMODE_SHIFT) /* Shift 2 bits per clock */
#  define SGPIO_SLICE_MUXCFG_PARMODE_SHIFT4  (2 << SGPIO_SLICE_MUXCFG_PARMODE_SHIFT) /* Shift 4 bits per clock */
#  define SGPIO_SLICE_MUXCFG_PARMODE_SHIFT8  (3 << SGPIO_SLICE_MUXCFG_PARMODE_SHIFT) /* Shift 1 byte per clock */
#define SGPIO_SLICE_MUXCFG_INVQUAL         (1 << 8)  /* Bit 8:  Inversion qualifier */
                                                     /* Bits 9-31: Reserved */
/* Slice data registers (32-bit data) */
/* Slice data shadow registers (32-bit data) */

/* COUNTn reload value (32-bit data) */

#define SGPIO_PRESET_MASK                  (0xfff)   /* Bits 0-11: Counter reload value */
                                                     /* Bits 12-31: Reserved */
/* Down counter registers */

#define SGPIO_COUNT_MASK                   (0xfff)   /* Bits 0-11: Down counter */
                                                     /* Bits 12-31: Reserved */
/* Position registers */

#define SGPIO_POS_POS_SHIFT                (0)       /* Bits 0-7: Each time COUNT reaches zero POS counts down */
#define SGPIO_POS_POS_MASK                 (0xffff << SGPIO_POS_POS_SHIFT)
#define SGPIO_POS_RESET_SHIFT              (8)       /* Bits 8-15: Reload value for POS after POS reaches zero */
#define SGPIO_POS_RESET_MASK               (0xffff << SGPIO_POS_RESET_SHIFT)
                                                     /* Bits 16-31: Reserved */
/* Mask for pattern match function of slice A (32-bit bit mask) */
/* Mask for pattern match function of slice H (32-bit bit mask) */
/* Mask for pattern match function of slice I (32-bit bit mask) */
/* Mask for pattern match function of slice P (32-bit bit mask) */

/* Common bit mask that can be used in all interrupt registers */

#define SGPIO_SLICE(n)                     (1 << (n)) /* Bits 0-15: Bit n corresponids to slice n */
                                                      /* Bits 16-31: Reserved */
/* GPIO input status register */

#define SGPIO_GPIO_INREG(n)                (1 << (n)) /* Bits 0-15: Bit i reflects the input state of SGPIO pin  */
                                                      /* Bits 16-31: Reserved */
/* GPIO output control register */

#define SGPIO_GPIO_OUTREG(n)               (1 << (n)) /* Bits 0-15: Bit i sets the output of SGPIO pin i */
                                                      /* Bits 16-31: Reserved */
/* GPIO output enable register */

#define SGPIO_GPIO_OENREG(n)               (1 << (n)) /* Bits 0-15: Bit i selects the output enable state of SGPIO pin i */
                                                      /* Bits 16-31: Reserved */
/* Slice count enable register */

#define SGPIO_CTRL_ENABLE(n)               (1 << (n)) /* Bits 0-15: Bit n controls slice n */
                                                      /* Bits 16-31: Reserved */
/* Slice count disable register */

#define SGPIO_CTRL_DISABLE(n)              (1 << (n)) /* Bits 0-15: Bit n controls slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt clear mask */

#define SGPIO_CLREN0(n)                    (1 << (n)) /* Bits 0-15: Bit n shift clock interrupt clear mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt set mask */

#define SGPIO_SETEN0(n)                    (1 << (n)) /* Bits 0-15: Bit n shift clock interrupt clear mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt enable */

#define SGPIO_ENABLE0(n)                   (1 << (n)) /* Bits 0-15: Bit n shift clock interrupt enable of slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt status */

#define SGPIO_STATUS0(n)                   (1 << (n)) /* Bits 0-15: Bit n shift clock interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt clear status */

#define SGPIO_CLRSTAT0(n)                  (1 << (n)) /* Bits 0-15: Bit n shift clears interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Shift clock interrupt set status */

#define SGPIO_SETSTAT0(n)                  (1 << (n)) /* Bits 0-15: Bit n shift sets interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt clear mask */

#define SGPIO_CLREN1(n)                    (1 << (n)) /* Bits 0-15: Bit n clears exchange clock interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt set mask */

#define SGPIO_SETEN1(n)                    (1 << (n)) /* Bits 0-15: Bit n sets exchange clock interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt enable */

#define SGPIO_ENABLE1(n)                   (1 << (n)) /* Bits 0-15: Bit n enables exchange clock interrupt of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt status */

#define SGPIO_STATUS1(n)                   (1 << (n)) /* Bits 0-15: Bit n status of exchange clock interrupt of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt clear status */

#define SGPIO_CLRSTAT1(n)                  (1 << (n)) /* Bits 0-15: Bit n clears exchange clock interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Exchange clock interrupt set status */

#define SGPIO_SETSTAT1(n)                  (1 << (n)) /* Bits 0-15: Bit n sets exchange clock interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt clear mask */

#define SGPIO_CLREN2(n)                    (1 << (n)) /* Bits 0-15: Bit n clears match interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt set mask */

#define SGPIO_SETEN2(n)                    (1 << (n)) /* Bits 0-15: Bit n sets match interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt enable */

#define SGPIO_ENABLE2(n)                   (1 << (n)) /* Bits 0-15: Bit n enables match interrupt of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt status */

#define SGPIO_STATUS2(n)                   (1 << (n)) /* Bits 0-15: Bit n is match interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt clear status */

#define SGPIO_CLRSTAT2(n)                  (1 << (n)) /* Bits 0-15: Bit n sets match interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Pattern match interrupt set status */

#define SGPIO_SETSTAT2(n)                  (1 << (n)) /* Bits 0-15: Bit n sets match interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input interrupt clear mask */

#define SGPIO_CLREN3(n)                    (1 << (n)) /* Bits 0-15: Bit n clears input interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input bit match interrupt set mask */

#define SGPIO_SETEN3(n)                    (1 << (n)) /* Bits 0-15: Bit n sets input interrupt mask of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input bit match interrupt enable */

#define SGPIO_ENABLE3(n)                   (1 << (n)) /* Bits 0-15: Bit n enables input interrupt of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input bit match interrupt status */

#define SGPIO_STATUS3(n)                   (1 << (n)) /* Bits 0-15: Bit n is input interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input bit match interrupt clear status */

#define SGPIO_CLRSTAT3(n)                  (1 << (n)) /* Bits 0-15: Bit n clears input interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */
/* Input bit match interrupt set status */

#define SGPIO_SETSTAT3(n)                  (1 << (n)) /* Bits 0-15: Bit n sets match interrupt status of slice n */
                                                      /* Bits 16-31: Reserved */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SGPIO_H */
