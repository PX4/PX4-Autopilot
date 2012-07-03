/****************************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_sct.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCT_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCT_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* Register Offsets *********************************************************************************/

#define LPC43_SCT_CONFIG_OFFSET     0x0000 /* SCT configuration register */
#define LPC43_SCT_CTRL_OFFSET       0x0004 /* SCT control register */
#define LPC43_SCT_CTRLL_OFFSET      0x0004 /* SCT control register low 16-bit */
#define LPC43_SCT_CTRLH_OFFSET      0x0006 /* SCT control register high 16-bit */
#define LPC43_SCT_LIMIT_OFFSET      0x0008 /* SCT limit register */
#define LPC43_SCT_LIMITL_OFFSET     0x0008 /* SCT limit register low 16-bit */
#define LPC43_SCT_LIMITH_OFFSET     0x000a /* SCT limit register high 16-bit */
#define LPC43_SCT_HALT_OFFSET       0x000c /* SCT halt condition register */
#define LPC43_SCT_HALTL_OFFSET      0x000c /* SCT halt condition register low 16-bit */
#define LPC43_SCT_HALTH_OFFSET      0x000e /* SCT halt condition register high 16-bit */
#define LPC43_SCT_STOP_OFFSET       0x0010 /* SCT stop condition register */
#define LPC43_SCT_STOPL_OFFSET      0x0010 /* SCT stop condition register low 16-bit */
#define LPC43_SCT_STOPH_OFFSET      0x0012 /* SCT stop condition register high 16-bit */
#define LPC43_SCT_START_OFFSET      0x0014 /* SCT start condition register */
#define LPC43_SCT_STARTL_OFFSET     0x0014 /* SCT start condition register low 16-bit */
#define LPC43_SCT_STARTH_OFFSET     0x0016 /* SCT start condition register high 16-bit */

#define LPC43_SCT_COUNT_OFFSET      0x0040 /* SCT counter register */
#define LPC43_SCT_COUNTL_OFFSET     0x0040 /* SCT counter register low 16-bit */
#define LPC43_SCT_COUNTH_OFFSET     0x0042 /* SCT counter register high 16-bit */
#define LPC43_SCT_STATE_OFFSET      0x0044 /* SCT state register */
#define LPC43_SCT_STATEL_OFFSET     0x0044 /* SCT state register low 16-bit */
#define LPC43_SCT_STATEH_OFFSET     0x0046 /* SCT state register high 16-bit */
#define LPC43_SCT_INPUT_OFFSET      0x0048 /* SCT input register */
#define LPC43_SCT_REGM_OFFSET       0x004c /* SCT match/capture registers mode register */
#define LPC43_SCT_REGML_OFFSET      0x004c /* SCT match/capture registers mode register low 16-bit */
#define LPC43_SCT_REGMH_OFFSET      0x004e /* SCT match/capture registers mode register high 16-bit */
#define LPC43_SCT_OUT_OFFSET        0x0050 /* SCT output register */
#define LPC43_SCT_OUTDIRC_OFFSET    0x0054 /* SCT output counter direction control register */
#define LPC43_SCT_RES_OFFSET        0x0058 /* SCT conflict resolution register */
#define LPC43_SCT_DMAREQ0_OFFSET    0x005c /* SCT DMA request 0 register */
#define LPC43_SCT_DMAREQ1_OFFSET    0x0060 /* SCT DMA request 1 register */

#define LPC43_SCT_EVEN_OFFSET       0x00f0 /* SCT event enable register */
#define LPC43_SCT_EVFLAG_OFFSET     0x00f4 /* SCT event flag register */
#define LPC43_SCT_CONEN_OFFSET      0x00f8 /* SCT conflict enable register */
#define LPC43_SCT_CONFLAG_OFFSET    0x00fC /* SCT conflict flag register */

#define LPC43_SCT_MATCH_OFFSET(n)   (0x0100 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0_OFFSET     0x0100 /* SCT match value register of match channel 0 */
#define LPC43_SCT_MATCH1_OFFSET     0x0104 /* SCT match value register of match channel 1 */
#define LPC43_SCT_MATCH2_OFFSET     0x0108 /* SCT match value register of match channel 2 */
#define LPC43_SCT_MATCH3_OFFSET     0x010c /* SCT match value register of match channel 3 */
#define LPC43_SCT_MATCH4_OFFSET     0x0110 /* SCT match value register of match channel 4 */
#define LPC43_SCT_MATCH5_OFFSET     0x0114 /* SCT match value register of match channel 5 */
#define LPC43_SCT_MATCH6_OFFSET     0x0118 /* SCT match value register of match channel 6 */
#define LPC43_SCT_MATCH7_OFFSET     0x011c /* SCT match value register of match channel 7 */
#define LPC43_SCT_MATCH8_OFFSET     0x0120 /* SCT match value register of match channel 8 */
#define LPC43_SCT_MATCH9_OFFSET     0x0124 /* SCT match value register of match channel 9 */
#define LPC43_SCT_MATCH10_OFFSET    0x0128 /* SCT match value register of match channel 10 */
#define LPC43_SCT_MATCH11_OFFSET    0x012c /* SCT match value register of match channel 11 */
#define LPC43_SCT_MATCH12_OFFSET    0x0130 /* SCT match value register of match channel 12 */
#define LPC43_SCT_MATCH13_OFFSET    0x0134 /* SCT match value register of match channel 13 */
#define LPC43_SCT_MATCH14_OFFSET    0x0138 /* SCT match value register of match channel 14 */
#define LPC43_SCT_MATCH15_OFFSET    0x013c /* SCT match value register of match channel 15 */

#define LPC43_SCT_MATCHL_OFFSET(n) (0x0100 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0L_OFFSET    0x0100 /* SCT match value register of match channel 0; low 16-bit */
#define LPC43_SCT_MATCH1L_OFFSET    0x0104 /* SCT match value register of match channel 1; low 16-bit */
#define LPC43_SCT_MATCH2L_OFFSET    0x0108 /* SCT match value register of match channel 2; low 16-bit */
#define LPC43_SCT_MATCH3L_OFFSET    0x010c /* SCT match value register of match channel 3; low 16-bit */
#define LPC43_SCT_MATCH4L_OFFSET    0x0110 /* SCT match value register of match channel 4; low 16-bit */
#define LPC43_SCT_MATCH5L_OFFSET    0x0114 /* SCT match value register of match channel 5; low 16-bit */
#define LPC43_SCT_MATCH6L_OFFSET    0x0118 /* SCT match value register of match channel 6; low 16-bit */
#define LPC43_SCT_MATCH7L_OFFSET    0x011c /* SCT match value register of match channel 7; low 16-bit */
#define LPC43_SCT_MATCH8L_OFFSET    0x0120 /* SCT match value register of match channel 8; low 16-bit */
#define LPC43_SCT_MATCH9L_OFFSET    0x0124 /* SCT match value register of match channel 9; low 16-bit */
#define LPC43_SCT_MATCH10L_OFFSET   0x0128 /* SCT match value register of match channel 10; low 16-bit */
#define LPC43_SCT_MATCH11L_OFFSET   0x012c /* SCT match value register of match channel 11; low 16-bit */
#define LPC43_SCT_MATCH12L_OFFSET   0x0130 /* SCT match value register of match channel 12; low 16-bit */
#define LPC43_SCT_MATCH13L_OFFSET   0x0134 /* SCT match value register of match channel 13; low 16-bit */
#define LPC43_SCT_MATCH14L_OFFSET   0x0138 /* SCT match value register of match channel 14; low 16-bit */
#define LPC43_SCT_MATCH15L_OFFSET   0x013c /* SCT match value register of match channel 15; low 16-bit */

#define LPC43_SCT_MATCHH_OFFSET(n)  (0x0102 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0H_OFFSET    0x0102 /* SCT match value register of match channel 0; high 16-bit */
#define LPC43_SCT_MATCH1H_OFFSET    0x0106 /* SCT match value register of match channel 1; high 16-bit */
#define LPC43_SCT_MATCH2H_OFFSET    0x010a /* SCT match value register of match channel 2; high 16-bit */
#define LPC43_SCT_MATCH3H_OFFSET    0x010e /* SCT match value register of match channel 3; high 16-bit */
#define LPC43_SCT_MATCH4H_OFFSET    0x0112 /* SCT match value register of match channel 4; high 16-bit */
#define LPC43_SCT_MATCH5H_OFFSET    0x0116 /* SCT match value register of match channel 5; high 16-bit */
#define LPC43_SCT_MATCH6H_OFFSET    0x011a /* SCT match value register of match channel 6; high 16-bit */
#define LPC43_SCT_MATCH7H_OFFSET    0x011e /* SCT match value register of match channel 7; high 16-bit */
#define LPC43_SCT_MATCH8H_OFFSET    0x0122 /* SCT match value register of match channel 8; high 16-bit */
#define LPC43_SCT_MATCH9H_OFFSET    0x0126 /* SCT match value register of match channel 9; high 16-bit */
#define LPC43_SCT_MATCH10H_OFFSET   0x012a /* SCT match value register of match channel 10; high 16-bit */
#define LPC43_SCT_MATCH11H_OFFSET   0x012e /* SCT match value register of match channel 11; high 16-bit */
#define LPC43_SCT_MATCH12H_OFFSET   0x0132 /* SCT match value register of match channel 12; high 16-bit */
#define LPC43_SCT_MATCH13H_OFFSET   0x0136 /* SCT match value register of match channel 13; high 16-bit */
#define LPC43_SCT_MATCH14H_OFFSET   0x013a /* SCT match value register of match channel 14; high 16-bit */
#define LPC43_SCT_MATCH15H_OFFSET   0x013e /* SCT match value register of match channel 15; high 16-bit */

#define LPC43_SCT_CAP_OFFSET(n)     (0x0100 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0_OFFSET       0x0100 /* SCT capture value register Ch0 */
#define LPC43_SCT_CAP1_OFFSET       0x0104 /* SCT capture value register Ch1 */
#define LPC43_SCT_CAP2_OFFSET       0x0108 /* SCT capture value register Ch2 */
#define LPC43_SCT_CAP3_OFFSET       0x010c /* SCT capture value register Ch3 */
#define LPC43_SCT_CAP4_OFFSET       0x0110 /* SCT capture value register Ch4 */
#define LPC43_SCT_CAP5_OFFSET       0x0114 /* SCT capture value register Ch5 */
#define LPC43_SCT_CAP6_OFFSET       0x0118 /* SCT capture value register Ch6 */
#define LPC43_SCT_CAP7_OFFSET       0x011c /* SCT capture value register Ch7 */
#define LPC43_SCT_CAP8_OFFSET       0x0120 /* SCT capture value register Ch8 */
#define LPC43_SCT_CAP9_OFFSET       0x0124 /* SCT capture value register Ch9 */
#define LPC43_SCT_CAP10_OFFSET      0x0128 /* SCT capture value register Ch10 */
#define LPC43_SCT_CAP11_OFFSET      0x012c /* SCT capture value register Ch11 */
#define LPC43_SCT_CAP12_OFFSET      0x0130 /* SCT capture value register Ch12 */
#define LPC43_SCT_CAP13_OFFSET      0x0134 /* SCT capture value register Ch13 */
#define LPC43_SCT_CAP14_OFFSET      0x0138 /* SCT capture value register Ch14 */
#define LPC43_SCT_CAP15_OFFSET      0x013c /* SCT capture value register Ch15 */

#define LPC43_SCT_CAPL_OFFSET(n)    (0x0100 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0L_OFFSET      0x0100 /* SCT capture value register Ch0; low 16-bit */
#define LPC43_SCT_CAP1L_OFFSET      0x0104 /* SCT capture value register Ch1; low 16-bit */
#define LPC43_SCT_CAP2L_OFFSET      0x0108 /* SCT capture value register Ch2; low 16-bit */
#define LPC43_SCT_CAP3L_OFFSET      0x010c /* SCT capture value register Ch3; low 16-bit */
#define LPC43_SCT_CAP4L_OFFSET      0x0110 /* SCT capture value register Ch4; low 16-bit */
#define LPC43_SCT_CAP5L_OFFSET      0x0114 /* SCT capture value register Ch5; low 16-bit */
#define LPC43_SCT_CAP6L_OFFSET      0x0118 /* SCT capture value register Ch6; low 16-bit */
#define LPC43_SCT_CAP7L_OFFSET      0x011c /* SCT capture value register Ch7; low 16-bit */
#define LPC43_SCT_CAP8L_OFFSET      0x0120 /* SCT capture value register Ch8; low 16-bit */
#define LPC43_SCT_CAP9L_OFFSET      0x0124 /* SCT capture value register Ch9; low 16-bit */
#define LPC43_SCT_CAP10L_OFFSET     0x0128 /* SCT capture value register Ch10; low 16-bit */
#define LPC43_SCT_CAP11L_OFFSET     0x012c /* SCT capture value register Ch11; low 16-bit */
#define LPC43_SCT_CAP12L_OFFSET     0x0130 /* SCT capture value register Ch12; low 16-bit */
#define LPC43_SCT_CAP13L_OFFSET     0x0134 /* SCT capture value register Ch13; low 16-bit */
#define LPC43_SCT_CAP14L_OFFSET     0x0138 /* SCT capture value register Ch14; low 16-bit */
#define LPC43_SCT_CAP15L_OFFSET     0x013c /* SCT capture value register Ch15; low 16-bit */

#define LPC43_SCT_CAPH_OFFSET(n)    (0x0102 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0H_OFFSET      0x0102 /* SCT capture value register Ch0; high 16-bit */
#define LPC43_SCT_CAP1H_OFFSET      0x0106 /* SCT capture value register Ch1; high 16-bit */
#define LPC43_SCT_CAP2H_OFFSET      0x010a /* SCT capture value register Ch2; high 16-bit */
#define LPC43_SCT_CAP3H_OFFSET      0x010e /* SCT capture value register Ch3; high 16-bit */
#define LPC43_SCT_CAP4H_OFFSET      0x0112 /* SCT capture value register Ch4; high 16-bit */
#define LPC43_SCT_CAP5H_OFFSET      0x0116 /* SCT capture value register Ch5; high 16-bit */
#define LPC43_SCT_CAP6H_OFFSET      0x011a /* SCT capture value register Ch6; high 16-bit */
#define LPC43_SCT_CAP7H_OFFSET      0x011e /* SCT capture value register Ch7; high 16-bit */
#define LPC43_SCT_CAP8H_OFFSET      0x0122 /* SCT capture value register Ch8; high 16-bit */
#define LPC43_SCT_CAP9H_OFFSET      0x0126 /* SCT capture value register Ch9; high 16-bit */
#define LPC43_SCT_CAP10H_OFFSET     0x012a /* SCT capture value register Ch10; high 16-bit */
#define LPC43_SCT_CAP11H_OFFSET     0x012e /* SCT capture value register Ch11; high 16-bit */
#define LPC43_SCT_CAP12H_OFFSET     0x0132 /* SCT capture value register Ch12; high 16-bit */
#define LPC43_SCT_CAP13H_OFFSET     0x0136 /* SCT capture value register Ch13; high 16-bit */
#define LPC43_SCT_CAP14H_OFFSET     0x013a /* SCT capture value register Ch14; high 16-bit */
#define LPC43_SCT_CAP15H_OFFSET     0x013e /* SCT capture value register Ch15; high 16-bit */

#define LPC43_SCT_MATCHA_OFFSET(n)  (0x0180 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0A_OFFSET    0x0180 /* SCT match alias register of match channel 0 */
#define LPC43_SCT_MATCH1A_OFFSET    0x0184 /* SCT match alias register of match channel 1 */
#define LPC43_SCT_MATCH2A_OFFSET    0x0188 /* SCT match alias register of match channel 2 */
#define LPC43_SCT_MATCH3A_OFFSET    0x018c /* SCT match alias register of match channel 3 */
#define LPC43_SCT_MATCH4A_OFFSET    0x0190 /* SCT match alias register of match channel 4 */
#define LPC43_SCT_MATCH5A_OFFSET    0x0194 /* SCT match alias register of match channel 5 */
#define LPC43_SCT_MATCH6A_OFFSET    0x0198 /* SCT match alias register of match channel 6 */
#define LPC43_SCT_MATCH7A_OFFSET    0x019c /* SCT match alias register of match channel 7 */
#define LPC43_SCT_MATCH8A_OFFSET    0x01a0 /* SCT match alias register of match channel 8 */
#define LPC43_SCT_MATCH9A_OFFSET    0x01a4 /* SCT match alias register of match channel 9 */
#define LPC43_SCT_MATCH10A_OFFSET   0x01a8 /* SCT match alias register of match channel 10 */
#define LPC43_SCT_MATCH11A_OFFSET   0x01ac /* SCT match alias register of match channel 11 */
#define LPC43_SCT_MATCH12A_OFFSET   0x01b0 /* SCT match alias register of match channel 12 */
#define LPC43_SCT_MATCH13A_OFFSET   0x01b4 /* SCT match alias register of match channel 13 */
#define LPC43_SCT_MATCH14A_OFFSET   0x01b8 /* SCT match alias register of match channel 14 */
#define LPC43_SCT_MATCH15A_OFFSET   0x01bc /* SCT match alias register of match channel 15 */

#define LPC43_SCT_MATCHLA_OFFSET(n) (0x0180 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0LA_OFFSET   0x0180 /* SCT match alias register of match channel 0; low 16-bit */
#define LPC43_SCT_MATCH1LA_OFFSET   0x0184 /* SCT match alias register of match channel 1; low 16-bit */
#define LPC43_SCT_MATCH2LA_OFFSET   0x0188 /* SCT match alias register of match channel 2; low 16-bit */
#define LPC43_SCT_MATCH3LA_OFFSET   0x018c /* SCT match alias register of match channel 3; low 16-bit */
#define LPC43_SCT_MATCH4LA_OFFSET   0x0190 /* SCT match alias register of match channel 4; low 16-bit */
#define LPC43_SCT_MATCH5LA_OFFSET   0x0194 /* SCT match alias register of match channel 5; low 16-bit */
#define LPC43_SCT_MATCH6LA_OFFSET   0x0198 /* SCT match alias register of match channel 6; low 16-bit */
#define LPC43_SCT_MATCH7LA_OFFSET   0x019c /* SCT match alias register of match channel 7; low 16-bit */
#define LPC43_SCT_MATCH8LA_OFFSET   0x01a0 /* SCT match alias register of match channel 8; low 16-bit */
#define LPC43_SCT_MATCH9LA_OFFSET   0x01a4 /* SCT match alias register of match channel 9; low 16-bit */
#define LPC43_SCT_MATCH10LA_OFFSET  0x01a8 /* SCT match alias register of match channel 10; low 16-bit */
#define LPC43_SCT_MATCH11LA_OFFSET  0x01ac /* SCT match alias register of match channel 11; low 16-bit */
#define LPC43_SCT_MATCH12LA_OFFSET  0x01b0 /* SCT match alias register of match channel 12; low 16-bit */
#define LPC43_SCT_MATCH13LA_OFFSET  0x01b4 /* SCT match alias register of match channel 13; low 16-bit */
#define LPC43_SCT_MATCH14LA_OFFSET  0x01b8 /* SCT match alias register of match channel 14; low 16-bit */
#define LPC43_SCT_MATCH15LA_OFFSET  0x01bc /* SCT match alias register of match channel 15; low 16-bit */

#define LPC43_SCT_MATCHHA_OFFSET(n) (0x0182 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCH0HA_OFFSET   0x0182 /* SCT match alias register of match channel 0; high 16-bit */
#define LPC43_SCT_MATCH1HA_OFFSET   0x0186 /* SCT match alias register of match channel 1; high 16-bit */
#define LPC43_SCT_MATCH2HA_OFFSET   0x018a /* SCT match alias register of match channel 2; high 16-bit */
#define LPC43_SCT_MATCH3HA_OFFSET   0x018e /* SCT match alias register of match channel 3; high 16-bit */
#define LPC43_SCT_MATCH4HA_OFFSET   0x0192 /* SCT match alias register of match channel 4; high 16-bit */
#define LPC43_SCT_MATCH5HA_OFFSET   0x0196 /* SCT match alias register of match channel 5; high 16-bit */
#define LPC43_SCT_MATCH6HA_OFFSET   0x019a /* SCT match alias register of match channel 6; high 16-bit */
#define LPC43_SCT_MATCH7HA_OFFSET   0x019e /* SCT match alias register of match channel 7; high 16-bit */
#define LPC43_SCT_MATCH8HA_OFFSET   0x01a2 /* SCT match alias register of match channel 8; high 16-bit */
#define LPC43_SCT_MATCH9HA_OFFSET   0x01a6 /* SCT match alias register of match channel 9; high 16-bit */
#define LPC43_SCT_MATCH10HA_OFFSET  0x01aa /* SCT match alias register of match channel 10; high 16-bit */
#define LPC43_SCT_MATCH11HA_OFFSET  0x01ae /* SCT match alias register of match channel 11; high 16-bit */
#define LPC43_SCT_MATCH12HA_OFFSET  0x01b2 /* SCT match alias register of match channel 12; high 16-bit */
#define LPC43_SCT_MATCH13HA_OFFSET  0x01b6 /* SCT match alias register of match channel 13; high 16-bit */
#define LPC43_SCT_MATCH14HA_OFFSET  0x01ba /* SCT match alias register of match channel 14; high 16-bit */
#define LPC43_SCT_MATCH15HA_OFFSET  0x01be /* SCT match alias register of match channel 15; high 16-bit */

#define LPC43_SCT_CAPA_OFFSET(n)    (0x0180 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0A_OFFSET      0x0180 /* SCT capture alias register Ch0 */
#define LPC43_SCT_CAP1A_OFFSET      0x0184 /* SCT capture alias register Ch1 */
#define LPC43_SCT_CAP2A_OFFSET      0x0188 /* SCT capture alias register Ch2 */
#define LPC43_SCT_CAP3A_OFFSET      0x018c /* SCT capture alias register Ch3 */
#define LPC43_SCT_CAP4A_OFFSET      0x0190 /* SCT capture alias register Ch4 */
#define LPC43_SCT_CAP5A_OFFSET      0x0194 /* SCT capture alias register Ch5 */
#define LPC43_SCT_CAP6A_OFFSET      0x0198 /* SCT capture alias register Ch6 */
#define LPC43_SCT_CAP7A_OFFSET      0x019c /* SCT capture alias register Ch7 */
#define LPC43_SCT_CAP8A_OFFSET      0x01a0 /* SCT capture alias register Ch8 */
#define LPC43_SCT_CAP9A_OFFSET      0x01a4 /* SCT capture alias register Ch9 */
#define LPC43_SCT_CAP10A_OFFSET     0x01a8 /* SCT capture alias register Ch10 */
#define LPC43_SCT_CAP11A_OFFSET     0x01ac /* SCT capture alias register Ch11 */
#define LPC43_SCT_CAP12A_OFFSET     0x01b0 /* SCT capture alias register Ch12 */
#define LPC43_SCT_CAP13A_OFFSET     0x01b4 /* SCT capture alias register Ch13 */
#define LPC43_SCT_CAP14A_OFFSET     0x01b8 /* SCT capture alias register Ch14 */
#define LPC43_SCT_CAP15A_OFFSET     0x01bc /* SCT capture alias register Ch15 */

#define LPC43_SCT_CAPLA_OFFSET(n)   (0x0180 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0LA_OFFSET     0x0180 /* SCT capture alias register Ch0; low 16-bit */
#define LPC43_SCT_CAP1LA_OFFSET     0x0184 /* SCT capture alias register Ch1; low 16-bit */
#define LPC43_SCT_CAP2LA_OFFSET     0x0188 /* SCT capture alias register Ch2; low 16-bit */
#define LPC43_SCT_CAP3LA_OFFSET     0x018c /* SCT capture alias register Ch3; low 16-bit */
#define LPC43_SCT_CAP4LA_OFFSET     0x0190 /* SCT capture alias register Ch4; low 16-bit */
#define LPC43_SCT_CAP5LA_OFFSET     0x0194 /* SCT capture alias register Ch5; low 16-bit */
#define LPC43_SCT_CAP6LA_OFFSET     0x0198 /* SCT capture alias register Ch6; low 16-bit */
#define LPC43_SCT_CAP7LA_OFFSET     0x019c /* SCT capture alias register Ch7; low 16-bit */
#define LPC43_SCT_CAP8LA_OFFSET     0x01a0 /* SCT capture alias register Ch8; low 16-bit */
#define LPC43_SCT_CAP9LA_OFFSET     0x01a4 /* SCT capture alias register Ch9; low 16-bit */
#define LPC43_SCT_CAP10LA_OFFSET    0x01a8 /* SCT capture alias register Ch10; low 16-bit */
#define LPC43_SCT_CAP11LA_OFFSET    0x01ac /* SCT capture alias register Ch11; low 16-bit */
#define LPC43_SCT_CAP12LA_OFFSET    0x01b0 /* SCT capture alias register Ch12; low 16-bit */
#define LPC43_SCT_CAP13LA_OFFSET    0x01b4 /* SCT capture alias register Ch13; low 16-bit */
#define LPC43_SCT_CAP14LA_OFFSET    0x01b8 /* SCT capture alias register Ch14; low 16-bit */
#define LPC43_SCT_CAP15LA_OFFSET    0x01bc /* SCT capture alias register Ch15; low 16-bit */

#define LPC43_SCT_CAPHA_OFFSET(n)   (0x0182 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAP0HA_OFFSET     0x0182 /* SCT capture alias register Ch0; high 16-bit */
#define LPC43_SCT_CAP1HA_OFFSET     0x0186 /* SCT capture alias register Ch1; high 16-bit */
#define LPC43_SCT_CAP2HA_OFFSET     0x018a /* SCT capture alias register Ch2; high 16-bit */
#define LPC43_SCT_CAP3HA_OFFSET     0x018e /* SCT capture alias register Ch3; high 16-bit */
#define LPC43_SCT_CAP4HA_OFFSET     0x0192 /* SCT capture alias register Ch4; high 16-bit */
#define LPC43_SCT_CAP5HA_OFFSET     0x0196 /* SCT capture alias register Ch5; high 16-bit */
#define LPC43_SCT_CAP6HA_OFFSET     0x019a /* SCT capture alias register Ch6; high 16-bit */
#define LPC43_SCT_CAP7HA_OFFSET     0x019e /* SCT capture alias register Ch7; high 16-bit */
#define LPC43_SCT_CAP8HA_OFFSET     0x01a2 /* SCT capture alias register Ch8; high 16-bit */
#define LPC43_SCT_CAP9HA_OFFSET     0x01a6 /* SCT capture alias register Ch9; high 16-bit */
#define LPC43_SCT_CAP10HA_OFFSET    0x01aa /* SCT capture alias register Ch10; high 16-bit */
#define LPC43_SCT_CAP11HA_OFFSET    0x01ae /* SCT capture alias register Ch11; high 16-bit */
#define LPC43_SCT_CAP12HA_OFFSET    0x01b2 /* SCT capture alias register Ch12; high 16-bit */
#define LPC43_SCT_CAP13HA_OFFSET    0x01b6 /* SCT capture alias register Ch13; high 16-bit */
#define LPC43_SCT_CAP14HA_OFFSET    0x01ba /* SCT capture alias register Ch14; high 16-bit */
#define LPC43_SCT_CAP15HA_OFFSET    0x01be /* SCT capture alias register Ch15; high 16-bit */

#define LPC43_SCT_MATCHR_OFFSET(n)  (0x0200 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0_OFFSET    0x0200 /* SCT match reload register of match channel 0 */
#define LPC43_SCT_MATCHR1_OFFSET    0x0204 /* SCT match reload register of match channel 1 */
#define LPC43_SCT_MATCHR2_OFFSET    0x0208 /* SCT match reload register of match channel 2 */
#define LPC43_SCT_MATCHR3_OFFSET    0x020c /* SCT match reload register of match channel 3 */
#define LPC43_SCT_MATCHR4_OFFSET    0x0210 /* SCT match reload register of match channel 4 */
#define LPC43_SCT_MATCHR5_OFFSET    0x0214 /* SCT match reload register of match channel 5 */
#define LPC43_SCT_MATCHR6_OFFSET    0x0218 /* SCT match reload register of match channel 6 */
#define LPC43_SCT_MATCHR7_OFFSET    0x021c /* SCT match reload register of match channel 7 */
#define LPC43_SCT_MATCHR8_OFFSET    0x0220 /* SCT match reload register of match channel 8 */
#define LPC43_SCT_MATCHR9_OFFSET    0x0224 /* SCT match reload register of match channel 9 */
#define LPC43_SCT_MATCHR10_OFFSET   0x0228 /* SCT match reload register of match channel 10 */
#define LPC43_SCT_MATCHR11_OFFSET   0x022c /* SCT match reload register of match channel 11 */
#define LPC43_SCT_MATCHR12_OFFSET   0x0230 /* SCT match reload register of match channel 12 */
#define LPC43_SCT_MATCHR13_OFFSET   0x0234 /* SCT match reload register of match channel 13 */
#define LPC43_SCT_MATCHR14_OFFSET   0x0238 /* SCT match reload register of match channel 14 */
#define LPC43_SCT_MATCHR15_OFFSET   0x023c /* SCT match reload register of match channel 15 */

#define LPC43_SCT_MATCHRL_OFFSET(n) (0x0200 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0L_OFFSET   0x0200 /* SCT match reload register of match channel 0; low 16-bit */
#define LPC43_SCT_MATCHR1L_OFFSET   0x0204 /* SCT match reload register of match channel 1; low 16-bit */
#define LPC43_SCT_MATCHR2L_OFFSET   0x0208 /* SCT match reload register of match channel 2; low 16-bit */
#define LPC43_SCT_MATCHR3L_OFFSET   0x020c /* SCT match reload register of match channel 3; low 16-bit */
#define LPC43_SCT_MATCHR4L_OFFSET   0x0210 /* SCT match reload register of match channel 4; low 16-bit */
#define LPC43_SCT_MATCHR5L_OFFSET   0x0214 /* SCT match reload register of match channel 5; low 16-bit */
#define LPC43_SCT_MATCHR6L_OFFSET   0x0218 /* SCT match reload register of match channel 6; low 16-bit */
#define LPC43_SCT_MATCHR7L_OFFSET   0x021c /* SCT match reload register of match channel 7; low 16-bit */
#define LPC43_SCT_MATCHR8L_OFFSET   0x0220 /* SCT match reload register of match channel 8; low 16-bit */
#define LPC43_SCT_MATCHR9L_OFFSET   0x0224 /* SCT match reload register of match channel 9; low 16-bit */
#define LPC43_SCT_MATCHR10L_OFFSET  0x0228 /* SCT match reload register of match channel 10; low 16-bit */
#define LPC43_SCT_MATCHR11L_OFFSET  0x022c /* SCT match reload register of match channel 11; low 16-bit */
#define LPC43_SCT_MATCHR12L_OFFSET  0x0230 /* SCT match reload register of match channel 12; low 16-bit */
#define LPC43_SCT_MATCHR13L_OFFSET  0x0234 /* SCT match reload register of match channel 13; low 16-bit */
#define LPC43_SCT_MATCHR14L_OFFSET  0x0238 /* SCT match reload register of match channel 14; low 16-bit */
#define LPC43_SCT_MATCHR15L_OFFSET  0x023c /* SCT match reload register of match channel 15; low 16-bit */

#define LPC43_SCT_MATCHRH_OFFSET(n) (0x0202 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0H_OFFSET   0x0202 /* SCT match reload register of match channel 0; high 16-bit */
#define LPC43_SCT_MATCHR1H_OFFSET   0x0206 /* SCT match reload register of match channel 1; high 16-bit */
#define LPC43_SCT_MATCHR2H_OFFSET   0x020a /* SCT match reload register of match channel 2; high 16-bit */
#define LPC43_SCT_MATCHR3H_OFFSET   0x020e /* SCT match reload register of match channel 3; high 16-bit */
#define LPC43_SCT_MATCHR4H_OFFSET   0x0212 /* SCT match reload register of match channel 4; high 16-bit */
#define LPC43_SCT_MATCHR5H_OFFSET   0x0216 /* SCT match reload register of match channel 5; high 16-bit */
#define LPC43_SCT_MATCHR6H_OFFSET   0x021a /* SCT match reload register of match channel 6; high 16-bit */
#define LPC43_SCT_MATCHR7H_OFFSET   0x021e /* SCT match reload register of match channel 7; high 16-bit */
#define LPC43_SCT_MATCHR8H_OFFSET   0x0222 /* SCT match reload register of match channel 8; high 16-bit */
#define LPC43_SCT_MATCHR9H_OFFSET   0x0226 /* SCT match reload register of match channel 9; high 16-bit */
#define LPC43_SCT_MATCHR10H_OFFSET  0x022a /* SCT match reload register of match channel 10; high 16-bit */
#define LPC43_SCT_MATCHR11H_OFFSET  0x022e /* SCT match reload register of match channel 11; high 16-bit */
#define LPC43_SCT_MATCHR12H_OFFSET  0x0232 /* SCT match reload register of match channel 12; high 16-bit */
#define LPC43_SCT_MATCHR13H_OFFSET  0x0236 /* SCT match reload register of match channel 13; high 16-bit */
#define LPC43_SCT_MATCHR14H_OFFSET  0x023a /* SCT match reload register of match channel 14; high 16-bit */
#define LPC43_SCT_MATCHR15H_OFFSET  0x023e /* SCT match reload register of match channel 15; high 16-bit */

#define LPC43_SCT_CAPC_OFFSET(n)    (0x0200 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0_OFFSET      0x0200 /* SCT capture control register Ch0 */
#define LPC43_SCT_CAPC1_OFFSET      0x0204 /* SCT capture control register Ch1 */
#define LPC43_SCT_CAPC2_OFFSET      0x0208 /* SCT capture control register Ch2 */
#define LPC43_SCT_CAPC3_OFFSET      0x020c /* SCT capture control register Ch3 */
#define LPC43_SCT_CAPC4_OFFSET      0x0210 /* SCT capture control register Ch4 */
#define LPC43_SCT_CAPC5_OFFSET      0x0214 /* SCT capture control register Ch5 */
#define LPC43_SCT_CAPC6_OFFSET      0x0218 /* SCT capture control register Ch6 */
#define LPC43_SCT_CAPC7_OFFSET      0x021c /* SCT capture control register Ch7 */
#define LPC43_SCT_CAPC8_OFFSET      0x0220 /* SCT capture control register Ch8 */
#define LPC43_SCT_CAPC9_OFFSET      0x0224 /* SCT capture control register Ch9 */
#define LPC43_SCT_CAPC10_OFFSET     0x0228 /* SCT capture control register Ch10 */
#define LPC43_SCT_CAPC11_OFFSET     0x022c /* SCT capture control register Ch11 */
#define LPC43_SCT_CAPC12_OFFSET     0x0230 /* SCT capture control register Ch12 */
#define LPC43_SCT_CAPC13_OFFSET     0x0234 /* SCT capture control register Ch13 */
#define LPC43_SCT_CAPC14_OFFSET     0x0238 /* SCT capture control register Ch14 */
#define LPC43_SCT_CAPC15_OFFSET     0x023c /* SCT capture control register Ch15 */

#define LPC43_SCT_CAPCL_OFFSET(n)   (0x0200 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0L_OFFSET     0x0200 /* SCT capture control register Ch0; low 16-bit */
#define LPC43_SCT_CAPC1L_OFFSET     0x0204 /* SCT capture control register Ch1; low 16-bit */
#define LPC43_SCT_CAPC2L_OFFSET     0x0208 /* SCT capture control register Ch2; low 16-bit */
#define LPC43_SCT_CAPC3L_OFFSET     0x020c /* SCT capture control register Ch3; low 16-bit */
#define LPC43_SCT_CAPC4L_OFFSET     0x0210 /* SCT capture control register Ch4; low 16-bit */
#define LPC43_SCT_CAPC5L_OFFSET     0x0214 /* SCT capture control register Ch5; low 16-bit */
#define LPC43_SCT_CAPC6L_OFFSET     0x0218 /* SCT capture control register Ch6; low 16-bit */
#define LPC43_SCT_CAPC7L_OFFSET     0x021c /* SCT capture control register Ch7; low 16-bit */
#define LPC43_SCT_CAPC8L_OFFSET     0x0220 /* SCT capture control register Ch8; low 16-bit */
#define LPC43_SCT_CAPC9L_OFFSET     0x0224 /* SCT capture control register Ch9; low 16-bit */
#define LPC43_SCT_CAPC10L_OFFSET    0x0228 /* SCT capture control register Ch10; low 16-bit */
#define LPC43_SCT_CAPC11L_OFFSET    0x022c /* SCT capture control register Ch11; low 16-bit */
#define LPC43_SCT_CAPC12L_OFFSET    0x0230 /* SCT capture control register Ch12; low 16-bit */
#define LPC43_SCT_CAPC13L_OFFSET    0x0234 /* SCT capture control register Ch13; low 16-bit */
#define LPC43_SCT_CAPC14L_OFFSET    0x0238 /* SCT capture control register Ch14; low 16-bit */
#define LPC43_SCT_CAPC15L_OFFSET    0x023c /* SCT capture control register Ch15; low 16-bit */

#define LPC43_SCT_CAPCH_OFFSET(n)   (0x0202 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0H_OFFSET     0x0202 /* SCT capture control register Ch0; high 16-bit */
#define LPC43_SCT_CAPC1H_OFFSET     0x0206 /* SCT capture control register Ch1; high 16-bit */
#define LPC43_SCT_CAPC2H_OFFSET     0x020a /* SCT capture control register Ch2; high 16-bit */
#define LPC43_SCT_CAPC3H_OFFSET     0x020e /* SCT capture control register Ch3; high 16-bit */
#define LPC43_SCT_CAPC4H_OFFSET     0x0212 /* SCT capture control register Ch4; high 16-bit */
#define LPC43_SCT_CAPC5H_OFFSET     0x0216 /* SCT capture control register Ch5; high 16-bit */
#define LPC43_SCT_CAPC6H_OFFSET     0x021a /* SCT capture control register Ch6; high 16-bit */
#define LPC43_SCT_CAPC7H_OFFSET     0x021e /* SCT capture control register Ch7; high 16-bit */
#define LPC43_SCT_CAPC8H_OFFSET     0x0222 /* SCT capture control register Ch8; high 16-bit */
#define LPC43_SCT_CAPC9H_OFFSET     0x0226 /* SCT capture control register Ch9; high 16-bit */
#define LPC43_SCT_CAPC10H_OFFSET    0x022a /* SCT capture control register Ch10; high 16-bit */
#define LPC43_SCT_CAPC11H_OFFSET    0x022e /* SCT capture control register Ch11; high 16-bit */
#define LPC43_SCT_CAPC12H_OFFSET    0x0232 /* SCT capture control register Ch12; high 16-bit */
#define LPC43_SCT_CAPC13H_OFFSET    0x0236 /* SCT capture control register Ch13; high 16-bit */
#define LPC43_SCT_CAPC14H_OFFSET    0x023a /* SCT capture control register Ch14; high 16-bit */
#define LPC43_SCT_CAPC15H_OFFSET    0x023e /* SCT capture control register Ch15; high 16-bit */

#define LPC43_SCT_MATCHRA_OFFSET(n) (0x0280 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0A_OFFSET   0x0280 /* SCT match reload alias register of match channel 0 */
#define LPC43_SCT_MATCHR1A_OFFSET   0x0284 /* SCT match reload alias register of match channel 1 */
#define LPC43_SCT_MATCHR2A_OFFSET   0x0288 /* SCT match reload alias register of match channel 2 */
#define LPC43_SCT_MATCHR3A_OFFSET   0x028c /* SCT match reload alias register of match channel 3 */
#define LPC43_SCT_MATCHR4A_OFFSET   0x0290 /* SCT match reload alias register of match channel 4 */
#define LPC43_SCT_MATCHR5A_OFFSET   0x0294 /* SCT match reload alias register of match channel 5 */
#define LPC43_SCT_MATCHR6A_OFFSET   0x0298 /* SCT match reload alias register of match channel 6 */
#define LPC43_SCT_MATCHR7A_OFFSET   0x029c /* SCT match reload alias register of match channel 7 */
#define LPC43_SCT_MATCHR8A_OFFSET   0x02a0 /* SCT match reload alias register of match channel 8 */
#define LPC43_SCT_MATCHR9A_OFFSET   0x02a4 /* SCT match reload alias register of match channel 9 */
#define LPC43_SCT_MATCHR10A_OFFSET  0x02a8 /* SCT match reload alias register of match channel 10 */
#define LPC43_SCT_MATCHR11A_OFFSET  0x02ac /* SCT match reload alias register of match channel 11 */
#define LPC43_SCT_MATCHR12A_OFFSET  0x02b0 /* SCT match reload alias register of match channel 12 */
#define LPC43_SCT_MATCHR13A_OFFSET  0x02b4 /* SCT match reload alias register of match channel 13 */
#define LPC43_SCT_MATCHR14A_OFFSET  0x02b8 /* SCT match reload alias register of match channel 14 */
#define LPC43_SCT_MATCHR15A_OFFSET  0x02bc /* SCT match reload alias register of match channel 15 */

#define LPC43_SCT_MATCHRLA_OFFSET(n) (0x0280 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0LA_OFFSET  0x0280 /* SCT match reload alias register of match channel 0; low 16-bit */
#define LPC43_SCT_MATCHR1LA_OFFSET  0x0284 /* SCT match reload alias register of match channel 1; low 16-bit */
#define LPC43_SCT_MATCHR2LA_OFFSET  0x0288 /* SCT match reload alias register of match channel 2; low 16-bit */
#define LPC43_SCT_MATCHR3LA_OFFSET  0x028c /* SCT match reload alias register of match channel 3; low 16-bit */
#define LPC43_SCT_MATCHR4LA_OFFSET  0x0290 /* SCT match reload alias register of match channel 4; low 16-bit */
#define LPC43_SCT_MATCHR5LA_OFFSET  0x0294 /* SCT match reload alias register of match channel 5; low 16-bit */
#define LPC43_SCT_MATCHR6LA_OFFSET  0x0298 /* SCT match reload alias register of match channel 6; low 16-bit */
#define LPC43_SCT_MATCHR7LA_OFFSET  0x029c /* SCT match reload alias register of match channel 7; low 16-bit */
#define LPC43_SCT_MATCHR8LA_OFFSET  0x02a0 /* SCT match reload alias register of match channel 8; low 16-bit */
#define LPC43_SCT_MATCHR9LA_OFFSET  0x02a4 /* SCT match reload alias register of match channel 9; low 16-bit */
#define LPC43_SCT_MATCHR10LA_OFFSET 0x02a8 /* SCT match reload alias register of match channel 10; low 16-bit */
#define LPC43_SCT_MATCHR11LA_OFFSET 0x02ac /* SCT match reload alias register of match channel 11; low 16-bit */
#define LPC43_SCT_MATCHR12LA_OFFSET 0x02b0 /* SCT match reload alias register of match channel 12; low 16-bit */
#define LPC43_SCT_MATCHR13LA_OFFSET 0x02b4 /* SCT match reload alias register of match channel 13; low 16-bit */
#define LPC43_SCT_MATCHR14LA_OFFSET 0x02b8 /* SCT match reload alias register of match channel 14; low 16-bit */
#define LPC43_SCT_MATCHR15LA_OFFSET 0x02bc /* SCT match reload alias register of match channel 15; low 16-bit */

#define LPC43_SCT_MATCHRHA_OFFSET(n) (0x0282 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_MATCHR0HA_OFFSET  0x0282 /* SCT match reload alias register of match channel 0; high 16-bit */
#define LPC43_SCT_MATCHR1HA_OFFSET  0x0286 /* SCT match reload alias register of match channel 1; high 16-bit */
#define LPC43_SCT_MATCHR2HA_OFFSET  0x028a /* SCT match reload alias register of match channel 2; high 16-bit */
#define LPC43_SCT_MATCHR3HA_OFFSET  0x028e /* SCT match reload alias register of match channel 3; high 16-bit */
#define LPC43_SCT_MATCHR4HA_OFFSET  0x0292 /* SCT match reload alias register of match channel 4; high 16-bit */
#define LPC43_SCT_MATCHR5HA_OFFSET  0x0296 /* SCT match reload alias register of match channel 5; high 16-bit */
#define LPC43_SCT_MATCHR6HA_OFFSET  0x029a /* SCT match reload alias register of match channel 6; high 16-bit */
#define LPC43_SCT_MATCHR7HA_OFFSET  0x029e /* SCT match reload alias register of match channel 7; high 16-bit */
#define LPC43_SCT_MATCHR8HA_OFFSET  0x02a2 /* SCT match reload alias register of match channel 8; high 16-bit */
#define LPC43_SCT_MATCHR9HA_OFFSET  0x02a6 /* SCT match reload alias register of match channel 9; high 16-bit */
#define LPC43_SCT_MATCHR10HA_OFFSET 0x02aa /* SCT match reload alias register of match channel 10; high 16-bit */
#define LPC43_SCT_MATCHR11HA_OFFSET 0x02ae /* SCT match reload alias register of match channel 11; high 16-bit */
#define LPC43_SCT_MATCHR12HA_OFFSET 0x02b2 /* SCT match reload alias register of match channel 12; high 16-bit */
#define LPC43_SCT_MATCHR13HA_OFFSET 0x02b6 /* SCT match reload alias register of match channel 13; high 16-bit */
#define LPC43_SCT_MATCHR14HA_OFFSET 0x02ba /* SCT match reload alias register of match channel 14; high 16-bit */
#define LPC43_SCT_MATCHR15HA_OFFSET 0x02be /* SCT match reload alias register of match channel 15; high 16-bit */

#define LPC43_SCT_CAPCA_OFFSET(n)   (0x0280 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0A_OFFSET     0x0280 /* SCT capture control alias register Ch0 */
#define LPC43_SCT_CAPC1A_OFFSET     0x0284 /* SCT capture control alias register Ch1 */
#define LPC43_SCT_CAPC2A_OFFSET     0x0288 /* SCT capture control alias register Ch2 */
#define LPC43_SCT_CAPC3A_OFFSET     0x028c /* SCT capture control alias register Ch3 */
#define LPC43_SCT_CAPC4A_OFFSET     0x0290 /* SCT capture control alias register Ch4 */
#define LPC43_SCT_CAPC5A_OFFSET     0x0294 /* SCT capture control alias register Ch5 */
#define LPC43_SCT_CAPC6A_OFFSET     0x0298 /* SCT capture control alias register Ch6 */
#define LPC43_SCT_CAPC7A_OFFSET     0x029c /* SCT capture control alias register Ch7 */
#define LPC43_SCT_CAPC8A_OFFSET     0x02a0 /* SCT capture control alias register Ch8 */
#define LPC43_SCT_CAPC9A_OFFSET     0x02a4 /* SCT capture control alias register Ch9 */
#define LPC43_SCT_CAPC10A_OFFSET    0x02a8 /* SCT capture control alias register Ch10 */
#define LPC43_SCT_CAPC11A_OFFSET    0x02ac /* SCT capture control alias register Ch11 */
#define LPC43_SCT_CAPC12A_OFFSET    0x02b0 /* SCT capture control alias register Ch12 */
#define LPC43_SCT_CAPC13A_OFFSET    0x02b4 /* SCT capture control alias register Ch13 */
#define LPC43_SCT_CAPC14A_OFFSET    0x02b8 /* SCT capture control alias register Ch14 */
#define LPC43_SCT_CAPC15A_OFFSET    0x02bc /* SCT capture control alias register Ch15 */

#define LPC43_SCT_CAPCLA_OFFSET(n)  (0x0280 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0LA_OFFSET    0x0280 /* SCT capture control alias register Ch0; low 16-bit */
#define LPC43_SCT_CAPC1LA_OFFSET    0x0284 /* SCT capture control alias register Ch1; low 16-bit */
#define LPC43_SCT_CAPC2LA_OFFSET    0x0288 /* SCT capture control alias register Ch2; low 16-bit */
#define LPC43_SCT_CAPC3LA_OFFSET    0x028c /* SCT capture control alias register Ch3; low 16-bit */
#define LPC43_SCT_CAPC4LA_OFFSET    0x0290 /* SCT capture control alias register Ch4; low 16-bit */
#define LPC43_SCT_CAPC5LA_OFFSET    0x0294 /* SCT capture control alias register Ch5; low 16-bit */
#define LPC43_SCT_CAPC6LA_OFFSET    0x0298 /* SCT capture control alias register Ch6; low 16-bit */
#define LPC43_SCT_CAPC7LA_OFFSET    0x029c /* SCT capture control alias register Ch7; low 16-bit */
#define LPC43_SCT_CAPC8LA_OFFSET    0x02a0 /* SCT capture control alias register Ch8; low 16-bit */
#define LPC43_SCT_CAPC9LA_OFFSET    0x02a4 /* SCT capture control alias register Ch9; low 16-bit */
#define LPC43_SCT_CAPC10LA_OFFSET   0x02a8 /* SCT capture control alias register Ch10; low 16-bit */
#define LPC43_SCT_CAPC11LA_OFFSET   0x02ac /* SCT capture control alias register Ch11; low 16-bit */
#define LPC43_SCT_CAPC12LA_OFFSET   0x02b0 /* SCT capture control alias register Ch12; low 16-bit */
#define LPC43_SCT_CAPC13LA_OFFSET   0x02b4 /* SCT capture control alias register Ch13; low 16-bit */
#define LPC43_SCT_CAPC14LA_OFFSET   0x02b8 /* SCT capture control alias register Ch14; low 16-bit */
#define LPC43_SCT_CAPC15LA_OFFSET   0x02bc /* SCT capture control alias register Ch15; low 16-bit */

#define LPC43_SCT_CAPCHA_OFFSET(n)  (0x0282 + ((n) << 4)) /* n = 0..15 */
#define LPC43_SCT_CAPC0HA_OFFSET    0x0282 /* SCT capture control alias register Ch0; high 16-bit */
#define LPC43_SCT_CAPC1HA_OFFSET    0x0286 /* SCT capture control alias register Ch1; high 16-bit */
#define LPC43_SCT_CAPC2HA_OFFSET    0x028a /* SCT capture control alias register Ch2; high 16-bit */
#define LPC43_SCT_CAPC3HA_OFFSET    0x028e /* SCT capture control alias register Ch3; high 16-bit */
#define LPC43_SCT_CAPC4HA_OFFSET    0x0292 /* SCT capture control alias register Ch4; high 16-bit */
#define LPC43_SCT_CAPC5HA_OFFSET    0x0296 /* SCT capture control alias register Ch5; high 16-bit */
#define LPC43_SCT_CAPC6HA_OFFSET    0x029a /* SCT capture control alias register Ch6; high 16-bit */
#define LPC43_SCT_CAPC7HA_OFFSET    0x029e /* SCT capture control alias register Ch7; high 16-bit */
#define LPC43_SCT_CAPC8HA_OFFSET    0x02a2 /* SCT capture control alias register Ch8; high 16-bit */
#define LPC43_SCT_CAPC9HA_OFFSET    0x02a6 /* SCT capture control alias register Ch9; high 16-bit */
#define LPC43_SCT_CAPC10HA_OFFSET   0x02aa /* SCT capture control alias register Ch10; high 16-bit */
#define LPC43_SCT_CAPC11HA_OFFSET   0x02ae /* SCT capture control alias register Ch11; high 16-bit */
#define LPC43_SCT_CAPC12HA_OFFSET   0x02b2 /* SCT capture control alias register Ch12; high 16-bit */
#define LPC43_SCT_CAPC13HA_OFFSET   0x02b6 /* SCT capture control alias register Ch13; high 16-bit */
#define LPC43_SCT_CAPC14HA_OFFSET   0x02ba /* SCT capture control alias register Ch14; high 16-bit */
#define LPC43_SCT_CAPC15HA_OFFSET   0x02be /* SCT capture control alias register Ch15; high 16-bit */

#define LPC43_SCT_EVSM_OFFSET(n)    (0x0300 + ((n) << 3))
#define LPC43_SCT_EVC_OFFSET(n)     (0x0304 + ((n) << 3))

#define LPC43_SCT_EVSM0_OFFSET      0x0300 /* SCT event state register 0 */
#define LPC43_SCT_EVC0_OFFSET       0x0304 /* SCT event control register 0 */
#define LPC43_SCT_EVSM1_OFFSET      0x0308 /* SCT event state register 1 */
#define LPC43_SCT_EVC1_OFFSET       0x030c /* SCT event control register 1 */
#define LPC43_SCT_EVSM2_OFFSET      0x0310 /* SCT event state register 2 */
#define LPC43_SCT_EVC2_OFFSET       0x0314 /* SCT event control register 2 */
#define LPC43_SCT_EVSM3_OFFSET      0x0318 /* SCT event state register 3 */
#define LPC43_SCT_EVC3_OFFSET       0x031c /* SCT event control register 3 */
#define LPC43_SCT_EVSM4_OFFSET      0x0320 /* SCT event state register 4 */
#define LPC43_SCT_EVC4_OFFSET       0x0324 /* SCT event control register 4 */
#define LPC43_SCT_EVSM5_OFFSET      0x0328 /* SCT event state register 5 */
#define LPC43_SCT_EVC5_OFFSET       0x032c /* SCT event control register 5 */
#define LPC43_SCT_EVSM6_OFFSET      0x0330 /* SCT event state register 6 */
#define LPC43_SCT_EVC6_OFFSET       0x0334 /* SCT event control register 6 */
#define LPC43_SCT_EVSM7_OFFSET      0x0338 /* SCT event state register 7 */
#define LPC43_SCT_EVC7_OFFSET       0x033c /* SCT event control register 7 */
#define LPC43_SCT_EVSM8_OFFSET      0x0340 /* SCT event state register 8 */
#define LPC43_SCT_EVC8_OFFSET       0x0344 /* SCT event control register 8 */
#define LPC43_SCT_EVSM9_OFFSET      0x0348 /* SCT event state register 9 */
#define LPC43_SCT_EVC9_OFFSET       0x034c /* SCT event control register 9 */
#define LPC43_SCT_EVSM10_OFFSET     0x0350 /* SCT event state register 10 */
#define LPC43_SCT_EVC10_OFFSET      0x0354 /* SCT event control register 10 */
#define LPC43_SCT_EVSM11_OFFSET     0x0358 /* SCT event state register 11 */
#define LPC43_SCT_EVC11_OFFSET      0x035c /* SCT event control register 11 */
#define LPC43_SCT_EVSM12_OFFSET     0x0360 /* SCT event state register 12 */
#define LPC43_SCT_EVC12_OFFSET      0x0364 /* SCT event control register 12 */
#define LPC43_SCT_EVSM13_OFFSET     0x0368 /* SCT event state register 13 */
#define LPC43_SCT_EVC13_OFFSET      0x036c /* SCT event control register 13 */
#define LPC43_SCT_EVSM14_OFFSET     0x0370 /* SCT event state register 14 */
#define LPC43_SCT_EVC14_OFFSET      0x0374 /* SCT event control register 14 */
#define LPC43_SCT_EVSM15_OFFSET     0x0378 /* SCT event state register 15 */
#define LPC43_SCT_EVC15_OFFSET      0x037c /* SCT event control register 15 */

#define LPC43_SCT_OUTSET_OFFSET(n)  (0x0500 + ((n) << 3))
#define LPC43_SCT_OUTCLR_OFFSET(n)  (0x0504 + ((n) << 3))

#define LPC43_SCT_OUTSET0_OFFSET    0x0500 /* SCT output 0 set register */
#define LPC43_SCT_OUTCLR0_OFFSET    0x0504 /* SCT output 0 clear register */
#define LPC43_SCT_OUTSET1_OFFSET    0x0508 /* SCT output 1 set register */
#define LPC43_SCT_OUTCLR1_OFFSET    0x050c /* SCT output 1 clear register */
#define LPC43_SCT_OUTSET2_OFFSET    0x0510 /* SCT output 2 set register */
#define LPC43_SCT_OUTCLR2_OFFSET    0x0514 /* SCT output 2 clear register */
#define LPC43_SCT_OUTSET3_OFFSET    0x0518 /* SCT output 3 set register */
#define LPC43_SCT_OUTCLR3_OFFSET    0x051c /* SCT output 3 clear register */
#define LPC43_SCT_OUTSET4_OFFSET    0x0520 /* SCT output 4 set register */
#define LPC43_SCT_OUTCLR4_OFFSET    0x0524 /* SCT output 4 clear register */
#define LPC43_SCT_OUTSET5_OFFSET    0x0528 /* SCT output 5 set register */
#define LPC43_SCT_OUTCLR5_OFFSET    0x052c /* SCT output 5 clear register */
#define LPC43_SCT_OUTSET6_OFFSET    0x0530 /* SCT output 6 set register */
#define LPC43_SCT_OUTCLR6_OFFSET    0x0534 /* SCT output 6 clear register */
#define LPC43_SCT_OUTSET7_OFFSET    0x0538 /* SCT output 7 set register */
#define LPC43_SCT_OUTCLR7_OFFSET    0x053c /* SCT output 7 clear register */
#define LPC43_SCT_OUTSET8_OFFSET    0x0540 /* SCT output 8 set register */
#define LPC43_SCT_OUTCLR8_OFFSET    0x0544 /* SCT output 8 clear register */
#define LPC43_SCT_OUTSET9_OFFSET    0x0548 /* SCT output 9 set register */
#define LPC43_SCT_OUTCLR9_OFFSET    0x054c /* SCT output 9 clear register */
#define LPC43_SCT_OUTSET10_OFFSET   0x0550 /* SCT output 10 set register */
#define LPC43_SCT_OUTCLR10_OFFSET   0x0554 /* SCT output 10 clear register */
#define LPC43_SCT_OUTSET11_OFFSET   0x0558 /* SCT output 11 set register */
#define LPC43_SCT_OUTCLR11_OFFSET   0x055c /* SCT output 11 clear register */
#define LPC43_SCT_OUTSET12_OFFSET   0x0560 /* SCT output 12 set register */
#define LPC43_SCT_OUTCLR12_OFFSET   0x0564 /* SCT output 12 clear register */
#define LPC43_SCT_OUTSET13_OFFSET   0x0568 /* SCT output 13 set register */
#define LPC43_SCT_OUTCLR13_OFFSET   0x056c /* SCT output 13 clear register */
#define LPC43_SCT_OUTSET14_OFFSET   0x0570 /* SCT output 14 set register */
#define LPC43_SCT_OUTCLR14_OFFSET   0x0574 /* SCT output 14 clear register */
#define LPC43_SCT_OUTSET15_OFFSET   0x0578 /* SCT output 15 set register */
#define LPC43_SCT_OUTCLR15_OFFSET   0x057c /* SCT output 15 clear register */

/* Register Addresses *******************************************************************************/

#define LPC43_SCT_CONFIG            (LPC43_SCT_BASE+LPC43_SCT_CONFIG_OFFSET)
#define LPC43_SCT_CTRL              (LPC43_SCT_BASE+LPC43_SCT_CTRL_OFFSET)
#define LPC43_SCT_CTRLL             (LPC43_SCT_BASE+LPC43_SCT_CTRLL_OFFSET)
#define LPC43_SCT_CTRLH             (LPC43_SCT_BASE+LPC43_SCT_CTRLH_OFFSET)
#define LPC43_SCT_LIMIT             (LPC43_SCT_BASE+LPC43_SCT_LIMIT_OFFSET)
#define LPC43_SCT_LIMITL            (LPC43_SCT_BASE+LPC43_SCT_LIMITL_OFFSET)
#define LPC43_SCT_LIMITH            (LPC43_SCT_BASE+LPC43_SCT_LIMITH_OFFSET)
#define LPC43_SCT_HALT              (LPC43_SCT_BASE+LPC43_SCT_HALT_OFFSET)
#define LPC43_SCT_HALTL             (LPC43_SCT_BASE+LPC43_SCT_HALTL_OFFSET)
#define LPC43_SCT_HALTH             (LPC43_SCT_BASE+LPC43_SCT_HALTH_OFFSET)
#define LPC43_SCT_STOP              (LPC43_SCT_BASE+LPC43_SCT_STOP_OFFSET)
#define LPC43_SCT_STOPL             (LPC43_SCT_BASE+LPC43_SCT_STOPL_OFFSET)
#define LPC43_SCT_STOPH             (LPC43_SCT_BASE+LPC43_SCT_STOPH_OFFSET)
#define LPC43_SCT_START             (LPC43_SCT_BASE+LPC43_SCT_START_OFFSET)
#define LPC43_SCT_STARTL            (LPC43_SCT_BASE+LPC43_SCT_STARTL_OFFSET)
#define LPC43_SCT_STARTH            (LPC43_SCT_BASE+LPC43_SCT_STARTH_OFFSET)

#define LPC43_SCT_COUNT             (LPC43_SCT_BASE+LPC43_SCT_COUNT_OFFSET)
#define LPC43_SCT_COUNTL            (LPC43_SCT_BASE+LPC43_SCT_COUNTL_OFFSET)
#define LPC43_SCT_COUNTH            (LPC43_SCT_BASE+LPC43_SCT_COUNTH_OFFSET)
#define LPC43_SCT_STATE             (LPC43_SCT_BASE+LPC43_SCT_STATE_OFFSET)
#define LPC43_SCT_STATEL            (LPC43_SCT_BASE+LPC43_SCT_STATEL_OFFSET)
#define LPC43_SCT_STATEH            (LPC43_SCT_BASE+LPC43_SCT_STATEH_OFFSET)
#define LPC43_SCT_INPUT             (LPC43_SCT_BASE+LPC43_SCT_INPUT_OFFSET)
#define LPC43_SCT_REGM              (LPC43_SCT_BASE+LPC43_SCT_REGM_OFFSET)
#define LPC43_SCT_REGML             (LPC43_SCT_BASE+LPC43_SCT_REGML_OFFSET)
#define LPC43_SCT_REGMH             (LPC43_SCT_BASE+LPC43_SCT_REGMH_OFFSET)
#define LPC43_SCT_OUT               (LPC43_SCT_BASE+LPC43_SCT_OUT_OFFSET)
#define LPC43_SCT_OUTDIRC           (LPC43_SCT_BASE+LPC43_SCT_OUTDIRC_OFFSET)
#define LPC43_SCT_RES               (LPC43_SCT_BASE+LPC43_SCT_RES_OFFSET)
#define LPC43_SCT_DMAREQ0           (LPC43_SCT_BASE+LPC43_SCT_DMAREQ0_OFFSET)
#define LPC43_SCT_DMAREQ1           (LPC43_SCT_BASE+LPC43_SCT_DMAREQ1_OFFSET)

#define LPC43_SCT_EVEN              (LPC43_SCT_BASE+LPC43_SCT_EVEN_OFFSET)
#define LPC43_SCT_EVFLAG            (LPC43_SCT_BASE+LPC43_SCT_EVFLAG_OFFSET)
#define LPC43_SCT_CONEN             (LPC43_SCT_BASE+LPC43_SCT_CONEN_OFFSET)
#define LPC43_SCT_CONFLAG           (LPC43_SCT_BASE+LPC43_SCT_CONFLAG_OFFSET)

#define LPC43_SCT_MATCH(n)          (LPC43_SCT_BASE+LPC43_SCT_MATCH_OFFSET(n))
#define LPC43_SCT_MATCH0            (LPC43_SCT_BASE+LPC43_SCT_MATCH0_OFFSET)
#define LPC43_SCT_MATCH1            (LPC43_SCT_BASE+LPC43_SCT_MATCH1_OFFSET)
#define LPC43_SCT_MATCH2            (LPC43_SCT_BASE+LPC43_SCT_MATCH2_OFFSET)
#define LPC43_SCT_MATCH3            (LPC43_SCT_BASE+LPC43_SCT_MATCH3_OFFSET)
#define LPC43_SCT_MATCH4            (LPC43_SCT_BASE+LPC43_SCT_MATCH4_OFFSET)
#define LPC43_SCT_MATCH5            (LPC43_SCT_BASE+LPC43_SCT_MATCH5_OFFSET)
#define LPC43_SCT_MATCH6            (LPC43_SCT_BASE+LPC43_SCT_MATCH6_OFFSET)
#define LPC43_SCT_MATCH7            (LPC43_SCT_BASE+LPC43_SCT_MATCH7_OFFSET)
#define LPC43_SCT_MATCH8            (LPC43_SCT_BASE+LPC43_SCT_MATCH8_OFFSET)
#define LPC43_SCT_MATCH9            (LPC43_SCT_BASE+LPC43_SCT_MATCH9_OFFSET)
#define LPC43_SCT_MATCH10           (LPC43_SCT_BASE+LPC43_SCT_MATCH10_OFFSET)
#define LPC43_SCT_MATCH11           (LPC43_SCT_BASE+LPC43_SCT_MATCH11_OFFSET)
#define LPC43_SCT_MATCH12           (LPC43_SCT_BASE+LPC43_SCT_MATCH12_OFFSET)
#define LPC43_SCT_MATCH13           (LPC43_SCT_BASE+LPC43_SCT_MATCH13_OFFSET)
#define LPC43_SCT_MATCH14           (LPC43_SCT_BASE+LPC43_SCT_MATCH14_OFFSET)
#define LPC43_SCT_MATCH15           (LPC43_SCT_BASE+LPC43_SCT_MATCH15_OFFSET)

#define LPC43_SCT_MATCHL(n)         (LPC43_SCT_BASE+LPC43_SCT_MATCHL_OFFSET(n))
#define LPC43_SCT_MATCHL0           (LPC43_SCT_BASE+LPC43_SCT_MATCHL0_OFFSET)
#define LPC43_SCT_MATCHL1           (LPC43_SCT_BASE+LPC43_SCT_MATCHL1_OFFSET)
#define LPC43_SCT_MATCHL2           (LPC43_SCT_BASE+LPC43_SCT_MATCHL2_OFFSET)
#define LPC43_SCT_MATCHL3           (LPC43_SCT_BASE+LPC43_SCT_MATCHL3_OFFSET)
#define LPC43_SCT_MATCHL4           (LPC43_SCT_BASE+LPC43_SCT_MATCHL4_OFFSET)
#define LPC43_SCT_MATCHL5           (LPC43_SCT_BASE+LPC43_SCT_MATCHL5_OFFSET)
#define LPC43_SCT_MATCHL6           (LPC43_SCT_BASE+LPC43_SCT_MATCHL6_OFFSET)
#define LPC43_SCT_MATCHL7           (LPC43_SCT_BASE+LPC43_SCT_MATCHL7_OFFSET)
#define LPC43_SCT_MATCHL8           (LPC43_SCT_BASE+LPC43_SCT_MATCHL8_OFFSET)
#define LPC43_SCT_MATCHL9           (LPC43_SCT_BASE+LPC43_SCT_MATCHL9_OFFSET)
#define LPC43_SCT_MATCHL10          (LPC43_SCT_BASE+LPC43_SCT_MATCHL10_OFFSET)
#define LPC43_SCT_MATCHL11          (LPC43_SCT_BASE+LPC43_SCT_MATCHL11_OFFSET)
#define LPC43_SCT_MATCHL12          (LPC43_SCT_BASE+LPC43_SCT_MATCHL12_OFFSET)
#define LPC43_SCT_MATCHL13          (LPC43_SCT_BASE+LPC43_SCT_MATCHL13_OFFSET)
#define LPC43_SCT_MATCHL14          (LPC43_SCT_BASE+LPC43_SCT_MATCHL14_OFFSET)
#define LPC43_SCT_MATCHL15          (LPC43_SCT_BASE+LPC43_SCT_MATCHL15_OFFSET)

#define LPC43_SCT_MATCHH(n)         (LPC43_SCT_BASE+LPC43_SCT_MATCHH_OFFSET(n))
#define LPC43_SCT_MATCHH0           (LPC43_SCT_BASE+LPC43_SCT_MATCHH0_OFFSET)
#define LPC43_SCT_MATCHH1           (LPC43_SCT_BASE+LPC43_SCT_MATCHH1_OFFSET)
#define LPC43_SCT_MATCHH2           (LPC43_SCT_BASE+LPC43_SCT_MATCHH2_OFFSET)
#define LPC43_SCT_MATCHH3           (LPC43_SCT_BASE+LPC43_SCT_MATCHH3_OFFSET)
#define LPC43_SCT_MATCHH4           (LPC43_SCT_BASE+LPC43_SCT_MATCHH4_OFFSET)
#define LPC43_SCT_MATCHH5           (LPC43_SCT_BASE+LPC43_SCT_MATCHH5_OFFSET)
#define LPC43_SCT_MATCHH6           (LPC43_SCT_BASE+LPC43_SCT_MATCHH6_OFFSET)
#define LPC43_SCT_MATCHH7           (LPC43_SCT_BASE+LPC43_SCT_MATCHH7_OFFSET)
#define LPC43_SCT_MATCHH8           (LPC43_SCT_BASE+LPC43_SCT_MATCHH8_OFFSET)
#define LPC43_SCT_MATCHH9           (LPC43_SCT_BASE+LPC43_SCT_MATCHH9_OFFSET)
#define LPC43_SCT_MATCHH10          (LPC43_SCT_BASE+LPC43_SCT_MATCHH10_OFFSET)
#define LPC43_SCT_MATCHH11          (LPC43_SCT_BASE+LPC43_SCT_MATCHH11_OFFSET)
#define LPC43_SCT_MATCHH12          (LPC43_SCT_BASE+LPC43_SCT_MATCHH12_OFFSET)
#define LPC43_SCT_MATCHH13          (LPC43_SCT_BASE+LPC43_SCT_MATCHH13_OFFSET)
#define LPC43_SCT_MATCHH14          (LPC43_SCT_BASE+LPC43_SCT_MATCHH14_OFFSET)
#define LPC43_SCT_MATCHH15          (LPC43_SCT_BASE+LPC43_SCT_MATCHH15_OFFSET)

#define LPC43_SCT_CAP(n)            (LPC43_SCT_BASE+LPC43_SCT_CAP_OFFSET(n))
#define LPC43_SCT_CAP0              (LPC43_SCT_BASE+LPC43_SCT_CAP0_OFFSET)
#define LPC43_SCT_CAP1              (LPC43_SCT_BASE+LPC43_SCT_CAP1_OFFSET)
#define LPC43_SCT_CAP2              (LPC43_SCT_BASE+LPC43_SCT_CAP2_OFFSET)
#define LPC43_SCT_CAP3              (LPC43_SCT_BASE+LPC43_SCT_CAP3_OFFSET)
#define LPC43_SCT_CAP4              (LPC43_SCT_BASE+LPC43_SCT_CAP4_OFFSET)
#define LPC43_SCT_CAP5              (LPC43_SCT_BASE+LPC43_SCT_CAP5_OFFSET)
#define LPC43_SCT_CAP6              (LPC43_SCT_BASE+LPC43_SCT_CAP6_OFFSET)
#define LPC43_SCT_CAP7              (LPC43_SCT_BASE+LPC43_SCT_CAP7_OFFSET)
#define LPC43_SCT_CAP8              (LPC43_SCT_BASE+LPC43_SCT_CAP8_OFFSET)
#define LPC43_SCT_CAP9              (LPC43_SCT_BASE+LPC43_SCT_CAP9_OFFSET)
#define LPC43_SCT_CAP10             (LPC43_SCT_BASE+LPC43_SCT_CAP10_OFFSET)
#define LPC43_SCT_CAP11             (LPC43_SCT_BASE+LPC43_SCT_CAP11_OFFSET)
#define LPC43_SCT_CAP12             (LPC43_SCT_BASE+LPC43_SCT_CAP12_OFFSET)
#define LPC43_SCT_CAP13             (LPC43_SCT_BASE+LPC43_SCT_CAP13_OFFSET)
#define LPC43_SCT_CAP14             (LPC43_SCT_BASE+LPC43_SCT_CAP14_OFFSET)
#define LPC43_SCT_CAP15             (LPC43_SCT_BASE+LPC43_SCT_CAP15_OFFSET)

#define LPC43_SCT_CAPL(n)           (LPC43_SCT_BASE+LPC43_SCT_CAPL_OFFSET(n))
#define LPC43_SCT_CAPL0             (LPC43_SCT_BASE+LPC43_SCT_CAPL0_OFFSET)
#define LPC43_SCT_CAPL1             (LPC43_SCT_BASE+LPC43_SCT_CAPL1_OFFSET)
#define LPC43_SCT_CAPL2             (LPC43_SCT_BASE+LPC43_SCT_CAPL2_OFFSET)
#define LPC43_SCT_CAPL3             (LPC43_SCT_BASE+LPC43_SCT_CAPL3_OFFSET)
#define LPC43_SCT_CAPL4             (LPC43_SCT_BASE+LPC43_SCT_CAPL4_OFFSET)
#define LPC43_SCT_CAPL5             (LPC43_SCT_BASE+LPC43_SCT_CAPL5_OFFSET)
#define LPC43_SCT_CAPL6             (LPC43_SCT_BASE+LPC43_SCT_CAPL6_OFFSET)
#define LPC43_SCT_CAPL7             (LPC43_SCT_BASE+LPC43_SCT_CAPL7_OFFSET)
#define LPC43_SCT_CAPL8             (LPC43_SCT_BASE+LPC43_SCT_CAPL8_OFFSET)
#define LPC43_SCT_CAPL9             (LPC43_SCT_BASE+LPC43_SCT_CAPL9_OFFSET)
#define LPC43_SCT_CAPL10            (LPC43_SCT_BASE+LPC43_SCT_CAPL10_OFFSET)
#define LPC43_SCT_CAPL11            (LPC43_SCT_BASE+LPC43_SCT_CAPL11_OFFSET)
#define LPC43_SCT_CAPL12            (LPC43_SCT_BASE+LPC43_SCT_CAPL12_OFFSET)
#define LPC43_SCT_CAPL13            (LPC43_SCT_BASE+LPC43_SCT_CAPL13_OFFSET)
#define LPC43_SCT_CAPL14            (LPC43_SCT_BASE+LPC43_SCT_CAPL14_OFFSET)
#define LPC43_SCT_CAPL15            (LPC43_SCT_BASE+LPC43_SCT_CAPL15_OFFSET)

#define LPC43_SCT_CAPH(n)           (LPC43_SCT_BASE+LPC43_SCT_CAPH_OFFSET(n))
#define LPC43_SCT_CAPH0             (LPC43_SCT_BASE+LPC43_SCT_CAPH0_OFFSET)
#define LPC43_SCT_CAPH1             (LPC43_SCT_BASE+LPC43_SCT_CAPH1_OFFSET)
#define LPC43_SCT_CAPH2             (LPC43_SCT_BASE+LPC43_SCT_CAPH2_OFFSET)
#define LPC43_SCT_CAPH3             (LPC43_SCT_BASE+LPC43_SCT_CAPH3_OFFSET)
#define LPC43_SCT_CAPH4             (LPC43_SCT_BASE+LPC43_SCT_CAPH4_OFFSET)
#define LPC43_SCT_CAPH5             (LPC43_SCT_BASE+LPC43_SCT_CAPH5_OFFSET)
#define LPC43_SCT_CAPH6             (LPC43_SCT_BASE+LPC43_SCT_CAPH6_OFFSET)
#define LPC43_SCT_CAPH7             (LPC43_SCT_BASE+LPC43_SCT_CAPH7_OFFSET)
#define LPC43_SCT_CAPH8             (LPC43_SCT_BASE+LPC43_SCT_CAPH8_OFFSET)
#define LPC43_SCT_CAPH9             (LPC43_SCT_BASE+LPC43_SCT_CAPH9_OFFSET)
#define LPC43_SCT_CAPH10            (LPC43_SCT_BASE+LPC43_SCT_CAPH10_OFFSET)
#define LPC43_SCT_CAPH11            (LPC43_SCT_BASE+LPC43_SCT_CAPH11_OFFSET)
#define LPC43_SCT_CAPH12            (LPC43_SCT_BASE+LPC43_SCT_CAPH12_OFFSET)
#define LPC43_SCT_CAPH13            (LPC43_SCT_BASE+LPC43_SCT_CAPH13_OFFSET)
#define LPC43_SCT_CAPH14            (LPC43_SCT_BASE+LPC43_SCT_CAPH14_OFFSET)
#define LPC43_SCT_CAPH15            (LPC43_SCT_BASE+LPC43_SCT_CAPH15_OFFSET)

#define LPC43_SCT_MATCHA(n)         (LPC43_SCT_BASE+LPC43_SCT_MATCHA_OFFSET(n))
#define LPC43_SCT_MATCHA0           (LPC43_SCT_BASE+LPC43_SCT_MATCHA0_OFFSET)
#define LPC43_SCT_MATCHA1           (LPC43_SCT_BASE+LPC43_SCT_MATCHA1_OFFSET)
#define LPC43_SCT_MATCHA2           (LPC43_SCT_BASE+LPC43_SCT_MATCHA2_OFFSET)
#define LPC43_SCT_MATCHA3           (LPC43_SCT_BASE+LPC43_SCT_MATCHA3_OFFSET)
#define LPC43_SCT_MATCHA4           (LPC43_SCT_BASE+LPC43_SCT_MATCHA4_OFFSET)
#define LPC43_SCT_MATCHA5           (LPC43_SCT_BASE+LPC43_SCT_MATCHA5_OFFSET)
#define LPC43_SCT_MATCHA6           (LPC43_SCT_BASE+LPC43_SCT_MATCHA6_OFFSET)
#define LPC43_SCT_MATCHA7           (LPC43_SCT_BASE+LPC43_SCT_MATCHA7_OFFSET)
#define LPC43_SCT_MATCHA8           (LPC43_SCT_BASE+LPC43_SCT_MATCHA8_OFFSET)
#define LPC43_SCT_MATCHA9           (LPC43_SCT_BASE+LPC43_SCT_MATCHA9_OFFSET)
#define LPC43_SCT_MATCHA10          (LPC43_SCT_BASE+LPC43_SCT_MATCHA10_OFFSET)
#define LPC43_SCT_MATCHA11          (LPC43_SCT_BASE+LPC43_SCT_MATCHA11_OFFSET)
#define LPC43_SCT_MATCHA12          (LPC43_SCT_BASE+LPC43_SCT_MATCHA12_OFFSET)
#define LPC43_SCT_MATCHA13          (LPC43_SCT_BASE+LPC43_SCT_MATCHA13_OFFSET)
#define LPC43_SCT_MATCHA14          (LPC43_SCT_BASE+LPC43_SCT_MATCHA14_OFFSET)
#define LPC43_SCT_MATCHA15          (LPC43_SCT_BASE+LPC43_SCT_MATCHA15_OFFSET)

#define LPC43_SCT_MATCHLA(n)        (LPC43_SCT_BASE+LPC43_SCT_MATCHLA_OFFSET(n))
#define LPC43_SCT_MATCHLA0          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA0_OFFSET)
#define LPC43_SCT_MATCHLA1          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA1_OFFSET)
#define LPC43_SCT_MATCHLA2          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA2_OFFSET)
#define LPC43_SCT_MATCHLA3          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA3_OFFSET)
#define LPC43_SCT_MATCHLA4          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA4_OFFSET)
#define LPC43_SCT_MATCHLA5          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA5_OFFSET)
#define LPC43_SCT_MATCHLA6          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA6_OFFSET)
#define LPC43_SCT_MATCHLA7          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA7_OFFSET)
#define LPC43_SCT_MATCHLA8          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA8_OFFSET)
#define LPC43_SCT_MATCHLA9          (LPC43_SCT_BASE+LPC43_SCT_MATCHLA9_OFFSET)
#define LPC43_SCT_MATCHLA10         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA10_OFFSET)
#define LPC43_SCT_MATCHLA11         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA11_OFFSET)
#define LPC43_SCT_MATCHLA12         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA12_OFFSET)
#define LPC43_SCT_MATCHLA13         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA13_OFFSET)
#define LPC43_SCT_MATCHLA14         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA14_OFFSET)
#define LPC43_SCT_MATCHLA15         (LPC43_SCT_BASE+LPC43_SCT_MATCHLA15_OFFSET)

#define LPC43_SCT_MATCHHA(n)        (LPC43_SCT_BASE+LPC43_SCT_MATCHHA_OFFSET(n))
#define LPC43_SCT_MATCHHA0          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA0_OFFSET)
#define LPC43_SCT_MATCHHA1          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA1_OFFSET)
#define LPC43_SCT_MATCHHA2          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA2_OFFSET)
#define LPC43_SCT_MATCHHA3          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA3_OFFSET)
#define LPC43_SCT_MATCHHA4          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA4_OFFSET)
#define LPC43_SCT_MATCHHA5          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA5_OFFSET)
#define LPC43_SCT_MATCHHA6          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA6_OFFSET)
#define LPC43_SCT_MATCHHA7          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA7_OFFSET)
#define LPC43_SCT_MATCHHA8          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA8_OFFSET)
#define LPC43_SCT_MATCHHA9          (LPC43_SCT_BASE+LPC43_SCT_MATCHHA9_OFFSET)
#define LPC43_SCT_MATCHHA10         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA10_OFFSET)
#define LPC43_SCT_MATCHHA11         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA11_OFFSET)
#define LPC43_SCT_MATCHHA12         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA12_OFFSET)
#define LPC43_SCT_MATCHHA13         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA13_OFFSET)
#define LPC43_SCT_MATCHHA14         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA14_OFFSET)
#define LPC43_SCT_MATCHHA15         (LPC43_SCT_BASE+LPC43_SCT_MATCHHA15_OFFSET)

#define LPC43_SCT_CAPA(n)           (LPC43_SCT_BASE+LPC43_SCT_CAPA_OFFSET(n))
#define LPC43_SCT_CAPA0             (LPC43_SCT_BASE+LPC43_SCT_CAPA0_OFFSET)
#define LPC43_SCT_CAPA1             (LPC43_SCT_BASE+LPC43_SCT_CAPA1_OFFSET)
#define LPC43_SCT_CAPA2             (LPC43_SCT_BASE+LPC43_SCT_CAPA2_OFFSET)
#define LPC43_SCT_CAPA3             (LPC43_SCT_BASE+LPC43_SCT_CAPA3_OFFSET)
#define LPC43_SCT_CAPA4             (LPC43_SCT_BASE+LPC43_SCT_CAPA4_OFFSET)
#define LPC43_SCT_CAPA5             (LPC43_SCT_BASE+LPC43_SCT_CAPA5_OFFSET)
#define LPC43_SCT_CAPA6             (LPC43_SCT_BASE+LPC43_SCT_CAPA6_OFFSET)
#define LPC43_SCT_CAPA7             (LPC43_SCT_BASE+LPC43_SCT_CAPA7_OFFSET)
#define LPC43_SCT_CAPA8             (LPC43_SCT_BASE+LPC43_SCT_CAPA8_OFFSET)
#define LPC43_SCT_CAPA9             (LPC43_SCT_BASE+LPC43_SCT_CAPA9_OFFSET)
#define LPC43_SCT_CAPA10            (LPC43_SCT_BASE+LPC43_SCT_CAPA10_OFFSET)
#define LPC43_SCT_CAPA11            (LPC43_SCT_BASE+LPC43_SCT_CAPA11_OFFSET)
#define LPC43_SCT_CAPA12            (LPC43_SCT_BASE+LPC43_SCT_CAPA12_OFFSET)
#define LPC43_SCT_CAPA13            (LPC43_SCT_BASE+LPC43_SCT_CAPA13_OFFSET)
#define LPC43_SCT_CAPA14            (LPC43_SCT_BASE+LPC43_SCT_CAPA14_OFFSET)
#define LPC43_SCT_CAPA15            (LPC43_SCT_BASE+LPC43_SCT_CAPA15_OFFSET)

#define LPC43_SCT_CAPLA(n)          (LPC43_SCT_BASE+LPC43_SCT_CAPLA_OFFSET(n))
#define LPC43_SCT_CAPLA0            (LPC43_SCT_BASE+LPC43_SCT_CAPLA0_OFFSET)
#define LPC43_SCT_CAPLA1            (LPC43_SCT_BASE+LPC43_SCT_CAPLA1_OFFSET)
#define LPC43_SCT_CAPLA2            (LPC43_SCT_BASE+LPC43_SCT_CAPLA2_OFFSET)
#define LPC43_SCT_CAPLA3            (LPC43_SCT_BASE+LPC43_SCT_CAPLA3_OFFSET)
#define LPC43_SCT_CAPLA4            (LPC43_SCT_BASE+LPC43_SCT_CAPLA4_OFFSET)
#define LPC43_SCT_CAPLA5            (LPC43_SCT_BASE+LPC43_SCT_CAPLA5_OFFSET)
#define LPC43_SCT_CAPLA6            (LPC43_SCT_BASE+LPC43_SCT_CAPLA6_OFFSET)
#define LPC43_SCT_CAPLA7            (LPC43_SCT_BASE+LPC43_SCT_CAPLA7_OFFSET)
#define LPC43_SCT_CAPLA8            (LPC43_SCT_BASE+LPC43_SCT_CAPLA8_OFFSET)
#define LPC43_SCT_CAPLA9            (LPC43_SCT_BASE+LPC43_SCT_CAPLA9_OFFSET)
#define LPC43_SCT_CAPLA10           (LPC43_SCT_BASE+LPC43_SCT_CAPLA10_OFFSET)
#define LPC43_SCT_CAPLA11           (LPC43_SCT_BASE+LPC43_SCT_CAPLA11_OFFSET)
#define LPC43_SCT_CAPLA12           (LPC43_SCT_BASE+LPC43_SCT_CAPLA12_OFFSET)
#define LPC43_SCT_CAPLA13           (LPC43_SCT_BASE+LPC43_SCT_CAPLA13_OFFSET)
#define LPC43_SCT_CAPLA14           (LPC43_SCT_BASE+LPC43_SCT_CAPLA14_OFFSET)
#define LPC43_SCT_CAPLA15           (LPC43_SCT_BASE+LPC43_SCT_CAPLA15_OFFSET)

#define LPC43_SCT_CAPHA(n)          (LPC43_SCT_BASE+LPC43_SCT_CAPHA_OFFSET(n))
#define LPC43_SCT_CAPHA0            (LPC43_SCT_BASE+LPC43_SCT_CAPHA0_OFFSET)
#define LPC43_SCT_CAPHA1            (LPC43_SCT_BASE+LPC43_SCT_CAPHA1_OFFSET)
#define LPC43_SCT_CAPHA2            (LPC43_SCT_BASE+LPC43_SCT_CAPHA2_OFFSET)
#define LPC43_SCT_CAPHA3            (LPC43_SCT_BASE+LPC43_SCT_CAPHA3_OFFSET)
#define LPC43_SCT_CAPHA4            (LPC43_SCT_BASE+LPC43_SCT_CAPHA4_OFFSET)
#define LPC43_SCT_CAPHA5            (LPC43_SCT_BASE+LPC43_SCT_CAPHA5_OFFSET)
#define LPC43_SCT_CAPHA6            (LPC43_SCT_BASE+LPC43_SCT_CAPHA6_OFFSET)
#define LPC43_SCT_CAPHA7            (LPC43_SCT_BASE+LPC43_SCT_CAPHA7_OFFSET)
#define LPC43_SCT_CAPHA8            (LPC43_SCT_BASE+LPC43_SCT_CAPHA8_OFFSET)
#define LPC43_SCT_CAPHA9            (LPC43_SCT_BASE+LPC43_SCT_CAPHA9_OFFSET)
#define LPC43_SCT_CAPHA10           (LPC43_SCT_BASE+LPC43_SCT_CAPHA10_OFFSET)
#define LPC43_SCT_CAPHA11           (LPC43_SCT_BASE+LPC43_SCT_CAPHA11_OFFSET)
#define LPC43_SCT_CAPHA12           (LPC43_SCT_BASE+LPC43_SCT_CAPHA12_OFFSET)
#define LPC43_SCT_CAPHA13           (LPC43_SCT_BASE+LPC43_SCT_CAPHA13_OFFSET)
#define LPC43_SCT_CAPHA14           (LPC43_SCT_BASE+LPC43_SCT_CAPHA14_OFFSET)
#define LPC43_SCT_CAPHA15           (LPC43_SCT_BASE+LPC43_SCT_CAPHA15_OFFSET)

#define LPC43_SCT_MATCHR(n)         (LPC43_SCT_BASE+LPC43_SCT_MATCHR_OFFSET(n))
#define LPC43_SCT_MATCHR0           (LPC43_SCT_BASE+LPC43_SCT_MATCHR0_OFFSET)
#define LPC43_SCT_MATCHR1           (LPC43_SCT_BASE+LPC43_SCT_MATCHR1_OFFSET)
#define LPC43_SCT_MATCHR2           (LPC43_SCT_BASE+LPC43_SCT_MATCHR2_OFFSET)
#define LPC43_SCT_MATCHR3           (LPC43_SCT_BASE+LPC43_SCT_MATCHR3_OFFSET)
#define LPC43_SCT_MATCHR4           (LPC43_SCT_BASE+LPC43_SCT_MATCHR4_OFFSET)
#define LPC43_SCT_MATCHR5           (LPC43_SCT_BASE+LPC43_SCT_MATCHR5_OFFSET)
#define LPC43_SCT_MATCHR6           (LPC43_SCT_BASE+LPC43_SCT_MATCHR6_OFFSET)
#define LPC43_SCT_MATCHR7           (LPC43_SCT_BASE+LPC43_SCT_MATCHR7_OFFSET)
#define LPC43_SCT_MATCHR8           (LPC43_SCT_BASE+LPC43_SCT_MATCHR8_OFFSET)
#define LPC43_SCT_MATCHR9           (LPC43_SCT_BASE+LPC43_SCT_MATCHR9_OFFSET)
#define LPC43_SCT_MATCHR10          (LPC43_SCT_BASE+LPC43_SCT_MATCHR10_OFFSET)
#define LPC43_SCT_MATCHR11          (LPC43_SCT_BASE+LPC43_SCT_MATCHR11_OFFSET)
#define LPC43_SCT_MATCHR12          (LPC43_SCT_BASE+LPC43_SCT_MATCHR12_OFFSET)
#define LPC43_SCT_MATCHR13          (LPC43_SCT_BASE+LPC43_SCT_MATCHR13_OFFSET)
#define LPC43_SCT_MATCHR14          (LPC43_SCT_BASE+LPC43_SCT_MATCHR14_OFFSET)
#define LPC43_SCT_MATCHR15          (LPC43_SCT_BASE+LPC43_SCT_MATCHR15_OFFSET)

#define LPC43_SCT_MATCHRL(n)        (LPC43_SCT_BASE+LPC43_SCT_MATCHRL_OFFSET(n))
#define LPC43_SCT_MATCHRL0          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL0_OFFSET)
#define LPC43_SCT_MATCHRL1          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL1_OFFSET)
#define LPC43_SCT_MATCHRL2          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL2_OFFSET)
#define LPC43_SCT_MATCHRL3          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL3_OFFSET)
#define LPC43_SCT_MATCHRL4          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL4_OFFSET)
#define LPC43_SCT_MATCHRL5          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL5_OFFSET)
#define LPC43_SCT_MATCHRL6          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL6_OFFSET)
#define LPC43_SCT_MATCHRL7          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL7_OFFSET)
#define LPC43_SCT_MATCHRL8          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL8_OFFSET)
#define LPC43_SCT_MATCHRL9          (LPC43_SCT_BASE+LPC43_SCT_MATCHRL9_OFFSET)
#define LPC43_SCT_MATCHRL10         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL10_OFFSET)
#define LPC43_SCT_MATCHRL11         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL11_OFFSET)
#define LPC43_SCT_MATCHRL12         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL12_OFFSET)
#define LPC43_SCT_MATCHRL13         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL13_OFFSET)
#define LPC43_SCT_MATCHRL14         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL14_OFFSET)
#define LPC43_SCT_MATCHRL15         (LPC43_SCT_BASE+LPC43_SCT_MATCHRL15_OFFSET)

#define LPC43_SCT_MATCHRH(n)        (LPC43_SCT_BASE+LPC43_SCT_MATCHRH_OFFSET(n))
#define LPC43_SCT_MATCHRH0          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH0_OFFSET)
#define LPC43_SCT_MATCHRH1          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH1_OFFSET)
#define LPC43_SCT_MATCHRH2          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH2_OFFSET)
#define LPC43_SCT_MATCHRH3          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH3_OFFSET)
#define LPC43_SCT_MATCHRH4          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH4_OFFSET)
#define LPC43_SCT_MATCHRH5          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH5_OFFSET)
#define LPC43_SCT_MATCHRH6          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH6_OFFSET)
#define LPC43_SCT_MATCHRH7          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH7_OFFSET)
#define LPC43_SCT_MATCHRH8          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH8_OFFSET)
#define LPC43_SCT_MATCHRH9          (LPC43_SCT_BASE+LPC43_SCT_MATCHRH9_OFFSET)
#define LPC43_SCT_MATCHRH10         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH10_OFFSET)
#define LPC43_SCT_MATCHRH11         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH11_OFFSET)
#define LPC43_SCT_MATCHRH12         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH12_OFFSET)
#define LPC43_SCT_MATCHRH13         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH13_OFFSET)
#define LPC43_SCT_MATCHRH14         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH14_OFFSET)
#define LPC43_SCT_MATCHRH15         (LPC43_SCT_BASE+LPC43_SCT_MATCHRH15_OFFSET)

#define LPC43_SCT_CAPC(n)           (LPC43_SCT_BASE+LPC43_SCT_CAPC_OFFSET(n))
#define LPC43_SCT_CAPC0             (LPC43_SCT_BASE+LPC43_SCT_CAPC0_OFFSET)
#define LPC43_SCT_CAPC1             (LPC43_SCT_BASE+LPC43_SCT_CAPC1_OFFSET)
#define LPC43_SCT_CAPC2             (LPC43_SCT_BASE+LPC43_SCT_CAPC2_OFFSET)
#define LPC43_SCT_CAPC3             (LPC43_SCT_BASE+LPC43_SCT_CAPC3_OFFSET)
#define LPC43_SCT_CAPC4             (LPC43_SCT_BASE+LPC43_SCT_CAPC4_OFFSET)
#define LPC43_SCT_CAPC5             (LPC43_SCT_BASE+LPC43_SCT_CAPC5_OFFSET)
#define LPC43_SCT_CAPC6             (LPC43_SCT_BASE+LPC43_SCT_CAPC6_OFFSET)
#define LPC43_SCT_CAPC7             (LPC43_SCT_BASE+LPC43_SCT_CAPC7_OFFSET)
#define LPC43_SCT_CAPC8             (LPC43_SCT_BASE+LPC43_SCT_CAPC8_OFFSET)
#define LPC43_SCT_CAPC9             (LPC43_SCT_BASE+LPC43_SCT_CAPC9_OFFSET)
#define LPC43_SCT_CAPC10            (LPC43_SCT_BASE+LPC43_SCT_CAPC10_OFFSET)
#define LPC43_SCT_CAPC11            (LPC43_SCT_BASE+LPC43_SCT_CAPC11_OFFSET)
#define LPC43_SCT_CAPC12            (LPC43_SCT_BASE+LPC43_SCT_CAPC12_OFFSET)
#define LPC43_SCT_CAPC13            (LPC43_SCT_BASE+LPC43_SCT_CAPC13_OFFSET)
#define LPC43_SCT_CAPC14            (LPC43_SCT_BASE+LPC43_SCT_CAPC14_OFFSET)
#define LPC43_SCT_CAPC15            (LPC43_SCT_BASE+LPC43_SCT_CAPC15_OFFSET)

#define LPC43_SCT_CAPCL(n)          (LPC43_SCT_BASE+LPC43_SCT_CAPCL_OFFSET(n))
#define LPC43_SCT_CAPCL0            (LPC43_SCT_BASE+LPC43_SCT_CAPCL0_OFFSET)
#define LPC43_SCT_CAPCL1            (LPC43_SCT_BASE+LPC43_SCT_CAPCL1_OFFSET)
#define LPC43_SCT_CAPCL2            (LPC43_SCT_BASE+LPC43_SCT_CAPCL2_OFFSET)
#define LPC43_SCT_CAPCL3            (LPC43_SCT_BASE+LPC43_SCT_CAPCL3_OFFSET)
#define LPC43_SCT_CAPCL4            (LPC43_SCT_BASE+LPC43_SCT_CAPCL4_OFFSET)
#define LPC43_SCT_CAPCL5            (LPC43_SCT_BASE+LPC43_SCT_CAPCL5_OFFSET)
#define LPC43_SCT_CAPCL6            (LPC43_SCT_BASE+LPC43_SCT_CAPCL6_OFFSET)
#define LPC43_SCT_CAPCL7            (LPC43_SCT_BASE+LPC43_SCT_CAPCL7_OFFSET)
#define LPC43_SCT_CAPCL8            (LPC43_SCT_BASE+LPC43_SCT_CAPCL8_OFFSET)
#define LPC43_SCT_CAPCL9            (LPC43_SCT_BASE+LPC43_SCT_CAPCL9_OFFSET)
#define LPC43_SCT_CAPCL10           (LPC43_SCT_BASE+LPC43_SCT_CAPCL10_OFFSET)
#define LPC43_SCT_CAPCL11           (LPC43_SCT_BASE+LPC43_SCT_CAPCL11_OFFSET)
#define LPC43_SCT_CAPCL12           (LPC43_SCT_BASE+LPC43_SCT_CAPCL12_OFFSET)
#define LPC43_SCT_CAPCL13           (LPC43_SCT_BASE+LPC43_SCT_CAPCL13_OFFSET)
#define LPC43_SCT_CAPCL14           (LPC43_SCT_BASE+LPC43_SCT_CAPCL14_OFFSET)
#define LPC43_SCT_CAPCL15           (LPC43_SCT_BASE+LPC43_SCT_CAPCL15_OFFSET)

#define LPC43_SCT_CAPCH(n)          (LPC43_SCT_BASE+LPC43_SCT_CAPCH_OFFSET(n))
#define LPC43_SCT_CAPCH0            (LPC43_SCT_BASE+LPC43_SCT_CAPCH0_OFFSET)
#define LPC43_SCT_CAPCH1            (LPC43_SCT_BASE+LPC43_SCT_CAPCH1_OFFSET)
#define LPC43_SCT_CAPCH2            (LPC43_SCT_BASE+LPC43_SCT_CAPCH2_OFFSET)
#define LPC43_SCT_CAPCH3            (LPC43_SCT_BASE+LPC43_SCT_CAPCH3_OFFSET)
#define LPC43_SCT_CAPCH4            (LPC43_SCT_BASE+LPC43_SCT_CAPCH4_OFFSET)
#define LPC43_SCT_CAPCH5            (LPC43_SCT_BASE+LPC43_SCT_CAPCH5_OFFSET)
#define LPC43_SCT_CAPCH6            (LPC43_SCT_BASE+LPC43_SCT_CAPCH6_OFFSET)
#define LPC43_SCT_CAPCH7            (LPC43_SCT_BASE+LPC43_SCT_CAPCH7_OFFSET)
#define LPC43_SCT_CAPCH8            (LPC43_SCT_BASE+LPC43_SCT_CAPCH8_OFFSET)
#define LPC43_SCT_CAPCH9            (LPC43_SCT_BASE+LPC43_SCT_CAPCH9_OFFSET)
#define LPC43_SCT_CAPCH10           (LPC43_SCT_BASE+LPC43_SCT_CAPCH10_OFFSET)
#define LPC43_SCT_CAPCH11           (LPC43_SCT_BASE+LPC43_SCT_CAPCH11_OFFSET)
#define LPC43_SCT_CAPCH12           (LPC43_SCT_BASE+LPC43_SCT_CAPCH12_OFFSET)
#define LPC43_SCT_CAPCH13           (LPC43_SCT_BASE+LPC43_SCT_CAPCH13_OFFSET)
#define LPC43_SCT_CAPCH14           (LPC43_SCT_BASE+LPC43_SCT_CAPCH14_OFFSET)
#define LPC43_SCT_CAPCH15           (LPC43_SCT_BASE+LPC43_SCT_CAPCH15_OFFSET)

#define LPC43_SCT_MATCHRA(n)        (LPC43_SCT_BASE+LPC43_SCT_MATCHRA_OFFSET(n))
#define LPC43_SCT_MATCHRA0          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA0_OFFSET)
#define LPC43_SCT_MATCHRA1          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA1_OFFSET)
#define LPC43_SCT_MATCHRA2          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA2_OFFSET)
#define LPC43_SCT_MATCHRA3          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA3_OFFSET)
#define LPC43_SCT_MATCHRA4          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA4_OFFSET)
#define LPC43_SCT_MATCHRA5          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA5_OFFSET)
#define LPC43_SCT_MATCHRA6          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA6_OFFSET)
#define LPC43_SCT_MATCHRA7          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA7_OFFSET)
#define LPC43_SCT_MATCHRA8          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA8_OFFSET)
#define LPC43_SCT_MATCHRA9          (LPC43_SCT_BASE+LPC43_SCT_MATCHRA9_OFFSET)
#define LPC43_SCT_MATCHRA10         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA10_OFFSET)
#define LPC43_SCT_MATCHRA11         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA11_OFFSET)
#define LPC43_SCT_MATCHRA12         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA12_OFFSET)
#define LPC43_SCT_MATCHRA13         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA13_OFFSET)
#define LPC43_SCT_MATCHRA14         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA14_OFFSET)
#define LPC43_SCT_MATCHRA15         (LPC43_SCT_BASE+LPC43_SCT_MATCHRA15_OFFSET)

#define LPC43_SCT_MATCHRLA(n)       (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA_OFFSET(n))
#define LPC43_SCT_MATCHRLA0         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA0_OFFSET)
#define LPC43_SCT_MATCHRLA1         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA1_OFFSET)
#define LPC43_SCT_MATCHRLA2         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA2_OFFSET)
#define LPC43_SCT_MATCHRLA3         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA3_OFFSET)
#define LPC43_SCT_MATCHRLA4         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA4_OFFSET)
#define LPC43_SCT_MATCHRLA5         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA5_OFFSET)
#define LPC43_SCT_MATCHRLA6         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA6_OFFSET)
#define LPC43_SCT_MATCHRLA7         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA7_OFFSET)
#define LPC43_SCT_MATCHRLA8         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA8_OFFSET)
#define LPC43_SCT_MATCHRLA9         (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA9_OFFSET)
#define LPC43_SCT_MATCHRLA10        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA10_OFFSET)
#define LPC43_SCT_MATCHRLA11        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA11_OFFSET)
#define LPC43_SCT_MATCHRLA12        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA12_OFFSET)
#define LPC43_SCT_MATCHRLA13        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA13_OFFSET)
#define LPC43_SCT_MATCHRLA14        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA14_OFFSET)
#define LPC43_SCT_MATCHRLA15        (LPC43_SCT_BASE+LPC43_SCT_MATCHRLA15_OFFSET)

#define LPC43_SCT_MATCHRHA(n)       (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA_OFFSET(n))
#define LPC43_SCT_MATCHRHA0         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA0_OFFSET)
#define LPC43_SCT_MATCHRHA1         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA1_OFFSET)
#define LPC43_SCT_MATCHRHA2         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA2_OFFSET)
#define LPC43_SCT_MATCHRHA3         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA3_OFFSET)
#define LPC43_SCT_MATCHRHA4         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA4_OFFSET)
#define LPC43_SCT_MATCHRHA5         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA5_OFFSET)
#define LPC43_SCT_MATCHRHA6         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA6_OFFSET)
#define LPC43_SCT_MATCHRHA7         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA7_OFFSET)
#define LPC43_SCT_MATCHRHA8         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA8_OFFSET)
#define LPC43_SCT_MATCHRHA9         (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA9_OFFSET)
#define LPC43_SCT_MATCHRHA10        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA10_OFFSET)
#define LPC43_SCT_MATCHRHA11        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA11_OFFSET)
#define LPC43_SCT_MATCHRHA12        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA12_OFFSET)
#define LPC43_SCT_MATCHRHA13        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA13_OFFSET)
#define LPC43_SCT_MATCHRHA14        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA14_OFFSET)
#define LPC43_SCT_MATCHRHA15        (LPC43_SCT_BASE+LPC43_SCT_MATCHRHA15_OFFSET)

#define LPC43_SCT_CAPCA(n)          (LPC43_SCT_BASE+LPC43_SCT_CAPCA_OFFSET(n))
#define LPC43_SCT_CAPCA0            (LPC43_SCT_BASE+LPC43_SCT_CAPCA0_OFFSET)
#define LPC43_SCT_CAPCA1            (LPC43_SCT_BASE+LPC43_SCT_CAPCA1_OFFSET)
#define LPC43_SCT_CAPCA2            (LPC43_SCT_BASE+LPC43_SCT_CAPCA2_OFFSET)
#define LPC43_SCT_CAPCA3            (LPC43_SCT_BASE+LPC43_SCT_CAPCA3_OFFSET)
#define LPC43_SCT_CAPCA4            (LPC43_SCT_BASE+LPC43_SCT_CAPCA4_OFFSET)
#define LPC43_SCT_CAPCA5            (LPC43_SCT_BASE+LPC43_SCT_CAPCA5_OFFSET)
#define LPC43_SCT_CAPCA6            (LPC43_SCT_BASE+LPC43_SCT_CAPCA6_OFFSET)
#define LPC43_SCT_CAPCA7            (LPC43_SCT_BASE+LPC43_SCT_CAPCA7_OFFSET)
#define LPC43_SCT_CAPCA8            (LPC43_SCT_BASE+LPC43_SCT_CAPCA8_OFFSET)
#define LPC43_SCT_CAPCA9            (LPC43_SCT_BASE+LPC43_SCT_CAPCA9_OFFSET)
#define LPC43_SCT_CAPCA10           (LPC43_SCT_BASE+LPC43_SCT_CAPCA10_OFFSET)
#define LPC43_SCT_CAPCA11           (LPC43_SCT_BASE+LPC43_SCT_CAPCA11_OFFSET)
#define LPC43_SCT_CAPCA12           (LPC43_SCT_BASE+LPC43_SCT_CAPCA12_OFFSET)
#define LPC43_SCT_CAPCA13           (LPC43_SCT_BASE+LPC43_SCT_CAPCA13_OFFSET)
#define LPC43_SCT_CAPCA14           (LPC43_SCT_BASE+LPC43_SCT_CAPCA14_OFFSET)
#define LPC43_SCT_CAPCA15           (LPC43_SCT_BASE+LPC43_SCT_CAPCA15_OFFSET)

#define LPC43_SCT_CAPCLA(n)         (LPC43_SCT_BASE+LPC43_SCT_CAPCLA_OFFSET(n))
#define LPC43_SCT_CAPCLA0           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA0_OFFSET)
#define LPC43_SCT_CAPCLA1           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA1_OFFSET)
#define LPC43_SCT_CAPCLA2           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA2_OFFSET)
#define LPC43_SCT_CAPCLA3           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA3_OFFSET)
#define LPC43_SCT_CAPCLA4           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA4_OFFSET)
#define LPC43_SCT_CAPCLA5           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA5_OFFSET)
#define LPC43_SCT_CAPCLA6           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA6_OFFSET)
#define LPC43_SCT_CAPCLA7           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA7_OFFSET)
#define LPC43_SCT_CAPCLA8           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA8_OFFSET)
#define LPC43_SCT_CAPCLA9           (LPC43_SCT_BASE+LPC43_SCT_CAPCLA9_OFFSET)
#define LPC43_SCT_CAPCLA10          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA10_OFFSET)
#define LPC43_SCT_CAPCLA11          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA11_OFFSET)
#define LPC43_SCT_CAPCLA12          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA12_OFFSET)
#define LPC43_SCT_CAPCLA13          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA13_OFFSET)
#define LPC43_SCT_CAPCLA14          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA14_OFFSET)
#define LPC43_SCT_CAPCLA15          (LPC43_SCT_BASE+LPC43_SCT_CAPCLA15_OFFSET)

#define LPC43_SCT_CAPCHA(n)         (LPC43_SCT_BASE+LPC43_SCT_CAPCHA_OFFSET(n))
#define LPC43_SCT_CAPCHA0           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA0_OFFSET)
#define LPC43_SCT_CAPCHA1           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA1_OFFSET)
#define LPC43_SCT_CAPCHA2           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA2_OFFSET)
#define LPC43_SCT_CAPCHA3           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA3_OFFSET)
#define LPC43_SCT_CAPCHA4           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA4_OFFSET)
#define LPC43_SCT_CAPCHA5           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA5_OFFSET)
#define LPC43_SCT_CAPCHA6           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA6_OFFSET)
#define LPC43_SCT_CAPCHA7           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA7_OFFSET)
#define LPC43_SCT_CAPCHA8           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA8_OFFSET)
#define LPC43_SCT_CAPCHA9           (LPC43_SCT_BASE+LPC43_SCT_CAPCHA9_OFFSET)
#define LPC43_SCT_CAPCHA10          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA10_OFFSET)
#define LPC43_SCT_CAPCHA11          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA11_OFFSET)
#define LPC43_SCT_CAPCHA12          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA12_OFFSET)
#define LPC43_SCT_CAPCHA13          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA13_OFFSET)
#define LPC43_SCT_CAPCHA14          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA14_OFFSET)
#define LPC43_SCT_CAPCHA15          (LPC43_SCT_BASE+LPC43_SCT_CAPCHA15_OFFSET)

#define LPC43_SCT_EVSM(n)           (LPC43_SCT_BASE+LPC43_SCT_EVSM_OFFSET(n))
#define LPC43_SCT_EVC(n)            (LPC43_SCT_BASE+LPC43_SCT_EVC_OFFSET(n))

#define LPC43_SCT_EVSM0             (LPC43_SCT_BASE+LPC43_SCT_EVSM0_OFFSET)
#define LPC43_SCT_EVC0              (LPC43_SCT_BASE+LPC43_SCT_EVC0_OFFSET)
#define LPC43_SCT_EVSM1             (LPC43_SCT_BASE+LPC43_SCT_EVSM1_OFFSET)
#define LPC43_SCT_EVC1              (LPC43_SCT_BASE+LPC43_SCT_EVC1_OFFSET)
#define LPC43_SCT_EVSM2             (LPC43_SCT_BASE+LPC43_SCT_EVSM2_OFFSET)
#define LPC43_SCT_EVC2              (LPC43_SCT_BASE+LPC43_SCT_EVC2_OFFSET)
#define LPC43_SCT_EVSM3             (LPC43_SCT_BASE+LPC43_SCT_EVSM3_OFFSET)
#define LPC43_SCT_EVC3              (LPC43_SCT_BASE+LPC43_SCT_EVC3_OFFSET)
#define LPC43_SCT_EVSM4             (LPC43_SCT_BASE+LPC43_SCT_EVSM4_OFFSET)
#define LPC43_SCT_EVC4              (LPC43_SCT_BASE+LPC43_SCT_EVC4_OFFSET)
#define LPC43_SCT_EVSM5             (LPC43_SCT_BASE+LPC43_SCT_EVSM5_OFFSET)
#define LPC43_SCT_EVC5              (LPC43_SCT_BASE+LPC43_SCT_EVC5_OFFSET)
#define LPC43_SCT_EVSM6             (LPC43_SCT_BASE+LPC43_SCT_EVSM6_OFFSET)
#define LPC43_SCT_EVC6              (LPC43_SCT_BASE+LPC43_SCT_EVC6_OFFSET)
#define LPC43_SCT_EVSM7             (LPC43_SCT_BASE+LPC43_SCT_EVSM7_OFFSET)
#define LPC43_SCT_EVC7              (LPC43_SCT_BASE+LPC43_SCT_EVC7_OFFSET)
#define LPC43_SCT_EVSM8             (LPC43_SCT_BASE+LPC43_SCT_EVSM8_OFFSET)
#define LPC43_SCT_EVC8              (LPC43_SCT_BASE+LPC43_SCT_EVC8_OFFSET)
#define LPC43_SCT_EVSM9             (LPC43_SCT_BASE+LPC43_SCT_EVSM9_OFFSET)
#define LPC43_SCT_EVC9              (LPC43_SCT_BASE+LPC43_SCT_EVC9_OFFSET)
#define LPC43_SCT_EVSM10            (LPC43_SCT_BASE+LPC43_SCT_EVSM10_OFFSET)
#define LPC43_SCT_EVC10             (LPC43_SCT_BASE+LPC43_SCT_EVC10_OFFSET)
#define LPC43_SCT_EVSM11            (LPC43_SCT_BASE+LPC43_SCT_EVSM11_OFFSET)
#define LPC43_SCT_EVC11             (LPC43_SCT_BASE+LPC43_SCT_EVC11_OFFSET)
#define LPC43_SCT_EVSM12            (LPC43_SCT_BASE+LPC43_SCT_EVSM12_OFFSET)
#define LPC43_SCT_EVC12             (LPC43_SCT_BASE+LPC43_SCT_EVC12_OFFSET)
#define LPC43_SCT_EVSM13            (LPC43_SCT_BASE+LPC43_SCT_EVSM13_OFFSET)
#define LPC43_SCT_EVC13             (LPC43_SCT_BASE+LPC43_SCT_EVC13_OFFSET)
#define LPC43_SCT_EVSM14            (LPC43_SCT_BASE+LPC43_SCT_EVSM14_OFFSET)
#define LPC43_SCT_EVC14             (LPC43_SCT_BASE+LPC43_SCT_EVC14_OFFSET)
#define LPC43_SCT_EVSM15            (LPC43_SCT_BASE+LPC43_SCT_EVSM15_OFFSET)
#define LPC43_SCT_EVC15             (LPC43_SCT_BASE+LPC43_SCT_EVC15_OFFSET)

#define LPC43_SCT_OUTSET(n)         (LPC43_SCT_BASE+LPC43_SCT_OUTSET_OFFSET(n))
#define LPC43_SCT_OUTCLR(n)         (LPC43_SCT_BASE+LPC43_SCT_OUTCLR_OFFSET(n))

#define LPC43_SCT_OUTSET0           (LPC43_SCT_BASE+LPC43_SCT_OUTSET0_OFFSET)
#define LPC43_SCT_OUTCLR0           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR0_OFFSET)
#define LPC43_SCT_OUTSET1           (LPC43_SCT_BASE+LPC43_SCT_OUTSET1_OFFSET)
#define LPC43_SCT_OUTCLR1           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR1_OFFSET)
#define LPC43_SCT_OUTSET2           (LPC43_SCT_BASE+LPC43_SCT_OUTSET2_OFFSET)
#define LPC43_SCT_OUTCLR2           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR2_OFFSET)
#define LPC43_SCT_OUTSET3           (LPC43_SCT_BASE+LPC43_SCT_OUTSET3_OFFSET)
#define LPC43_SCT_OUTCLR3           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR3_OFFSET)
#define LPC43_SCT_OUTSET4           (LPC43_SCT_BASE+LPC43_SCT_OUTSET4_OFFSET)
#define LPC43_SCT_OUTCLR4           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR4_OFFSET)
#define LPC43_SCT_OUTSET5           (LPC43_SCT_BASE+LPC43_SCT_OUTSET5_OFFSET)
#define LPC43_SCT_OUTCLR5           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR5_OFFSET)
#define LPC43_SCT_OUTSET6           (LPC43_SCT_BASE+LPC43_SCT_OUTSET6_OFFSET)
#define LPC43_SCT_OUTCLR6           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR6_OFFSET)
#define LPC43_SCT_OUTSET7           (LPC43_SCT_BASE+LPC43_SCT_OUTSET7_OFFSET)
#define LPC43_SCT_OUTCLR7           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR7_OFFSET)
#define LPC43_SCT_OUTSET8           (LPC43_SCT_BASE+LPC43_SCT_OUTSET8_OFFSET)
#define LPC43_SCT_OUTCLR8           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR8_OFFSET)
#define LPC43_SCT_OUTSET9           (LPC43_SCT_BASE+LPC43_SCT_OUTSET9_OFFSET)
#define LPC43_SCT_OUTCLR9           (LPC43_SCT_BASE+LPC43_SCT_OUTCLR9_OFFSET)
#define LPC43_SCT_OUTSET10          (LPC43_SCT_BASE+LPC43_SCT_OUTSET10_OFFSET)
#define LPC43_SCT_OUTCLR10          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR10_OFFSET)
#define LPC43_SCT_OUTSET11          (LPC43_SCT_BASE+LPC43_SCT_OUTSET11_OFFSET)
#define LPC43_SCT_OUTCLR11          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR11_OFFSET)
#define LPC43_SCT_OUTSET12          (LPC43_SCT_BASE+LPC43_SCT_OUTSET12_OFFSET)
#define LPC43_SCT_OUTCLR12          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR12_OFFSET)
#define LPC43_SCT_OUTSET13          (LPC43_SCT_BASE+LPC43_SCT_OUTSET13_OFFSET)
#define LPC43_SCT_OUTCLR13          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR13_OFFSET)
#define LPC43_SCT_OUTSET14          (LPC43_SCT_BASE+LPC43_SCT_OUTSET14_OFFSET)
#define LPC43_SCT_OUTCLR14          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR14_OFFSET)
#define LPC43_SCT_OUTSET15          (LPC43_SCT_BASE+LPC43_SCT_OUTSET15_OFFSET)
#define LPC43_SCT_OUTCLR15          (LPC43_SCT_BASE+LPC43_SCT_OUTCLR15_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* SCT configuration register */

#define SCT_CONFIG_UNIFY            (1 << 0)  /* Bit 0:  0  SCT operation */
#define SCT_CONFIG_CLKMODE_SHIFT    (1)       /* Bits 1-2: SCT clock mode */
#define SCT_CONFIG_CLKMODE_MASK     (3 << SCT_CONFIG_CLKMODE_SHIFT)
#  define SCT_CONFIG_CLKMODE_BUS    (0 << SCT_CONFIG_CLKMODE_SHIFT) /* Bus clock clocks SCT and prescalers */
#  define SCT_CONFIG_CLKMODE_SCT    (1 << SCT_CONFIG_CLKMODE_SHIFT) /* SCT clock is the bus clock */
#  define SCT_CONFIG_CLKMODE_CLKSEL (2 << SCT_CONFIG_CLKMODE_SHIFT) /* CLKSEL clocks SCT and prescalers */
#  define SCT_CONFIG_CLKMODE_EDGE   (3 << SCT_CONFIG_CLKMODE_SHIFT) /* CLKSEL input edge clocks SCT and prescalers */
#define SCT_CONFIG_CLKSEL_SHIFT     (3)       /* Bits 3-6: SCT clock select */
#define SCT_CONFIG_CLKSEL_MASK      (15 << SCT_CONFIG_CLKSEL_SHIFT)
# define SCT_CONFIG_CLKSEL_REDGE0   (0 << SCT_CONFIG_CLKSEL_SHIFT)  /* Rising edges on input 0 */
# define SCT_CONFIG_CLKSEL_FEDGE0   (1 << SCT_CONFIG_CLKSEL_SHIFT)  /* Falling edges on input 0 */
# define SCT_CONFIG_CLKSEL_REDGE1   (2 << SCT_CONFIG_CLKSEL_SHIFT)  /* Rising edges on input 1 */
# define SCT_CONFIG_CLKSEL_FEDGE1   (3 << SCT_CONFIG_CLKSEL_SHIFT)  /* Falling edges on input 1 */
# define SCT_CONFIG_CLKSEL_REDGE2   (4 << SCT_CONFIG_CLKSEL_SHIFT)  /* Rising edges on input 2 */
# define SCT_CONFIG_CLKSEL_FEDGE2   (5 << SCT_CONFIG_CLKSEL_SHIFT)  /* Falling edges on input 2 */
# define SCT_CONFIG_CLKSEL_REDGE3   (6 << SCT_CONFIG_CLKSEL_SHIFT)  /* Rising edges on input 3 */
# define SCT_CONFIG_CLKSEL_FEDGE3   (7 << SCT_CONFIG_CLKSEL_SHIFT)  /* Falling edges on input 3 */
# define SCT_CONFIG_CLKSEL_REDGE4   (8 << SCT_CONFIG_CLKSEL_SHIFT)  /* Rising edges on input 4 */
# define SCT_CONFIG_CLKSEL_FEDGE4   (9 << SCT_CONFIG_CLKSEL_SHIFT)  /* Falling edges on input 4 */
# define SCT_CONFIG_CLKSEL_REDGE5   (10 << SCT_CONFIG_CLKSEL_SHIFT) /* Rising edges on input 5 */
# define SCT_CONFIG_CLKSEL_FEDGE5   (11 << SCT_CONFIG_CLKSEL_SHIFT) /* Falling edges on input 5 */
# define SCT_CONFIG_CLKSEL_REDGE6   (12 << SCT_CONFIG_CLKSEL_SHIFT) /* Rising edges on input 6 */
# define SCT_CONFIG_CLKSEL_FEDGE6   (13 << SCT_CONFIG_CLKSEL_SHIFT) /* Falling edges on input 6 */
# define SCT_CONFIG_CLKSEL_REDGE7   (14 << SCT_CONFIG_CLKSEL_SHIFT) /* Rising edges on input 7 */
# define SCT_CONFIG_CLKSEL_FEDGE7   (15 << SCT_CONFIG_CLKSEL_SHIFT) /* Falling edges on input 7 */
#define SCT_CONFIG_NORELOADU        (1 << 7)  /* Bit 7:  Disable unified match register reload */
#define SCT_CONFIG_NORELOADL        (1 << 7)  /* Bit 7:  Disable lower match registers reload */
#define SCT_CONFIG_NORELOADH        (1 << 8)  /* Bit 8:  Disable higher match register reload */
#define SCT_CONFIG_INSYNC_SHIFT     (9)       /* Bits 9-16: Synchronization for input n=1..7 */
#define SCT_CONFIG_INSYNC_MASK      (0xff << SCT_CONFIG_INSYNC_SHIFT)
#  define SCT_CONFIG_INSYNC(n)      (1 << (SCT_CONFIG_INSYNC_SHIFT+(n)))
                                              /* Bits 17-31: Reserved */
/* SCT control register */

#define SCT_CTRL_DOWNU              (1 << 0)  /* Bit 0:  Unified counter counts down */
#define SCT_CTRL_STOPU              (1 << 1)  /* Bit 1:  Unified counter stopped (I/O events can occur) */
#define SCT_CTRL_HALTU              (1 << 2)  /* Bit 2:  Unified counter halted (no events can occur) */
#define SCT_CTRL_CLRCTRU            (1 << 3)  /* Bit 3:  Clear unified counter */
#define SCT_CTRL_BIDIRU             (1 << 4)  /* Bit 4:  Unified counter direction select */
#define SCT_CTRL_PREU_SHIFT         (5)       /* Bits 5-12: Unified counter SCT clock prescale factor */
#define SCT_CTRL_PREU_MASK          (0xff << SCT_CTRL_PREU_SHIFT)

#define SCT_CTRL_DOWNL              (1 << 0)  /* Bit 0:  L counter counts down */
#define SCT_CTRL_STOPL              (1 << 1)  /* Bit 1:  L counter stopped (I/O events can occur) */
#define SCT_CTRL_HALTL              (1 << 2)  /* Bit 2:  L counter halted (no events can occur) */
#define SCT_CTRL_CLRCTRL            (1 << 3)  /* Bit 3:  Clear L counter */
#define SCT_CTRL_BIDIRL             (1 << 4)  /* Bit 4:  L counter direction select */
#define SCT_CTRL_PREL_SHIFT         (5)       /* Bits 5-12: L counter SCT clock prescale factor */
#define SCT_CTRL_PREL_MASK          (0xff << SCT_CTRL_PREL_SHIFT)
                                              /* Bits 13-15: Reserved */
#define SCT_CTRL_DOWNH              (1 << 16) /* Bit 16: H counter counts down */
#define SCT_CTRL_STOPH              (1 << 17) /* Bit 17: H counter stopped (I/O events can occur) */
#define SCT_CTRL_HALTH              (1 << 18) /* Bit 18: H counter halted (no events can occur) */
#define SCT_CTRL_CLRCTRH            (1 << 19) /* Bit 19: Clear H counter */
#define SCT_CTRL_BIDIRH             (1 << 20) /* Bit 20: H counter direction select */
#define SCT_CTRL_PREH_SHIFT         (21)      /* Bits 21-28: H counter SCT clock prescale factor */
#define SCT_CTRL_PREH_MASK          (0xff << yy)
                                                /* Bits 29-31: Reserved */
/* SCT control register low/high 16-bit */

#define SCT_CTRL_DOWN               (1 << 0)  /* Bit 0:  Unified counter counts down */
#define SCT_CTRL_STOP               (1 << 1)  /* Bit 1:  Unified counter stopped (I/O events can occur) */
#define SCT_CTRL_HALT               (1 << 2)  /* Bit 2:  Unified counter halted (no events can occur) */
#define SCT_CTRL_CLRCTR             (1 << 3)  /* Bit 3:  Clear unified counter */
#define SCT_CTRL_BIDIR              (1 << 4)  /* Bit 4:  Unified counter direction select */
#define SCT_CTRL_PRE_SHIFT          (5)       /* Bits 5-12: Unified counter SCT clock prescale factor */
#define SCT_CTRL_PRE_MASK           (0xff << SCT_CTRL_PRE_SHIFT)
                                              /* Bits 13-16: Reserved */
/* SCT limit register (all 32-bits for unified counter) */

#define SCT_LIMITL_SHIFT            (0)       /* Bits 0-15: Limit for L counter */
#define SCT_LIMITL_MASK             (0xffff << SCT_LIMITL_SHIFT)
#define SCT_LIMITH_SHIFT            (16)      /* Bits 16-31: Limit for H counter */
#define SCT_LIMITH_MASK             (0xffff << SCT_LIMITH_SHIFT)

/* SCT limit register low/high 16-bit (all 16-bits for limit value) */

/* SCT halt condition register (all 32-bits for unified counter) */

#define SCT_HALTU(n)                (1 << (n))
#define SCT_HALTL_SHIFT             (0)       /* Bits 0-15: Set HALTL in CTRL register */
#define SCT_HALTL_MASK              (0xffff << SCT_HALTL_SHIFT)
#  define SCT_HALTL(n)              (1 << (n))
#define SCT_HALTH_SHIFT             (16)      /* Bits 16-31:Set HALTL in CTRL register */
#define SCT_HALTH_MASK              (0xffff << SCT_HALTH_SHIFT)
#  define SCT_HALTH(n)              (1 << (SCT_HALTH_SHIFT+(n)))

/* SCT halt condition register low/high 16-bit (all 16-bits for halt condition) */

#define SCT_HALT(n)                 (1 << (n))

/* SCT stop condition register (all 32-bits for unified counter) */

#define SCT_STOPU(n)                (1 << (n))
#define SCT_STOPL_SHIFT             (0)       /* Bits 0-15: Set STOPL in CTRL register */
#define SCT_STOPL_MASK              (0xffff << SCT_STOPL_SHIFT)
#  define SCT_STOPL(n)              (1 << (n))
#define SCT_STOPH_SHIFT             (16)      /* Bits 16-31:Set STOPL in CTRL register */
#define SCT_STOPH_MASK              (0xffff << SCT_STOPH_SHIFT)
#  define SCT_STOPH(n)              (1 << (SCT_STOPH_SHIFT+(n)))

/* SCT stop condition register low 16-bit (all 16-bits for stop condition) */

#define SCT_STOP(n)                 (1 << (n))

/* SCT start condition register (all 32-bits for unified counter) */

#define SCT_STARTU(n)               (1 << (n))
#define SCT_STARTL_SHIFT            (0)       /* Bits 0-15: Clear STOPL in CTRL register */
#define SCT_STARTL_MASK             (0xffff << SCT_STARTL_SHIFT)
#  define SCT_STARTL(n)             (1 << (n))
#define SCT_STARTH_SHIFT            (16)      /* Bits 16-31: Clear STOPL in CTRL register */
#define SCT_STARTH_MASK             (0xffff << SCT_STARTH_SHIFT)
#  define SCT_STARTH(n)             (1 << (SCT_STARTH_SHIFT+(n)))

/* SCT start condition register low 16-bit (all 16-bits for start condition) */

#define SCT_START(n)                (1 << (n))

/* SCT counter register (all 32-bits for unified counter) */

#define SCT_COUNTL_SHIFT            (0)       /* Bits 0-15: L counter value */
#define SCT_COUNTL_MASK             (0xffff << SCT_COUNTL_SHIFT)
#define SCT_COUNTH_SHIFT            (16)      /* Bits 16-31: H counter value */
#define SCT_COUNTH_MASK             (0xffff << SCT_COUNTH_SHIFT)

/* SCT counter register low/high 16-bit (all 16-bits for counter value)*/

/* SCT state register */

#define SCT_STATEU_SHIFT            (0)       /* Bits 0-5: Unified counter state */
#define SCT_STATEU_MASK             (31 << SCT_STATEL_SHIFT)
#define SCT_STATEL_SHIFT            (0)       /* Bits 0-5: L counter state */
#define SCT_STATEL_MASK             (31 << SCT_STATEL_SHIFT)
                                              /* Bits 6-15: Reserved */
#define SCT_STATEH_SHIFT            (16)      /* Bits 16-20: H counter state */
#define SCT_STATEH_MASK             (31 << SCT_STATEH_SHIFT)
                                              /* Bits 21-31: Reserved */
/* SCT state register low/high 16-bit */

#define SCT_STATE_SHIFT             (0)       /* Bits 0-5: Counter state */
#define SCT_STATE_MASK              (31 << SCT_STATE_SHIFT)
                                              /* Bits 6-15: Reserved */
/* SCT input register */

#define SCT_INPUT_AIN(n)            (1 << (n))
#define SCT_INPUT_AIN0              (1 << 0)  /* Bit 0:  Real-time status of input 0 */
#define SCT_INPUT_AIN1              (1 << 1)  /* Bit 1:  Real-time status of input 1 */
#define SCT_INPUT_AIN2              (1 << 2)  /* Bit 2:  Real-time status of input 2 */
#define SCT_INPUT_AIN3              (1 << 3)  /* Bit 3:  Real-time status of input 3 */
#define SCT_INPUT_AIN4              (1 << 4)  /* Bit 4:  Real-time status of input 4 */
#define SCT_INPUT_AIN5              (1 << 5)  /* Bit 5:  Real-time status of input 5 */
#define SCT_INPUT_AIN6              (1 << 6)  /* Bit 6:  Real-time status of input 6 */
#define SCT_INPUT_AIN7              (1 << 7)  /* Bit 7:  Real-time status of input 7 */
                                              /* Bits 8-15: Reserved */
#define SCT_INPUT_SIN(n)            (1 << ((n)+16))
#define SCT_INPUT_SIN0              (1 << 16) /* Bit 16: Synchronized input 0 state */
#define SCT_INPUT_SIN1              (1 << 17) /* Bit 17: Synchronized input 1 state */
#define SCT_INPUT_SIN2              (1 << 18) /* Bit 18: Synchronized input 2 state */
#define SCT_INPUT_SIN3              (1 << 19) /* Bit 19: Synchronized input 3 state */
#define SCT_INPUT_SIN4              (1 << 20) /* Bit 20: Synchronized input 4 state */
#define SCT_INPUT_SIN5              (1 << 21) /* Bit 21: Synchronized input 5 state */
#define SCT_INPUT_SIN6              (1 << 22) /* Bit 22: Synchronized input 6 state */
#define SCT_INPUT_SIN7              (1 << 23) /* Bit 23: Synchronized input 7 state */
                                              /* Bits 24-31: Reserved */
/* SCT match/capture registers mode register (all 32-bits for unified counter)*/

#define SCT_REGMU(n)                (1 << (n))
#define SCT_REGML_SHIFT             (0)       /* Bits 0-15: Match/capture registers n */
#define SCT_REGML_MASK              (0xffff << SCT_REGML_SHIFT)
#  define SCT_REGML(n)              (1 << (n))
#define SCT_REGMH_SHIFT             (16)      /* Bits 16-31: Match/capture registers n */
#define SCT_REGMH_MASK              (0xffff << SCT_REGMH_SHIFT)
#  define SCT_REGMH(n)              (1 << (SCT_REGMH_SHIFT+(n)))

/* SCT match/capture registers mode register low 16-bit (all 16-bits)*/

#define SCT_REGM(n)                 (1 << (n))

/* SCT output register */

#define SCT_OUTU(n)                 (1 << (n)) /* Bits 0-15: Set output n */

/* SCT output counter direction control register */

#define SCT_OUTDIRC_UNCOND          (0)        /* Set and clear do not depend on any counter */
#define SCT_OUTDIRC_REVU            (1)        /* Reversed when unified counter is counting down */
#define SCT_OUTDIRC_REVL            (1)        /* Reversed when L counter is counting down */
#define SCT_OUTDIRC_REVH            (2)        /* Reversed when H counter is counting down */

#define SCT_OUTDIRC_SETCLR_SHIFT(c) ((c) << 1)
#define SCT_OUTDIRC_SETCLR_SHIFT(c) (3 << SCT_OUTDIRC_SETCLR_SHIFT(c))
#  define SCT_OUTDIRC_SETCLR(c,n)   ((n) << SCT_OUTDIRC_SETCLR_SHIFT(c))

#define SCT_OUTDIRC_SETCLR0_SHIFT   (0)       /* Bits 0-1: Set/clear operation on output 0 */
#define SCT_OUTDIRC_SETCLR0_MASK    (3 << SCT_OUTDIRC_SETCLR0_SHIFT)
#  define SCT_OUTDIRC_SETCLR0(n)    ((n) << SCT_OUTDIRC_SETCLR0_SHIFT)
#define SCT_OUTDIRC_SETCLR1_SHIFT   (2)       /* Bits 2-3: Set/clear operation on output 1 */
#define SCT_OUTDIRC_SETCLR1_MASK    (3 << SCT_OUTDIRC_SETCLR1_SHIFT)
#  define SCT_OUTDIRC_SETCLR1(n)    ((n) << SCT_OUTDIRC_SETCLR1_SHIFT)
#define SCT_OUTDIRC_SETCLR2_SHIFT   (4)       /* Bits 4-5: Set/clear operation on output 2 */
#define SCT_OUTDIRC_SETCLR2_MASK    (3 << SCT_OUTDIRC_SETCLR2_SHIFT)
#  define SCT_OUTDIRC_SETCLR2(n)    ((n) << SCT_OUTDIRC_SETCLR2_SHIFT)
#define SCT_OUTDIRC_SETCLR3_SHIFT   (6)       /* Bits 6-7: Set/clear operation on output 3 */
#define SCT_OUTDIRC_SETCLR3_MASK    (3 << SCT_OUTDIRC_SETCLR3_SHIFT)
#  define SCT_OUTDIRC_SETCLR3(n)    ((n) << SCT_OUTDIRC_SETCLR3_SHIFT)
#define SCT_OUTDIRC_SETCLR4_SHIFT   (8)       /* Bits 8-9: Set/clear operation on output 4 */
#define SCT_OUTDIRC_SETCLR4_MASK    (3 << SCT_OUTDIRC_SETCLR4_SHIFT)
#  define SCT_OUTDIRC_SETCLR4(n)    ((n) << SCT_OUTDIRC_SETCLR4_SHIFT)
#define SCT_OUTDIRC_SETCLR5_SHIFT   (10)       /* Bits 10-11: Set/clear operation on output 5 */
#define SCT_OUTDIRC_SETCLR5_MASK    (3 << SCT_OUTDIRC_SETCLR5_SHIFT)
#  define SCT_OUTDIRC_SETCLR5(n)    ((n) << SCT_OUTDIRC_SETCLR5_SHIFT)
#define SCT_OUTDIRC_SETCLR6_SHIFT   (12)       /* Bits 12-13: Set/clear operation on output 6 */
#define SCT_OUTDIRC_SETCLR6_MASK    (3 << SCT_OUTDIRC_SETCLR6_SHIFT)
#  define SCT_OUTDIRC_SETCLR6(n)    ((n) << SCT_OUTDIRC_SETCLR6_SHIFT)
#define SCT_OUTDIRC_SETCLR7_SHIFT   (14)       /* Bits 14-15: Set/clear operation on output 7 */
#define SCT_OUTDIRC_SETCLR7_MASK    (3 << SCT_OUTDIRC_SETCLR7_SHIFT)
#  define SCT_OUTDIRC_SETCLR7(n)    ((n) << SCT_OUTDIRC_SETCLR7_SHIFT)
#define SCT_OUTDIRC_SETCLR8_SHIFT   (16)       /* Bits 16-17: Set/clear operation on output 8 */
#define SCT_OUTDIRC_SETCLR8_MASK    (3 << SCT_OUTDIRC_SETCLR8_SHIFT)
#  define SCT_OUTDIRC_SETCLR8(n)    ((n) << SCT_OUTDIRC_SETCLR8_SHIFT)
#define SCT_OUTDIRC_SETCLR9_SHIFT   (18)       /* Bits 18-19: Set/clear operation on output 9 */
#define SCT_OUTDIRC_SETCLR9_MASK    (3 << SCT_OUTDIRC_SETCLR9_SHIFT)
#  define SCT_OUTDIRC_SETCLR9(n)    ((n) << SCT_OUTDIRC_SETCLR9_SHIFT)
#define SCT_OUTDIRC_SETCLR10_SHIFT  (20)       /* Bits 20-21: Set/clear operation on output 10 */
#define SCT_OUTDIRC_SETCLR10_MASK   (3 << SCT_OUTDIRC_SETCLR10_SHIFT)
#  define SCT_OUTDIRC_SETCLR10(n)   ((n) << SCT_OUTDIRC_SETCLR10_SHIFT)
#define SCT_OUTDIRC_SETCLR11_SHIFT  (22)       /* Bits 22-23: Set/clear operation on output 11 */
#define SCT_OUTDIRC_SETCLR11_MASK   (3 << SCT_OUTDIRC_SETCLR11_SHIFT)
#  define SCT_OUTDIRC_SETCLR11(n)   ((n) << SCT_OUTDIRC_SETCLR11_SHIFT)
#define SCT_OUTDIRC_SETCLR12_SHIFT  (24)       /* Bits 24-25: Set/clear operation on output 12 */
#define SCT_OUTDIRC_SETCLR12_MASK   (3 << SCT_OUTDIRC_SETCLR12_SHIFT)
#  define SCT_OUTDIRC_SETCLR12(n)   ((n) << SCT_OUTDIRC_SETCLR12_SHIFT)
#define SCT_OUTDIRC_SETCLR13_SHIFT  (26)       /* Bits 26-27: Set/clear operation on output 13 */
#define SCT_OUTDIRC_SETCLR13_MASK   (3 << SCT_OUTDIRC_SETCLR13_SHIFT)
#  define SCT_OUTDIRC_SETCLR13(n)   ((n) << SCT_OUTDIRC_SETCLR13_SHIFT)
#define SCT_OUTDIRC_SETCLR14_SHIFT  (28)       /* Bits 28-29: Set/clear operation on output 14 */
#define SCT_OUTDIRC_SETCLR14_MASK   (3 << SCT_OUTDIRC_SETCLR14_SHIFT)
#  define SCT_OUTDIRC_SETCLR14(n)   ((n) << SCT_OUTDIRC_SETCLR14_SHIFT)
#define SCT_OUTDIRC_SETCLR15_SHIFT  (30)       /* Bits 30-31: Set/clear operation on output 15 */
#define SCT_OUTDIRC_SETCLR15_MASK   (3 << SCT_OUTDIRC_SETCLR15_SHIFT)
#  define SCT_OUTDIRC_SETCLR15(n)   ((n) << SCT_OUTDIRC_SETCLR15_SHIFT)

/* SCT conflict resolution register */

#define SCT_RES_NOCHANGE            (0)        /* No change */
#define SCT_RES_SET                 (1)        /* Set output */
#define SCT_RES_CLEAR               (1)        /* Clear output */
#define SCT_RES_TOGGLE              (2)        /* Toggle output */

#define SCT_RES_OUT_SHIFT(c)        ((c) << 1)
#define SCT_RES_OUT_SHIFT(c)        (3 << SCT_RES_OUT_SHIFT(c))
#  define SCT_RES_OUT(c,n)          ((n) << SCT_RES_OUT_SHIFT(c))

#define SCT_RES_OUT0_SHIFT          (0)       /* Bits 0-1: Effect of simultaneous set and clear on output 0 */
#define SCT_RES_OUT0_MASK           (3 << SCT_RES_OUT0_SHIFT)
#  define SCT_RES_OUT0(n)           ((n) << SCT_RES_OUT0_SHIFT)
#define SCT_RES_OUT1_SHIFT          (2)       /* Bits 2-3: Effect of simultaneous set and clear on output 1 */
#define SCT_RES_OUT1_MASK           (3 << SCT_RES_OUT1_SHIFT)
#  define SCT_RES_OUT1(n)           ((n) << SCT_RES_OUT1_SHIFT)
#define SCT_RES_OUT2_SHIFT          (4)       /* Bits 4-5: Effect of simultaneous set and clear on output 2 */
#define SCT_RES_OUT2_MASK           (3 << SCT_RES_OUT2_SHIFT)
#  define SCT_RES_OUT2(n)           ((n) << SCT_RES_OUT2_SHIFT)
#define SCT_RES_OUT3_SHIFT          (6)       /* Bits 6-7: Effect of simultaneous set and clear on output 3 */
#define SCT_RES_OUT3_MASK           (3 << SCT_RES_OUT3_SHIFT)
#  define SCT_RES_OUT3(n)           ((n) << SCT_RES_OUT3_SHIFT)
#define SCT_RES_OUT4_SHIFT          (8)       /* Bits 8-9: Effect of simultaneous set and clear on output 4 */
#define SCT_RES_OUT4_MASK           (3 << SCT_RES_OUT4_SHIFT)
#  define SCT_RES_OUT4(n)           ((n) << SCT_RES_OUT4_SHIFT)
#define SCT_RES_OUT5_SHIFT          (10)       /* Bits 10-11: Effect of simultaneous set and clear on output 5 */
#define SCT_RES_OUT5_MASK           (3 << SCT_RES_OUT5_SHIFT)
#  define SCT_RES_OUT5(n)           ((n) << SCT_RES_OUT5_SHIFT)
#define SCT_RES_OUT6_SHIFT          (12)       /* Bits 12-13: Effect of simultaneous set and clear on output 6 */
#define SCT_RES_OUT6_MASK           (3 << SCT_RES_OUT6_SHIFT)
#  define SCT_RES_OUT6(n)           ((n) << SCT_RES_OUT6_SHIFT)
#define SCT_RES_OUT7_SHIFT          (14)       /* Bits 14-15: Effect of simultaneous set and clear on output 7 */
#define SCT_RES_OUT7_MASK           (3 << SCT_RES_OUT7_SHIFT)
#  define SCT_RES_OUT7(n)           ((n) << SCT_RES_OUT7_SHIFT)
#define SCT_RES_OUT8_SHIFT          (16)       /* Bits 16-17: Effect of simultaneous set and clear on output 8 */
#define SCT_RES_OUT8_MASK           (3 << SCT_RES_OUT8_SHIFT)
#  define SCT_RES_OUT8(n)           ((n) << SCT_RES_OUT8_SHIFT)
#define SCT_RES_OUT9_SHIFT          (18)       /* Bits 18-19: Effect of simultaneous set and clear on output 9 */
#define SCT_RES_OUT9_MASK           (3 << SCT_RES_OUT9_SHIFT)
#  define SCT_RES_OUT9(n)           ((n) << SCT_RES_OUT9_SHIFT)
#define SCT_RES_OUT10_SHIFT         (20)       /* Bits 20-21: Effect of simultaneous set and clear on output 10 */
#define SCT_RES_OUT10_MASK          (3 << SCT_RES_OUT10_SHIFT)
#  define SCT_RES_OUT10(n)          ((n) << SCT_RES_OUT10_SHIFT)
#define SCT_RES_OUT11_SHIFT         (22)       /* Bits 22-23: Effect of simultaneous set and clear on output 11 */
#define SCT_RES_OUT11_MASK          (3 << SCT_RES_OUT11_SHIFT)
#  define SCT_RES_OUT11(n)          ((n) << SCT_RES_OUT11_SHIFT)
#define SCT_RES_OUT12_SHIFT         (24)       /* Bits 24-25: Effect of simultaneous set and clear on output 12 */
#define SCT_RES_OUT12_MASK          (3 << SCT_RES_OUT12_SHIFT)
#  define SCT_RES_OUT12(n)          ((n) << SCT_RES_OUT12_SHIFT)
#define SCT_RES_OUT13_SHIFT         (26)       /* Bits 26-27: Effect of simultaneous set and clear on output 13 */
#define SCT_RES_OUT13_MASK          (3 << SCT_RES_OUT13_SHIFT)
#  define SCT_RES_OUT13(n)          ((n) << SCT_RES_OUT13_SHIFT)
#define SCT_RES_OUT14_SHIFT         (28)       /* Bits 28-29: Effect of simultaneous set and clear on output 14 */
#define SCT_RES_OUT14_MASK          (3 << SCT_RES_OUT14_SHIFT)
#  define SCT_RES_OUT14(n)          ((n) << SCT_RES_OUT14_SHIFT)
#define SCT_RES_OUT15_SHIFT         (30)       /* Bits 30-31: Effect of simultaneous set and clear on output 15 */
#define SCT_RES_OUT15_MASK          (3 << SCT_RES_OUT15_SHIFT)
#  define SCT_RES_OUT15(n)          ((n) << SCT_RES_OUT15_SHIFT)

/* SCT DMA request 0/1 registers */

#define SCT_DMAREQ_DEV_SHIFT        (0)       /* Bits 0-15:  Event n sets DMA request */
#define SCT_DMAREQ_DEV_MASK         (0xffff << SCT_DMAREQ_DEV_SHIFT)
#  define SCT_DMAREQ_DEV(n)         (1 << ((n) + SCT_DMAREQ_DEV_SHIFT))
                                              /* Bits 16-29: Reserved */
#define SCT_DMAREQ_DRL              (1 << 30) /* Bit 30: DMA request when counter reloaded */
#define SCT_DMAREQ_DRQ              (1 << 31) /* Bit 31: State of DMA Request */

/* SCT event enable register */

#define SCT_EVEN_SHIFT              (0)       /* Bits 0-15:  Enable event n interrupts */
#define SCT_EVEN_MASK               (0xffff << SCT_EVEN_SHIFT)
#  define SCT_EVEN(n)               (1 << ((n) + SCT_EVEN_SHIFT))
                                              /* Bits 1-31: Reserved */
/* SCT event flag register */

#define SCT_EVFLAG_SHIFT            (0)       /* Bits 0-15:  Event n status */
#define SCT_EVFLAG_MASK             (0xffff << SCT_EVFLAG_SHIFT)
#  define SCT_EVFLAG(n)             (1 << ((n) + SCT_EVFLAG_SHIFT))
                                              /* Bits 1-31: Reserved */
/* SCT conflict enable register */

#define SCT_CONEN_SHIFT             (0)       /* Bits 0-15:  Event n conflict interrupts */
#define SCT_CONEN_MASK              (0xffff << SCT_CONEN_SHIFT)
#  define SCT_CONEN(n)              (1 << ((n) + SCT_CONEN_SHIFT))
                                              /* Bits 1-31: Reserved */
/* SCT conflict flag register */

#define SCT_CONFLAG_DEV_SHIFT       (0)       /* Bits 0-15:  No-change conflict on output n */
#define SCT_CONFLAG_DEV_MASK        (0xffff << SCT_CONFLAG_DEV_SHIFT)
#  define SCT_CONFLAG_DEV(n)        (1 << ((n) + SCT_CONFLAG_DEV_SHIFT))
                                              /* Bits 16-29: Reserved */
#define SCT_CONFLAG_BUSERRU         (1 << 30) /* Bit 30: Unified counter bus error */
#define SCT_CONFLAG_BUSERRL         (1 << 30) /* Bit 30: L counter bus error bus error */
#define SCT_CONFLAG_BUSERRH         (1 << 31) /* Bit 31: H counter bus error bus error */

/* SCT match value register of match channels 0-15 (all 32-bits for unified counter) */
/* SCT match alias register of match channels 0-15 (all 32-bits for unified counter) */

#define SCT_MATCHL_SHIFT            (0)       /* Bits 0-15: L match value */
#define SCT_MATCHL_MASK             (0xffff << SCT_MATCHL_SHIFT)
#define SCT_MATCHH_SHIFT            (16)      /* Bits 16-31: H match value */
#define SCT_MATCHH_MASK             (0xffff << SCT_MATCHH_SHIFT)

/* SCT high/low match value register of match channels 0-15 */
/* SCT high/low match alias register of match channels 0-15 */

#define SCT_MATCH_SHIFT             (0)       /* Bits 0-15: match value */
#define SCT_MATCH_MASK              (0xffff << SCT_MATCH_SHIFT))

/* SCT match reload register of match channels 0-15 (all 32-bits for unified counter) */
/* SCT match reload alias register of match channels 0-15 (all 32-bits for unified counter) */

#define SCT_RELOADL_SHIFT           (0)       /* Bits 0-15: L reload value */
#define SCT_RELOADL_MASK            (0xffff << SCT_RELOADL_SHIFT)
#define SCT_RELOADH_SHIFT           (16)      /* Bits 16-31: H reload value */
#define SCT_RELOADH_MASK            (0xffff << SCT_RELOADH_SHIFT)

/* SCT high/low match reload register of match channels 0-15 */
/* SCT high/low match reload alias register of match channels 0-15 */

#define SCT_RELOAD_SHIFT            (0)       /* Bits 0-15: Reload value */
#define SCT_RELOAD_MASK             (0xffff << SCT_RELOAD_SHIFT)

/* SCT capture value register of capture channels 0-15 (all 32-bits for unified counter) */
/* SCT capture alias register of capture channels 0-15 (all 32-bits for unified counter) */

#define SCT_CAPL_SHIFT              (0)       /* Bits 0-15: L capture value */
#define SCT_CAPL_MASK               (0xffff << SCT_CAPL_SHIFT)
#define SCT_CAPH_SHIFT              (16)      /* Bits 16-31: H capture value */
#define SCT_CAPH_MASK               (0xffff << SCT_CAPH_SHIFT)

/* SCT high/low capture value register of capture channels 0-15 */
/* SCT high/low capture alias register of capture channels 0-15 */

#define SCT_CAP_SHIFT               (0)       /* Bits 0-15: Capture value */
#define SCT_CAP_MASK                (0xffff << SCT_CAP_SHIFT)

/* SCT capture control register of capture channels 0-15 (all 32-bits for unified counter) */
/* SCT capture control alias register of capture channels 0-15 (all 32-bits for unified counter) */

#define SCT_CAPCONU(n)              (1 << (n))
#define SCT_CAPCONL_SHIFT           (0)       /* Bits 0-15: L capture controls */
#define SCT_CAPCONL_MASK            (0xffff << SCT_CAPCONL_SHIFT)
#  define SCT_CAPCONL(n)            (1 << ((n)+SCT_CAPCONL_SHIFT))
#define SCT_CAPCONH_SHIFT           (16)      /* Bits 16-31: H capture controls */
#define SCT_CAPCONH_MASK            (0xffff << SCT_CAPCONH_SHIFT)
#  define SCT_CAPCONH(n)            (1 << ((n)+SCT_CAPCONH_SHIFT))

/* SCT high/low capture control register of capture channels 0-15 */
/* SCT high/low capture control alias register of capture channels 0-15 */

#define SCT_CAPCON_SHIFT            (0)       /* Bits 0-15: Capture controls */
#define SCT_CAPCON_MASK             (0xffff << SCT_CAPCON_SHIFT)
#  define SCT_CAPCON(n)             (1 << ((n)+SCT_CAPCON_SHIFT))

/* SCT event state mask registers 0 to 15 */

#define SCT_EVSM(n)                 (1 << (n))

/* SCT event control registers 0 to 15 */

#define SCT_EVC_MATCHSEL_SHIFT      (0)       /* Bits 0-3: Selects Match register associated event */
#define SCT_EVC_MATCHSEL_MASK       (15 << SCT_EVC_MATCHSEL_SHIFT)
#define SCT_EVC_HEVENT              (1 << 4)  /* Bit 4:  Select L/H counter */
#define SCT_EVC_OUTSEL              (1 << 5)  /* Bit 5:  Input/output select*/
#define SCT_EVC_IOSEL_SHIFT         (6)       /* Bits 6-9: Selects input or output signal associated event */
#define SCT_EVC_IOSEL_MASK          (15 << SCT_EVC_IOSEL_SHIFT)
#define SCT_EVC_IOCOND_SHIFT        (10)      /* Bits 10-11: Selects I/O condition for event n */
#define SCT_EVC_IOCOND_MASK         (3 << SCT_EVC_IOCOND_SHIFT)
#  define SCT_EVC_IOCOND_LOW        (0 << SCT_EVC_IOCOND_SHIFT)
#  define SCT_EVC_IOCOND_RISE       (1 << SCT_EVC_IOCOND_SHIFT)
#  define SCT_EVC_IOCOND_FALL       (2 << SCT_EVC_IOCOND_SHIFT)
#  define SCT_EVC_IOCOND_HIGH       (3 << SCT_EVC_IOCOND_SHIFT)
#define SCT_EVC_COMBMODE_SHIFT      (12)      /* Bits 12-13: Match and I/O condition usage */
#define SCT_EVC_COMBMODE_MASK       (3 << SCT_EVC_COMBMODE_SHIFT)
#  define SCT_EVC_COMBMODE_OR       (0 << SCT_EVC_COMBMODE_SHIFT)
#  define SCT_EVC_COMBMODE_MATCH    (1 << SCT_EVC_COMBMODE_SHIFT)
#  define SCT_EVC_COMBMODE_IO       (2 << SCT_EVC_COMBMODE_SHIFT)
#  define SCT_EVC_COMBMODE_AND      (3 << SCT_EVC_COMBMODE_SHIFT)
#define SCT_EVC_STATELD             (1 << 14)  /* Bit 14:  STATEV control */
#define SCT_EVC_STATEV_SHIFT        (15)       /* Bits 15-19: State value */
#define SCT_EVC_STATEV_MASK         (31 << SCT_EVC_STATEV_SHIFT)
                                               /* Bits 20-31: Reserved */
/* SCT output set registers 0 to 15 */

#define SCT_OUTSET_SHIFT            (0)        /* Bits 0-15: Bit m selects event m to set output n */
#define SCT_OUTSET_MASK             (0xffff << SCT_OUTSET_SHIFT)
#  define SCT_OUTSET_MASK(m)        (1 << ((n)SCT_OUTSET_SHIFT))
                                               /* Bits 16-31: Reserved */
/* SCT output clear registers 0 to 15 */

#define SCT_OUTCLR_SHIFT            (0)        /* Bits 0-15: Bit m selects event m to clear output n */
#define SCT_OUTCLR_MASK             (0xffff << SCT_OUTCLR_SHIFT)
#  define SCT_OUTCLR_MASK(m)        (1 << ((n)SCT_OUTCLR_SHIFT))
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

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_SCT_H */
