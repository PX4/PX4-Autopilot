/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_analogdie.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_ANALOGDIE_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_ANALOGDIE_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/
/* I2C slave address ****************************************************************************/
/* "The analog die has its own register set which is accessed through the I2C1 interface. The
 *  analog block has one device slave address 0001100. All blocks in the analog die behave
 *  like one I2C slave device to the I2C1 interface."
 */

#define LPC31_ANALOG_I2CADDR            0x0c

/* I2C register base addresses ******************************************************************/

#define LPC31_ANALOG_CHARGER_BASE       0x0000
#define LPC31_ANALOG_CHARGER_SIZE       0x0010

#define LPC31_ANALOG_CODEC_BASE         0x0010
#define LPC31_ANALOG_CODEC_SIZE         0x0010

#define LPC31_ANALOG_RTC_BASE           0x0020
#define LPC31_ANALOG_RTC_SIZE           0x0010

/* Charger register addresses *******************************************************************/

#define LPC31_CHARGER_OTGDCLIC          0x0000 /* PSU and Li-ion charger control register */
#define LPC31_CHARGER_DCDCLIC           0x0001 /* PSU and Li-ion charger status register */
#define LPC31_CHARGER_CGU_ANALOG        0x0002 /* Analog die CGU control register */

/* Audio CODEC register addresses ***************************************************************/

#define LPC31_CODEC_AIN0                0x0010 /* Analog input PGA control */
#define LPC31_CODEC_AIN1                0x0011 /* Analog input control */
#define LPC31_CODEC_AOUT                0x0012 /* Analog output control */
#define LPC31_CODEC_DEC                 0x0013 /* Decimator control */
#define LPC31_CODEC_INT0                0x0014 /* Interpolator control */
#define LPC31_CODEC_INT1                0x0015 /* Interpolator volume control */
#define LPC31_CODEC_I2S1MUX             0x0016 /* I2S1 digital audio multiplexer control */
#define LPC31_CODEC_AOUTDECINT          0x0017 /* Analog out status */

/* RTC register addresses ***********************************************************************/

#define LPC31_RTC_TIME                  0x0020 /* RTC time shadow register */
#define LPC31_RTC_ALARMTIME             0x0021 /* RTC alarm time register */
#define LPC31_RTC_STATUS                0x0022 /* RTC status register */
#define LPC31_RTC_SETENASTAT            0x0023 /* RTC set/enable register */
#define LPC31_RTC_CLRENASTAT            0x0024 /* RTC clear register */

/* Charger register bit definitions *************************************************************/

/* PSU and Li-ion charger control register */

#define CHARGER_LIC_PONTBAT             (1 << 0)  /* Bit 0:  Enable temperature sensing for Li-ion battery */
#define CHARGER_LIC_PONTLIM             (1 << 1)  /* Bit 1:  Power for chip temperature control loop */
#define CHARGER_LIC_TT_SHIFT            (2)       /* Bits 2-4: Chip temperature control loop threshold */
#define CHARGER_LIC_TT_MASK             (7 << CHARGER_LIC_TT_SHIFT)
#define CHARGER_LIC_G_SHIFT             (5)       /* Bits 5-7: Chip temperature control loop gain */
#define CHARGER_LIC_G_MASK              (7 << CHARGER_LIC_G_SHIFT)
#define CHARGER_LIC_CS_SHIFT            (8)       /* Bits 8-11: Compensation setting */
#define CHARGER_LIC_CS_MASK             (15 << CHARGER_LIC_CS_SHIFT)
#define CHARGER_LIC_CHARGEENABLE        (1 << 12) /* Bit 12: Start charger */
#define CHARGER_DCDC_SEL1V8             (1 << 16) /* Bit 16: LDO3 settings */
#define CHARGER_DCDC2_ADJUST_SHIFT      (17)      /* Bits 17-19: DCDC2 voltage setting */
#define CHARGER_DCDC2_ADJUST_MASK       (7 << CHARGER_DCDC2_ADJUST_SHIFT)
#  define CHARGER_DCDC2_ADJUST_1p4V     (0 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.40 */
#  define CHARGER_DCDC2_ADJUST_1p33V    (1 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.33 */
#  define CHARGER_DCDC2_ADJUST_1p26V    (2 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.26 */
#  define CHARGER_DCDC2_ADJUST_1p19V    (3 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.19 */
#  define CHARGER_DCDC2_ADJUST_1p11V    (4 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.11 */
#  define CHARGER_DCDC2_ADJUST_1p04V    (5 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 1.04 */
#  define CHARGER_DCDC2_ADJUST_p97V     (6 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 0.97 */
#  define CHARGER_DCDC2_ADJUST_p9V      (7 << CHARGER_DCDC2_ADJUST_SHIFT) /* Output voltage 0.90 */
#define CHARGER_DCDC1_ADJUST_SHIFT      (20)      /* Bits 20-22: DCDC1 voltage setting */
#define CHARGER_DCDC1_ADJUST_MASK       (7 << CHARGER_DCDC1_ADJUST_SHIFT)
#  define CHARGER_DCDC1_ADJUST_3p2V     (0 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 3.2 */
#  define CHARGER_DCDC1_ADJUST_3p09V    (1 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 3.09 */
#  define CHARGER_DCDC1_ADJUST_2p97V    (2 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.97 */
#  define CHARGER_DCDC1_ADJUST_2p86V    (3 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.86 */
#  define CHARGER_DCDC1_ADJUST_2p74V    (4 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.74 */
#  define CHARGER_DCDC1_ADJUST_2p63V    (5 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.63 */
#  define CHARGER_DCDC1_ADJUST_2p51V    (6 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.51 */
#  define CHARGER_DCDC1_ADJUST_2p4V     (7 << CHARGER_DCDC1_ADJUST_SHIFT) /* Output voltage 2.40 */
#define CHARGER_DCDC_LDOON              (1 << 23) /* Bit 23: LDO on/off control */
#define CHARGER_DCDC_CLKSTABLE          (1 << 24) /* Bit 24: DCDC clock control */
#define CHARGER_USBOTG                  (1 << 28) /* Bit 28: USBOTG charge pump control */

/* PSU and Li-ion charger status register */

#define CHARGER_LIC_TEMPBATOK           (1 << 0)  /* Bit 0:  Battery temperature OK indicator */
#define CHARGER_LIC_NONTC               (1 << 1)  /* Bit 1:  No NTC indicator */
#define CHARGER_LIC_CVCHARGE            (1 << 2)  /* Bit 2:  Constant-voltage charge indicator */
#define CHARGER_LIC_FASTCHARGE          (1 << 3)  /* Bit 3:  Fast-charge indicator (100% current) */
#define CHARGER_LIC_TRICKLECHARGE       (1 << 4)  /* Bit 4:  Trickle-charge indicator (10% current) */
#define CHARGER_LIC_BATTERYFULL         (1 << 5)  /* Bit 5:  Battery full indicator */
#define CHARGER_LIC_CHARGERON           (1 << 6)  /* Bit 6:  charger on indicator */
#define CHARGER_DCDC_INVRAMP_3V3        (1 << 16) /* Bit 16: DCDC1 mode status 4 */
#define CHARGER_DCDC_INVRAMP_1V2        (1 << 17) /* Bit 17: DCDC2 mode status 4 */
#define CHARGER_DCDC_INVINIT_3V3        (1 << 18) /* Bit 18: DCDC1 mode status 3 */
#define CHARGER_DCDC_INVINIT_1V2        (1 << 19) /* Bit 19: DCDC2 mode status 3 */
#define CHARGER_DCDC_INVDISC_3V3        (1 << 20) /* Bit 20: DCDC1 mode status 2 */
#define CHARGER_DCDC_INVDISC_1V2        (1 << 21) /* Bit 21: DCDC2 mode status 2 */
#define CHARGER_DCDC_INVCONT_3V3        (1 << 22) /* Bit 22: DCDC1 mode status 1 */
#define CHARGER_DCDC_INVCONT_1V2        (1 << 23) /* Bit 23: DCDC2 mode status 1 */
#define CHARGER_DCDC_USBDETECT          (1 << 24) /* Bit 24: USB VBUS supply detect */

/* Analog die CGU control register */

#define CHARGER_PD_I2C_CLK256FS         (1 << 0)  /* Bit 0:  Power-down for I2C clock I2C_CLK_256FS */
#define CHARGER_PD_INT_CLKDSP           (1 << 1)  /* Bit 1:  Power-down for interpolator clock INT_CLK_DSP */
#define CHARGER_PD_INT_CLKNS            (1 << 2)  /* Bit 2:  Power-down for interpolator clock INT_CLK_NS */
#define CHARGER_PD_DEC_CLK              (1 << 3)  /* Bit 3:  Power-down for decimator clock DEC_CLK */
#define CHARGER_PD_I2STX_BITCLK         (1 << 4)  /* Bit 4:  Power down I2STX_BCK */
#define CHARGER_PD_I2STX_SYSCLK         (1 << 5)  /* Bit 5:  Power-down for I2STX_SYSCLK */
#define CHARGER_PD_I2SRX_BITCLK         (1 << 6)  /* Bit 6:  Power-down I2SRX_BCK */
#define CHARGER_PD_I2SRX_SYSCLK         (1 << 7)  /* Bit 7:  Power-down for I2SRX_SYSCLK */
#define CHARGER_PD_LIC_CLK              (1 << 8)  /* Bit 8:  Power-down for the Li-ion charger controller clock LIC_CLK */
#define CHARGER_PD_AOUT_DAC             (1 << 11) /* Bit 11: Power-down for the DAC clock AOUT_CLK_DAC */
#define CHARGER_PD_AIN_ADC2             (1 << 12) /* Bit 12: Power-down for the SADC clock AIN_CLK_ADC2 */
#define CHARGER_PD_AIN_ADC1             (1 << 13) /* Bit 13: Power-down for the SADC clock AIN_CLK_ADC1 */
#define CHARGER_PD_AIN_ADCSYS           (1 << 14) /* Bit 14: Power-down for SADC clock AIN_CLK_ADCSYS */
#define CHARGER_INT_CLKNS256FS          (1 << 16) /* Bit 16: Select clock INT_CLK_NS for interpolator */
#define CHARGER_AOUT_CLKDAC256FS        (1 << 17) /* Bit 17: Select DAC clock */
#define CHARGER_AIN_ADC2128FS           (1 << 18) /* Bit 18: 128fs */
#define CHARGER_AIN_ADC1OFF_1           (1 << 19) /* Bit 19: Tied to '1' */
#define CHARGER_AIN_ADCSYS256FS         (1 << 20) /* Bit 20: Select sampling clock for the SADC AIN_CLK_ADCSYS */
#define CHARGER_I2C_CLK                 (1 << 21) /* Bit 21: Select clock for the analog die I2C block */
#define CHARGER_CGU_LSOFF               (1 << 22) /* Bit 22: Level shifter RTC off */
#define CHARGER_CLKDAC_SAMEPHASE        (1 << 23) /* Bit 23: Clock phase DA not inverted */

/* Audio CODEC register bit definitions *********************************************************/

/* Analog input PGA control */

#define CODEC_AIN0_0DB                  0
#define CODEC_AIN0_3DB                  1
#define CODEC_AIN0_6DB                  2
#define CODEC_AIN0_9DB                  3
#define CODEC_AIN0_12DB                 4
#define CODEC_AIN0_15DB                 5
#define CODEC_AIN0_18DB                 6
#define CODEC_AIN0_21DB                 7
#define CODEC_AIN0_24DB                 8

#define CODEC_AIN0_PGA3_SHIFT           (0)      /* Bits 0-3: Gain of PGA3 (right channel) */
#define CODEC_AIN0_PGA3_MASK            (15 << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_0DB           (CODEC_AIN0_0DB  << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_3DB           (CODEC_AIN0_3DB  << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_6DB           (CODEC_AIN0_6DB  << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_9DB           (CODEC_AIN0_9DB  << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_12DB          (CODEC_AIN0_12DB << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_15DB          (CODEC_AIN0_15DB << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_18DB          (CODEC_AIN0_18DB << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_21DB          (CODEC_AIN0_21DB << CODEC_AIN0_PGA3_SHIFT)
#  define CODEC_AIN0_PGA3_24DB          (CODEC_AIN0_24DB << CODEC_AIN0_PGA3_SHIFT)
#define CODEC_AIN0_PGA2_SHIFT           (4)      /* Bits 4-7: Gain of PGA2 (microphone input) */
#define CODEC_AIN0_PGA2_MASK            (0xff << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_0DB           (CODEC_AIN0_0DB  << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_3DB           (CODEC_AIN0_3DB  << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_6DB           (CODEC_AIN0_6DB  << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_9DB           (CODEC_AIN0_9DB  << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_12DB          (CODEC_AIN0_12DB << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_15DB          (CODEC_AIN0_15DB << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_18DB          (CODEC_AIN0_18DB << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_21DB          (CODEC_AIN0_21DB << CODEC_AIN0_PGA2_SHIFT)
#  define CODEC_AIN0_PGA2_24DB          (CODEC_AIN0_24DB << CODEC_AIN0_PGA2_SHIFT)
#define CODEC_AIN0_PGA1_SHIFT           (8)      /* Bits 8-11: Gain of PGA1 (left channel) */
#define CODEC_AIN0_PGA1_MASK            (0xff << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_0DB           (CODEC_AIN0_0DB  << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_3DB           (CODEC_AIN0_3DB  << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_6DB           (CODEC_AIN0_6DB  << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_9DB           (CODEC_AIN0_9DB  << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_12DB          (CODEC_AIN0_12DB << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_15DB          (CODEC_AIN0_15DB << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_18DB          (CODEC_AIN0_18DB << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_21DB          (CODEC_AIN0_21DB << CODEC_AIN0_PGA1_SHIFT)
#  define CODEC_AIN0_PGA1_24DB          (CODEC_AIN0_24DB << CODEC_AIN0_PGA1_SHIFT)

/* Analog input control */

#define CODEC_AIN1_XFBAD2               (1 << 0)  /* Bit 0: ADC enable feedback in loopfilter (right channel) */
#define CODEC_AIN1_XFBAD1               (1 << 1)  /* Bit 1:  ADC enable feedback in loopfilter (left channel) */
#define CODEC_AIN1_DITHER2              (1 << 2)  /* Bit 2:  ADC dither input (right channel) */
#define CODEC_AIN1_DITHER1              (1 << 3)  /* Bit 3:  ADC dither input (left channel) */
#define CODEC_AIN1_PD_VCOM_VREF1        (1 << 4)  /* Bit 4:  ADC_VREF power down */
#define CODEC_AIN1_PD_SDC3              (1 << 5)  /* Bit 5:  BIAS1 power down */
#define CODEC_AIN1_PD_SDC2              (1 << 9)  /* Bit 9:  SDC2 (microphone input) power down */
#define CODEC_AIN1_PD_SDC1              (1 << 10) /* Bit 10: SDC1 (left channel) power down */
#define CODEC_AIN1_PD_PGA3              (1 << 11) /* Bit 11: PGA3 (right channel) power down */
#define CODEC_AIN1_PD_PGA2              (1 << 12) /* Bit 12: PGA2 (microphone input) power down */
#define CODEC_AIN1_PD_PGA1              (1 << 13) /* Bit 13: PGA1 (left channel) power down */
#define CODEC_AIN1_PD_LNA1              (1 << 14) /* Bit 14: LNA power down */
#define CODEC_AIN1_MUXR_SHIFT           (15)      /* Bits 15-16: MUX0 & MUX1 input selection for right channel */
#define CODEC_AIN1_MUXR_MASK            (3 << CODEC_AIN1_MUXR_SHIFT)
#  define CODEC_AIN1_MUXR_TUNER         (0 << CODEC_AIN1_MUXR_SHIFT) /* Tuner input */
#  define CODEC_AIN1_MUXR_LINE          (1 << CODEC_AIN1_MUXR_SHIFT) /* Line input */
#  define CODEC_AIN1_MUXR_MICTBYP       (2 << CODEC_AIN1_MUXR_SHIFT) /* Mic input tuner by-pass */
#  define CODEC_AIN1_MUXR_MICLBYP       (3 << CODEC_AIN1_MUXR_SHIFT) /* Mic input Line-in by-pass */
#define CODEC_AIN1_MUXL_SHIFT           (17)      /* Bits 17-18: MUX0 & MUX1 input selection for left channel */
#define CODEC_AIN1_MUXL_MASK            (3 << CODEC_AIN1_MUXL_SHIFT)
#  define CODEC_AIN1_MUXL_TUNER         (0 << CODEC_AIN1_MUXL_SHIFT) /* Tuner input */
#  define CODEC_AIN1_MUXL_LINE          (1 << CODEC_AIN1_MUXL_SHIFT) /* Line input */
#  define CODEC_AIN1_MUXL_MICTBYP       (2 << CODEC_AIN1_MUXL_SHIFT) /* Mic input tuner by-pass */
#  define CODEC_AIN1_MUXL_MICLBYP       (3 << CODEC_AIN1_MUXL_SHIFT) /* Mic input Line-in by-pass */

/* Analog output control */

#define CODEC_AOUT_PD_ANVC_R            (1 << 0) /* Bit 0:  Power down the Analog Volume Control (AVC) Right */
#define CODEC_AOUT_PD_ANVC_L            (1 << 1) /* Bit 1:  Power down the Analog Volume Control (AVC) Left */
#define CODEC_AOUT_PD_SET_DWA           (1 << 2) /* Bit 2:  Data Weight Algorithm */
#define CODEC_AOUT_PD_SET_FORMAT        (1 << 3) /* Bit 3:  Input format of the I2S bus */
#define CODEC_AOUT_PD_SDAC_R            (1 << 4) /* Bit 4:  Power down the SDAC Right */
#define CODEC_AOUT_PD_SDAC_L            (1 << 5) /* Bit 5:  Power down the SDAC Left */
#define CODEC_AOUT_LIMITERR_SHIFT       (6)      /* Bit 6-7: Current limiter setting (short-circuit protection) right channel */
#define CODEC_AOUT_LIMITERR_MASK        (3 << CODEC_AOUT_LIMITERR_SHIFT)
#  define CODEC_AOUT_LIMITERR_OFF       (0 << CODEC_AOUT_LIMITERR_SHIFT) /* Max current: off */
#  define CODEC_AOUT_LIMITERR_100MA     (1 << CODEC_AOUT_LIMITERR_SHIFT) /* Max current: 100 mA */
#  define CODEC_AOUT_LIMITERR_120MA     (2 << CODEC_AOUT_LIMITERR_SHIFT) /* Max current: 120 mA */
#  define CODEC_AOUT_LIMITERR_140MA     (3 << CODEC_AOUT_LIMITERR_SHIFT) /* Max current: 140 mA */
#define CODEC_AOUT_LIMITERC_SHIFT       (8)      /* Bit 8-9: Current limiter setting (short-circuit protection) common ground channel */
#define CODEC_AOUT_LIMITERC_MASK        (3 << CODEC_AOUT_LIMITERC_SHIFT)
#  define CODEC_AOUT_LIMITERC_OFF       (0 << CODEC_AOUT_LIMITERC_SHIFT) /* Max current: off */
#  define CODEC_AOUT_LIMITERC_200MA     (1 << CODEC_AOUT_LIMITERC_SHIFT) /* Max current: 200 mA */
#  define CODEC_AOUT_LIMITERC_240MA     (2 << CODEC_AOUT_LIMITERC_SHIFT) /* Max current: 240 mA */
#  define CODEC_AOUT_LIMITERC_280MA     (3 << CODEC_AOUT_LIMITERC_SHIFT) /* Max current: 280 mA */
#define CODEC_AOUT_LIMITERL_SHIFT       (10)      /* Bit 10-11: Current limiter setting (short-circuit protection) left channel */
#define CODEC_AOUT_LIMITERL_MASK        (3 << CODEC_AOUT_LIMITERL_SHIFT)
#  define CODEC_AOUT_LIMITERL_OFF       (0 << CODEC_AOUT_LIMITERL_SHIFT) /* Max current: off */
#  define CODEC_AOUT_LIMITERL_100MA     (1 << CODEC_AOUT_LIMITERL_SHIFT) /* Max current: 100 mA */
#  define CODEC_AOUT_LIMITERL_120MA     (2 << CODEC_AOUT_LIMITERL_SHIFT) /* Max current: 120 mA */
#  define CODEC_AOUT_LIMITERL_140MA     (3 << CODEC_AOUT_LIMITERL_SHIFT) /* Max current: 140 mA */
#define CODEC_AOUT_PD_HP_R              (1 << 12) /* Bit 12: Power down the right headphone amplifier */
#define CODEC_AOUT_PD_HP_C              (1 << 13) /* Bit 13: Power down the common ground headphone amplifier */
#define CODEC_AOUT_PD_HP_L              (1 << 14) /* Bit 14: Power down the left headphone amplifier */
#define CODEC_AOUT_PD_VREF_SLOW         (1 << 15) /* Bit 15: Power down the reference */
#define CODEC_AOUT_GAIN_AVC_SHIFT       (16)      /* Bit 16-29: Gain of the Analog Volume Control (AVC) */
#define CODEC_AOUT_GAIN_AVC_MASK        (0x3fff << CODEC_AOUT_GAIN_AVC_SHIFT)
#define CODEC_AOUT_VREF_SLOW_UP         (1 << 30) /* Bit 30: Control for slow reference voltage */
#define CODEC_AOUT_SWDAC_ON             (1 << 31) /* Bit 31: Control the AVC switches of SDAC */

/* Decimator control */

#define CODEC_DEC_AGCTIM_SHIFT          (25)      /* Bits 25-27:  AGC Time Constant */
#define CODEC_DEC_AGCTIM_MASK           (7 << CODEC_DEC_AGCTIM_SHIFT)
#define CODEC_DEC_AGCLVL_SHIFT          (23)      /* Bits 23-24: AGC Level */
#define CODEC_DEC_AGCLVL_MASK           (3 << CODEC_DEC_AGCLVL_SHIFT)
#  define CODEC_DEC_AGCLVL_M5p5DBFS     (0 << CODEC_DEC_AGCLVL_SHIFT) /* AGC Target level -5.5 dBFS */
#  define CODEC_DEC_AGCLVL_M8P0DBFS     (1 << CODEC_DEC_AGCLVL_SHIFT) /* AGC Target level -8.0 dBFS */
#  define CODEC_DEC_AGCLVL_M11P5DBFS    (2 << CODEC_DEC_AGCLVL_SHIFT) /* AGC Target level -11.5 dBFS */
#  define CODEC_DEC_AGCLVL_M14p0DBFS    (3 << CODEC_DEC_AGCLVL_SHIFT) /* AGC Target level -14.0 dBFS */
#define CODEC_DEC_AGCEN                 (1 << 22) /* Bit 22: AGC Enable */
#define CODEC_DEC_MUTE                  (1 << 21) /* Bit 21: Enable mute */
#define CODEC_DEC_POLINV                (1 << 20) /* Bit 20: Enable polarity inversion */
#define CODEC_DEC_DCFILTI               (1 << 19) /* Bit 19: Enable input DC blocking filter */
#define CODEC_DEC_DCFILTO               (1 << 18) /* Bit 18: Enable DC blocking filter after decimation filters */
#define CODEC_DEC_DBLIN                 (1 << 17) /* Bit 17: Enable soft start-up after a reset */
#define CODEC_DEC_DELAY_DBLIN           (1 << 16) /* Bit 16: Enable delay timer after a reset */
#define CODEC_DEC_GAINL_SHIFT           (8)       /* Bits 8-15: Gain settings, LEFT channel (2’s compliment format 0.5dB/bit) */
#define CODEC_DEC_GAINL_MASK            (0xff << CODEC_DEC_GAINL_SHIFT)
#define CODEC_DEC_GAINR_SHIFT           (0)       /* Bits 0-7: Gain settings RIGHT channel (2’s compliment format 0.5dB/bit) */
#define CODEC_DEC_GAINR_MASK            (0xff << CODEC_DEC_GAINR_SHIFT)

/* Interpolator control */

#define CODEC_INT0_DEEM_CHAN1_SHIFT     (0)       /* Bits 0-2:  Set de-emphasis channel 1 */
#define CODEC_INT0_DEEM_CHAN1_MASK      (7 << CODEC_INT0_DEEM_CHAN1_SHIFT)
#  define CODEC_INT0_DEEM_CHAN1_NONE    (0 << CODEC_INT0_DEEM_CHAN1_SHIFT) /* No digital de-emphasis */
#  define CODEC_INT0_DEEM_CHAN1_32KHZ   (1 << CODEC_INT0_DEEM_CHAN1_SHIFT) /* De-emphasis for fs = 32 kHz */
#  define CODEC_INT0_DEEM_CHAN1_44p1KHz (2 << CODEC_INT0_DEEM_CHAN1_SHIFT) /* De-emphasis for fs = 44.1 kHz */
#  define CODEC_INT0_DEEM_CHAN1_48KHz   (3 << CODEC_INT0_DEEM_CHAN1_SHIFT) /* De-emphasis for fs = 48 kHz */
#  define CODEC_INT0_DEEM_CHAN1_96KHz   (4 << CODEC_INT0_DEEM_CHAN1_SHIFT) /* De-emphasis for fs = 96 kHz */
#define CODEC_INT0_SET_SILENCE          (1 << 3)  /* Bit 3:  overruling silence switch input */
#define CODEC_INT0_SD_VALUE_SHIFT       (4)       /* Bits 4-5: Silence detection time window */
#define CODEC_INT0_SD_VALUE_MASK        (3 << CODEC_INT0_SD_VALUE_SHIFT)
#  define CODEC_INT0_SD_VALUE_3200      (0 << CODEC_INT0_SD_VALUE_SHIFT) /* 3200 fs samples */
#  define CODEC_INT0_SD_VALUE_4800      (1 << CODEC_INT0_SD_VALUE_SHIFT) /* 4800 fs samples */
#  define CODEC_INT0_SD_VALUE_9600      (2 << CODEC_INT0_SD_VALUE_SHIFT) /* 9600 fs samples */
#  define CODEC_INT0_SD_VALUE_19200     (3 << CODEC_INT0_SD_VALUE_SHIFT) /* 19200 fs samples */
#define CODEC_INT0_SD                   (1 << 6)  /* Bit 6:  Silence detection enable */
#define CODEC_INT0_PD_DAC               (1 << 7)  /* Bit 7:  Enable power down sequence interpolator */
#define CODEC_INT0_PD_SLOPE             (1 << 8)  /* Bit 8:  DC ramp up/down slope setting */
#define CODEC_INT0_NS_SHIFT             (9)       /* Bits 9-10:  Noise shaper settings (1-bit, 4-bit, 5-bit or 6-bit) */
#define CODEC_INT0_NS_MASK              (3 << CODEC_INT0_NS_SHIFT)
#  define CODEC_INT0_NS_1BIT            (0 << CODEC_INT0_NS_SHIFT) /* 1-bit noise shaped output */
#  define CODEC_INT0_NS_4BIT            (1 << CODEC_INT0_NS_SHIFT) /* 4-bit noise shaped output */
#  define CODEC_INT0_NS_5BIT            (2 << CODEC_INT0_NS_SHIFT) /* 5-bit noise shaped output */
#  define CODEC_INT0_NS_6BIT            (3 << CODEC_INT0_NS_SHIFT) /* 6-bit noise shaped output */
#define CODEC_INT0_SPEED_MODE_SHIFT     (11)      /* Bits 11-12: Input data rate settings (1fs or 8 fs) */
#define CODEC_INT0_SPEED_MODE_MASK      (3 << CODEC_INT0_SPEED_MODE_SHIFT)
#  define CODEC_INT0_SPEED_MODE_1FS     (0 << CODEC_INT0_SPEED_MODE_SHIFT) /* 1fs (normal) speed mode */
#  define CODEC_INT0_SPEED_MODE_2FS     (1 << CODEC_INT0_SPEED_MODE_SHIFT) /* 2fs (double) speed mode */
#  define CODEC_INT0_SPEED_MODE_8FS     (3 << CODEC_INT0_SPEED_MODE_SHIFT) /* 8fs speed mode */
#define CODEC_INT0_FILTER_SHIFT         (13)      /* Bits 13-14: Filter coefficient settings for sharp/slow roll-off */
#define CODEC_INT0_FILTER_MASK          (3 << CODEC_INT0_FILTER_SHIFT)
#  define CODEC_INT0_FILTER_SLOWEST     (0 << CODEC_INT0_FILTER_SHIFT) /* Slow roll-off */
#  define CODEC_INT0_FILTER_SLOWER      (1 << CODEC_INT0_FILTER_SHIFT) /* Slow roll-off */
#  define CODEC_INT0_FILTER_SLOW        (2 << CODEC_INT0_FILTER_SHIFT) /* Slow roll-off */
#  define CODEC_INT0_FILTER_SHARP       (3 << CODEC_INT0_FILTER_SHIFT) /* Sharp roll-off */
#define CODEC_INT0_POLINV               (1 << 15) /* Bit 15:  Enable polarity inversion */

/* Interpolator volume control */

#define CODEC_INT1_MUTE                 (1 << 16) /* Bit 16: Mute interpolator */
#define CODEC_INT1_MASTERVOLR_SHIFT     (8)       /* Bits 8-15: Set volume right channel */
#define CODEC_INT1_MASTERVOLR_MASK      (0xff << CODEC_INT1_MASTERVOLR_SHIFT)
#define CODEC_INT1_MASTERVOLL_SHIFT     (0)       /* Bits 0-7: Set volume left channel */
#define CODEC_INT1_MASTERVOLL_MASK      (0xff << CODEC_INT1_MASTERVOLL_SHIFT)

/* I2S1 digital audio multiplexer control */

#define CODEC_I2S1MUX_TXCTRL_SHIFT      (0)       /* Bit 0-2: Serial interface mode I2STX interface */
#define CODEC_I2S1MUX_TXCTRL_MASK       (7 << CODEC_I2S1MUX_TXCTRL_SHIFT)
#  define CODEC_I2S1MUX_TXCTRL_NXPI2S   (3 << CODEC_I2S1MUX_TXCTRL_SHIFT) /* NXP I2S */
#  define CODEC_I2S1MUX_TXCTRL_LJ16     (4 << CODEC_I2S1MUX_TXCTRL_SHIFT) /* LSB justified 16 bits */
#  define CODEC_I2S1MUX_TXCTRL_LJ18     (5 << CODEC_I2S1MUX_TXCTRL_SHIFT) /* LSB justified 18 bits */
#  define CODEC_I2S1MUX_TXCTRL_LJ20     (6 << CODEC_I2S1MUX_TXCTRL_SHIFT) /* LSB justified 20 bits */
#  define CODEC_I2S1MUX_TXCTRL_LJ24     (7 << CODEC_I2S1MUX_TXCTRL_SHIFT) /* LSB justified 24 bits */
#define CODEC_I2S1MUX_RXCTRL_SHIFT      (8)       /* Bit 8-10: Serial interface mode I2SRX interface */
#define CODEC_I2S1MUX_RXCTRL_MASK       (7 << CODEC_I2S1MUX_RXCTRL_SHIFT)
#  define CODEC_I2S1MUX_RXCTRL_DIL      (0 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* DAD/ISN/LIRS */
#  define CODEC_I2S1MUX_RXCTRL_SPD3     (1 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* SPD3 format */
#  define CODEC_I2S1MUX_RXCTRL_NXPI2S   (3 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* NXP I2S */
#  define CODEC_I2S1MUX_RXCTRL_LJ16     (4 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* LSB justified 16 bits */
#  define CODEC_I2S1MUX_RXCTRL_LJ18     (5 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* LSB justified 18 bits */
#  define CODEC_I2S1MUX_RXCTRL_LJ20     (6 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* LSB justified 20 bits */
#  define CODEC_I2S1MUX_RXCTRL_LJ24     (7 << CODEC_I2S1MUX_RXCTRL_SHIFT) /* LSB justified 24 bits */
#define CODEC_I2S1MUX_BYPASS            (1 << 16) /* Bit 16: Selection for Digital Mux */

/* Analog out status */

#define CODEC_INT_FSPULSE               (1 << 0)  /* Bit 0:  One interpolator DSP clock cycle pulse at 1fs frequency */
#define CODEC_INT_DACPC_SHIFT           (1)       /* Bits 1-7: Program counter on the CLKIN_DSP clock for interpolator */
#define CODEC_INT_DACPC_MASK            (0x7f << CODEC_INT_DACPC_SHIFT)
#define CODEC_INT_SDETECTEDR1           (1 << 8)  /* Bit 8:  Silence detection output channel 1 RIGHT */
#define CODEC_INT_SDETECTEDL1           (1 << 9)  /* Bit 9:  Silence detection ouput channel 1 LEFT */
#define CODEC_INT_MUTESTATEM            (1 << 10) /* Bit 10: Mute status of master channel */
#define CODEC_INT_INVNDAC               (1 << 11) /* Bit 11: Control signal to invert neg. channel data for differential application */
#define CODEC_INT_DSR                   (1 << 12) /* Bit 12: DAC silence switch control signal RIGHT */
#define CODEC_INT_DSL                   (1 << 13) /* Bit 13: DAC silence switch control signal LEFT */
#define CODEC_INT_PDREADY               (1 << 14) /* Bit 14: Power down sequence completed; actual power down signal for DAC */
#define CODEC_DEC_OVERFLOW              (1 << 16) /* Bit 16: Overflow indicator */
#define CODEC_DEC_MUTESTATE             (1 << 17) /* Bit 17: Output muted status indicator */
#define CODEC_DEC_AGCSTAT               (1 << 18) /* Bit 18: T.B.F. */
#define CODEC_AOUT_CLIPR                (1 << 24) /* Bit 24: Output of right headphone amplifier is clipped */
#define CODEC_AOUT_CLIPC                (1 << 25) /* Bit 25: Output of common ground headphone amplifier is clipped */
#define CODEC_AOUT_CLIPL                (1 << 26) /* Bit 26: Output of left headphone amplifier is clipped */

/* RTC register bit definitions *****************************************************************/

/* RTC time shadow register -- 32-bit time in seconds since epoch */
/* RTC alarm time register -- 32-bit time in seconds since epoch */

/* RTC status register (See also common interrupt bits below) */

#define RTC_STATUS_RST                  (1 << 12) /* Bit 12: RTC is in reset */
#define RTC_STATUS_PENDING              (1 << 16) /* Bit 16: Time in RTC_TIME has not yet been updated */
#define RTC_STATUS_LSENA                (1 << 13) /* Bit 13: Software access (via level shifters) is enabled */

/* RTC status register, RTC set/enable register, and RTC clear register common interrupt bits  */

#define RTC_INT_ALARM                   (1 << 0)  /* Bit 0:  RTC time counter matched alarm time */
#define RTC_INT_UNSET                   (1 << 1)  /* Bit 1:  Time undefined */
#define RTC_INT_ADENA                   (1 << 8)  /* Bit 8:  Alarm to the event router */
#define RTC_INT_RTCENA                  (1 << 9)  /* Bit 9:  Alarm assertion to RTC_INT disabled */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_ANALOGDIE_H */
