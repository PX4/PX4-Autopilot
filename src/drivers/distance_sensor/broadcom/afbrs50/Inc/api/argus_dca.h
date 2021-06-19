/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the dynamic configuration adaption (DCA) setup parameters
 * 				and data structure.
 *
 * @copyright
 *
 * Copyright (c) 2021, Broadcom Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARGUS_DCA_H
#define ARGUS_DCA_H

/*!***************************************************************************
 * @defgroup 	argusdca Dynamic Configuration Adaption
 * @ingroup		argusapi
 *
 * @brief		Dynamic Configuration Adaption (DCA) parameter definitions and API functions.
 *
 * @details		The DCA contains an algorithms that detect ambient conditions
 * 				and adopt the device configuration to the changing parameters
 * 				dynamically while operating the sensor. This is achieved by
 * 				rating the currently received signal quality and changing the
 * 				device configuration accordingly to the gathered information
 * 				from the current measurement frame results before the next
 * 				integration cycle starts.
 *
 * 				The DCA consists of the following features:
 * 			 	- Static or Dynamic mode. The first is utilizing the nominal
 * 			 	  values while the latter is dynamically adopting between min.
 * 			 	  and max. value and starting from the nominal values.
 * 			 	- Analog Integration Depth Adaption (from multiple patterns down to single pulses)
 * 			 	- Optical Output Power Adaption
 * 			 	- Pixel Input Gain Adaption (w/ ambient light rejection)
 * 			 	- ADC Sensitivity (i.e. ADC Range) Adaption
 * 			 	- Power Saving Ratio (to decrease the average output power and thus the current consumption)
 * 			 	- All that features are heeding the Laser Safety limits.
 * 				.
 *
 * @addtogroup 	argusdca
 * @{
 *****************************************************************************/

#include "argus_def.h"



/*! The minimum amplitude threshold value. */
#define ARGUS_CFG_DCA_ATH_MIN		(1U << 6U)

/*! The maximum amplitude threshold value. */
#define ARGUS_CFG_DCA_ATH_MAX		(0xFFFFU)


/*! The minimum saturated pixel threshold value. */
#define ARGUS_CFG_DCA_PXTH_MIN		(1U)

/*! The maximum saturated pixel threshold value. */
#define ARGUS_CFG_DCA_PXTH_MAX		(33U)


/*! The maximum analog integration depth in UQ10.6 format,
 * i.e. the maximum pattern count per sample. */
#define ARGUS_CFG_DCA_DEPTH_MAX 	((uq10_6_t)(ADS_SEQCT_N_MASK << (6U - ADS_SEQCT_N_SHIFT)))

/*! The minimum analog integration depth in UQ10.6 format,
 *  i.e. the minimum pattern count per sample. */
#define ARGUS_CFG_DCA_DEPTH_MIN 	((uq10_6_t)(1U)) // 1/64, i.e. 1/2 nibble


/*! The maximum optical output power, i.e. the maximum VCSEL 1 high current in LSB. */
#define ARGUS_CFG_DCA_POWER_MAX_LSB (ADS_LASET_VCSEL_HC1_MASK >> ADS_LASET_VCSEL_HC1_SHIFT)

/*! The minimum optical output power, i.e. the minimum VCSEL 1 high current in mA. */
#define ARGUS_CFG_DCA_POWER_MIN_LSB (1)

/*! The maximum optical output power, i.e. the maximum VCSEL 1 high current in LSB. */
#define ARGUS_CFG_DCA_POWER_MAX		(ADS0032_HIGH_CURRENT_LSB2MA(ARGUS_CFG_DCA_POWER_MAX_LSB + 1))

/*! The minimum optical output power, i.e. the minimum VCSEL 1 high current in mA. */
#define ARGUS_CFG_DCA_POWER_MIN 	(1)








/*! The dynamic configuration algorithm Pixel Input Gain stage count. */
#define ARGUS_DCA_GAIN_STAGE_COUNT (4U)

/*! The dynamic configuration algorithm state mask for the Pixel Input Gain stage. */
#define ARGUS_STATE_DCA_GAIN_MASK (0x03U)

/*! The dynamic configuration algorithm state mask for the Pixel Input Gain stage. */
#define ARGUS_STATE_DCA_GAIN_SHIFT (14U)

/*! Getter for the dynamic configuration algorithm Pixel Input Gain stage. */
#define ARGUS_STATE_DCA_GAIN_GET(state) \
	(((state) >> ARGUS_STATE_DCA_GAIN_SHIFT) & ARGUS_STATE_DCA_GAIN_MASK)


/*! The dynamic configuration algorithm Optical Output Power stage count. */
#define ARGUS_DCA_POWER_STAGE_COUNT (4U)

/*! The dynamic configuration algorithm state mask for the Optical Output Power stage. */
#define ARGUS_STATE_DCA_POWER_MASK (0x03U)

/*! The dynamic configuration algorithm state mask for the Optical Output Power stage. */
#define ARGUS_STATE_DCA_POWER_SHIFT (12U)

/*! Getter for the dynamic configuration algorithm Optical Output Power stage. */
#define ARGUS_STATE_DCA_POWER_GET(state) \
	(((state) >> ARGUS_STATE_DCA_POWER_SHIFT) & ARGUS_STATE_DCA_POWER_MASK)


/*! The dynamic configuration algorithm state mask for the Max. Analog Integration Depth shift value. */
#define ARGUS_STATE_DCA_DEPTH_SHFT_MASK (0x0FU)

/*! The dynamic configuration algorithm state mask for the Max. Analog Integration Depth shift value. */
#define ARGUS_STATE_DCA_DEPTH_SHFT_SHIFT (8U)

/*! Getter for the dynamic configuration algorithm Max. Analog Integration Depth shift value. */
#define ARGUS_STATE_DCA_DEPTH_SHFT_GET(state) \
	(((state) >> ARGUS_STATE_DCA_DEPTH_SHFT_SHIFT) & ARGUS_STATE_DCA_DEPTH_SHFT_MASK)


/*!***************************************************************************
 * @brief	The dynamic configuration algorithm enable flags.
 *****************************************************************************/
typedef enum {
	/*! DCA is disabled and will be completely skipped. */
	DCA_ENABLE_OFF = 0,

	/*! DCA is enabled and will dynamically adjust the device configuration. */
	DCA_ENABLE_DYNAMIC = 1,

	/*! DCA is enabled and will apply the static (nominal) values to the device. */
	DCA_ENABLE_STATIC = -1,

} argus_dca_enable_t;

/*!***************************************************************************
 * @brief	The dynamic configuration algorithm Optical Output Power stages enumerator.
 *****************************************************************************/
typedef enum {
	/*! Low output power stage. */
	DCA_POWER_LOW = 0,

	/*! Medium low output power stage. */
	DCA_POWER_MEDIUM_LOW = 1,

	/*! Medium high output power stage. */
	DCA_POWER_MEDIUM_HIGH = 2,

	/*! High output power stage. */
	DCA_POWER_HIGH = 3

} argus_dca_power_t;

/*!***************************************************************************
 * @brief	The dynamic configuration algorithm Pixel Input Gain stages enumerator.
 *****************************************************************************/
typedef enum {
	/*! Low gain stage. */
	DCA_GAIN_LOW = 0,

	/*! Medium low gain stage. */
	DCA_GAIN_MEDIUM_LOW = 1,

	/*! Medium high gain stage. */
	DCA_GAIN_MEDIUM_HIGH = 2,

	/*! High gain stage. */
	DCA_GAIN_HIGH = 3

} argus_dca_gain_t;


/*!***************************************************************************
 * @brief	State flags for the current frame.
 * @details	State flags determine the current state of the measurement frame:
 * 			- [0]: #ARGUS_STATE_MEASUREMENT_MODE: Measurement Mode:
 * 				- 0: Mode A
 * 				- 1: Mode B
 * 				.
 * 			- [1]: #ARGUS_STATE_DUAL_FREQ_MODE: Dual Frequency Mode Enabled Flag
 * 				- 0: Disabled, measurements w/ base frequency only
 * 				- 1: Enabled, measurements w/ detuned frequency
 * 				.
 * 			- [2]: #ARGUS_STATE_MEASUREMENT_FREQ: Measurement Frequency for
 * 			       Dual Frequency Mode, (only valid if #ARGUS_STATE_DUAL_FREQ_MODE
 * 			       flag is set)
 * 				- 0: A-Frame w/ detuned frequency
 * 				- 1: B-Frame w/ detuned frequency
 * 				.
 * 			- [3]: #ARGUS_STATE_DEBUG_MODE
 * 			- [4]: #ARGUS_STATE_GOLDEN_PIXEL_MODE
 * 			- [5]: #ARGUS_STATE_BGL_WARNING
 * 			- [6]: #ARGUS_STATE_BGL_ERROR
 * 			- [7]: #ARGUS_STATE_PLL_LOCKED
 * 				- 0: PLL_LOCKED bit was not set at start of integration;
 * 				- 0: PLL_LOCKED bit was set at start of integration;
 * 				.
 * 			- [8-11]: Max. Depth Shift Value
 * 			- [12-13]: Power Stages
 * 			- [14-15]: Gain Stages
 * 			.
 *****************************************************************************/
typedef enum {
	/*! No state flag set. */
	ARGUS_STATE_NONE = 0,

	/*! 0x01: Measurement Mode.
	 *  - 0: Mode A: Long Range / Medium Precision
	 *  - 1: Mode B: Short Range / High Precision */
	ARGUS_STATE_MEASUREMENT_MODE = 1U << 0U,

	/*! 0x02: Dual Frequency Mode Enabled.
	 *  - 0: Disabled: measurements with base frequency,
	 *  - 1: Enabled: measurement with detuned frequency. */
	ARGUS_STATE_DUAL_FREQ_MODE = 1U << 1U,

	/*! 0x04: Measurement Frequency for Dual Frequency Mode
	 *        (only if #ARGUS_STATE_DUAL_FREQ_MODE flag is set).
	 *  - 0: A-Frame w/ detuned frequency,
	 *  - 1: B-Frame w/ detuned frequency */
	ARGUS_STATE_MEASUREMENT_FREQ = 1U << 2U,

	/*! 0x08: Debug Mode. If set, the range value of erroneous pixels are not
	 * 		  cleared or reset.
	 * 	- 0: Disabled (default).
	 *  - 1: Enabled. */
	ARGUS_STATE_DEBUG_MODE = 1U << 3U,

	/*! 0x10: Golden Pixel Mode Flag.
	 *  	  Set whenever the Pixel Binning Algorithm is operating in the
	 *  	  Golden Pixel Mode.
	 * 	- 0: Normal Pixel Binning Mode.
	 *  - 1: Golden Pixel Mode. */
	ARGUS_STATE_GOLDEN_PIXEL_MODE = 1U << 4U,

	/*! 0x20: Background Light Warning Flag.
	 *        Set whenever the background light is very high and the
	 *        measurement data might be unreliable.
	 *  - 0: No Warning Background Light is within valid range.
	 *  - 1: Warning: Background Light is very high. */
	ARGUS_STATE_BGL_WARNING = 1U << 5U,

	/*! 0x40: Background Light Error Flag.
	 *        Set whenever the background light is too high and the
	 *        measurement data is unreliable or invalid.
	 *  - 0: No Error, Background Light is within valid range.
	 *  - 1: Error: Background Light is too high. */
	ARGUS_STATE_BGL_ERROR = 1U << 6U,

	/*! 0x80: PLL_LOCKED bit.
	 *  - 0: PLL not locked at start of integration.
	 *  - 1: PLL locked at start of integration. */
	ARGUS_STATE_PLL_LOCKED = 1U << 7U,

	/*! DCA is in low Optical Output Power stage. */
	ARGUS_STATE_DCA_POWER_LOW = DCA_GAIN_LOW << ARGUS_STATE_DCA_POWER_SHIFT,

	/*! DCA is in medium-low Optical Output Power stage. */
	ARGUS_STATE_DCA_POWER_MED_LOW = DCA_GAIN_MEDIUM_LOW << ARGUS_STATE_DCA_POWER_SHIFT,

	/*! DCA is in medium-high Optical Output Power stage. */
	ARGUS_STATE_DCA_POWER_MED_HIGH = DCA_GAIN_MEDIUM_HIGH << ARGUS_STATE_DCA_POWER_SHIFT,

	/*! DCA is in high Optical Output Power stage. */
	ARGUS_STATE_DCA_POWER_HIGH = DCA_GAIN_HIGH << ARGUS_STATE_DCA_POWER_SHIFT,


	/*! DCA is in low Pixel Input Gain stage. */
	ARGUS_STATE_DCA_GAIN_LOW = DCA_GAIN_LOW << ARGUS_STATE_DCA_GAIN_SHIFT,

	/*! DCA is in medium-low Pixel Input Gain stage. */
	ARGUS_STATE_DCA_GAIN_MED_LOW = DCA_GAIN_MEDIUM_LOW << ARGUS_STATE_DCA_GAIN_SHIFT,

	/*! DCA is in medium-high Pixel Input Gain stage. */
	ARGUS_STATE_DCA_GAIN_MED_HIGH = DCA_GAIN_MEDIUM_HIGH << ARGUS_STATE_DCA_GAIN_SHIFT,

	/*! DCA is in high Pixel Input Gain stage. */
	ARGUS_STATE_DCA_GAIN_HIGH = DCA_GAIN_HIGH << ARGUS_STATE_DCA_GAIN_SHIFT,

} argus_state_t;

/*!***************************************************************************
 * @brief	Dynamic Configuration Adaption (DCA) Parameters.
 * @details	DCA contains:
 * 			 - Static or dynamic mode. The first is utilizing the nominal values
 * 			   while the latter is dynamically adopting between min. and max.
 * 			   value and starting form the nominal values.
 * 			 - Analog Integration Depth Adaption down to single pulses.
 * 			 - Optical Output Power Adaption
 * 			 - Pixel Input Gain Adaption
 * 			 - Digital Integration Depth Adaption
 * 			 - Dynamic Global Phase Shift Injection.
 * 			 - All that features are heeding the Laser Safety limits.
 * 			 .
 *****************************************************************************/
typedef struct {
	/*! Enables the automatic configuration adaption features.
	 *  Enables the dynamic part if #DCA_ENABLE_DYNAMIC and the static only if
	 *  #DCA_ENABLE_STATIC. If set to DCA_ENABLE_OFF, the DCA is completely
	 *  skipped and the static register values are considered which is
	 *  recommended for advanced debugging only. */
	argus_dca_enable_t Enabled;

	/*! The threshold value of saturated pixels that causes a linear reduction
	 *  of the integration energy, i.e. if the number of saturated pixels are
	 *  larger or equal to this value, the integration energy will be reduced
	 *  by a single step (one pattern if the current integration depth is > 1,
	 *  one pulse if the current integration depth is <= 1 or one power LSB for
	 *  the optical power range).
	 *
	 *  Valid values: 1, ..., 33; (use 33 to disable the linear decrease)
	 *  Note that the linear value must be smaller or equal to the exponential
	 *  value. To sum up, it must hold:
	 *  1 <= SatPxThLin <= SatPxThExp <= SatPxThRst <= 33 */
	uint8_t SatPxThLin;

	/*! The threshold number of saturated pixels that causes a exponential
	 *  reduction of the integration energy, i.e. if the number of saturated
	 *  pixels is larger or equal to this value, the integration energy will be
	 *  halved.
	 *
	 *  Valid values: 1, ..., 33; (use 33 to disable the exponential decrease)
	 *  Note that the exponential value must be between the linear and reset
	 *  values. To sum up, it must hold:
	 *  1 <= SatPxThLin <= SatPxThExp <= SatPxThRst <= 33 */
	uint8_t SatPxThExp;

	/*! The threshold number of saturated pixels that causes a sudden reset of
	 *  the integration energy to the minimal value, i.e. if the number of
	 *  saturated pixels are larger or equal to this value, the integration
	 *  energy will suddenly be reset to the minimum values. The gain setting
	 *  will stay at the mid value and a decrease happens after the next step
	 *  if still required.
	 *
	 *  Valid values: 1, ..., 33; (use 33 to disable the sudden reset)
	 *  Note that the reset value must be larger or equal to the exponential
	 *  value. To sum up, it must hold:
	 *  1 <= SatPxThLin <= SatPxThExp <= SatPxThRst <= 33 */
	uint8_t SatPxThRst;

	/*! The amplitude to be targeted from the lower regime. If the amplitude
	 *  lower than the target value, a linear increase of integration energy
	 *  will happen in order to optimize for best performance.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_ATH_MIN, ... #ARGUS_CFG_DCA_ATH_MAX or 0
	 *  Set 0 to disable optimization toward the target amplitude.
	 *  Note further that the following condition must hold:
	 *  'MIN' <= AthLow <= Atarget <= AthHigh <= 'MAX' */
	uq12_4_t Atarget;

	/*! The low threshold value for the max. amplitude. If the max. amplitude
	 *  falls below this value, the integration depth will be increases.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_ATH_MIN, ... #ARGUS_CFG_DCA_ATH_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= AthLow <= Atarget <= AthHigh <= 'MAX' */
	uq12_4_t AthLow;

	/*! The high threshold value for the max. amplitude. If the max. amplitude
	 *  exceeds this value, the integration depth will be decreases. Note that
	 *  also saturated pixels will cause a decrease of the integration depth.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_ATH_MIN, ... #ARGUS_CFG_DCA_ATH_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= AthLow <= Atarget <= AthHigh <= 'MAX' */
	uq12_4_t AthHigh;

	/*! The nominal analog integration depth in UQ10.6 format,
	 *  i.e. the nominal pattern count per sample.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_DEPTH_MIN, ... #ARGUS_CFG_DCA_DEPTH_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= DepthLow <= DepthNom <= DepthHigh <= 'MAX' */
	uq10_6_t DepthNom;

	/*! The minimum analog integration depth in UQ10.6 format,
	 *  i.e. the minimum pattern count per sample.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_DEPTH_MIN, ... #ARGUS_CFG_DCA_DEPTH_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= DepthLow <= DepthNom <= DepthHigh <= 'MAX' */
	uq10_6_t DepthMin;

	/*! The maximum analog integration depth in UQ10.6 format,
	 *  i.e. the maximum pattern count per sample.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_DEPTH_MIN, ... #ARGUS_CFG_DCA_DEPTH_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= DepthMin <= DepthNom <= DepthMax <= 'MAX' */
	uq10_6_t DepthMax;

	/*! The nominal optical output power in mA,
	 *  i.e. the nominal VCSEL_HC1 setting.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_POWER_MIN, ... #ARGUS_CFG_DCA_POWER_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= PowerMin <= PowerNom <= 'MAX' */
	uq12_4_t PowerNom;

	/*! The minimum optical output power in mA,
	 *  i.e. the minimum VCSEL_HC1 setting.
	 *
	 *  Valid values: #ARGUS_CFG_DCA_POWER_MIN, ... #ARGUS_CFG_DCA_POWER_MAX
	 *  Note further that the following condition must hold:
	 *  'MIN' <= PowerMin <= PowerNom <= 'MAX' */
	uq12_4_t PowerMin;

	/*! The nominal pixel gain setting, i.e. the setting for
	 *  nominal/default gain stage.
	 *
	 *  Valid values: 0,..,3: #DCA_GAIN_LOW, ... #DCA_GAIN_HIGH
	 *  Note further that the following condition must hold:
	 *  'MIN' <= GainMin <= GainNom <= GainMax <= 'MAX' */
	argus_dca_gain_t GainNom;

	/*! The minimal pixel gain setting, i.e. the setting for
	 *  minimum gain stage.
	 *
	 *  Valid values: 0,..,3: #DCA_GAIN_LOW, ... #DCA_GAIN_HIGH
	 *  Note further that the following condition must hold:
	 *  'MIN' <= GainMin <= GainNom <= GainMax <= 'MAX' */
	argus_dca_gain_t GainMin;

	/*! The maximum pixel gain setting, i.e. the setting for
	 *  maximum gain stage.
	 *
	 *  Valid values: 0,..,3: #DCA_GAIN_LOW, ... #DCA_GAIN_HIGH
	 *  Note further that the following condition must hold:
	 *  'MIN' <= GainMin <= GainNom <= GainMax <= 'MAX' */
	argus_dca_gain_t GainMax;

	/*! Power Saving Ratio value.
	 *
	 *  Determines the percentage of the full available frame time that is not
	 *  exploited for digital integration. Thus the device is idle within the
	 *  specified portion of the frame time and does consume less energy.
	 *
	 *  Note that the laser safety might already limit the maximum integration
	 *  depth and the power saving ratio might not take effect for all ambient
	 *  situations. Thus the Power Saving Ratio is to be understood as a minimum
	 *  percentage where the device is idle per frame.
	 *
	 *  The value is a UQ0.8 format that ranges from 0.0 (=0x00) to 0.996 (=0xFF),
	 *  where 0 means no power saving (i.e. feature disabled) and 0xFF determines
	 *  maximum power saving, i.e. the digital integration depth is limited to a
	 *  single sample.
	 *
	 *  Range: 0x00, .., 0xFF; set 0 to disable. */
	uq0_8_t PowerSavingRatio;

} argus_cfg_dca_t;

/*! @} */
#endif /* ARGUS_DCA_H */
