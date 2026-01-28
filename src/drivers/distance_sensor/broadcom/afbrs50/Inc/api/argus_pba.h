/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Defines the pixel binning algorithm (PBA) setup parameters and
 *              data structure.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
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

#ifndef ARGUS_PBA_H
#define ARGUS_PBA_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_pba Pixel Binning Algorithm
 * @ingroup     argus_api
 *
 * @brief       Pixel Binning Algorithm (PBA) parameter definitions and API functions.
 *
 * @details     Defines the generic Pixel Binning Algorithm (PBA) setup parameters
 *              and data structure.
 *
 *              The PBA module contains filter algorithms that determine the
 *              pixels with the best signal quality and extract an 1D distance
 *              information from the filtered pixels by averaging them in a
 *              specified way.
 *
 *              Basically, the Pixel Binning Algorithm is a multi-stage filter:
 *
 *              -# A fixed pre-filter mask is applied to statically disable
 *                  specified pixels.
 *
 *              -# A relative and absolute amplitude filter is applied in the
 *                  second stage. The relative filter is determined by a ratio
 *                  of the maximum amplitude off all available (i.e. not filtered
 *                  in stage 1) pixels. Pixels that have an amplitude below the
 *                  relative threshold are dismissed. The same holds true for
 *                  the absolute amplitude threshold. All pixel with smaller
 *                  amplitude are dismissed.\n
 *                  Note that the absolute amplitude threshold is disabled if
 *                  the Golden Pixel (see below) is also disabled in order to
 *                  prevent invalid filtering for multi-pixel devices.\n
 *                  The relative threshold is useful to setup a distance
 *                  measurement scenario. All well illuminated pixels are
 *                  selected and considered for the final 1D distance. The
 *                  absolute threshold is used to dismiss pixels that are below
 *                  the noise level. The latter would be considered for the 1D
 *                  result if the maximum amplitude is already very low.\n
 *                  Those threshold are implemented using a hysteresis behavior.
 *                  For its configuration, see the following parameters:
 *                  - #argus_cfg_pba_t::RelativeAmplitudeInclusion
 *                  - #argus_cfg_pba_t::RelativeAmplitudeExclusion
 *                  - #argus_cfg_pba_t::AbsoluteAmplitudeInclusion
 *                  - #argus_cfg_pba_t::AbsoluteAmplitudeExclusion
 *                  .
 *
 *              -# An absolute minimum distance filter is applied in addition
 *                  to the amplitude filter. This removes all pixel that have
 *                  a lower distance than the specified threshold. This is used
 *                  to remove invalid pixels that can be detected by a physically
 *                  not correct negative distance.\n
 *                  For its configuration, see the following parameters:
 *                  - #PBA_ENABLE_MIN_DIST_SCOPE
 *                  - #argus_cfg_pba_t::AbsoluteDistanceScopeInclusion
 *                  - #argus_cfg_pba_t::AbsoluteDistanceScopeExclusion
 *                  - #argus_cfg_pba_t::RelativeDistanceScopeInclusion
 *                  - #argus_cfg_pba_t::RelativeDistanceScopeExclusion
 *                  .
 *
 *              -# A distance filter is used to distinguish pixels that target
 *                  the actual object from pixels that see the brighter background,
 *                  e.g. white walls. Thus, the pixel with the minimum distance
 *                  is referenced and all pixel that have a distance between
 *                  the minimum and the given minimum distance scope are selected
 *                  for the 1D distance result. The minimum distance scope is
 *                  determined by an relative (to the current minimum distance)
 *                  and an absolute value. The larger scope value is the
 *                  relevant one, i.e. the relative distance scope can be used
 *                  to heed the increasing noise at larger distances.\n
 *                  For its configuration, see the following parameters:
 *                  - #argus_cfg_pba_t::AbsoluteMinimumDistanceThreshold
 *                  .
 *
 *              -# If all of the above filters fail to determine a single valid
 *                  pixel, the Golden Pixel is used as a fallback value. The
 *                  Golden Pixel is the pixel that sits right at the focus point
 *                  of the optics at large distances. Thus, it is expected to
 *                  have the best signal at large distances.\n
 *                  For its configuration, see the following parameters:
 *                  - #PBA_ENABLE_GOLDPX_FALLBACK_MODE
 *                  .
 *
 *              -# In order to avoid unwanted effects from "out-of-focus" pixels
 *                  in application that require a smaller focus, the Golden Pixel
 *                  Priority Mode prioritizes a valid signal on the central
 *                  Golden Pixel over other pixels. That is, while the Golden
 *                  Pixel has a reasonable signal strength, it is the only pixel
 *                  considered for the 1D result.\n
 *                  For its configuration, see the following parameters:
 *                  - #PBA_ENABLE_GOLDPX_FALLBACK_MODE
 *                  - #argus_cfg_pba_t::GoldenPixelPriorityAmplitudeInclusion
 *                  - #argus_cfg_pba_t::GoldenPixelPriorityAmplitudeExclusion
 *                  .
 *              .
 *
 *              After filtering is done, there may be more than a single pixel
 *              left to determine the 1D signal. Therefore several averaging
 *              methods are implemented to obtain the best 1D result from many
 *              pixels. See #argus_pba_averaging_mode_t for details.
 *
 *
 * @addtogroup  argus_pba
 * @{
 *****************************************************************************/

#include "argus_def.h"

/*!***************************************************************************
 * @brief   Enable flags for the pixel binning algorithm.
 *
 * @details Determines the pixel binning algorithm feature enable status.
 *
 *          - [0]: #PBA_ENABLE: Enables the pixel binning feature.
 *          - [1]: reserved
 *          - [2]: reserved
 *          - [3]: reserved
 *          - [4]: #PBA_ENABLE_GOLDPX_PRIORITY_MODE: Enables the Golden Pixel
 *                                                   priority mode feature.
 *          - [5]: #PBA_ENABLE_GOLDPX_FALLBACK_MODE: Enables the Golden Pixel
 *                                                   fallback mode feature.
 *          - [6]: #PBA_ENABLE_MIN_DIST_SCOPE: Enables the minimum distance
 *                                             scope feature.
 *          - [7]: reserved
 *          .
 *****************************************************************************/
typedef enum argus_pba_flags_t {
	/*! Enables the pixel binning feature. */
	PBA_ENABLE = 1U << 0U,

	/*! Enables the Golden Pixel Priority Mode.
	 *  If enabled, the Golden Pixel is prioritized over other Pixels as long
	 *  as it has a good signal (determined by # */
	PBA_ENABLE_GOLDPX_PRIORITY_MODE = 1U << 4U,

	/*! Enables the Golden Pixel Fallback Mode.
	 *  If enabled, the Golden Pixel is used as a last fallback pixel to obtain
	 *  a valid signal from. This is recommended for all non-multi pixel
	 *  devices whose TX field-of-view is aligned to target the Golden Pixel in
	 *  factory calibration. */
	PBA_ENABLE_GOLDPX_FALLBACK_MODE = 1U << 5U,

	/*! Enables the minimum distance scope filter. */
	PBA_ENABLE_MIN_DIST_SCOPE = 1U << 6U,

} argus_pba_flags_t;

/*!***************************************************************************
 * @brief   The averaging modes for the pixel binning algorithm.
 *****************************************************************************/
typedef enum argus_pba_averaging_mode_t {
	/*! Evaluate the 1D range from all available pixels using
	 *  a simple average. */
	PBA_SIMPLE_AVG = 1U,

	/*! Evaluate the 1D range from all available pixels using
	 *  a linear amplitude weighted averaging method.
	 *  Formula: x_mean = sum(x_i * A_i) / sum(A_i) */
	PBA_LINEAR_AMPLITUDE_WEIGHTED_AVG = 2U,

} argus_pba_averaging_mode_t;

/*!***************************************************************************
 * @brief   The pixel binning algorithm settings data structure.
 * @details Describes the pixel binning algorithm settings.
 *****************************************************************************/
typedef struct {
	/*! Enables the Pixel Binning Algorithm.
	 *
	 *  Each bit may enable a different feature. See #argus_pba_flags_t
	 *  for details about the enabled flags. */
	argus_pba_flags_t Enabled;

	/*! Determines the PBA averaging mode which is used to obtain the
	 *  final range value from the algorithm, for example, the average
	 *  of all pixels. See #argus_pba_averaging_mode_t for more details
	 *  about the individual evaluation modes. */
	argus_pba_averaging_mode_t AveragingMode;

	/*! The relative amplitude inclusion threshold (in %) of the max. amplitude.
	 *
	 *  Pixels, whose amplitudes raise above this inclusion threshold, are
	 *  added to the pixel binning. The amplitude must fall below the
	 *  exclusion (#RelativeAmplitudeExclusion) threshold to be removed from
	 *  the pixel binning again.
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *  Note: in addition to the relative criteria, there is also the absolute
	 *  criteria (#AbsoluteAmplitudeInclusion, #AbsoluteAmplitudeExclusion).
	 *  The pixels are added to the pixel binning if their respective amplitude
	 *  is larger than the absolute AND relative inclusion values. On the other
	 *  hand, they are removed if their amplitude falls below the absolute OR
	 *  relative exclusion threshold.
	 *
	 *  Must be greater than or equal to the #RelativeAmplitudeExclusion.
	 *
	 *  Use #RelativeAmplitudeExclusion == #RelativeAmplitudeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only.
	 *
	 *  Use 0 (for both, #RelativeAmplitudeExclusion and
	 *  #RelativeAmplitudeInclusion) to disable the relative amplitude
	 *  hysteresis. */
	uq0_8_t RelativeAmplitudeInclusion;

	/*! The relative amplitude exclusion threshold (in %) of the max. amplitude.
	 *
	 *  Pixels, whose amplitudes fall below this exclusion threshold, are
	 *  removed from the pixel binning. The amplitude must raise above the
	 *  inclusion (#RelativeAmplitudeInclusion) threshold to be added back
	 *  to be pixel binning again.
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *  Note: in addition to the relative criteria, there is also the absolute
	 *  criteria (#AbsoluteAmplitudeInclusion, #AbsoluteAmplitudeExclusion).
	 *  The pixels are added to the pixel binning if their respective amplitude
	 *  is larger than the absolute AND relative inclusion values. On the other
	 *  hand, they are removed if their amplitude falls below the absolute OR
	 *  relative exclusion threshold.
	 *
	 *  Must be less than or equal to #RelativeAmplitudeInclusion.
	 *
	 *  Use #RelativeAmplitudeExclusion == #RelativeAmplitudeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only.
	 *
	 *  Use 0 (for both, #RelativeAmplitudeExclusion and
	 *  #RelativeAmplitudeInclusion) to disable the relative amplitude
	 *  hysteresis. */
	uq0_8_t RelativeAmplitudeExclusion;

	/*! The absolute amplitude inclusion threshold in LSB.
	 *
	 *  Pixels, whose amplitudes raise above this inclusion threshold, are
	 *  added to the pixel binning. The amplitude must fall below the
	 *  exclusion (#RelativeAmplitudeExclusion) threshold to be removed from
	 *  the pixel binning again.
	 *
	 *  The absolute amplitude hysteresis is only valid if the Golden Pixel
	 *  mode is enabled. Otherwise, the thresholds are set to 0 LSB internally
	 *  which disables the absolute criteria.
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/16.
	 *
	 *  Note: in addition to the absolute criteria, there is also the relative
	 *  criteria (#RelativeAmplitudeInclusion, #RelativeAmplitudeExclusion).
	 *  The pixels are added to the pixel binning if their respective amplitude
	 *  is larger than the absolute AND relative inclusion values. On the other
	 *  hand, they are removed if their amplitude falls below the absolute OR
	 *  relative exclusion threshold.
	 *
	 *  Must be greater than or equal to #AbsoluteAmplitudeExclusion.
	 *
	 *  Use #AbsoluteAmplitudeExclusion == #AbsoluteAmplitudeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only.
	 *
	 *  Use 0 (for both, #AbsoluteAmplitudeExclusion and
	 *  #AbsoluteAmplitudeInclusion) to disable the absolute amplitude
	 *  hysteresis. */
	uq12_4_t AbsoluteAmplitudeInclusion;

	/*! The absolute amplitude exclusion threshold in LSB.
	 *
	 *  Pixels, whose amplitudes fall below this exclusion threshold, are
	 *  removed from the pixel binning. The amplitude must raise above the
	 *  inclusion (#RelativeAmplitudeInclusion) threshold to be added back
	 *  to be pixel binning again.
	 *
	 *  The absolute amplitude hysteresis is only valid if the Golden Pixel
	 *  mode is enabled. Otherwise, the thresholds are set to 0 LSB internally
	 *  which disables the absolute criteria.
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/16.
	 *
	 *  Note: in addition to the absolute criteria, there is also the relative
	 *  criteria (#RelativeAmplitudeInclusion, #RelativeAmplitudeExclusion).
	 *  The pixels are added to the pixel binning if their respective amplitude
	 *  is larger than the absolute AND relative inclusion values. On the other
	 *  hand, they are removed if their amplitude falls below the absolute OR
	 *  relative exclusion threshold.
	 *
	 *  Must be less than or equal to #AbsoluteAmplitudeInclusion.
	 *
	 *  Use #AbsoluteAmplitudeExclusion == #AbsoluteAmplitudeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only.
	 *
	 *  Use 0 (for both, #AbsoluteAmplitudeExclusion and
	 *  #AbsoluteAmplitudeInclusion) to disable the absolute amplitude
	 *  hysteresis. */
	uq12_4_t AbsoluteAmplitudeExclusion;

	/*! The Golden Pixel Priority Mode inclusion threshold in LSB.
	 *
	 *  The Golden Pixel Priority Mode prioritizes a valid signal on the
	 *  Golden Pixel over other pixel to avoid unwanted effects from
	 *  "out-of-focus" pixels in application that require a smaller focus.
	 *
	 *  If the Golden Pixel priority mode is enabled (see
	 *  #PBA_ENABLE_GOLDPX_PRIORITY_MODE) and the Golden Pixel has a valid signal
	 *  with amplitude higher than this inclusion threshold, its priority state
	 *  is enabled and the binning exits early by dismissing all other pixels
	 *  regardless of their respective amplitude or state. The Golden Pixel
	 *  priority state is disabled if the Golden Pixel amplitude falls below
	 *  the exclusion threshold (#GoldenPixelPriorityAmplitudeExclusion) or its
	 *  state becomes invalid (e.g. #PIXEL_SAT).
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/16.
	 *
	 *  Use 0 to disable the Golden Pixel priority mode hysteresis. */
	uq12_4_t GoldenPixelPriorityAmplitudeInclusion;

	/*! The Golden Pixel Priority Mode exclusion threshold in LSB.
	 *
	 *  The Golden Pixel Priority Mode prioritizes a valid signal on the
	 *  Golden Pixel over other pixel to avoid unwanted effects from
	 *  "out-of-focus" pixels in application that require a smaller focus.
	 *
	 *  If the Golden Pixel priority mode is enabled (see
	 *  #PBA_ENABLE_GOLDPX_PRIORITY_MODE) and the Golden Pixel has a valid
	 *  signal with amplitude higher than the exclusion threshold
	 *  (#GoldenPixelPriorityAmplitudeInclusion), its priority state is enabled
	 *  and the binning exits early by dismissing all other pixels regardless
	 *  of their respective amplitude or state. The Golden Pixel priority state
	 *  is disabled if the Golden Pixel amplitude falls below this exclusion
	 *  threshold or its state becomes invalid (e.g. #PIXEL_SAT).
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/16.
	 *
	 *  Use 0 to disable the Golden Pixel priority mode hysteresis. */
	uq12_4_t GoldenPixelPriorityAmplitudeExclusion;

	/*! The relative minimum distance scope inclusion threshold (in %).
	 *
	 *  Pixels, whose range is smaller than the minimum distance inclusion
	 *  threshold (x_min + dx_incl) are added to the pixel binning. The
	 *  range must raise above the exclusion
	 *  (#RelativeDistanceScopeExclusion) threshold to be removed
	 *  from the pixel binning again. The relative value is determined
	 *  by multiplying the percentage with the minimum distance.
	 *
	 *  The distance scope determines an interval within that pixels
	 *  are considered valid, originating at the minimum distance (x_min).
	 *  The width of the interval is specified by the relative and absolute
	 *  minimum distance scope thresholds. The actual values it the
	 *  maximum of both, the relative and absolute inclusion values
	 *  (#AbsoluteDistanceScopeInclusion).
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *  Must be smaller than or equal to the #RelativeDistanceScopeExclusion.
	 *
	 *  Use #RelativeDistanceScopeExclusion == #RelativeDistanceScopeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only. */
	uq0_8_t RelativeDistanceScopeInclusion;

	/*! The relative distance scope exclusion threshold (in %).
	 *
	 *  Pixels, whose range is larger than the minimum distance exclusion
	 *  threshold (x_min + dx_excl) are removed from the pixel binning. The
	 *  range must fall below the inclusion
	 *  (#RelativeDistanceScopeInclusion) threshold to be added
	 *  to the pixel binning again. The relative value is determined
	 *  by multiplying the percentage with the minimum distance.
	 *
	 *  The distance scope determines an interval within that pixels
	 *  are considered valid, originating at the minimum distance (x_min).
	 *  The width of the interval is specified by the relative and absolute
	 *  minimum distance scope thresholds. The actual values it the
	 *  maximum of both, the relative and absolute exclusion values
	 *  (#AbsoluteDistanceScopeExclusion).
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *  Must be larger than or equal to the #RelativeDistanceScopeInclusion.
	 *
	 *  Use #RelativeDistanceScopeExclusion == #RelativeDistanceScopeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only. */
	uq0_8_t RelativeDistanceScopeExclusion;

	/*! The absolute minimum distance scope inclusion threshold (in m).
	 *
	 *  Pixels, whose range is smaller than the minimum distance inclusion
	 *  threshold (x_min + dx_incl) are added to the pixel binning. The
	 *  range must raise above the exclusion
	 *  (#AbsoluteDistanceScopeExclusion) threshold to be added
	 *  to the pixel binning again.
	 *
	 *  The distance scope determines an interval within that pixels
	 *  are considered valid, originating at the minimum distance (x_min).
	 *  The width of the interval is specified by the relative and absolute
	 *  minimum distance scope thresholds. The actual values it the
	 *  maximum of both, the relative and absolute exclusion values
	 *  (#RelativeDistanceScopeInclusion).
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/2^15.
	 *
	 *  Must be smaller than or equal to the #AbsoluteDistanceScopeExclusion.
	 *
	 *  Use #AbsoluteDistanceScopeExclusion == #AbsoluteDistanceScopeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only. */
	uq1_15_t AbsoluteDistanceScopeInclusion;

	/*! The absolute minimum distance scope exclusion threshold (in m).
	 *
	 *  Pixels, whose range is larger than the minimum distance exclusion
	 *  threshold (x_min + dx_excl) are removed from the pixel binning. The
	 *  range must fall below the inclusion
	 *  (#AbsoluteDistanceScopeInclusion) threshold to be added
	 *  to the pixel binning again.
	 *
	 *  The distance scope determines an interval within that pixels
	 *  are considered valid, originating at the minimum distance (x_min).
	 *  The width of the interval is specified by the relative and absolute
	 *  minimum distance scope thresholds. The actual values it the
	 *  maximum of both, the relative and absolute exclusion values
	 *  (#RelativeDistanceScopeExclusion).
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/2^15.
	 *
	 *  Must be larger than or equal to the #AbsoluteDistanceScopeInclusion.
	 *
	 *  Use #AbsoluteDistanceScopeExclusion == #AbsoluteDistanceScopeInclusion to
	 *  disable the hysteresis behavior and use it as a threshold only. */
	uq1_15_t AbsoluteDistanceScopeExclusion;

	/*! The Golden Pixel Saturation Filter Pixel Threshold.
	 *
	 *  The Golden Pixel Saturation Filter will evaluate the status of the
	 *  Golden Pixel to #PIXEL_INVALID if a certain number of active pixels,
	 *  i.e. pixels that are not removed by the static pre-filter mask
	 *  (#PrefilterMask), are saturated (#PIXEL_SAT).
	 *
	 *  The purpose of this filter is to avoid erroneous situations with highly
	 *  reflective targets (e.g. retro-reflectors) that can invalidate the
	 *  Golden Pixel such that it would not show the correct saturation state.
	 *  In order to avoid using the Golden Pixel in that scenario, this filter
	 *  mechanism can be used to remove the Golden Pixel if a specified number
	 *  of other pixels show saturation state.
	 *
	 *  Use 0 to disable the Golden Pixel Saturation Filter. */
	uint8_t GoldenPixelSaturationFilterPixelThreshold;

	/*! The Golden Pixel out-of-sync age limit for the GPPM.
	 *
	 *  The Golden Pixel out-of-sync age is the number of consecutive frames
	 *  where the Golden Pixel is out-of-sync. This parameters is the threshold
	 *  to distinguish between temporary and permanent out-of-sync states.
	 *
	 *  Temporary out-of-sync states happen when the target rapidly changes. In
	 *  this case, the Golden Pixel Priority Mode (GPPM) is not exited. Only if
	 *  the out-of-sync age exceeds the specified threshold, the Golden Pixel is
	 *  considered erroneous and the GPPM is exited.
	 *
	 *  Use 0 to disable the Golden Pixel out-of-sync aging (= infinity). */
	uint8_t GoldenPixelOutOfSyncAgeThreshold;

	/*! The absolute minimum distance threshold value in m.
	 *
	 *  Pixels with distance below this threshold value are dismissed. */
	q9_22_t AbsoluteMinimumDistanceThreshold;

	/*! The pre-filter pixel mask determines the pixel channels that are
	 *  statically excluded from the pixel binning (i.e. 1D distance) result.
	 *
	 *  The pixel enabled mask is an 32-bit mask that determines the
	 *  device internal channel number. It is recommended to use the
	 *   - #PIXELXY_ISENABLED(msk, x, y)
	 *   - #PIXELXY_ENABLE(msk, x, y)
	 *   - #PIXELXY_DISABLE(msk, x, y)
	 *   .
	 *  macros to work with the pixel enable masks. */
	uint32_t PrefilterMask;

} argus_cfg_pba_t;

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_PBA_H */
