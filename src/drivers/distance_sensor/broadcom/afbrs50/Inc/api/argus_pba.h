/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the pixel binning algorithm (PBA) setup parameters and
 * 				data structure.
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

#ifndef ARGUS_PBA_H
#define ARGUS_PBA_H

/*!***************************************************************************
 * @defgroup 	arguspba Pixel Binning Algorithm
 * @ingroup		argusapi
 *
 * @brief		Pixel Binning Algorithm (PBA) parameter definitions and API functions.
 *
 * @details		Defines the generic pixel binning algorithm (PBA) setup parameters
 * 				and data structure.
 *
 * 				The PBA module contains filter algorithms that determine the
 * 				pixels with the best signal quality and extract an 1d distance
 * 				information from the filtered pixels.
 *
 * 				The pixel filter algorithm is a three-stage filter with a
 * 				fallback value:
 *
 * 				-# A fixed pre-filter mask is applied to statically disable
 * 				    specified pixels.
 * 				-# A relative and absolute amplitude filter is applied in the
 * 				 	second stage. The relative filter is determined by a ratio
 * 				 	of the maximum amplitude off all available (i.e. not filtered
 * 				 	in stage 1) pixels. Pixels that have an amplitude below the
 * 				 	relative threshold are dismissed. The same holds true for
 * 				 	the absolute amplitude threshold. All pixel with smaller
 * 				 	amplitude are dismissed.\n
 * 				 	The relative threshold is useful to setup a distance
 * 				 	measurement scenario. All well illuminated pixels are
 * 				 	selected and considered for the final 1d distance. The
 * 				 	absolute threshold is used to dismiss pixels that are below
 * 				 	the noise level. The latter would be considered for the 1d
 * 				 	result if the maximum amplitude is already very low.
 * 				-# A distance filter is used to distinguish pixels that target
 * 				    the actual object from pixels that see the brighter background,
 * 				    e.g. white walls. Thus, the pixel with the minimum distance
 * 				    is referenced and all pixel that have a distance between
 * 				    the minimum and the given minimum distance scope are selected
 * 				    for the 1d distance result. The minimum distance scope is
 * 				    determined by an relative (to the current minimum distance)
 * 				    and an absolute value. The larger scope value is the
 * 				    relevant one, i.e. the relative distance scope can be used
 * 				    to heed the increasing noise at larger distances.
 * 				-# If all of the above filters fail to determine a single valid
 * 				    pixel, the golden pixel is used as a fallback value. The
 * 				    golden pixel is the pixel that sits right at the focus point
 * 				    of the optics at large distances.
 * 				.
 *
 *				After filtering is done, there may be more than a single pixel
 *				left to determine the 1d signal. Therefore several averaging
 *				methods are implemented to obtain the best 1d result from many
 *				pixels. See #argus_pba_averaging_mode_t for details.
 *
 *
 * @addtogroup 	arguspba
 * @{
 *****************************************************************************/

#include "argus_def.h"

/*!***************************************************************************
 * @brief	Enable flags for the pixel binning algorithm.
 *
 * @details Determines the pixel binning algorithm feature enable status.
 * 			- [0]: #PBA_ENABLE: Enables the pixel binning feature.
 * 			- [1]: reserved
 * 			- [2]: reserved
 * 			- [3]: reserved
 * 			- [4]: reserved
 * 			- [5]: #PBA_ENABLE_GOLDPX: Enables the golden pixel feature.
 * 			- [6]: #PBA_ENABLE_MIN_DIST_SCOPE: Enables the minimum distance scope
 * 					 feature.
 * 			- [7]: reserved
 * 			.
 *****************************************************************************/
typedef enum {
	/*! Enables the pixel binning feature. */
	PBA_ENABLE = 1U << 0U,

	/*! Enables the golden pixel. */
	PBA_ENABLE_GOLDPX = 1U << 5U,

	/*! Enables the minimum distance scope filter. */
	PBA_ENABLE_MIN_DIST_SCOPE = 1U << 6U,

} argus_pba_flags_t;

/*!***************************************************************************
 * @brief	The averaging modes for the pixel binning algorithm.
 *****************************************************************************/
typedef enum {
	/*! Evaluate the 1D range from all available pixels using
	 *  a simple average. */
	PBA_SIMPLE_AVG = 1U,

	/*! Evaluate the 1D range from all available pixels using
	 *  a linear amplitude weighted averaging method.
	 *  Formula: x_mean = sum(x_i * A_i) / sum(A_i) */
	PBA_LINEAR_AMPLITUDE_WEIGHTED_AVG = 2U,

} argus_pba_averaging_mode_t;

/*!***************************************************************************
 * @brief	The pixel binning algorithm settings data structure.
 * @details	Describes the pixel binning algorithm settings.
 *****************************************************************************/
typedef struct {
	/*! Enables the pixel binning features.
	 *  Each bit may enable a different feature. See #argus_pba_flags_t
	 *  for details about the enabled flags. */
	argus_pba_flags_t Enabled;

	/*! Determines the PBA averaging mode which is used to obtain the
	 *  final range value from the algorithm, for example, the average
	 *  of all pixels. See #argus_pba_averaging_mode_t for more details
	 *  about the individual evaluation modes. */
	argus_pba_averaging_mode_t AveragingMode;

	/*! The Relative amplitude threshold value (in %) of the max. amplitude.
	 *  Pixels with amplitude below this threshold value are dismissed.
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *  Use 0 to disable the relative amplitude threshold. */
	uq0_8_t RelAmplThreshold;

	/*! The relative minimum distance scope value in %.
	 * 	Pixels that have a range value within [x0, x0 + dx] are considered
	 * 	for the pixel binning, where x0 is the minimum distance of all
	 * 	amplitude picked pixels and dx is the minimum distance scope value.
	 * 	The minimum distance scope value will be the maximum of relative
	 * 	and absolute value.
	 *
	 *  All available values from the 8-bit representation are valid.
	 *  The actual percentage value is determined by 100%/256*x.
	 *
	 *	Special values:
	 *  - 0: Use 0 for absolute value only or to choose the pixel with the
	 *  	 minimum distance only (of also the absolute value is 0)! */
	uq0_8_t RelMinDistanceScope;

	/*! The Absolute amplitude threshold value in LSB.
	 * 	Pixels with amplitude below this threshold value are dismissed.
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/16.
	 *
	 *  Use 0 to disable the absolute amplitude threshold. */
	uq12_4_t AbsAmplThreshold;

	/*! The absolute minimum distance scope value in m.
	 * 	Pixels that have a range value within [x0, x0 + dx] are considered
	 * 	for the pixel binning, where x0 is the minimum distance of all
	 * 	amplitude picked pixels and dx is the minimum distance scope value.
	 * 	The minimum distance scope value will be the maximum of relative
	 * 	and absolute value.
	 *
	 *  All available values from the 16-bit representation are valid.
	 *  The actual LSB value is determined by x/2^15.
	 *
	 *	Special values:
	 *  - 0: Use 0 for relative value only or to choose the pixel with the
	 *  	 minimum distance only (of also the relative value is 0)! */
	uq1_15_t AbsMinDistanceScope;

	/*! The pre-filter pixel mask determines the pixel channels that are
	 * 	statically excluded from the pixel binning (i.e. 1D distance) result.
	 *
	 * 	The pixel enabled mask is an 32-bit mask that determines the
	 * 	device internal channel number. It is recommended to use the
	 * 	 - #PIXELXY_ISENABLED(msk, x, y)
	 * 	 - #PIXELXY_ENABLE(msk, x, y)
	 * 	 - #PIXELXY_DISABLE(msk, x, y)
	 * 	 .
	 * 	macros to work with the pixel enable masks. */
	uint32_t PrefilterMask;

} argus_cfg_pba_t;

/*! @} */
#endif /* ARGUS_PBA_H */
