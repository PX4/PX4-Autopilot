/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     Defines the dynamic configuration adaption (DCA) setup parameters
 *              and data structure.
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

#ifndef ARGUS_ND_H
#define ARGUS_ND_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_nd Noise Detectors
 * @ingroup     argus_api
 *
 * @brief       Noise Detectors parameter definitions and API functions.
 *
 * @details     The module contains algorithms for noise detection and is comprised
 *              of 2 sections: Saturation Detector and Optical Disturber Detector.
 *              The Saturation Detector is designed to mark as saturated the pixels
 *              that have a too high phase noise (UCNoise).
 *              The ODD is designed to exclude pixels with bad SNR values or with
 *              phase noise (UCNoise) from the final result.
 *
 * @addtogroup  argus_nd
 * @{
 *****************************************************************************/

#include "argus_def.h"


/*!***************************************************************************
 * @brief   Noise Detectors Parameters for Saturation Detector and for
 *          Optical Disturber Detector.
 * @details Config contains:
 *
 *            Saturation Detector:
 *             - Absolute Noise Threshold (when UCNoise is over this threshold
 *             and Amplitude is over the corresponding threshold, the
 *             pixel is marked as saturated)
 *             - Amplitude Threshold (when Amplitude is over this threshold
 *             and UCNoise is over the corresponding threshold, the
 *             pixel is marked as saturated)
 *             - UCNoise EMA Weight (for smoothing the UC Noise)
 *
 *            ODD:
 *             - Enable switch
 *             - Activation Threshold (over which Amplitude to apply the ODD)
 *             - Inclusion Threshold (over which the pixel is marked valid)
 *             - Exclusion Threshold (under which the pixel is marked invalid)
 *****************************************************************************/
typedef struct argus_cfg_noise_detectors_t {
	struct {
		/*! The uncorrelated noise threshold to saturate a pixel.
		 *
		 *  This is an AND condition together with the AmplitudeThreshold.
		 *
		 *  Units: LSB
		 *  Valid Values 1.0, ..., 4095.0 */
		uq12_4_t AbsoluteNoiseThreshold;

		/*! The amplitude threshold to saturate a pixel from the uncorrelated noise figure.
		 *
		 *  This is an AND condition together with the AbsoluteNoiseThreshold.
		 *
		 *  Units: LSB
		 *  Valid Values 1.0, ..., 4095.0 */
		uq12_4_t AmplitudeThreshold;

		/*! The weight for the exponentially moving average (EMA) that is used to
		 *  smooth the uncorrelated noise figure. */
		uq0_8_t UncorrelatedNoiseEMAWeight;

	} Saturation;

	struct {
		/*! Enables the ODD. */
		bool Enabled;

		/*! Amplitude threshold for enabling filter for pixel N
		 * (if amplitude N is over the limit).
		 *
		 *  Units: LSB
		 *  Valid Values 1.0, ..., 4095.0 */
		uq12_4_t Activation;

		/*! The SNR threshold to validate a pixel (SNR high enough).
		 *
		 *  Units: LSB
		 *  Valid Values 1.0, ..., 4095.0 */
		uq12_4_t Inclusion;

		/*! The SNR threshold to invalidate a pixel (too low SNR).
		 *
		 *  Units: LSB
		 *  Valid Values 1.0, ..., 4095.0 */
		uq12_4_t Exclusion;
	} ODD;

} argus_cfg_nd_t;


/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_ND_H */
