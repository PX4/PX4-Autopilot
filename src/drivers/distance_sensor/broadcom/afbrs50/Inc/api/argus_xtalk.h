/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 hardware API.
 * @details     Defines the generic device calibration API.
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

#ifndef ARGUS_XTALK_H
#define ARGUS_XTALK_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @addtogroup  argus_cal
 * @{
 *****************************************************************************/

#include "argus_def.h"
#include "argus_dfm.h"

/*!***************************************************************************
 * @brief   Pixel Crosstalk Compensation Vector.
 * @details Contains calibration data (per pixel) that belongs to the
 *          RX-TX-Crosstalk compensation feature.
 *          The crosstalk vector consists of a Sine and Cosine component in LSB.
 *****************************************************************************/
typedef struct xtalk_t {
	/*! Crosstalk Vector - Sine component.
	 *  Units: LSB
	 *  Special Value: Q11_4_MIN == not available */
	q11_4_t dS;

	/*! Crosstalk Vector - Cosine component.
	 *  Units: LSB
	 *  Special Value: Q11_4_MIN == not available */
	q11_4_t dC;

} xtalk_t;

/*!***************************************************************************
 * @brief   Pixel Crosstalk Vector Table.
 * @details Contains crosstalk vector values for all 32 active pixels,
 *          separated for A/B-Frames.
 *****************************************************************************/
typedef union argus_cal_xtalk_table_t {
	struct {
		/*! The crosstalk vector table for A-Frames. */
		xtalk_t FrameA[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];

		/*! The crosstalk vector table for B-Frames. */
		xtalk_t FrameB[ARGUS_PIXELS_X][ARGUS_PIXELS_Y];
	};

	/*! The crosstalk vector table for A/B-Frames of all 32 pixels.*/
	xtalk_t Table[ARGUS_DFM_FRAME_COUNT][ARGUS_PIXELS_X][ARGUS_PIXELS_Y];

} argus_cal_xtalk_table_t;


/*!***************************************************************************
 * @brief   Electrical Pixel-To-Pixel Crosstalk Compensation Parameters.
 * @details Contains calibration data that belongs to the electrical
 *          pixel-to-pixel crosstalk compensation feature.
 *****************************************************************************/
typedef struct argus_cal_electrical_p2pxtalk_t {
	/*! Electrical Pixel-To-Pixel Compensation on/off. */
	bool Enabled;

	/*! The relative threshold determines when the compensation is active for
	 *  each individual pixel. The value determines the ratio of the individual
	 *  pixel signal with respect to the overall average signal. If the
	 *  ratio is smaller than the value, the compensation is active. Absolute
	 *  and relative conditions are connected with AND logic. */
	uq0_8_t RelativeThreshold;

	/*! The absolute threshold determines the minimum total crosstalk
	 *  amplitude (i.e. the average amplitude of all pixels weighted by
	 *  the Kc factor) that is required for the compensation to become
	 *  active. Set to 0 to always enable. Absolute and relative
	 *  conditions are connected with AND logic. */
	uq12_4_t AbsoluteTreshold;

	/*! The sine component of the Kc factor that determines the amount of the total
	 *  signal of all pixels that influences the individual signal of each pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t KcFactorS;

	/*! The cosine component of the Kc factor that determines the amount of the total
	 *  signal of all pixels that influences the individual signal of each pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t KcFactorC;

	/*! The sine component of the reference pixel Kc factor that determines the
	 *  amount of the total signal on all pixels that influences the individual
	 *  signal of the reference pixel.
	 *  Higher values determine more influence on the reference pixel signal. */
	q3_12_t KcFactorSRefPx;

	/*! The cosine component of the reference pixel Kc factor that determines the
	 *  amount of the total signal on all pixels that influences the individual
	 *  signal of the reference pixel.
	 *  Higher values determine more influence on the reference pixel signal. */
	q3_12_t KcFactorCRefPx;

} argus_cal_electrical_p2pxtalk_t;

/*!***************************************************************************
 * @brief   Optical Pixel-To-Pixel Crosstalk Compensation Parameters.
 * @details Contains calibration data that belongs to the optical
 *          pixel-to-pixel crosstalk compensation feature.
 *****************************************************************************/
typedef struct argus_cal_optical_p2pxtalk_t {
	/*! Optical Pixel-To-Pixel Compensation on/off. */
	bool Enabled;

	/*! The sine component of the coupling coefficient that determines the amount
	 *  of a neighbour pixel signal that influences the raw signal of certain pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t CouplingCoeffS;

	/*! The cosine component of the coupling coefficient that determines the amount
	 *  of a neighbour pixel signal that influences the raw signal of a certain pixel.
	 *  Higher values determine more influence on the individual pixel signal. */
	q3_12_t CouplingCoeffC;

} argus_cal_optical_p2pxtalk_t;

/*!***************************************************************************
 * @brief   Pixel-To-Pixel Crosstalk Compensation Parameters.
 * @details Contains combined calibration data for electrical and
 *          optical pixel-to-pixel crosstalk compensation feature.
 *****************************************************************************/
typedef struct argus_cal_p2pxtalk_t {
	argus_cal_electrical_p2pxtalk_t Electrical;

	argus_cal_optical_p2pxtalk_t Optical;
} argus_cal_p2pxtalk_t;

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_XTALK_H */
