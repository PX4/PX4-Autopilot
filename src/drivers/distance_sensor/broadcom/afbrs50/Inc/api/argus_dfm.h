/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		Defines the dual frequency mode (DFM) setup parameters.
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

#ifndef ARGUS_DFM_H
#define ARGUS_DFM_H

/*!***************************************************************************
 * @defgroup 	argusdfm Dual Frequency Mode
 * @ingroup		argusdev
 *
 * @brief		Dual Frequency Mode (DFM) parameter definitions and API functions.
 *
 * @details		The DFM is an algorithm to extend the unambiguous range of the
 * 				sensor by utilizing two detuned measurement frequencies.
 *
 *				The AFBR-S50 API provides three measurement modes:
 *				- 1X: Single Frequency Measurement
 *				- 4X: Dual Frequency Measurement w/ 4 times the unambiguous
 *				      range of the Single Frequency Measurement
 *				- 8X: Dual Frequency Measurement w/ 8 times the unambiguous
 *				      range of the Single Frequency Measurement
 *
 * @addtogroup 	argusdfm
 * @{
 *****************************************************************************/

/*! The Dual Frequency Mode frequency count. */
#define ARGUS_DFM_FRAME_COUNT (2U)

/*! The Dual Frequency Mode measurement modes count. Excluding the disabled mode. */
#define ARGUS_DFM_MODE_COUNT (2U) // expect off-mode!

/*! The Dual Frequency Mode measurement modes enumeration. */
typedef enum {
	/*! Single Frequency Measurement Mode (w/ 1x Unambiguous Range). */
	DFM_MODE_OFF = 0U,

	/*! 4X Dual Frequency Measurement Mode (w/ 4x Unambiguous Range). */
	DFM_MODE_4X = 1U,

	/*! 8X Dual Frequency Measurement Mode (w/ 8x Unambiguous Range). */
	DFM_MODE_8X = 2U,

} argus_dfm_mode_t;


/*! @} */
#endif /* ARGUS_DFM_H */
