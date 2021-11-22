/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 hardware API.
 * @details		This file provides generic definitions belonging to all
 * 				devices from the AFBR-S50 product family.
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

#ifndef ARGUS_DEF_H
#define ARGUS_DEF_H

/*!***************************************************************************
 * Include files
 *****************************************************************************/
#include "argus_status.h"
#include "argus_version.h"
#include "utility/fp_def.h"
#include "utility/time.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*!***************************************************************************
 * @addtogroup 	argusapi
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief	Maximum number of phases per measurement cycle.
 * @details	The actual phase number is defined in the register configuration.
 * 			However the software does only support a fixed value of 4 yet.
 *****************************************************************************/
#define ARGUS_PHASECOUNT 4U

/*!***************************************************************************
 * @brief	The device pixel field size in x direction (long edge).
 *****************************************************************************/
#define ARGUS_PIXELS_X	8U

/*!***************************************************************************
 * @brief	The device pixel field size in y direction (short edge).
 *****************************************************************************/
#define ARGUS_PIXELS_Y	4U

/*!***************************************************************************
 * @brief	The total device pixel count.
 *****************************************************************************/
#define ARGUS_PIXELS	((ARGUS_PIXELS_X)*(ARGUS_PIXELS_Y))

/*!***************************************************************************
 * @brief	The AFBR-S50 module types.
 *****************************************************************************/
typedef enum {
	/*! No device connected or not recognized. */
	MODULE_NONE = 0,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 1 - legacy version! */
	AFBR_S50MV85G_V1 = 1,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 2 - legacy version! */
	AFBR_S50MV85G_V2 = 2,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 7 - current version! */
	AFBR_S50MV85G_V3 = 7,

	/*! AFBR-S50LV85D: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  long range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50LV85D_V1 = 3,

	/*! AFBR-S50MV68B: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and red, 680 nm, laser source for
	 *  medium range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50MV68B_V1 = 4,

	/*! AFBR-S50MV85I: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50MV85I_V1 = 5,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  short range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50SV85K_V1 = 6,


	/*! Reserved for future extensions. */
	Reserved = 0x3F

} argus_module_version_t;

/*!***************************************************************************
 * @brief	The AFBR-S50 laser configurations.
 *****************************************************************************/
typedef enum {
	/*! No laser connected. */
	LASER_NONE = 0,

	/*! 850nm Infra-Red VCSEL v1 */
	LASER_H_V1 = 1,

	/*! 850nm Infra-Red VCSEL v2 */
	LASER_H_V2 = 2,

	/*! 680nm Red VCSEL v1 */
	LASER_R_V1 = 3,

	/*! 850nm Infra-Red VCSEL v2 /w extended mode. */
	LASER_H_V2X = 4,

} argus_laser_type_t;

/*!***************************************************************************
 * @brief	The AFBR-S50 chip versions.
 *****************************************************************************/
typedef enum {
	/*! No device connected or not recognized. */
	ADS0032_NONE = 0,

	/*! ADS0032 v1a */
	ADS0032_V1A = 1,

	/*! ADS0032 v1b */
	ADS0032_V1B = 2,

	/*! ADS0032 v1c */
	ADS0032_V1C = 3,

	/*! ADS0032 v1d */
	ADS0032_V1D = 4,

	/*! ADS0032 v1e */
	ADS0032_V1E = 5,

} argus_chip_version_t;

/*!***************************************************************************
 * @brief	The number of measurement modes with distinct configuration and
 * 			calibration records.
 *****************************************************************************/
#define ARGUS_MODE_COUNT (2)

/*!***************************************************************************
 * @brief	The measurement modes.
 *****************************************************************************/
typedef enum {
	/*! Measurement Mode A: Long Range Mode. */
	ARGUS_MODE_A = 1,

	/*! Measurement Mode B: Short Range Mode. */
	ARGUS_MODE_B = 2,

} argus_mode_t;

/*!***************************************************************************
 * @brief	Generic API callback function.
 * @details	Invoked by the API. The content of the abstract data pointer
 * 			depends upon the context.
 * @param	status The module status that caused the callback. #STATUS_OK if
 * 				   everything was as expected.
 * @param	data An abstract pointer to an user defined data. This will usually
 * 				 be passed to the function that also takes the callback as an
 * 				 parameter. Otherwise it has a special meaning such as
 * 				 configuration or calibration data.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*argus_callback_t)(status_t status, void *data);

/*! @} */
#endif /* ARGUS_DEF_H */
