/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 hardware API.
 * @details     This file provides generic definitions belonging to all
 *              devices from the AFBR-S50 product family.
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

#ifndef ARGUS_DEF_H
#define ARGUS_DEF_H
#ifdef __cplusplus
extern "C" {
#endif

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
 * @addtogroup  argus_api
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief   Maximum number of phases per measurement cycle.
 * @details The actual phase number is defined in the register configuration.
 *          However the software does only support a fixed value of 4 yet.
 *****************************************************************************/
#define ARGUS_PHASECOUNT 4

/*!***************************************************************************
 * @brief   The device pixel field size in x direction (long edge).
 *****************************************************************************/
#define ARGUS_PIXELS_X  8

/*!***************************************************************************
 * @brief   The device pixel field size in y direction (short edge).
 *****************************************************************************/
#define ARGUS_PIXELS_Y  4

/*!***************************************************************************
 * @brief   The total device pixel count.
 *****************************************************************************/
#define ARGUS_PIXELS    ((ARGUS_PIXELS_X)*(ARGUS_PIXELS_Y))

/*!***************************************************************************
 * @brief   A flag indicating that the device is a extended range device.
 *****************************************************************************/
#define MODULE_EXTENDED_FLAG (0x40U)

/*!***************************************************************************
 * @brief   The AFBR-S50 module types.
 *****************************************************************************/
typedef enum argus_module_version_t {
	/*! No device connected or not recognized. */
	MODULE_NONE = 0,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 1 - legacy version! */
	AFBR_S50MV85G_V1 = 0x01,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 2 - legacy version! */
	AFBR_S50MV85G_V2 = 0x02,

	/*! AFBR-S50LV85D: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  long range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50LV85D_V1 = 0x03,

	/*! AFBR-S50MV68B: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and red, 680 nm, laser source for
	 *  medium range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50MV68B_V1 = 0x04,

	/*! AFBR-S50MV85I: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50MV85I_V1 = 0x05,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  short range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50SV85K_V1 = 0x06,

	/*! AFBR-S50MV85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  medium range 3D applications.
	 *  Version 3 - current version! */
	AFBR_S50MV85G_V3 = 0x07,

	/*! AFBR-S50LX85D: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  extended long range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50LX85D_V1 = AFBR_S50LV85D_V1 | MODULE_EXTENDED_FLAG,

	/*! AFBR-S50MX68B: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and red, 680 nm, laser source for
	 *  extended medium range 1D applications.
	 *  Version 1 - current version! */
	AFBR_S50MX68B_V1 = AFBR_S50MV68B_V1 | MODULE_EXTENDED_FLAG,

	/*! AFBR-S50MX85I: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  extended medium range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50MX85I_V1 = AFBR_S50MV85I_V1 | MODULE_EXTENDED_FLAG,

	/*! AFBR-S50MX85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  extended short range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50SX85K_V1 = AFBR_S50SV85K_V1 | MODULE_EXTENDED_FLAG,

	/*! AFBR-S50MX85G: an ADS0032 based multi-pixel range finder device
	 *  w/ 4x8 pixel matrix and infra-red, 850 nm, laser source for
	 *  extended medium range 3D applications.
	 *  Version 1 - current version! */
	AFBR_S50MX85G_V1 = AFBR_S50MV85G_V3 | MODULE_EXTENDED_FLAG,

} argus_module_version_t;

/*!***************************************************************************
 * @brief   The AFBR-S50 laser configurations.
 *****************************************************************************/
typedef enum argus_laser_type_t {
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

	/*! 680nm Red VCSEL v1 w/ extended mode. */
	LASER_R_V1X = 5,

} argus_laser_type_t;

/*!***************************************************************************
 * @brief   The AFBR-S50 chip versions.
 *****************************************************************************/
typedef enum argus_chip_version_t {
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
 * @brief   The measurement mode flags.
 * @details The measurement mode flags that can be combined to a measurement
 *          mode, e.g. high speed short range mode. See #argus_mode_t for
 *          a complete list of available measurement modes.
 *
 *          - Bit 0: Short Range Mode
 *          - Bit 1: Long Range Mode
 *          - Bit 2: High Speed Mode
 *
 *          Note that the Long and Short Range Flags are mutual exclusive but
 *          any of those 2 must be set. Thus the value 0 is invalid!
 *          All other flags enhance the base configurations, e.g. the High
 *          Speed flag create the high speed mode of the selected base
 *          measurement mode.
 *****************************************************************************/
typedef enum argus_mode_flags_t {
	/*! Measurement Mode Flag for Short Range Base Mode. */
	ARGUS_MODE_FLAG_SHORT_RANGE = 0x01 << 0,

	/*! Measurement Mode Flag for Long Range Base Mode. */
	ARGUS_MODE_FLAG_LONG_RANGE = 0x01 << 1,

	/*! Measurement Mode Flag for High Speed Mode. */
	ARGUS_MODE_FLAG_HIGH_SPEED = 0x01 << 2

} argus_mode_flags_t;

/*!***************************************************************************
 * @brief   The measurement modes.
 * @details The measurement modes are composed in binary from of the flags
 *          define in #argus_mode_flags_t, i.e. each bit has a special meaning:
 *
 *          - Bit 0: Short Range Mode
 *          - Bit 1: Long Range Mode
 *          - Bit 2: High Speed Mode
 *
 *          Note that the Long and Short Range Bits are mutual exclusive but any
 *          of those 2 must be set. Thus the value 0 is invalid!
 *****************************************************************************/
typedef enum argus_mode_t {
	/*! Measurement Mode: Short Range Mode. */
	ARGUS_MODE_SHORT_RANGE =                                // = 0x01 = 0b0001
		ARGUS_MODE_FLAG_SHORT_RANGE,

	/*! Measurement Mode: Long Range Mode. */
	ARGUS_MODE_LONG_RANGE =                                 // = 0x02 = 0b0010
		ARGUS_MODE_FLAG_LONG_RANGE,

	/*! Measurement Mode: High Speed Short Range Mode. */
	ARGUS_MODE_HIGH_SPEED_SHORT_RANGE =                     // = 0x05 = 0b0101
		ARGUS_MODE_FLAG_SHORT_RANGE | ARGUS_MODE_FLAG_HIGH_SPEED,

	/*! Measurement Mode: High Speed Long Range Mode. */
	ARGUS_MODE_HIGH_SPEED_LONG_RANGE =                      // = 0x06 = 0b0110
		ARGUS_MODE_FLAG_LONG_RANGE | ARGUS_MODE_FLAG_HIGH_SPEED,

} argus_mode_t;

/*! The data structure for the API representing a AFBR-S50 device instance. */
typedef struct argus_hnd_t argus_hnd_t;

/*!***************************************************************************
 * @brief   Measurement Ready API callback function.
 *
 * @details Invoked by the API whenever a measurement cycle is finished and
 *          new data is ready to be evaluated via the #Argus_EvaluateData API
 *          function.
 *          The callback is passed to the API via the #Argus_TriggerMeasurement
 *          or #Argus_StartMeasurementTimer API functions.
 *          The API passes the status of the currently finished measurement
 *          cycle to the callback as first parameters. The second parameter is
 *          a pointer the API handle structure. The latter is used to identify
 *          the calling instance of the API in case of multiple devices.
 *          Further it can be passed to the #Argus_EvaluateData function.
 *
 * @warning Since the callback is called from an interrupt context, the
 *          callback execution must return as fast as possible. The usual task
 *          in the callback is to post an event to the main thread to inform it
 *          about the new data and that is must call the #Argus_EvaluateData
 *          function.
 *
 * @param   status The module status that caused the callback. #STATUS_OK if
 *                 everything was as expected.
 *
 * @param   hnd The API handle pointer to the calling instance. Identifies the
 *              instance of the API that was invoking the callback and thus the
 *              instance that must call the #Argus_EvaluateData for.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
typedef status_t (*argus_measurement_ready_callback_t)(status_t status, argus_hnd_t *hnd);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_DEF_H */
