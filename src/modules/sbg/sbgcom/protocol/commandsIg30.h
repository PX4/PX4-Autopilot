/*!
 *	\file		commandsIg30.h
 *	\author		SBG Systems (Alexis Guinamard)
 *	\date		04/03/2012
 *
 *	\brief		This file contains IG-20 / IG-30 specific commands
 *
 *	\section CodeCopyright Copyright Notice
 *	Copyright (C) 2007-2012, SBG Systems SAS. All rights reserved.
 *
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 */
#ifndef __COMMANDS_IG30_H__
#define __COMMANDS_IG30_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"


//----------------------------------------------------------------------//
//- Masks definitons about orientation computation                     -//
//----------------------------------------------------------------------//

#define SBG_FILTER_OPTION_ENABLE_ATTITUDE		(0x10)			/*!< Enable the attitude/navigation computation.<br>If disabled, the device can output calibrated data at 200 Hz. */

//----------------------------------------------------------------------//
//-  Definitions concerning GPS                                        -//
//----------------------------------------------------------------------//

#define SBG_GPS_DISABLE_SBAS					(0x00)			/*!< GPS option, disable SBAS and ranging */
#define SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS	(0x01)			/*!< GPS option, enable SBAS corrections */
#define SBG_GPS_ENABLE_SBAS_RANGING				(0x02)			/*!< GPS option, enable ranging for SBAS */
#define SBG_GPS_ALTITUDE_ABOVE_MSL				(0x04)			/*!< GPS option, When set the GPS outputs an altitude above Mean Sea Level. Otherwise, altitude is above Ellipsoid */

//----------------------------------------------------------------------//
//- Definition concerning GPS and navigation                           -//
//----------------------------------------------------------------------//

/*!
 *	GPS dynamic platform model enumeration.
 */
typedef enum _SbgGpsDynamicModel
{
	SBG_GPS_MODEL_STATIONARY	= 1,		/*!< Stationary model, low bandwith */
	SBG_GPS_MODEL_PEDESTRIAN	= 2,		/*!< Pedestrian model, low dynamic, low bandwith */
	SBG_GPS_MODEL_AUTOMOTIVE	= 3,		/*!< Automotive model, mid dynamic, mid bandwith */
	SBG_GPS_MODEL_SEA			= 4,		/*!< Sea model, mid dynamic, mid bandwith */
	SBG_GPS_MODEL_AIRBONE_1G	= 5,		/*!< Airbone model with less than 1g accelerations, mid bandwith */
	SBG_GPS_MODEL_AIRBONE_2G	= 6,		/*!< Airbone model with less than 2g accelerations, high bandwith */
	SBG_GPS_MODEL_AIRBONE_4G	= 7			/*!< Airbone model with less than 4g accelerations, very high bandwith and recommanded for most applications */
} SbgGpsDynamicModel;

//----------------------------------------------------------------------//
//- Sensor sampling and orientation computation relative commands      -//   
//----------------------------------------------------------------------//

/*!
 *	Defines some options regarding the Kalman Filter.<br>
 *	It's possible, for example, to enable/disable the attitude computation or to enable/disable gyro-bias estimation.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	filterOptions			The Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 filterOptions);

/*!
 *	Retreives some options regarding the Kalman Filter.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pFilterOptions			The Kalman filter options mask.<br>
 *										Possible values are:<br>
 *										- #SBG_FILTER_OPTION_ENABLE_ATTITUDE
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterAttitudeOptions(SbgProtocolHandle handle, uint32 *pFilterOptions);

/*!
 *	Defines the sensors filter cut-off frequencies and the update rate for the Kalman Filter.<br>
 *	If you set a setting to 0.0, the value will remain unchanged.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	gyroAccelsSampling		Reserved for backward compatibility. Leave to 0.
 *	\param[in]	cutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffAccel				Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	cutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[in]	kalmanFreq				The Kalman filter refresh rate.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterFrequencies(SbgProtocolHandle handle, float gyroAccelsSampling, float cutoffGyro, float cutoffAccel, float cutoffMagneto, float kalmanFreq);

/*!
 *	Retrives the sensors filter cut-off frequencies and the update rate for the Kalman Filter.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pGyroAccelsSampling		The accelerometers and gyroscopes sampling frequency in Hz.
 *	\param[out]	pCutoffGyro				Gyroscopes low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffAccel			Accelerometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pCutoffMagneto			Magnetometers low-pass filter cut-off frequency in Hz.
 *	\param[out]	pKalmanFreq				The Kalman filter refresh rate.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterFrequencies(SbgProtocolHandle handle, float *pGyroAccelsSampling, float *pCutoffGyro, float *pCutoffAccel, float *pCutoffMagneto, float *pKalmanFreq);

//----------------------------------------------------------------------//
//- GPS and altimeter relative commands                                -//   
//----------------------------------------------------------------------//

/*!
 *	Configures the advanced GPS options. (IG-30G only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	model				Dynamic platform model:
 * 									- SBG_GPS_MODEL_STATIONARY<br>
 * 									- SBG_GPS_MODEL_PEDESTRIAN<br>
 * 									- SBG_GPS_MODEL_AUTOMOTIVE<br>
 * 									- SBG_GPS_MODEL_SEA<br>
 * 									- SBG_GPS_MODEL_AIRBONE_1G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_2G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_4G<br>
 *  \param[in]	options				GPS options using masks, such as SBAS corrections.<br>
 *									Available options masks are:
 *									- SBG_GPS_DISABLE_SBAS<br>
 *									- SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS<br>
 *									- SBG_GPS_ENABLE_SBAS_RANGING<br>
 *									- SBG_GPS_ALTITUDE_ABOVE_MSL<br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgSetGpsOptions(SbgProtocolHandle handle, SbgGpsDynamicModel model, uint8 options);

/*!
 *	Get the advanced GPS options. (IG-30G only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pModel				Dynamic platform model:
 * 									- SBG_GPS_MODEL_STATIONARY<br>
 * 									- SBG_GPS_MODEL_PEDESTRIAN<br>
 * 									- SBG_GPS_MODEL_AUTOMOTIVE<br>
 * 									- SBG_GPS_MODEL_SEA<br>
 * 									- SBG_GPS_MODEL_AIRBONE_1G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_2G<br>
 * 									- SBG_GPS_MODEL_AIRBONE_4G<br>
 *  \param[out]	pOptions			GPS options using masks, such as SBAS corrections.<br>
 *									Available options masks are:
 *									- SBG_GPS_DISABLE_SBAS<br>
 *									- SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS<br>
 *									- SBG_GPS_ENABLE_SBAS_RANGING<br>
 *									- SBG_GPS_ALTITUDE_ABOVE_MSL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGpsOptions(SbgProtocolHandle handle, SbgGpsDynamicModel *pModel, uint8 *pOptions);

#endif