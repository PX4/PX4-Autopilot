/*!
 *	\file		commandsCalib.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Calibration specific commands implementation.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __SBG_COMMANDS_CALIB_H__
#define __SBG_COMMANDS_CALIB_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

//----------------------------------------------------------------------//
//  Calibration commands arguments                                      //
//----------------------------------------------------------------------//

/*!
 *	For the magnetometers calibration command, defines which action to execute.
 */
typedef enum _SbgCalibMagsAction
{
	SBG_CALIB_MAGS_LOAD_DEFAULT			= 0x00,		/*!< Load into volatile memory the default magnetometers calibration data. */
	SBG_CALIB_MAGS_SAVE					= 0x05		/*!< Save the current magnetometers calibration data into FLASH memory. */
} SbgCalibMagsAction;

/*!
 *	For the gyroscopes calibration command, defined which action to execute
 */
typedef enum _SbgCalibGyrosAction
{
	SBG_CALIB_GYROS_LOAD_DEFAULT		= 0x00,		/*!< Load into volatile memory the default gyroscopes bias data. */
	SBG_CALIB_GYROS_MEASURE_COARSE		= 0x04,		/*!< Calibrate the zero bias for gyroscopes during 250 ms */
	SBG_CALIB_GYROS_SAVE				= 0x05,		/*!< Save the current gyroscopes bias data into FLASH memory. */
	SBG_CALIB_GYROS_MEASURE_MEDIUM		= 0x06,		/*!< Calibrate the zero bias for gyroscopes during 1000 ms */
	SBG_CALIB_GYROS_MEASURE_FINE		= 0x07		/*!< Calibrate the zero bias for gyroscopes during 3000 ms */
} SbgCalibGyrosAction;

//----------------------------------------------------------------------//
//- Calibration commands                                               -//
//----------------------------------------------------------------------//

/*!
 *	Command used to start/stop/save a magnetometer calibration procedure.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	argument				Define the action we would like to execute.
 *	\return								SBG_NO_ERROR if the action has been executed sucessfully.
 */
SbgErrorCode sbgCalibMagnetometers(SbgProtocolHandle handle, SbgCalibMagsAction argument);

/*!
 *	Define the current magnetometer calibration parameters.<br>
 *	This command is used, for example, by the sbgCenter when the magnetometer calibration is exectured on the computer.<br> 
 *	When this command is called, the new settings are only stored in volatile memory.<br>
 *	To save this calibration permanently, use sbgCalibMagnetometers with SBG_CALIB_SAVE argument.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offset					X,Y,Z offset of the magnetic field.
 *	\param[in]	crossAxis				3x3 matrix containing the rotation and deformation of the magnetic field.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibMagnetometersSetTransformations(SbgProtocolHandle handle, const float offset[3], const float crossAxis[9]);

/*!
 *	Returns the current magnetometer calibration parameters.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	offset					X,Y,Z offset of the magnetic field.
 *	\param[out]	crossAxis				3x3 matrix containing the rotation and deformation of the magnetic field.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibMagnetometersGetTransformations(SbgProtocolHandle handle, float offset[3], float crossAxis[9]);

/*!
 *	Acquiere and save the current gyroscope bias value.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	argument				Define the action we would like to execute.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCalibGyroBias(SbgProtocolHandle handle, SbgCalibGyrosAction argument);

#endif
