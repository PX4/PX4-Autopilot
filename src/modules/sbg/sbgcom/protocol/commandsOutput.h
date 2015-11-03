/*!
 *	\file		commandsOutput.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		02/04/07
 *
 *	\brief		Orientation specific commands
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef COMMANDS_OUTPUT_H_
#define COMMANDS_OUTPUT_H_

#include "../sbgCommon.h"
#include "protocol.h"

//----------------------------------------------------------------------//
//- Trigger mask definitions                                           -//
//----------------------------------------------------------------------//
#define SBG_TRIGGER_DISABLED					(0x00000000)			/*!< Helper used to disable an output using the trigger mechanism. */
#define SBG_TRIGGER_MAIN_LOOP_DIVIDER			(0x00000001)			/*!< Enable trigger on the main loop frequency divider */
#define SBG_TRIGGER_MAGNETOMETERS				(0x00000002)			/*!< Enable trigger on a new magnetometers data */
#define SBG_TRIGGER_BAROMETER					(0x00000004)			/*!< Enable trigger on a new barometer data */
#define SBG_TRIGGER_GPS_VELOCITY				(0x00000008)			/*!< Enable a trigger on a new GPS velocity */
#define SBG_TRIGGER_GPS_POSITION				(0x00000010)			/*!< Enable a trigger on a new GPS position */
#define SBG_TRIGGER_GPS_COURSE					(0x00000020)			/*!< Enable a trigger on a new GPS course (heading) */
#define SBG_TRIGGER_TIME_PULSE					(0x00000040)			/*!< Enable a trigger on an input time pulse */
#define SBG_TRIGGER_EXT_EVENT					(0x00000080)			/*!< Enable a trigger on a new external event */
#define SBG_TRIGGER_ODO_VELOCITY_0				(0x00000100)			/*!< Enable a trigger when a new odometer velocity (channel 0) event occurs*/
#define SBG_TRIGGER_ODO_VELOCITY_1				(0x00000200)			/*!< Enable a trigger when a new odometer velocity (channel 1) event occurs */
#define SBG_TRIGGER_EXT_TRUE_HEADING			(0x00000400)			/*!< Enable a trigger when a new true heading information is available */
#define SBG_TRIGGER_VIRTUAL_ODOMETER			(0x00000800)			/*!< Enable a trigger when the virtual odometer reached the desired distance */

//----------------------------------------------------------------------//
//- Output type definitions                                            -//
//----------------------------------------------------------------------//

/*!
 * Enum defining all possible types of communication modes
 */
typedef enum _SbgContOutputTypes
{
	SBG_CONT_TRIGGER_MODE_DISABLE				= 0x00,					/*!< Disable all types of continuous/triggered outputs; Only Question/Answers are used */
	SBG_CONTINUOUS_MODE_ENABLE					= 0x01,					/*!< Enable basic continuous mode (one continuous frame at a regular time */
	SBG_TRIGGERED_MODE_ENABLE					= 0x02					/*!< Enable advanced triggered mode */
} SbgContOutputTypes;

//----------------------------------------------------------------------//
//- NMEA and ASCII protocol output types definition                    -//
//----------------------------------------------------------------------//

/*!
 * This enum defines all possible NMEA and ascii frames provided by the IG-500
 */
typedef enum _SbgAsciiOutputIds
{
	SBG_OUTPUT_NMEA_GPGGA						= 0x00,					/*!< Standard NMEA GGA frame */
	SBG_OUTPUT_NMEA_GPRMC						= 0x01,					/*!< Standard NMEA RMC frame */
	SBG_OUTPUT_NMEA_GPZDA						= 0x02,					/*!< Standard NMEA ZDA frame */
	SBG_OUTPUT_ASCII_SBG01						= 0x03,					/*!< Proprietary SBG output frame (euler angles) */
} SbgAsciiOutputIds;

//----------------------------------------------------------------------//
//- Output configuration commands                                      -//
//----------------------------------------------------------------------//

/*!
 *	Define the default output mask used to enable outputs for the continous mode.
 *	\param[in]	handle							A valid sbgCom library handle.

 *	\param[in]	defaultOutputMask				Outputs to enable using a mask system.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetDefaultOutputMask(SbgProtocolHandle handle, uint32 defaultOutputMask);

/*!
 *	Returns the current default output mask used by the command SBG_GET_DEFAULT_OUTPUT.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pDefaultOutputMask				The current output mask used by the device.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetDefaultOutputMask(SbgProtocolHandle handle, uint32 *pDefaultOutputMask);


/*!
 *	Define continous mode options, enabled/disabled and output frames speed.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	contMode						Defines which continuous mode is set. Possible values are: <br>
 *												- SBG_CONT_TRIGGER_MODE_DISABLE <br>
 *												- SBG_CONTINUOUS_MODE_ENABLE <br>
 *												- SBG_TRIGGERED_MODE_ENABLE <br>
 *	\param[in]	divider							Main loop frequency divider applied on output.<br>
 *												For example if the device filter frequency has been set to 100Hz and we are using a divider of 4,<br>
 *												we will have data outputted at 100/4 = 25Hz.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetContinuousMode(SbgProtocolHandle handle, SbgContOutputTypes contMode, uint8 divider);

/*!
 *	Returns the continous mode option.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pContMode						Returns the continuous mode
 *	\param[out]	pDivider						The current devider used to define the speed of the continous mode.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetContinuousMode(SbgProtocolHandle handle, SbgContOutputTypes *pContMode, uint8 *pDivider);

/*!
 *	Define triggered output options
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	condId							The condition number may be 0 to 3
 *	\param[in]	triggerMask						The trigger bit mask that will generate the triggered output
 *  \param[in]	outputMask						Output mask defining the output buffer sent when a trigger is detected
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetTriggeredMode(SbgProtocolHandle handle, uint8 condId, uint32 triggerMask, uint32 outputMask);

/*!
 *	Return a triggered output condition parameters
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	condId							The condition number may be 0 to 3
 *	\param[out]	pTriggerMask					The trigger bit mask that will generate the triggered output
 *  \param[out]	pOutputMask						Output mask defining the output buffer sent when a trigger is detected
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetTriggeredMode(SbgProtocolHandle handle, uint8 condId, uint32 *pTriggerMask, uint32 *pOutputMask);

/*!
 *	Configure one ASCII / NMEA output message
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	frameId							ASCII Frame ID to configure
 *	\param[in]	divider							Main loop divider to apply on output (2 to divide main loop frequency by 2).
 *	\param[in]	triggerMask						Trigger mask that enables the corresponding frame output
 *	\return										SBG_NO_ERROR in case of a good operation
 */
SbgErrorCode sbgSetAsciiOutputConf(SbgProtocolHandle handle, SbgAsciiOutputIds frameId, uint8 divider, uint32 triggerMask);

/*!
 *	Get one ASCII / NMEA output sentence configuration
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	frameId							ASCII Frame ID to get its configuration
 *	\param[out]	pDivider						Main loop divider applied on output (2 to divide main loop frequency by 2).
 *	\param[out]	pTriggerMask					Associated trigger mask that enables the output
 *	\return										SBG_NO_ERROR in case of a good operation
 */
SbgErrorCode sbgGetAsciiOutputConf(SbgProtocolHandle handle, SbgAsciiOutputIds frameId, uint8 *pDivider, uint32 *pTriggerMask);

//----------------------------------------------------------------------//
//- Applications commands operations                                   -//
//----------------------------------------------------------------------//

/*!
 *	Ask the device to returns the default ouput configured using sbgSetDefaultOutputMask.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pOutput							The returned output filled by the device according to the default output mask.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetDefaultOutput(SbgProtocolHandle handle, SbgOutput *pOutput);

/*!
 *	Ask the device to returns a specific combinaison of ouputs.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	outputMask						Using a mask system, define which outputs should be returned.<br>
 *												See the file protocolOutput.h for a description of all available masks.
 *	\param[out]	pOutput							The returned output filled by the device according to the default output mask.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetSpecificOutput(SbgProtocolHandle handle, uint32 outputMask, SbgOutput *pOutput);

#endif
