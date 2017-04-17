/*!
 *	\file		commandsOdo.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		11/10/2010
 *
 *	\brief		Odometer parameters on IG-500E.<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */

#ifndef __COMMANDS_ODO_H__
#define __COMMANDS_ODO_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"


//----------------------------------------------------------------------//
//- Odometer definitions                                               -//
//----------------------------------------------------------------------//

#define SBG_ODO_AUTO_GPS_GAIN		(0x80)	/*!< If set in the GainOpts field, the GPS will automatically enhance odometer's gain */

//----------------------------------------------------------------------//
//- Odometer Types definition                                          -//
//----------------------------------------------------------------------//

/*!
 *  This enum defines sensitive axis of the corresponding odometer channel
 */
typedef enum _SbgOdoAxis
{
	SBG_ODO_X				= 0x00,			/*!< Odometer sensitive axis is X */
	SBG_ODO_Y				= 0x01,			/*!< Odometer sensitive axis is Y */
	SBG_ODO_Z				= 0x02,			/*!< Odometer sensitive axis is Z */
} SbgOdoAxis;

/*!
 *  This enum defines the direction of the corresponding odometer channel
 */
typedef enum _SbgOdoDirection
{
	SBG_ODO_DIR_POSITIVE	=	0x00,		/*!< Odometer velocity is always positive */
	SBG_ODO_DIR_NEGATIVE	=	0x01,		/*!< Odometer velocity is always negative */
} SbgOdoDirection;

//----------------------------------------------------------------------//
//- Odometer commands                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Set the main configuration of the external odometer channels
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	axis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_ODO_X
 *										- SBG_ODO_Y
 *										- SBG_ODO_Z
 *  \param[in]	pulsesPerMeter		decimal number of pulses per meter
 *  \param[in]	gainError			Error in percent on the previous gain value
 *  \param[in]	gpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain 
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoConfig(SbgProtocolHandle handle, uint8 channel, SbgOdoAxis axis, float pulsesPerMeter, uint8 gainError, bool gpsGainCorrection);

/*!
 *	Get the main configuration of the external odometer channels
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pAxis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_ODO_X
 *										- SBG_ODO_Y
 *										- SBG_ODO_Z
 *  \param[out]	pPulsesPerMeter		decimal number of pulses per meter
 *  \param[out]	pGainError			Error in percent on the previous gain value
 *	\param[out]	pGpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoConfig(SbgProtocolHandle handle, uint8 channel, SbgOdoAxis *pAxis, float *pPulsesPerMeter, uint8 *pGainError, bool *pGpsGainCorrection);

/*!
 *  Configures the odometer direction for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoDirection		Direction of the odometer. May be:
 *									- SBG_ODO_DIR_POSITIVE
 *									- SBG_ODO_DIR_NEGATIVE
 *									- SBG_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoDirection(SbgProtocolHandle handle, uint8 channel, SbgOdoDirection odoDirection);

/*!
 *  Get the odometer direction for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pOdoDirection		Direction of the odometer. May be:
 *									- SBG_ODO_DIR_POSITIVE
 *									- SBG_ODO_DIR_NEGATIVE
 *									- SBG_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoDirection(SbgProtocolHandle handle, uint8 channel, SbgOdoDirection *pOdoDirection);

/*!
 *  Configures the odometer lever arm for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoLeverArm			Lever arm of the odometer channel, with respect to the device. in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOdoLeverArm(SbgProtocolHandle handle, uint8 channel, const float odoLeverArm[3]);

/*!
 *  Get the odometer lever arm for the corresponding channel
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	odoLeverArm			Lever arm of the odometer channel, with respect to the device. in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOdoLeverArm(SbgProtocolHandle handle, uint8 channel, float odoLeverArm[3]);

#endif