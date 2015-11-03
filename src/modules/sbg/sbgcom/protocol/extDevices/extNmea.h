/*!
 *	\file		extNmea.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		29/11/10
 *
 *	\brief		NMEA external modules specific commands implementation<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef __EXTNMEA_H__
#define __EXTNMEA_H__

#include "../../sbgCommon.h"
#include "../protocol.h"

//----------------------------------------------------------------------//
//- NMEA definition                                                    -//
//----------------------------------------------------------------------//

#define SBG_NMEA_OPT_HDT_AFTER_RMC		(0x0002)	/*!< If set the HDT frame will be time stamped with the PREVIOUS RMC time data. */
#define SBG_NMEA_OPT_HDT_BEFORE_RMC		(0x0000)	/*!< If set the HDT frame will be time stamped with the NEXT RMC time data. */

//----------------------------------------------------------------------//
//- NMEA Types definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * NMEA protocol external configuration commands
 */
typedef enum _SbgNmeaCommand
{
	SBG_EXT_CMD_NMEA_ACK						= 0x00,		/*!< Acknowledge returned by the device */
	
	SBG_EXT_CMD_NMEA_SET_OPTIONS				= 0x01,		/*!< Configure the GPS options: Altitude reference and distance between antennas if available*/
	SBG_EXT_CMD_NMEA_GET_OPTIONS				= 0x02,		/*!< Get the GPS options: Altitude reference and distance between antennas if available*/
	SBG_EXT_CMD_NMEA_RET_OPTIONS				= 0x03,		/*!< Return of the GPS options: Altitude reference and distance between antennas if available*/

	SBG_EXT_CMD_NMEA_SET_YAW_OFFSET				= 0x04,		/*!< Set an offset on the yaw angle returned by the GPS True Heading */
	
	SBG_EXT_CMD_NMEA_SET_MATRIX_OFFSET			= 0x07,		/*!< Set a full orientation offset for true heading data (used for heading offset and accuracy) */
	SBG_EXT_CMD_NMEA_GET_MATRIX_OFFSET			= 0x08,		/*!< Get a full orientation offset for true heading data (used for heading offset and accuracy) */
	SBG_EXT_CMD_NMEA_RET_MATRIX_OFFSET			= 0x09,		/*!< Ret a full orientation offset for true heading data (used for heading offset and accuracy) */
} SbgExtNmeaCommand;

//----------------------------------------------------------------------//
//- NMEA operations                                                    -//
//----------------------------------------------------------------------//

/*!
 *	Configures the remote NMEA GPS options.
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	options				NMEA GPS options. Possible choices are:
 *									- SBG_NMEA_OPT_HDT_AFTER_RMC
 *									- SBG_NMEA_OPT_HDT_BEFORE_RMC
 *	\param[in]	stdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetOptions(SbgProtocolHandle handle, uint16 options, uint32 stdHeadingAcuracy);

/*!
 *	Get the remote NMEA GPS options.
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pOptions			NMEA GPS options. Possible choices are:
 *									- SBG_NMEA_OPT_HDT_AFTER_RMC
 *									- SBG_NMEA_OPT_HDT_BEFORE_RMC
 *	\param[out]	pStdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaGetOptions(SbgProtocolHandle handle, uint16 *pOptions, uint32 *pStdHeadingAcuracy);

/*!
 *	Configures the offset between the IG-500E orientation and the GPS true Heading data. <br>
 *  IG-500E heading is then GPS True Heading - offset; This offset actually is stored as a rotation matrix. <br>
 *	See sbgExtNmeaSetMatrixOffset or sbgExtNmeagetMatrixOffset for more information
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	offset				offset between the GPS true heading and IG-500E heading
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetYawOffset(SbgProtocolHandle handle, float offset);

/*!
 *	Configures the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	matrixOffset		rientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetMatrixOffset(SbgProtocolHandle handle, const float matrixOffset[9]);

/*!
 *	Get the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	matrixOffset		Orientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaGetMatrixOffset(SbgProtocolHandle handle, float matrixOffset[9]);

#endif