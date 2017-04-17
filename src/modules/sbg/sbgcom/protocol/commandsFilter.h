/*!
 *	\file		commandsFilter.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Filter related commands used to define Kalman Filter options and sensors filtering settings.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef _COMMANDS_FILTER_H_
#define _COMMANDS_FILTER_H_

#include "../sbgCommon.h"
#include "protocol.h"

//--------------------------------------------------------------------------------//
//- Definitons about kalman filter                                               -//
//--------------------------------------------------------------------------------//

#define SBG_MP_MAX_SIZE								(2048)		/*!< Maximum size allowed for a motion profile buffer */
#define SBG_MP_DEFAULT_SIZE							(0)			/*!< Default size of a motion profile with current DK version */

//--------------------------------------------------------------------------------//
//- Types definitons about kalman filter                                         -//
//--------------------------------------------------------------------------------//

/*!
 *	List of available sources for heading corrections.
 */
typedef enum _SbgHeadingSource
{
	SBG_HEADING_SOURCE_NONE							=	0x00,	/*!< The heading is calculated using gyroscopes only. */
	SBG_HEADING_SOURCE_MAGNETOMETERS				=	0x01,	/*!< The heading is based on magnetometers information and gyroscopes. */
	SBG_HEADING_SOURCE_GPS_COURSE					=	0x02,	/*!< GPS course is used */
	SBG_HEADING_GPS_ACCELERATIONS					=	0x03,	/*!< Heading is derived from GPS acceleration measurements */
	SBG_HEADING_SOURCE_USER							=	0x05,	/*!< Heading is fed by user via the main protocol */
	SBG_HEADING_SOURCE_REMOTE_MAG					=	0x06,	/*!< Remote magnetometers used as a heading source */
	SBG_HEADING_SOURCE_REMOTE_TRUE_HEADING			=	0x07	/*!< Remote true heading sensor used (dual antenna)  */
} SbgHeadingSource;

//----------------------------------------------------------------------//
//- Kalman Filter commands                                             -//
//----------------------------------------------------------------------//

/*!
 * Defines the kalman filter motion profile to be used
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pMpBuffer				Motion profile buffer pointer
 * \param[in]	mpSize					Motion profile buffer size
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgSetMotionProfile(SbgProtocolHandle handle, void *pMpBuffer, uint16 mpSize);

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Sends a motion profile buffer part
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pMpBuffer				Motion profile buffer pointer
 * \param[in]	index					Index in the IG-500 motion profile buffer
 * \param[in]	size					Number of bytes to transmit
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgSendMPBuffer(SbgProtocolHandle handle, void *pMpBuffer, uint16 index, uint16 size);

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Once the buffer is fully sent, Validate and use a motion profile buffer
 * \param[in]	handle					Valid sbgCom library handle
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgValidateMPBuffer(SbgProtocolHandle handle);

/*!
 * Retrives the kalman filter motion profile information
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pId						Motion profile Identifier is returned here
 * \param[in]	pVersion				Motion profile version is returned here
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgGetMotionProfileInfo(SbgProtocolHandle handle, uint32 *pId, uint32 *pVersion);

/*!
 *	Defines the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	source					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource source);

/*!
 *	Retrives the kalman filter source for heading estimate.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pSource					Source used in heading calculation.<br>
 *										Possible values are:<br>
 *										- SBG_HEADING_SOURCE_NONE, unavailable for IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_MAGNETOMETERS<br>
 *										- SBG_HEADING_SOURCE_GPS, only for IG-30G and IG-500N devices<br>
 *										- SBG_HEADING_SOURCE_EXTERNAL<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource *pSource);

/*!
 *	Defines the magnetic declination in radians.<br>
 *	The declination is important for good results in navigation estimation with IG-500N devices.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	declination				The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetMagneticDeclination(SbgProtocolHandle handle, float declination);

/*!
 *	Returns the magnetic declination in radians.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pDeclination			The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetMagneticDeclination(SbgProtocolHandle handle, float *pDeclination);

/*!
 *	Send a new heading inforamtion to the Kalman filter.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	heading					The new heading in radians.
 *	\param[in]	accuracy				The heading accuracy in radians.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendFilterHeading(SbgProtocolHandle handle, float heading, float accuracy);

//----------------------------------------------------------------------//
//- Additional configuration commands                                  -//
//----------------------------------------------------------------------//

/*!
 *	Defines the Heave configuration
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	enableHeave				Set to true if heave has to be computed
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetHeaveConf(SbgProtocolHandle handle, bool enableHeave);

/*!
 *	Returns the heave configuration
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	pEnableHeave			Set to true if heave has to be computed
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetHeaveConf(SbgProtocolHandle handle, bool *pEnableHeave);

#endif
