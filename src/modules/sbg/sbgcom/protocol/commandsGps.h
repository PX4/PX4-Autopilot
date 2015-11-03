/*!
 *	\file		commandsGps.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		29/01/10
 *
 *	\brief		Commands used to configure internal GPS and altimeter of the IG-500N<br>
 *				 Copyright 20010 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_GPS_H__
#define __COMMANDS_GPS_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

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
//- GPS and altimeter relative commands                                -//   
//----------------------------------------------------------------------//

/*!
 *	Configures the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *  \param[in]	reference			Reference pressure at ground in pascals.<br>
 *									If set to 0, the current pressure is considered as ground pressure.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetReferencePressure(SbgProtocolHandle handle, bool permanent, uint32 reference);

/*!
 *	Get the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pReference			Pointer to the reference pressure in pascals.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetReferencePressure(SbgProtocolHandle handle, uint32 *pReference);

/*!
 *	Configures the advanced GPS options. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
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
SbgErrorCode sbgSetGpsOptions(SbgProtocolHandle handle, bool permanent, SbgGpsDynamicModel model, uint8 options);

/*!
 *	Get the advanced GPS options. (IG-30G and IG-500N only)
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