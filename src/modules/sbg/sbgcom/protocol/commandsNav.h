/*!
 *	\file		commandsNav.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		25/07/08
 *
 *	\brief		Commands used to define navigation options on IG-500N and IG-500E<br>
 *				 Copyright 20010 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_NAV_H__
#define __COMMANDS_NAV_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

//----------------------------------------------------------------------//
//-  Definitions concerning GPS                                        -//
//----------------------------------------------------------------------//

#define SBG_SV_USED_FOR_NAV						(0x01)			/*!< SV is used for navigation */
#define SBG_SV_DIFF_AVAILABLE					(0x02)			/*!< Differential correction data is available for this SV */
#define SBG_SV_ORBIT_AVAILABLE					(0x04)			/*!< Orbit information is available for this SV (Ephemeris or Almanach) */
#define SBG_SV_ORBIT_EPHEMERIS					(0x08)			/*!< Orbit information is Ephemeris */
#define SBG_SV_UNHEALTHY						(0x10)			/*!< SV is unhealthy / shall not be used */

#define SBG_SV_QUAL_IDLE						(0x00)			/*!< This channel is idle */
#define SBG_SV_QUAL_SEARCHING_1					(0x01)			/*!< Channel is searching */
#define SBG_SV_QUAL_SERACHING_2					(0x02)			/*!< Channel is searching */
#define SBG_SV_QUAL_DETECTED_UNUSABLE			(0x03)			/*!< Signal detected but unusable */
#define SBG_SV_QUAL_CODE_LOCK_ON				(0x04)			/*!< Code Lock on Signal */
#define SBG_SV_QUAL_CODE_AND_CARRIER_LOCKED_1	(0x05)			/*!< Code and Carrier locked */
#define SBG_SV_QUAL_CODE_AND_CARRIER_LOCKED_2	(0x06)			/*!< Code and Carrier locked */
#define SBG_SV_QUAL_RECEIVING_DATA				(0x07)			/*!< Code and Carrier locked, receiving 50bps data */

//----------------------------------------------------------------------//
//- Definition concerning GPS and navigation                           -//
//----------------------------------------------------------------------//

/*!
 *	GPS Space vehicles information struct.<br>
 *	Angles are in 32/45th of degrees.
 */
typedef struct _SbgGpsSVInfo
{
	uint8	satelliteID;					/*!< ID of the satellite followed */
	uint8	flagsQuality;					/*!< flags and signal quality indicator (quality bits 7,6,5 flags 4,3,2,1,0) */
	uint8	signalStrength;					/*!< Carrier to noise Ratio */
	int8	azimuth;						/*!< Azimuth of the SV(signed) (1LSB = 32/45 degrees) */
	int8	elevation; 						/*!< Elevation of the SV(signed) (1LSB = 32/45 degrees) */
} SbgGpsSVInfo;

/*!
 *	Define the position aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgAidingPosSrc
{
	SBG_POS_SRC_GPS				= 0x00,		/*!< We are using the GPS position as the aiding source for the navigation filter */
	SBG_POS_SRC_GPS_AND_BARO	= 0x01,		/*!< We are using the GPS position and the barometric altitude as the aiding source for the navigation filter */
	SBG_POS_SRC_USER			= 0x02		/*!< We are using the user provided source for position as the aiding source for the navigation filter */
} SbgAidingPosSrc;

/*!
 *	Define the velocity aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgAidingVelSrc
{
	SBG_VEL_SRC_GPS				= 0x00,		/*!< We are using the GPS velocity as the aiding source for the navigation filter */
	SBG_VEL_SRC_USER			= 0x02,		/*!< We are using the a user provided source for velocity as the aiding source for the navigation filter */
	SBG_VEL_SRC_ODO				= 0x03,		/*! We are using the external odometer as velocity source for the navigation filter */
} SbgAidingVelSrc;

//----------------------------------------------------------------------//
//- Navigation relative commands                                       -//   
//----------------------------------------------------------------------//

/*!
 *	Get the advanced information about Space Vehicles seen by GPS. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pSvInfo				Pointer to an array of SbgGpsSVInfo structure that will contain SV information (std size 16 channels)
 *	\param[out]	pNbSV				Pointer to the number of satellites managed
 *	\param[in]	maxNbSv				Maximum number of nbSv that can be contained in the pSvInfo list.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgGetSvInfo(SbgProtocolHandle handle, SbgGpsSVInfo *pSvInfo, uint8 *pNbSV, uint8 maxNbSv);

/*!
 *	Defines which source to use to aid the velocity in the navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	aidingSrc			The aiding source to use in the navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavVelocitySrc(SbgProtocolHandle handle, SbgAidingVelSrc aidingSrc);

/*!
 *	Returns which source is used to aid the velocity in the navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in the navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavVelocitySrc(SbgProtocolHandle handle, SbgAidingVelSrc *pAidingSrc);

/*!
 *	Defines which source to use to aid the position in the navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	aidingSrc			The aiding source to use in the navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetNavPositionSrc(SbgProtocolHandle handle, SbgAidingPosSrc aidingSrc);

/*!
 *	Returns which source is used to aid the position in the navigation filter. (IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pAidingSrc			The aiding source used in the navigation filter.<br>
 *									Valid options are:<br>
 *									- SBG_SRC_GPS<br>
 *									- SBG_SRC_GPS_AND_BARO<br>
 *									- SBG_SRC_EXTERNAL<br>
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetNavPositionSrc(SbgProtocolHandle handle, SbgAidingPosSrc *pAidingSrc);

/*!
 *	Defines the GPS lever arm. (IG-500N only).<br>
 *	Use this command to specifiy the vector between the device and the GPS antenna.<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGpsLeverArm(SbgProtocolHandle handle, const float gpsLeverArm[3]);

/*!
 *	Returns the GPS lever arm. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	gpsLeverArm			X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGpsLeverArm(SbgProtocolHandle handle, float gpsLeverArm[3]);

/*!
 *	Defines the local gravity magnitude. (IG-500N only).
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	gravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetGravityMagnitude(SbgProtocolHandle handle, float gravityMagnitude);

/*!
 *	Returns the local gravity magnitude. (IG-500N only).<br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pGravityMagnitude	The local gravity magnitude in m.s^-2.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetGravityMagnitude(SbgProtocolHandle handle, float *pGravityMagnitude);

//----------------------------------------------------------------------//
//- Altimeter relative commands                                        -//   
//----------------------------------------------------------------------//

/*!
 *	Configures the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	reference			Reference pressure at ground in pascals.<br>
 *									If set to 0, the current pressure is considered as ground pressure.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetReferencePressure(SbgProtocolHandle handle, uint32 reference);

/*!
 *	Get the reference pressure used for altitude calculation. (IG-30G and IG-500N only)
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	pReference			Pointer to the reference pressure in pascals.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetReferencePressure(SbgProtocolHandle handle, uint32 *pReference);

//----------------------------------------------------------------------//
//- User position and velocity input commands                          -//   
//----------------------------------------------------------------------//

/*!
 *	Send a new velocity information to the Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	velocity			The new X,Y,Z velocity that should be used by the navigation filter in m/s.
 *	\param[in]	accuracy			The velocity accuracy in m/s.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavVelocity(SbgProtocolHandle handle, const float velocity[3], float accuracy);

/*!
 *	Send a new position information to the Navigation filter.
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	position			The new WGS84 position : latitude, longitude and altitude (above ellipsoid) in [deg, deg, meters].
 *	\param[in]	hAccuracy			The horizontal accuracy in meters.
 *	\param[in]	vAccuracy			The vertical accuracy in meters.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendNavPosition(SbgProtocolHandle handle, const double position[3], float hAccuracy, float vAccuracy);
#endif
