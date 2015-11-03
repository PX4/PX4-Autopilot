/*!
 *	\file		commandsOrientation.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Commands used to align the device frame.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_ORIENTATION_H__
#define __COMMANDS_ORIENTATION_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

//----------------------------------------------------------------------//
//  Orientatio command arguments definitions                            //
//----------------------------------------------------------------------//

/*!
 *	Type of orientation reset to do.
 */
typedef enum _SbgOffsetType
{
	SBG_OFFSET_PRE_ROT				= 1,	/*!< We would like to define or get a pre rotation matrix*/
	SBG_OFFSET_POST_ROT				= 2,	/*!< We would like to define or get a post rotation matrix*/

	SBG_OFFSET_PRE_ROT_Z_RESET		= 3,	/*!< Calculate a pre rotation on Z axis */
	SBG_OFFSET_PRE_ROT_XY_RESET		= 4,	/*!< Calculate a pre rotation on X and Y axis */
	SBG_OFFSET_PRE_ROT_XYZ_RESET	= 5,	/*!< Calculate a pre rotation on X,Y and Z axis */

	SBG_OFFSET_POST_ROT_Z_RESET		= 6,	/*!< Calculate a post rotation on Z axis */
	SBG_OFFSET_POST_ROT_XY_RESET	= 7,	/*!< Calculate a post rotation on X and Y axis */
	SBG_OFFSET_POST_ROT_XYZ_RESET	= 8		/*!< Calculate a post rotation on X,Y and Z axis */
} SbgOffsetType;

//----------------------------------------------------------------------//
//  Orientation commands                                                //
//----------------------------------------------------------------------//

/*!
 *	Defines the pre or post rotation to applied to the device.<br>
 *	This command is usefull to define an orientation 'offset'<br>
 *	You can either apply a pre rotation that applies directly on sensors values or<br>
 *	use a post rotation that only rotate attitude output such as euler angles.<br>
 *	<br>
 *	The post rotation isn't available for the IG-500N because, sensors, attitude,<br>
 *	velocity and position should be expressed in the same frame.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offsetType				Define if it's a pre or post rotation using:<br>
 *										- SBG_OFFSET_PRE_ROT<br>
 *										- SBG_OFFSET_POST_ROT<br>
 *	\param[in]	rotationMatrix			3x3 matrix that represents the rotation to apply.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetManualOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType, const float rotationMatrix[9]);

/*!
 *	Command used to automatically calcualte a pre or post rotation matrix.<br>
 *	Please refers to the device User Manual for more inforamtion.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offsetType				Define which type of orientation reset we have to do.<br>
 *										Available options are:<br>
 *										- SBG_OFFSET_PRE_ROT_Z_RESET<br>
 *										- SBG_OFFSET_PRE_ROT_XY_RESET<br>
 *										- SBG_OFFSET_PRE_ROT_XYZ_RESET<br>
 *										- SBG_OFFSET_POST_ROT_Z_RESET<br>
 *										- SBG_OFFSET_POST_ROT_XY_RESET<br>
 *										- SBG_OFFSET_POST_ROT_XYZ_RESET<br>
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetAutoOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType);

/*!
 *	Returns the pre or post rotation applied to the device.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offsetType				Define if we would like the pre or post rotation matrix using:
 *										- SBG_OFFSET_PRE_ROT<br>
 *										- SBG_OFFSET_POST_ROT<br>
 *	\param[out]	rotationMatrix			3x3 matrix that represents the applied rotation.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType, float rotationMatrix[9]);


#endif
