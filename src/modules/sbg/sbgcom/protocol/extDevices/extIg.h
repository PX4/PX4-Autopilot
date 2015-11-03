/*!
 *	\file		extIg.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		29/11/10
 *
 *	\brief		Remote IG-Device specific commands implementation<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef __EXTIG_H__
#define __EXTIG_H__

#include "../../sbgCommon.h"
#include "../protocol.h"

//----------------------------------------------------------------------//
//- IG-Device definition                                                -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- IG-Device Types definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * IG-Device protocol external configuration commands
 */
typedef enum _SbgIgdCommand
{
	SBG_EXT_CMD_IG_ACK							= 0x00,		/*!< Acknowledge returned by the IG-Device management module */
	SBG_EXT_CMD_IG_SET_MATRIX_OFFSET			= 0x02,		/*!< Set a manual rotation matrix offset for device alignment */
	SBG_EXT_CMD_IG_GET_MATRIX_OFFSET			= 0x03,		/*!< Get a manual rotation matrix offset for device alignment */
	SBG_EXT_CMD_IG_RET_MATRIX_OFFSET			= 0x04,		/*!< Return a manual rotation matrix offset for device alignment */
} SbgIgdCommand;

//----------------------------------------------------------------------//
//- Remote IG-Device operations                                        -//
//----------------------------------------------------------------------//


/*!
 *	Set a manual orientation offset between the IG-500E and remote device, in a matrix form
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	matrixOffset		orientation offset between the IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgSetMatrixOffset(SbgProtocolHandle handle, const float matrixOffset[9]);

/*!
 *	Get the offset between the IG-500E orientation and the IG-Device orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	matrixOffset		Orientation offset between the remote IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtIgGetMatrixOffset(SbgProtocolHandle handle, float matrixOffset[9]);


#endif
