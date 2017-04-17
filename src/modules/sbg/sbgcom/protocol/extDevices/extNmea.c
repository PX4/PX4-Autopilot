#include "extNmea.h"
#include "../protocolOutputMode.h"
#include "../commandsExt.h"

//----------------------------------------------------------------------//
//- NMEA operations                                                    -//
//----------------------------------------------------------------------//

/*!
 *	Configures the remote NMEA GPS options.
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	options				NMEA GPS options. Possible choices are:
 *									- SBG_NMEA_OPT_HDT_AFTER_RMC
 *									- SBG_NMEA_OPT_HDT_BEFORE_RMC
 *	\param[in]	stdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °.
									Leave to 100000 if not used.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetOptions(SbgProtocolHandle handle, uint16 options, uint32 stdHeadingAcuracy)
{
	uint8 command[2+sizeof(uint16)+sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_NMEA_SET_OPTIONS;
	command[1] = 0;
	*((uint16*)(command+2*sizeof(uint8))) = sbgHostToTarget16(handle->targetOutputMode,options);
	*((uint32*)(command+2*sizeof(uint8)+sizeof(uint16))) = sbgHostToTarget32(handle->targetOutputMode,stdHeadingAcuracy);

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,2*sizeof(uint8)+sizeof(uint16)+sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_NMEA_ACK))
		{
			error = (SbgErrorCode)answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Get the remote NMEA GPS options.
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pOptions			NMEA GPS options. Possible choices are:
 *									- SBG_NMEA_OPT_HDT_AFTER_RMC
 *									- SBG_NMEA_OPT_HDT_BEFORE_RMC
 *	\param[out]	pStdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaGetOptions(SbgProtocolHandle handle, uint16 *pOptions, uint32 *pStdHeadingAcuracy)
{
	uint8 command[sizeof(uint8)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;


	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_NMEA_GET_OPTIONS;

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually a matrix output
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_EXT_CMD_NMEA_RET_OPTIONS) && (answerSize == (sizeof(uint8)+sizeof(uint16)+sizeof(uint32))))
		{
			if (pOptions)
			{
				*pOptions = sbgTargetToHost16(handle->targetOutputMode, *(uint16*)(answer+sizeof(uint8)));
			}
			if (pStdHeadingAcuracy)
			{
				*pStdHeadingAcuracy = sbgTargetToHost32(handle->targetOutputMode, *((uint32*)(answer+sizeof(uint8)+sizeof(uint16))));
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_NMEA_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = (SbgErrorCode)answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Configures the offset between the IG-500E orientation and the GPS true Heading data. <br>
 *  IG-500E heading is then GPS True Heading - offset; This offset actually is stored as a rotation matrix. <br>
 *	See sbgExtNmeaSetMatrixOffset or sbgExtNmeagetMatrixOffset for more information
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	offset				offset between the GPS true heading and IG-500E heading
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetYawOffset(SbgProtocolHandle handle, float offset)
{
	uint8 command[2+sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_NMEA_SET_YAW_OFFSET;
	command[1] = 0;
	*((uint32*)(command+2*sizeof(uint8))) = sbgHostToTargetFloat(handle->targetOutputMode,offset);

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,2*sizeof(uint8)+sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_NMEA_ACK))
		{
			error = (SbgErrorCode)answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Configures the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	matrixOffset		rientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaSetMatrixOffset(SbgProtocolHandle handle, const float matrixOffset[9])
{
	uint8 command[2*sizeof(uint8)+9*sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_NMEA_SET_MATRIX_OFFSET;
	command[1] = 0;
	for (i=0;i<9;i++)
	{
		*((uint32*)(command+2*sizeof(uint8)+i*sizeof(uint32))) = sbgHostToTargetFloat(handle->targetOutputMode,matrixOffset[i]);
	}

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,2*sizeof(uint8)+9*sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_NMEA_ACK))
		{
			error = (SbgErrorCode)answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Get the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[out]	matrixOffset		Orientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtNmeaGetMatrixOffset(SbgProtocolHandle handle, float matrixOffset[9])
{
	uint8 command[sizeof(uint8)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	uint32* matrix;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_EXT_CMD_NMEA_GET_MATRIX_OFFSET;

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgExtDeviceConfig(handle,command,sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually a matrix output
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_EXT_CMD_NMEA_RET_MATRIX_OFFSET) && (answerSize == (sizeof(uint8)+9*sizeof(uint32))))
		{
			if (matrixOffset)
			{
				matrix = (uint32*)(answer + sizeof(uint8));
				for (i=0;i<9;i++)
				{
					matrixOffset[i] = sbgTargetToHostFloat(handle->targetOutputMode, matrix[i]);
				}
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_EXT_CMD_NMEA_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}
