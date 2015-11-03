#include "commandsOrientation.h"
#include "protocolOutputMode.h"

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
 *										- OFFSET_PRE_ROT<br>
 *										- OFFSET_POST_ROT<br>
 *	\param[in]	rotationMatrix			3x3 matrix that represents the rotation to apply.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetManualOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType, const float rotationMatrix[9])
{
	uint8 buffer[2*sizeof(uint8) + 9*sizeof(uint32)];
	SbgErrorCode error;
	uint32 *outputMatrix;
	uint16 i;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(buffer+sizeof(uint8)) = (uint8)offsetType;
	
	//
	// Get a pointer to the manual matrix
	//
	outputMatrix = (uint32*)(buffer+2*sizeof(uint8));

	//
	// Fill the manual matrix
	//
	for (i=0; i<9; i++)
	{
		outputMatrix[i] = sbgHostToTargetFloat(handle->targetOutputMode, rotationMatrix[i]);
	}

	//
	// Send the command
	//
	error = sbgProtocolSend(handle, SBG_SET_MANUAL_ORIENTATION_OFFSET, buffer, 2*sizeof(uint8) + 9*sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK (wait for 1s because pre rotation could take a while)
		//
		error = sbgWaitForAck(handle, 1000);
	}

	return error;
}

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
SbgErrorCode sbgSetAutoOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType)
{
	uint8 buffer[2*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(buffer+sizeof(uint8)) = (uint8)offsetType;

	//
	// Send the command
	//
	error = sbgProtocolSend(handle, SBG_SET_AUTO_ORIENTATION_OFFSET, buffer, 2*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK (wait for 1s because pre rotation could take a while)
		//
		error = sbgWaitForAck(handle, 1000);
	}

	return error;
}

/*!
 *	Returns the pre or post rotation applied to the device.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	offsetType				Define if we would like the pre or post rotation matrix using:
 *										- SBG_OFFSET_PRE_ROT<br>
 *										- SBG_OFFSET_POST_ROT<br>
 *	\param[out]	rotationMatrix			3x3 matrix that represents the applied rotation.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOrientationOffset(SbgProtocolHandle handle, SbgOffsetType offsetType, float rotationMatrix[9])
{
	uint8 cmd;
	uint16 size;
	uint32 matrix[9];
	SbgErrorCode error;
	uint8 offsetTypeValue;
	uint32 i;

	//
	// Get the value defining the type of orientation reset.
	//
	offsetTypeValue = (uint8)offsetType;

	//
	// Send the command
	//
	error = sbgProtocolSend(handle, SBG_GET_ORIENTATION_OFFSET, &offsetTypeValue, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)matrix, &size, 9*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_ORIENTATION_OFFSET) && (size == 9*sizeof(uint32)) )
			{
				//
				// Returns the rotation matrix
				//
				if (rotationMatrix)
				{
					for (i=0;i<9;i++)
					{
						rotationMatrix[i] = sbgTargetToHostFloat(handle->targetOutputMode, matrix[i]);
					}
				}
			}
			else if ( (cmd == SBG_ACK) && (size == sizeof(uint8)) )
			{
				//
				// Get the error code contained in the ACK frame
				//
				error = (SbgErrorCode)(matrix[0]);

				//
				// We should receive an NACK so check if it's the case and return the error
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have received an invalid frame, maybe a question/answer desync!
					//
					error = SBG_INVALID_FRAME;
				}
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
	}

	return error;
}
