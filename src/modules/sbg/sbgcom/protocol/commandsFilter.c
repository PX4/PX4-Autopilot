#include "commandsFilter.h"
#include "../sbgCom.h"
#include <stdlib.h>
#include <string.h>

/*!
 * Defines the kalman filter motion profile to be used
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pMpBuffer				Motion profile buffer pointer
 * \param[in]	mpSize					Motion profile buffer size
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgSetMotionProfile(SbgProtocolHandle handle, void *pMpBuffer, uint16 mpSize)
{
	SbgErrorCode error = SBG_NO_ERROR;
	const uint16 subBufferSize = 64-8-2;
	uint16 numBuffers;
	uint16 i;
	uint16 index;
	uint16 size;
	uint16 trial;

	//
	// In order to comply with low level protocol, we have to split the buffer in several parts
	// Each sub buffer will contain at 256 bytes maximum
	//
	numBuffers = mpSize / subBufferSize;
	if (mpSize % subBufferSize)
	{
		numBuffers++;
	}

	//
	// Send each sub buffer to the IG-Device
	//
	for (i = 0; i < numBuffers; i++)
	{
		//
		// Compute the index of the sub buffer to transmit
		//
		index = i*subBufferSize;
		
		//
		// Compute the size in bytes of the sub buffer to transmit
		//
		if (i < (numBuffers - 1))
		{
			size = subBufferSize;
		}
		else
		{
			size = mpSize % subBufferSize;
		}

		//
		// Now send the sub buffer and wait for the answer
		// We have three tries to send this sub buffer
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Send the sub buffer
			//
			error = sbgSendMPBuffer(handle, ((uint8*)pMpBuffer) + index, index, size);

			//
			// Test if the sub buffer has been sent
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Sub buffer successfully sent so exit the trial loop
				//
				break;
			}
		}

		//
		// Test if the sub buffer has been sent successfully
		//
		if (error != SBG_NO_ERROR)
		{
			//
			// An error has occured while sending the sub buffer so exit
			//
			break;
		}
	}

	//
	// Test if the buffer has been sent successfully to the device
	//
	if (error == SBG_NO_ERROR)
	{
		//
		// Try to validate the motion profile buffer
		//
		error = sbgValidateMPBuffer(handle);
	}
	
	return error;
}

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Sends a motion profile buffer part
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pMpBuffer				Motion profile buffer pointer
 * \param[in]	index					Index in the IG-500 motion profile buffer
 * \param[in]	size					Number of bytes to transmit
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgSendMPBuffer(SbgProtocolHandle handle, void *pMpBuffer, uint16 index, uint16 size)
{
	SbgErrorCode error;
	uint8 *pBuffer;

	//
	// Allocate memory for the buffer
	// And check resulting pointer
	//
	pBuffer = (uint8*)malloc(sizeof(uint16) + (size * sizeof(uint8)));

	if (pBuffer)
	{
		//
		// Build up the frame
		//
		*((uint16*)pBuffer) = sbgHostToTarget16(handle->targetOutputMode, index);
		memcpy(pBuffer + sizeof(uint16), pMpBuffer, size);

		//
		// Send the command used to set motion profile data
		//
		error = sbgProtocolSend(handle, SBG_SEND_MP_BUFFER, pBuffer, size + sizeof(uint16));

		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			// We let some time for the system to recover because it will reset partially
			//
			error = sbgWaitForAck(handle, 3*SBG_FRAME_RECEPTION_TIME_OUT);
		}

		//
		// Release the allocated memory
		//
		free(pBuffer);
	}
	else
	{
		//
		// Not enough memory
		//
		error = SBG_MALLOC_FAILED;
	}

	return error;
}

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Once the buffer is fully sent, Validate and use a motion profile buffer
 * \param[in]	handle					Valid sbgCom library handle
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgValidateMPBuffer(SbgProtocolHandle handle)
{
	SbgErrorCode error;

	//
	// Send the command used to set motion profile data
	//
	error = sbgProtocolSend(handle, SBG_VALIDATE_MP_BUFFER, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK, the device will answer and then reset so don't wait too long
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 * Retrives the kalman filter motion profile information
 * \param[in]	handle					Valid sbgCom library handle
 * \param[in]	pId						Motion profile Identifier is returned here
 * \param[in]	pVersion				Motion profile version is returned here
 * \return								SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgGetMotionProfileInfo(SbgProtocolHandle handle, uint32 *pId, uint32 *pVersion)
{
	uint8 cmd;
	uint16 size;
	uint32 bufferIn[2];
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_MP_INFO, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &bufferIn, &size, 2*sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_MP_INFO) && (size == 2*sizeof(uint32)) )
			{
				//
				// Returns, if possible, the heading source
				//
				if (pId)
				{
					*pId = sbgTargetToHost32(handle->targetOutputMode, bufferIn[0]);
				}
				if (pVersion)
				{
					*pVersion = sbgTargetToHost32(handle->targetOutputMode, bufferIn[1]);
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
SbgErrorCode sbgSetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource source)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	buffer[0] = 0;
	buffer[1] = (uint8)source;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_SET_FILTER_HEADING_SOURCE, buffer, 2*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// We should receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

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
SbgErrorCode sbgGetFilterHeadingSource(SbgProtocolHandle handle, SbgHeadingSource *pSource)
{
	uint8 cmd;
	uint16 size;
	uint8 buffer;
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_FILTER_HEADING_SOURCE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &buffer, &size, sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_FILTER_HEADING_SOURCE) && (size == sizeof(uint8)) )
			{
				//
				// Returns, if possible, the heading source
				//
				if (pSource)
				{
					*pSource = (SbgHeadingSource)buffer;
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

/*!
 *	Defines the magnetic declination in radians.<br>
 *	The declination is important for good results in navigation estimation with IG-500N devices.
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[in]	declination				The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetMagneticDeclination(SbgProtocolHandle handle, float declination)
{
	uint8 buffer[sizeof(uint8) + sizeof(uint32)];
	SbgErrorCode error;

	//
	// Check if the handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build the arguments buffer
		//
		*(uint8*)(buffer)					= 0;
		*(uint32*)(buffer + sizeof(uint8))	= sbgHostToTargetFloat(handle->targetOutputMode, declination);

		//
		// Send the command used to define the magnectic declination
		//
		error = sbgProtocolSend(handle, SBG_SET_MAGNETIC_DECLINATION, buffer, sizeof(uint8)+sizeof(uint32));

		//
		// Check if we have sent the frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			//
			error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Returns the magnetic declination in radians.<br>
 *	\param[in]	handle					Valid sbgCom library handle.
 *	\param[out]	pDeclination			The local magnetic declination in radians from -PI to +PI.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetMagneticDeclination(SbgProtocolHandle handle, float *pDeclination)
{
	uint8 cmd;
	uint16 size;
	uint32 buffer;
	SbgErrorCode error;

	//
	// Check if the handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send the command used to get the current magnetic declination
		//
		error = sbgProtocolSend(handle, SBG_GET_MAGNETIC_DECLINATION, NULL, 0);

		//
		// Check if we have sent the frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read the answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, (uint8*)&buffer, &size, sizeof(uint32));

			//
			// Check if we have received the frame successfully
			//
			if (error == SBG_NO_ERROR)
			{
				if ( (cmd == SBG_RET_MAGNETIC_DECLINATION) && (size == sizeof(uint32)) )
				{
					//
					// Returns, if possible, the magnetic declination
					//
					if (pDeclination)
					{
						*pDeclination = sbgTargetToHostFloat(handle->targetOutputMode, buffer);
					}
				}
				else
				{
					error = SBG_INVALID_FRAME;
				}
			}
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Send a new heading inforamtion to the Kalman filter.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	heading					The new heading in radians.
 *	\param[in]	accuracy				The heading accuracy in radians.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendFilterHeading(SbgProtocolHandle handle, float heading, float accuracy)
{
	uint32 buffer[2];
	SbgErrorCode error;

	//
	// Check if the handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Build the arguments buffer
		//
		buffer[0] = sbgHostToTargetFloat(handle->targetOutputMode, heading);
		buffer[1] = sbgHostToTargetFloat(handle->targetOutputMode, accuracy);

		//
		// Send the command used to inform the device we have a new heading data
		//
		error = sbgProtocolSend(handle, SBG_SEND_FILTER_HEADING, buffer, 2*sizeof(uint32));
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

//----------------------------------------------------------------------//
//- Additional configuration commands                                  -//
//----------------------------------------------------------------------//

/*!
 *	Defines the Heave configuration
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	enableHeave				Set to true if heave has to be computed
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetHeaveConf(SbgProtocolHandle handle, bool enableHeave)
{
	SbgErrorCode error;
	uint8 heaveConf[8];

	//
	// Check if the handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Define the output buffer
		//
		memset(heaveConf, 0, 8*sizeof(uint8));
		heaveConf[0] = enableHeave;

		//
		// Send the command used to inform the device we have a new heading data
		//
		error = sbgProtocolSend(handle, SBG_SET_HEAVE_CONF, heaveConf, sizeof(heaveConf));

		//
		// Check if we have sent the frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// We should receive an ACK
			//
			error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Returns the heave configuration
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	pEnableHeave			Set to true if heave has to be computed
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetHeaveConf(SbgProtocolHandle handle, bool *pEnableHeave)
{
	uint8 cmd;
	uint16 size;
	uint8 heaveConf[8];
	SbgErrorCode error;

	//
	// Check if the handle is valid
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Send the command used to get the heave configuration
		//
		error = sbgProtocolSend(handle, SBG_GET_HEAVE_CONF, NULL, 0);

		//
		// Check if we have sent the frame successfully
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Try to read the answer
			//
			error = sbgProtocolReceiveTimeOut(handle, &cmd, heaveConf, &size, sizeof(heaveConf));

			//
			// Check if we have received the frame successfully
			//
			if (error == SBG_NO_ERROR)
			{
				if ( (cmd == SBG_RET_HEAVE_CONF) && (size == sizeof(heaveConf)) )
				{
					//
					// Returns, if possible, the magnetic declination
					//
					if (pEnableHeave)
					{
						*pEnableHeave = heaveConf[0];
					}
				}
				else
				{
					error = SBG_INVALID_FRAME;
				}
			}
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}