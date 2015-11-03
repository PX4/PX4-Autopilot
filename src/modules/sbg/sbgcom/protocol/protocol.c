#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "protocol.h"
#include "commands.h"
#include "../time/sbgTime.h"
#include "protocolOutputMode.h"

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Init the protocol system used to communicate with the product and return the created handle.
 *	\param[in]	deviceName				Device location to open such as COM1 on windows platforms.
 *	\param[in]	baudRate				Baud rate to use to communication with the device such as 115200.
 *	\param[out]	pHandle					Pointer used to handle the created sbgCom library handle.
 *	\return								SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgProtocolInit(const char *deviceName, uint32 baudRate, SbgProtocolHandle *pHandle)
{
	SbgErrorCode errorCode;
	SbgDeviceHandle deviceHandle;
	SbgProtocolHandle protocolHandle;

	//
	// Check if we can store the new handle
	//
	if (pHandle)
	{
		//
		// First set the returned handle to an invalid handle
		//
		*pHandle = SBG_INVALID_PROTOCOL_HANDLE;

		//
		// Try to open the device
		//
		errorCode = sbgDeviceOpen(deviceName, baudRate, &deviceHandle);

		//
		// Check if we have opened the device
		//
		if ( (errorCode == SBG_NO_ERROR) && (deviceHandle != SBG_INVALID_DEVICE_HANDLE) )
		{
			//
			// Create a new protocol handle
			//
			protocolHandle = (SbgProtocolHandle)malloc(sizeof(SbgProtocolHandleInt));

			//
			// Init the new protocol handle
			//
			if (protocolHandle)
			{
				protocolHandle->serialBufferSize = 0;
				protocolHandle->serialHandle = deviceHandle;
				protocolHandle->targetOutputMode = 0;
				protocolHandle->targetDefaultOutputMask = 0;
				protocolHandle->pUserHandlerContinuousError = NULL;
				protocolHandle->pUserHandlerDefaultOutput = NULL;
				protocolHandle->pUserArgContinuousError = NULL;
				protocolHandle->pUserArgDefaultOutput = NULL;
				protocolHandle->pUserHandlerTriggeredOutput = NULL;
				protocolHandle->pUserArgTriggeredOutput = NULL;

				//
				// We have a valid protocol handle so return it
				//
				*pHandle = protocolHandle;
			}
			else
			{
				//
				// Close the opened device
				//
				sbgDeviceClose(deviceHandle);
				errorCode = SBG_MALLOC_FAILED;
			}
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	handle					A valid sbgCom library handle to close.
 *	\return								SBG_NO_ERROR if we have closed and released the protocol system.
 */
SbgErrorCode sbgProtocolClose(SbgProtocolHandle handle)
{
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Close the device
		//
		sbgDeviceClose(handle->serialHandle);

		//
		// Release the protocol handle
		//
		free(handle);

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Compute a CRC 16 for a specified buffer using a polynomial 0x8408 
 *	\param[in]	pFrame						Buffer to compute the CRC on.
 *	\param[in]	bufferSize					Buffer size in bytes.
 *	\return									CRC 16 computed for the buffer.
 */
uint16 sbgProtocolCalcCRC(const void *pFrame, uint16 bufferSize)
{
	const uint8 *pBuffer = (const uint8*)pFrame;
	uint16 poly = 0x8408;
	uint16 crc = 0;
	uint8 carry;
	uint8 i_bits;
	uint16 j;
	
	for (j=0 ; j < bufferSize ; j++)
	{
		crc = crc ^ pBuffer[j] ;
		for (i_bits=0 ; i_bits < 8 ; i_bits++)
		{
			carry = crc & 1 ;
			crc = crc / 2 ;
			if (carry)
			{
				crc = crc ^ poly;
			}
		}
	}

	return crc;	
}


/*!
 *	Flush all data and the serial com port.
 *	\param[in]	handle		A valid sbgCom library handle.
 *	\return					SBG_NO_ERROR if the data has been flused.
 */
SbgErrorCode sbgProtocolFlush(SbgProtocolHandle handle)
{
	SbgErrorCode errorCode = SBG_ERROR;

	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Flush the serial port
		//
		sbgDeviceFlush(handle->serialHandle);

		//
		// Flush the protocol buffer
		//
		handle->serialBufferSize = 0;

		//
		// No error.
		//
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Change the baud rate used to communication with the device.<br>
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	baudRate				New baudrate to use.
 *	\return								SBG_NO_ERROR if we have changed the serial baud rate.
 */
SbgErrorCode sbgProtocolChangeBaud(SbgProtocolHandle handle, uint32 baudRate)
{
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		return sbgDeviceChangeBaud(handle->serialHandle, baudRate);
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Send a frame to the device (size should be less than 504 bytes).
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	cmd						Command number to send.
 *	\param[in]	pData					Pointer to the data field to send.
 *	\param[in]	size					Size of the data field to send.
 *	\return								SBG_NO_ERROR if the frame has been sent.
 */
SbgErrorCode sbgProtocolSend(SbgProtocolHandle handle, uint8 cmd, const void *pData, uint16 size)
{
	SbgErrorCode errorCode;
	uint8 *pFrame;
	uint16 realSize;
	uint16 crc;
	
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Check if the size is valid
		//
		if (size <= SBG_MAX_DATA_LENGTH)
		{
			//
			// Allocate the frame (8 fixed minimal frame + data size)
			//
			pFrame = (uint8*)malloc(8+size);
			
			if (pFrame)
			{
				//
				// Flush the com to remove old data (re-sync)
				//
				sbgDeviceFlush(handle->serialHandle);

				//
				// Create the frame
				//
				pFrame[0] = SBG_SYNC;
				pFrame[1] = SBG_STX;
				pFrame[2] = cmd;
				
				//
				// Copy the data
				//
				if ( (size > 0) && (pData) )
				{
					memcpy(pFrame+5, pData, size);
					realSize = size;	
				}
				else
				{
					// Data field empty	
					realSize = 0;
				}

				//
				// Define the size
				//
				pFrame[3] = (uint8)(realSize>>8);		// MSB
				pFrame[4] = (uint8)(realSize   );		// LSB
				
				//
				// Calculate the crc (from CMD to end of DATA)
				//
				crc = sbgProtocolCalcCRC(pFrame+2, realSize+3);

				//
				// Fill the end of the frame
				//
				pFrame[realSize+5] = (uint8)(crc>>8);		// MSB
				pFrame[realSize+6] = (uint8)(crc   );		// LSB
				pFrame[realSize+7] = SBG_ETC;
				
				//
				// Send the frame over the device
				//
				errorCode = sbgDeviceWrite(handle->serialHandle, pFrame, realSize+8);

				//
				// Free the sent frame
				//
				free(pFrame);

				return errorCode;
			}
			else
			{
				return SBG_MALLOC_FAILED;
			}
		}
		else
		{
			return SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Try to receive a frame from the device and returns the cmd, data and size of data field (maxSize less than 504 bytes)
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 */
SbgErrorCode sbgProtocolReceive(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize)
{
	SbgErrorCode errorCode = SBG_NOT_READY;
	bool syncFound;
	uint16 dataSize = 0;
	uint16 frameCrc;
	uint16 computedCrc;
	uint16 i;
	uint32 numBytesRead;

	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		// 
		// Set the size to 0 in order to avoid possible bugs
		//
		if (pSize)
		{
			*pSize = 0;
		}

		//
		// The max data size is SBG_MAX_DATA_LENGTH
		//
		if (maxSize > SBG_MAX_DATA_LENGTH)
		{
			maxSize = SBG_MAX_DATA_LENGTH;
		}

		//
		// Check if we can receive some new data (the receive buffer isn't full)
		//
		if (handle->serialBufferSize < SBG_RX_BUFFER_SIZE)
		{
			//
			// First try to read some new data
			//
			if (sbgDeviceRead(handle->serialHandle, handle->serialBuffer+handle->serialBufferSize, 
				              SBG_RX_BUFFER_SIZE-handle->serialBufferSize, &numBytesRead) == SBG_NO_ERROR)
			{
				handle->serialBufferSize += (uint16)numBytesRead;
			}
			else
			{
				//
				// Nothing read
				//
				numBytesRead = 0;
			}
		}

		//
		// We have read all available data from the serial buffer.
		// We will try to process all received data until we have found a valid frame.
		//
		while (handle->serialBufferSize > 0)
		{
			//
			// For now, we haven't found any start of frame
			//
			syncFound = FALSE;

			//
			// To find a valid start of frame we need at least 2 bytes in the reception buffer
			//
			if (handle->serialBufferSize >= 2)
			{
				//
				// Try to find a valid start of frame by looking for SYNC and STX chars
				//
				for (i=0; i<handle->serialBufferSize-1; i++)
				{
					//
					// A valid start of frame should begin with SYNC and when STX chars
					//
					if ( (handle->serialBuffer[i] == SBG_SYNC) && (handle->serialBuffer[i+1] == SBG_STX) )
					{
						//
						// We have found the sync char so remove all dumy received bytes before the begining of the frame
						//
						if (i>0)
						{
							memmove(handle->serialBuffer,handle->serialBuffer+i,handle->serialBufferSize-i);
							handle->serialBufferSize = handle->serialBufferSize-i;
						}

						//
						// The sync has been found
						//
						syncFound = TRUE;
						break;
					}
				}
			}

			//
			// Check if a valid start of frame has been found
			//
			if (syncFound)
			{
				//
				// A valid start of frame found, try to extract the frame if we have at least a whole frame.
				//
				if (handle->serialBufferSize < 8)
				{
					//
					// Don't have enough data for a valid frame
					//
					return SBG_NOT_READY;
				}

				//
				// Extract the frame size (MSB first)
				//
				dataSize = ((uint16)handle->serialBuffer[3]<<8) | handle->serialBuffer[4];

				//
				// Check if the frame size is valid
				//
				if (dataSize <= SBG_MAX_DATA_LENGTH)
				{
					//
					// Check if we have received the whole frame
					//
					if (handle->serialBufferSize < dataSize+8)
					{
						//
						// Don't have received the whole frame
						//
						return SBG_NOT_READY;
					}

					//
					// We have the whole frame so check the ETC char (end of frame)
					//
					if (handle->serialBuffer[dataSize+7] == SBG_ETC)
					{
						//
						// Read the CRC from the received frame (MSB first)
						// 
						frameCrc = ((uint16)handle->serialBuffer[dataSize+5]<<8) | handle->serialBuffer[dataSize+6];

						//
						// Compute the CRC of the received frame
						//
						computedCrc = sbgProtocolCalcCRC(handle->serialBuffer+2, dataSize+3);

						//
						// Check if the received frame has a valid CRC
						//
						if (frameCrc == computedCrc)
						{
							//
							// We have a valid frame so return the received command
							//
							if (pCmd)
							{
								*pCmd = handle->serialBuffer[2];
							}

							//
							// Extract the data field if needed
							//
							if (dataSize > 0)
							{
								//
								// Check if input parameters are valid
								//
								if ( (pData) && (pSize) )
								{
									//
									// Check if we have enough space to store the data field
									//
									if (dataSize > maxSize)
									{
										*pSize = maxSize;
										memcpy(pData, handle->serialBuffer+5, maxSize);
										errorCode = SBG_BUFFER_OVERFLOW;
									}
									else
									{
										*pSize = dataSize;
										memcpy(pData, handle->serialBuffer+5, dataSize);
										errorCode = SBG_NO_ERROR;
									}
								}
								else
								{
									errorCode = SBG_NULL_POINTER;
								}
							}
							else
							{
								errorCode = SBG_NO_ERROR;
							}

							//
							// We have read a whole valid frame so remove it from the buffer
							//
							if (handle->serialBufferSize>dataSize+8)
							{
								memmove(handle->serialBuffer, handle->serialBuffer+dataSize+8, handle->serialBufferSize-(dataSize+8));
								handle->serialBufferSize = handle->serialBufferSize-(dataSize+8);
							}
							else
							{
								handle->serialBufferSize = 0;
							}

							//
							// A valid frame has been received
							//
							return errorCode;
						}
						else
						{
							//
							// We have an invalid frame CRC but we have also read the whole frame so remove it from the buffer
							//
							if (handle->serialBufferSize>dataSize+8)
							{
								memmove(handle->serialBuffer, handle->serialBuffer+dataSize+8, handle->serialBufferSize-(dataSize+8));
								handle->serialBufferSize = handle->serialBufferSize-(dataSize+8);
							}
							else
							{
								handle->serialBufferSize = 0;
							}
						}
					}
					else
					{
						//
						// End of frame not found so the frame is invalid, we should have incorrectly detected a start of frame.
						// Remove the SYNC and STX char because we should have an invalid sync
						//
						memmove(handle->serialBuffer, handle->serialBuffer+2, handle->serialBufferSize-2);
						handle->serialBufferSize = handle->serialBufferSize-2;
					}
				}
				else
				{						
					//
					// Frame size invalid, so we should have incorrectly detected a start of frame.
					// Remove the SYNC and STX char and retry to read a valid frame
					//
					memmove(handle->serialBuffer, handle->serialBuffer+2, handle->serialBufferSize-2);
					handle->serialBufferSize = handle->serialBufferSize-2;
				}
			}
			else
			{
				//
				// Unable to find a valid start of frame so check if the last byte is a SYNC char in order to keep it for next time
				//
				if (handle->serialBuffer[handle->serialBufferSize-1] == SBG_SYNC)
				{
					//
					// Report the SYNC char and discard all other bytes in the buffer
					//
					handle->serialBuffer[0] = SBG_SYNC;
					handle->serialBufferSize = 1;
				}
				else
				{
					//
					// Discard the whole buffer
					//
					handle->serialBufferSize = 0;
				}

				//
				// Unable to find a start of frame
				//
				return SBG_NOT_READY;
			}
		}

		//
		// The whole buffer has been paresed and no valid frame has been found
		//
		return SBG_NOT_READY;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Try to receive a frame during a time out.<br>
 *	This function also handle continuous frame present in the serial buffer.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolReceiveTimeOut(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize)
{
	
	return sbgProtocolReceiveTimeOutMs(handle, pCmd, pData, pSize, maxSize, SBG_FRAME_RECEPTION_TIME_OUT);
}


/*!
 *	Try to receive a frame during a time out.
 *	This function also handle continuous frame present in the serial buffer.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOutMs				Time in millisecond during which a frame can be received.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolReceiveTimeOutMs(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize, uint32 timeOutMs)
{
	uint32 endReceiveTime = sbgGetTime() + timeOutMs;
	SbgErrorCode errorCode;
	uint8 fullFrame[512];
	uint16 sizeTmp;
	uint8 cmdTmp;

	//
	// Check if we have a valid protocol handle
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Try to read the frame until the time out has elapsed
		//
		while (sbgGetTime() < endReceiveTime)
		{
			//
			// Max size is here set to 512 because continuous information can occur before the
			// desired answer
			//
			errorCode = sbgProtocolReceive(handle, &cmdTmp, fullFrame, &sizeTmp, 512);

			//
			// Check if we have received a valid frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Depending on the received frame, handle continuous or triggered frames if necessary
				//
				switch (cmdTmp)
				{
				case SBG_CONTINUOUS_DEFAULT_OUTPUT:
					//
					// We have received a continuous frame so extract data from it
					//
					sbgProtocolManageContinuousFrame(handle, fullFrame, sizeTmp);
					break;

				case SBG_TRIGGERED_OUTPUT:
					//
					// We have received a triggered output frame so extract data from it
					//
					sbgProtocolManageTriggeredFrame(handle, fullFrame, sizeTmp);
					break;

				default:
					//
					// Not a continuous frame, so we return its parameters
					// Check if the Data buffer has enough space to contain the received answer
					//
					if (sizeTmp <= maxSize)
					{
						//
						// Extract frame data
						//
						if (pCmd)
						{
							*pCmd = cmdTmp;
						}
						if (pSize)
						{
							*pSize = sizeTmp;
						}
						if (pData)
						{
							memcpy(pData, fullFrame, sizeTmp*sizeof(uint8));
						}

						//
						// We have a valid frame so return a no error
						//
						return SBG_NO_ERROR;
					}
					else
					{
						//
						// The pData buffer is too small to contain the received buffer
						//
						return SBG_BUFFER_OVERFLOW;
					}
				}
			}
			else if (errorCode == SBG_NOT_READY)
			{
				//
				// We only sleep if nothing was received
				//
				sbgSleep(1);
			}
			else
			{
				return errorCode;
			}
		}

		//
		// No frame received during the time out so exit.
		//
		return SBG_TIME_OUT;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Handles a received continuous frame and call the user continuous mode callback if the received frame is valid.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pData					Received data field buffer.
 *	\param[out]	size					Size of the received data field buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolManageContinuousFrame(SbgProtocolHandle handle, uint8 *pFullFrame, uint16 size)
{
	SbgOutput output;
	SbgErrorCode errorCode = SBG_ERROR;

	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Check if we have a user handler
		//
		if (handle->pUserHandlerDefaultOutput)
		{
			//
			// First, compute the SbgOutput structure
			//
			errorCode = sbgFillOutputFromBuffer(handle->targetOutputMode, handle->targetDefaultOutputMask, pFullFrame, size, &output);

			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Then, call the user handler
				//
				handle->pUserHandlerDefaultOutput(handle, &output, handle->pUserArgDefaultOutput);
			}
			else if (handle->pUserHandlerContinuousError)
			{
				//
				// We have an error in continuous frame interpretation and we hae a valid continuous error callback
				// so call the continuous error callback
				//
				handle->pUserHandlerContinuousError(handle, errorCode, handle->pUserArgContinuousError);
			}
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Handles a received triggered frame and call the user continuous mode callback if the received frame is valid.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pFullFrame				Received data field buffer.
 *	\param[out]	size					Size of the received data field buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolManageTriggeredFrame(SbgProtocolHandle handle, uint8 *pFullFrame, uint16 size)
{
	SbgErrorCode errorCode = SBG_ERROR;
	SbgOutput output;
	uint32 triggerMask, outputMask;


	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// Check if we have a user handler
		//
		if (handle->pUserHandlerTriggeredOutput)
		{
			if (size >= 2*sizeof(uint32))
			{
				//
				// First, get the trigger mask and output mask
				//
				triggerMask = sbgTargetToHost32( handle->targetOutputMode, *(uint32*)pFullFrame );
				outputMask  = sbgTargetToHost32( handle->targetOutputMode, *(uint32*)(pFullFrame + sizeof(uint32)) );
				
				//
				// Then compute the SbgOutput structure
				//
				errorCode = sbgFillOutputFromBuffer(handle->targetOutputMode, outputMask, pFullFrame+2*sizeof(uint32), size-2*sizeof(uint32), &output);

				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Then, call the user handler
					//
					handle->pUserHandlerTriggeredOutput(handle, triggerMask, &output, handle->pUserArgTriggeredOutput);
				}
				else if (handle->pUserHandlerContinuousError)
				{
					//
					// We have an error in continuous frame interpretation and we hae a valid continuous error callback
					// so call the continuous error callback
					//
					handle->pUserHandlerContinuousError(handle, errorCode, handle->pUserArgContinuousError);
				}
			}
			else
			{
				errorCode = SBG_INVALID_FRAME;
			}
		}
		else
		{
			errorCode =  SBG_NO_ERROR;
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Scan the serial port for continuous frames.<br>
 *	For each received frame, call the continuous user callback function.<br>
 *	For each error on a frame reception, call the continuous error callback function.<br>
 *	This function returns only when no more frames are available in the serial port.
 *	\param[in]	handle		A valid sbgCom library handle.
 *	\return					SBG_NO_ERROR if all the frames has been read sucessfully.
 */
SbgErrorCode sbgProtocolContinuousModeHandle(SbgProtocolHandle handle)
{
	uint8 fullFrame[512];
	uint8 cmd;
	uint16 size;
	SbgErrorCode errorCode = SBG_ERROR;

	//
	// Check if we have a valid protocol handle
	//
	if (handle != SBG_INVALID_PROTOCOL_HANDLE)
	{
		//
		// We read all received frames (until we have a SBG_NOT_READY error on the receive function)
		//
		do
		{
			//
			// Try to receive a frame
			//
			errorCode = sbgProtocolReceive(handle, &cmd, fullFrame, &size, 512);
			//
			// Check if we have received a valid frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the received frame is a continuous frame
				//
				switch (cmd)
				{
				case SBG_CONTINUOUS_DEFAULT_OUTPUT:
					//
					// We have received a continuous frame so extract data from it
					//
					sbgProtocolManageContinuousFrame(handle, fullFrame, size);
					break;

				case SBG_TRIGGERED_OUTPUT:
					//
					// We have received a triggered output frame so extract data from it
					//
					sbgProtocolManageTriggeredFrame(handle, fullFrame, size);
					break;

				default:
					//
					// Not a continuous frame we have lost this frame so return an error using the continuous error callback
					//
					if (handle->pUserHandlerContinuousError)
					{
						handle->pUserHandlerContinuousError(handle, SBG_NOT_CONTINUOUS_FRAME, handle->pUserArgContinuousError);
					}
					break;
				}
			}
			else if ( (handle->pUserHandlerContinuousError) && (errorCode != SBG_NOT_READY) )
			{
				//
				// We have an error during the frame reception so return the error using the continuous error callback
				//
				handle->pUserHandlerContinuousError(handle, errorCode, handle->pUserArgContinuousError);
			}
		} while (errorCode != SBG_NOT_READY);

		//
		// We have read all available frames in the buffer
		//
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Defines the handle function to call when we have received a valid triggered frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the Triggered frame handler function.
 *	\param[in]	pUserArg					User argument to pass to the error handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetTriggeredModeCallback(SbgProtocolHandle handle, TriggeredModeCallback callback, void *pUserArg)
{
	//
	// Check if we have both a valid handle and a valid callback function
	//
	if ( (handle != NULL) && (callback != NULL) )
	{
		handle->pUserHandlerTriggeredOutput = callback;
		handle->pUserArgTriggeredOutput = pUserArg;
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Defines the handle function to call when we have an error on a continuous frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the error handler function.
 *	\param[in]	pUserArg					User argument to pass to the error handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetContinuousErrorCallback(SbgProtocolHandle handle, ContinuousErrorCallback callback, void *pUserArg)
{
	//
	// Check if we have both a valid handle and a valid callback function
	//
	if ( (handle != NULL) && (callback != NULL) )
	{
		handle->pUserHandlerContinuousError = callback;
		handle->pUserArgContinuousError = pUserArg;
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Defines the handle function to call when we have received a valid continuous frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the error handler function.
 *	\param[in]	pUserArg					User argument to pass to the error handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetContinuousModeCallback(SbgProtocolHandle handle, ContinuousModeCallback callback, void *pUserArg)
{
	//
	// Check if we have both a valid handle and a valid callback function
	//
	if ( (handle != NULL) && (callback != NULL) )
	{
		handle->pUserHandlerDefaultOutput = callback;
		handle->pUserArgDefaultOutput = pUserArg;
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}
