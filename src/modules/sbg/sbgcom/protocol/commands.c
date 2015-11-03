#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "protocolOutputMode.h"
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
//- Common commands operations                                         -//
//----------------------------------------------------------------------//

/*!
 *	Send an ACK frame to the device with a specified error code.
 *	\param[in]	handle							A valid sbgMatLab library handle.
 *	\param[in]	error							The error code field of the ACK frame.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSendAck(SbgProtocolHandle handle, SbgErrorCode error)
{
	return sbgProtocolSend(handle, SBG_ACK, &error, sizeof(uint8));
}

/*!
 *	Wait for an ACK or NACK frame for ms number of milliseconds.<br>
 *	If we have received a frame, the function returns the error code field<br>
 *	contained in the received frame.<br>
 *	If the received frame contains SBG_NO_ERROR, we have received an ACK otherwise it's an NACK.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	ms								Number of milliseconds to wait for a frame.
 *	\return										SBG_NO_ERROR if we have received an ACK frame within ms milliseconds.<br>
 *												Error code if we have received an NACK frame<br>
 *												SBG_TIME_OUT if no ACK/NACK frame was received.<br>
 */
SbgErrorCode sbgWaitForAck(SbgProtocolHandle handle, uint32 ms)
{
	SbgErrorCode errorCode;
	uint8 receivedErrorCode;
	uint16 receivedSize;
	uint8 cmd;

	//
	// Try to receive an ACK during the specified time
	//
	errorCode = sbgProtocolReceiveTimeOutMs(handle, &cmd, &receivedErrorCode, &receivedSize, sizeof(uint8), ms);

	//
	// Check if a valid frame has been received
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Check if its a valid ACK frame
		//
		if ( (cmd == SBG_ACK) && (receivedSize == sizeof(uint8)) )
		{
			//
			// Extract the error code contained in the ACK frame
			//
			errorCode = (SbgErrorCode)(receivedErrorCode);
		}
		else
		{
			errorCode = SBG_INVALID_FRAME;
		}
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Settings commands operations                                       -//
//----------------------------------------------------------------------//

/*!
 *	Gets device information such as product code, hardware and firmware revisions.<br>
 *	For versions, use SBG_VERSION_GET_MAJOR, SBG_VERSION_GET_MINOR, SBG_VERSION_GET_REV and SBG_VERSION_GET_BUILD<br>
 *	to extract versions inforamtion.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	productCode						Device product code string.
 *	\param[out]	pDeviceId						Device id.
 *	\param[out]	pFirmwareVersion				The device firmware version. ('1.0.0.0')
 *	\param[out]	pCalibDataVersion				The device calibration data version. ('1.0.0.0')
 *	\param[out]	pMainBoardVersion				The device main board hardware version. ('1.0.0.0')
 *	\param[out]	pGpsBoardVersion				The device gps/top board hardware version. ('1.0.0.0')
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetInfos(SbgProtocolHandle handle, char productCode[32], uint32 *pDeviceId, uint32 *pFirmwareVersion, uint32 *pCalibDataVersion, uint32 *pMainBoardVersion, uint32 *pGpsBoardVersion)
{
	uint8 cmd;
	uint16 size;
	uint8 buffer[sizeof(char)*32+sizeof(uint32)*5];
	uint32 deviceId;
	uint32 firmwareVersion;
	uint32 calibDataVersion;
	uint32 mainBoardVersion;
	uint32 gpsBoardVersion;
	SbgErrorCode error;

	//
	// Send the command used to get hardware and software version
	//
	error = sbgProtocolSend(handle, SBG_GET_INFOS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, sizeof(char)*32+sizeof(uint32)*5);

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_INFOS) && (size == sizeof(char)*32+sizeof(uint32)*5) )
			{
				//
				// Copy the string product if possible
				//
				if (productCode)
				{
					//
					// Copy the product code and ensure that the string is NULL terminated
					//
					memcpy(productCode, buffer, sizeof(char)*31);
					productCode[31] = '\0';
				}

				//
				// When extract the three 32 bits data
				//
				deviceId			= *(uint32*)(buffer+sizeof(char)*32+sizeof(uint32)*0);
				firmwareVersion		= *(uint32*)(buffer+sizeof(char)*32+sizeof(uint32)*1);
				calibDataVersion	= *(uint32*)(buffer+sizeof(char)*32+sizeof(uint32)*2);
				mainBoardVersion	= *(uint32*)(buffer+sizeof(char)*32+sizeof(uint32)*3);
				gpsBoardVersion		= *(uint32*)(buffer+sizeof(char)*32+sizeof(uint32)*4);

				//
				// Convert the data to host format
				//
				deviceId			= sbgTargetToHost32(handle->targetOutputMode, deviceId);
				firmwareVersion		= sbgTargetToHost32(handle->targetOutputMode, firmwareVersion);
				calibDataVersion	= sbgTargetToHost32(handle->targetOutputMode, calibDataVersion);
				mainBoardVersion	= sbgTargetToHost32(handle->targetOutputMode, mainBoardVersion);
				gpsBoardVersion		= sbgTargetToHost32(handle->targetOutputMode, gpsBoardVersion);

				//
				// Returns the values if needed
				//
				if (pDeviceId)
				{
					*pDeviceId = deviceId;
				}
				if (pFirmwareVersion)
				{
					*pFirmwareVersion = firmwareVersion;
				}
				if (pCalibDataVersion)
				{
					*pCalibDataVersion = calibDataVersion;
				}
				if (pMainBoardVersion)
				{
					*pMainBoardVersion = mainBoardVersion;
				}
				if (pGpsBoardVersion)
				{
					*pGpsBoardVersion = gpsBoardVersion;
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
 *	Defines a user selectable ID for the device.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	userId							An uint32 used to hold the device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetUserId(SbgProtocolHandle handle, uint32 userId)
{
	uint8 buffer[sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(uint32*)(buffer+sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, userId);

	//
	// Send the command used to set the user ID
	//
	error = sbgProtocolSend(handle, SBG_SET_USER_ID, buffer, sizeof(uint8)+sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}
	
	return error;
}

/*!
 *	Returns the device user id.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pUserId							Pointer to an uint32 used to hold the device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetUserId(SbgProtocolHandle handle, uint32 *pUserId)
{
	uint8 cmd;
	uint16 size;
	uint32 userId;
	SbgErrorCode error;

	//
	// Send the command used to get the user defined ID
	//
	error = sbgProtocolSend(handle, SBG_GET_USER_ID, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &userId, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_USER_ID) && (size == sizeof(uint32)) )
			{
				//
				// Returns the user id value
				//
				if (pUserId)
				{
					*pUserId = sbgTargetToHost32(handle->targetOutputMode, userId);
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
 *	Defines the baud rate of the device uart communication.<br>
 *	If the command is valid, the device acknoledge the baudrate change at the current speed and THEN change it's baud rate.<br>
 *	This command only change the baud rate on the device. It doesn't change the sbgCom library baud rate.<br>
 *	To change only the baud rate used by the library, you have to use the function sbgProtocolChangeBaud.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	baudRate						The new device baud rate to use.<br>
 *												Valid values are [9600, 19200,38400,57600,115200,230400,460800,921600].
 *	\param[in]	uartOptions						Options applied on the COM port: (only with supported hardware)
 *												 - SBG_PROTOCOL_DIS_TX_EMI_REDUCTION : Normal / Fast mode of operation
 *												 - SBG_PROTOCOL_EN_TX_EMI_REDUCTION: Slow operation for EMI reduction. 
 *												   Baudrate is then limited at 230400bps.
 *	\return										If SBG_NO_ERROR, the device has been configured to the new speed.
 */
SbgErrorCode sbgSetProtocolMode(SbgProtocolHandle handle, uint32 baudRate, uint32 uartOptions)
{
	uint8 cmd;
	uint16 size;
	uint8 buffer[sizeof(uint8)+sizeof(uint32)];
	uint32 realBaudRate;
	uint32 realUartOption;
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(uint32*)(buffer+sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, baudRate | uartOptions);

	//
	// Send the command used to set the protocol mode
	//
	error = sbgProtocolSend(handle, SBG_SET_PROTOCOL_MODE, buffer, sizeof(uint8)+sizeof(uint32));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, buffer, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			//
			// Check that answer is valid
			//
			if ( (cmd == SBG_RET_PROTOCOL_MODE) && (size == sizeof(uint32)) )
			{
				//
				// Get the baud rate only
				//
				realBaudRate = sbgTargetToHost32(handle->targetOutputMode, *(uint32*)buffer)&0x00FFFFFF;
				realUartOption = sbgTargetToHost32(handle->targetOutputMode, *(uint32*)buffer)&0xFF000000;

				//
				// Check that the baud rate difference is less than 5% and that the uart options are the same
				//
				if ( (((float)(abs(baudRate-realBaudRate)/(float)baudRate)) < 0.05f) && (realUartOption == uartOptions) )
				{
					error = SBG_NO_ERROR;
				}
				else
				{
					error = SBG_INVALID_PARAMETER;
				}
			}
			else
			{
				error = SBG_INVALID_PARAMETER;
			}

			//
			// Wait 10 ms in order to let the device change its speed.
			//
			sbgSleep(10);
		}
		else if ( (cmd == SBG_ACK) && (size == sizeof(uint8)) )
		{
			//
			// Get the error code contained in the ACK frame
			//
			error = (SbgErrorCode)(buffer[0]);

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

	return error;
}

/*!
 *	Command used to get the current theorical baudrate used by the device.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pBaudRate						Theorical baud rate used by the device.
 *  \param[out]	pUartOptions					Options applied on the COM port: (only with supported hardware)
 *												 - SBG_PROTOCOL_DIS_TX_EMI_REDUCTION : Normal / Fast mode of operation
 *												 - SBG_PROTOCOL_EN_TX_EMI_REDUCTION: Slow operation for EMI reduction.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetProtocolMode(SbgProtocolHandle handle, uint32 *pBaudRate, uint32 *pUartOptions)
{
	uint8 cmd;
	uint16 size;
	SbgErrorCode error;
	uint32 options;

	//
	// Send the command used to get the protocol mode
	//
	error = sbgProtocolSend(handle, SBG_GET_PROTOCOL_MODE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &options, &size, sizeof(uint32));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_PROTOCOL_MODE) && (size == sizeof(uint32)) )
			{
				//
				// Extract param data
				//
				options = sbgTargetToHost32(handle->targetOutputMode, options);
	
				//
				// Returns the baudrate and uart option if required
				//
				if (pBaudRate)
				{
					*pBaudRate = options & (~SBG_PROTOCOL_EN_TX_EMI_REDUCTION);
				}
				if (pUartOptions)
				{
					*pUartOptions = options & SBG_PROTOCOL_EN_TX_EMI_REDUCTION;
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
 *	Defines the output mode of the target, big/little endian and float/fixed format.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	outputMode						The output mode configuration using masks<br>
 *												- #SBG_OUTPUT_MODE_DEFAULT<br>
 *												- #SBG_OUTPUT_MODE_BIG_ENDIAN<br>
 *												- #SBG_OUTPUT_MODE_LITTLE_ENDIAN<br>
 *												- #SBG_OUTPUT_MODE_FLOAT<br>
 *												- #SBG_OUTPUT_MODE_FIXED<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetOutputMode(SbgProtocolHandle handle, uint8 outputMode)
{
	uint8 buffer[sizeof(uint8)*2];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	*(buffer+sizeof(uint8)) = outputMode;

	//
	// Send the command used to set the output mode
	//
	error = sbgProtocolSend(handle, SBG_SET_OUTPUT_MODE, buffer, sizeof(uint8)*2);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);

		//
		// Check if we have received an ACK
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// The new output mode has been processed so update the protocol instance output mode
			//
			handle->targetOutputMode = outputMode;
		}
	}

	return error;
}

/*!
 *	Returns the output mode of the target, big/little endian and float/fixed format.
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pOutputMode						Current output mask used by the device.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetOutputMode(SbgProtocolHandle handle, uint8 *pOutputMode)
{
	uint8 cmd;
	uint16 size;
	uint8 outputMode;
	SbgErrorCode error;

	//
	// Send the command used to get the output mode
	//
	error = sbgProtocolSend(handle, SBG_GET_OUTPUT_MODE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &outputMode, &size, sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_OUTPUT_MODE) && (size == sizeof(uint8)) )
			{
				//
				// We have received the ouput mode so update it for the current protocol instance
				//
				handle->targetOutputMode = outputMode;

				//
				// Returns the flags
				//
				if (pOutputMode)
				{
					*pOutputMode = outputMode;
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
 *	Sets the Low power modes for the IG-Device
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	devicePowerMode					Defines the device power mode.
 *	\param[in]	gpsPowerMode					Defines the the GPS receiver power mode. (leave to SBG_GPS_MAX_PERF if there is no GPS reveicer)	<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetLowPowerModes(SbgProtocolHandle handle, SbgPowerModeDevice devicePowerMode, SbgPowerModeGps gpsPowerMode)
{
	uint8 buffer[3*sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	buffer[0] = 0;
	buffer[1] = (uint8)devicePowerMode;
	buffer[2] = (uint8)gpsPowerMode;

	//
	// Send the command used to set the low power mode for the device
	//
	error = sbgProtocolSend(handle, SBG_SET_LOW_POWER_MODE, buffer, 3*sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK (this command could take some time to complete)
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT*2);
	}

	return error;
}

/*!
 *	Gets the Low power modes for the IG-Device
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[out]	pDevicePowerMode				Returns the device power mode. (pass NULL if not used)
 *	\param[out]	pGpsPowerMode					Returns the GPS receiver power mode. (pass NULL if not used)	<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetLowPowerModes(SbgProtocolHandle handle, SbgPowerModeDevice *pDevicePowerMode, SbgPowerModeGps *pGpsPowerMode)
{
	uint8 cmd;
	uint16 size;
	uint8 frame[2];
	SbgErrorCode error;

	//
	// Send the command used to get the low power mode
	//
	error = sbgProtocolSend(handle, SBG_GET_LOW_POWER_MODE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, &frame, &size, 2*sizeof(uint8));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_LOW_POWER_MODE) && (size ==  2*sizeof(uint8)) )
			{
				//
				// Fill outputs if needed
				//
				if (pDevicePowerMode)
				{
					*pDevicePowerMode = (SbgPowerModeDevice)frame[0];
				}
				if (pGpsPowerMode)
				{
					*pGpsPowerMode = (SbgPowerModeGps)frame[1];
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
 *	Write a user buffer in the IG device's memory
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\param[in]	index							Address in the user memory where the data has to be written.	<br>
 *												Max address: 0x3F (64 bytes)
 *	\param[in]	size							Size of the user buffer to write in bytes						<br>
 *												Max size: (64 - index)
 *  \param[in]	pBuffer							Buffer that will be written to IG device's memory
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetUserBuffer(SbgProtocolHandle handle, uint16 index, uint16 size, const void *pBuffer)
{
	uint8 frame[504];
	SbgErrorCode error;
	
	//
	// Check that user buffer is allocated
	//
	if (pBuffer)
	{
		//
		// Check that a single frame can contain the user buffer
		//
		if (size <= 500)
		{
			//
			// Build the frame
			//
			frame[0] = 0;
			*((uint16*)(frame+sizeof(uint8))) = sbgHostToTarget16(handle->targetOutputMode, index);
			*((uint16*)(frame+sizeof(uint8)+sizeof(uint16))) = sbgHostToTarget16(handle->targetOutputMode, size);
			memcpy(frame+sizeof(uint8)+2*sizeof(uint16), pBuffer, size);

			//
			// And send it with command SBG_SET_USER_BUFFER
			//
			error = sbgProtocolSend(handle, SBG_SET_USER_BUFFER, frame, size + 2*sizeof(uint16) + sizeof(uint8));

			if (error == SBG_NO_ERROR)
			{
				//
				// We should receive an ACK if the user buffer has been sucessfully written.
				//
				error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
			}
		}
		else
		{
			error = SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Read a user buffer in the IG device's memory
 *	\param[in]	handle							A valid sbgCom library handle.
 *  \param[out]	pBuffer							Buffer that will be read from the IG device's memory. Needs to be preallocated
 *	\param[in]	index							Address in the user memory where the data has to be read.	<br>
 *												Max address: 0x3F (64 bytes)
 *	\param[in]	size							Size of the user buffer to read in bytes					<br>
 *												Max size: (64 - index)
 */
SbgErrorCode sbgGetUserBuffer(SbgProtocolHandle handle, void *pBuffer, uint16 index, uint16 size)
{
	SbgErrorCode error;
	uint8 frame[504];
	uint16 frameSize;
	uint8 cmd;

	//
	// Check that the user buffer is allocated
	//
	if (pBuffer)
	{
		//
		// Check buffer size
		//
		if (size <= SBG_MAX_DATA_LENGTH)
		{
			//
			// Build the frame
			//
			*((uint16*)(frame)) = sbgHostToTarget16(handle->targetOutputMode, index);
			*((uint16*)(frame+sizeof(uint16))) = sbgHostToTarget16(handle->targetOutputMode, size);

			//
			// And send it with command SBG_GET_USER_BUFFER
			//
			error = sbgProtocolSend(handle, SBG_GET_USER_BUFFER, frame, 2*sizeof(uint16));

			if (error == SBG_NO_ERROR)
			{
				//
				// Question were sent. Wait for an answer
				// Try to read the answer
				//
				error = sbgProtocolReceiveTimeOut(handle, &cmd, frame, &frameSize, size);

				if (error == SBG_NO_ERROR)
				{
					if ( (cmd == SBG_RET_USER_BUFFER) && (frameSize ==  size) )
					{
						//
						// Fill output for user
						//
						memcpy(pBuffer, frame, size);
					}
					else if ( cmd == SBG_ACK)
					{
						//
						// Get the error code contained in the ACK frame
						//
						error = (SbgErrorCode)(frame[0]);

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
		}
		else
		{
			error = SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Restore all settings to factory defaults (excepted for calibration data such as gyros bias and magnetometers calibration).<br>
 *	The device baud rate is reseted to 115200 bauds.<br>
 *	This command dosen't change the sbgCom library baud rate.<br>
 *	To change only the baud rate used by the library, you have to use the function sbgProtocolChangeBaud.<br>
 *	You should also call sbgGetOutputMode to update the output mode format. <br>
  * This command forces the IMU to reboot so the system may not respond for some time after this function call
 *	\param[in]	handle							A valid sbgCom library handle.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgRestoreDefaultSettings(SbgProtocolHandle handle)
{
	uint8 buffer[sizeof(uint8)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;

	//
	// Send the command used to restore the default settings
	//
	error = sbgProtocolSend(handle, SBG_RESTORE_DEFAULT_SETTINGS, buffer, sizeof(uint8));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 *Save current settings into the flash non volatile memory
 * \param[in]	handle							A valid sbgCom library handle.
 * \return										SBG_NO_ERROR in case of a good operation
 */
SbgErrorCode sbgSaveSettings(SbgProtocolHandle handle)
{
	SbgErrorCode error;

	//
	// Send the command used to save configuration into flash memory
	//
	error = sbgProtocolSend(handle, SBG_SAVE_SETTINGS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 * This command set advanced settings options
 * \param[in]	handle							A valid sbgCom library handle.
 * \param[in]	advancedOptions					Options bit mask
 * \return										SBG_NO_ERROR in case of a good operation
 */
SbgErrorCode sbgSetAdvancedOptions(SbgProtocolHandle handle, uint32 advancedOptions)
{
	uint32 buffer[2];
	SbgErrorCode error;

	//
	// Format output buffer
	//
	buffer[0] = sbgHostToTarget32(handle->targetOutputMode, advancedOptions);
	buffer[1] = 0;

	//
	// Send the command used to save configuration into flash memory
	//
	error = sbgProtocolSend(handle, SBG_SET_ADVANCED_OPTIONS, buffer, sizeof(buffer));

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to receive an ACK
		//
		error = sbgWaitForAck(handle, SBG_FRAME_RECEPTION_TIME_OUT);
	}

	return error;
}

/*!
 * This command is used to retrieve advanced options from the device
 * \param[in]	handle							A valid sbgCom library handle.
 * \param[in]	pAdvancedOptions				Options bit mask
 * \return										SBG_NO_ERROR in case of a good operation
 */
SbgErrorCode sbgGetAdvancedOptions(SbgProtocolHandle handle, uint32 *pAdvancedOptions)
{
	uint8 cmd;
	uint16 size;
	uint32 options[2];
	SbgErrorCode error;

	//
	// Send the command used to get the advanced options
	//
	error = sbgProtocolSend(handle, SBG_GET_ADVANCED_OPTIONS, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, options, &size, sizeof(options));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_ADVANCED_OPTIONS) && (size == sizeof(options)) )
			{
				//
				// Returns the user id value
				//
				if (pAdvancedOptions)
				{
					*pAdvancedOptions = sbgTargetToHost32(handle->targetOutputMode, options[0]);
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
