#include "commandsExt.h"
#include "protocolOutputMode.h"

/*!
 *	Select the external device connected to the IG-500E and its UART configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	deviceType			The device type connected
 *  \param[in]	baudRate			The baud rate used to communicate with the device at initialization
 *  \param[in]	uartOptions			Some uart Options (bitwize ORed) needed to communicate with the device. Restart the device if changed:
 *									- SBG_EXT_PORT_RS232 or SBG_EXT_PORT_RS422
 *									- SBG_EXT_PORT_DIS_EMI_REDUCTION or SBG_EXT_PORT_EN_EMI_REDUCTION
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetExtDevice(SbgProtocolHandle handle, SbgExtDeviceType deviceType, uint32 baudRate, uint16 uartOptions)
{
	uint8 buffer[2*sizeof(uint8)+sizeof(uint32)+sizeof(uint16)];
	SbgErrorCode error;

	//
	// Build the arguments buffer
	//
	*buffer = 0;
	buffer[1] = (uint8)deviceType;
	*(uint32*)(buffer + 2*sizeof(uint8)) = sbgHostToTarget32(handle->targetOutputMode, baudRate);
	*(uint16*)(buffer + 2*sizeof(uint8) + sizeof(uint32)) = sbgHostToTarget16(handle->targetOutputMode, uartOptions);

	//
	// Send the command used to set the external device
	//
	error = sbgProtocolSend(handle, SBG_SET_EXTERNAL_DEVICE, buffer, 2*sizeof(uint8)+sizeof(uint32)+sizeof(uint16));

	if (error == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		error = sbgWaitForAck(handle, 5000);
	}
	
	return error;
}

/*!
 *	Get the external device connected to the IG-500E and its UART configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pDeviceType			The device type connected
 *  \param[out]	pBaudRate			The baud rate used to communicate with the device at initialization
 *  \param[out]	pUartOptions		Some uart Options (bitwize ORed) used to communicate with the device
 *									- SBG_EXT_PORT_RS232 or SBG_EXT_PORT_RS422
 *									- SBG_EXT_PORT_DIS_EMI_REDUCTION or SBG_EXT_PORT_EN_EMI_REDUCTION
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetExtDevice(SbgProtocolHandle handle, SbgExtDeviceType *pDeviceType, uint32 *pBaudRate, uint16 *pUartOptions)
{
	uint8 cmd;
	uint16 size;
	uint8 frame[sizeof(uint8)+sizeof(uint32)+sizeof(uint16)];
	SbgErrorCode error;

	//
	// Send the command used to get the external device params
	//
	error = sbgProtocolSend(handle, SBG_GET_EXTERNAL_DEVICE, NULL, 0);

	if (error == SBG_NO_ERROR)
	{
		//
		// Try to read the answer
		//
		error = sbgProtocolReceiveTimeOut(handle, &cmd, frame, &size, sizeof(uint8)+sizeof(uint32)+sizeof(uint16));

		if (error == SBG_NO_ERROR)
		{
			if ( (cmd == SBG_RET_EXTERNAL_DEVICE) && (size == sizeof(uint8)+sizeof(uint32)+sizeof(uint16)) )
			{
				//
				// Returns, if possible, the output parameters
				//
				if (pDeviceType)
				{
					*pDeviceType = (SbgExtDeviceType)(frame[0]);
				}
				if (pBaudRate)
				{
					*pBaudRate = sbgTargetToHost32(handle->targetOutputMode, *(uint32*)(frame+sizeof(uint8)));
				}
				if (pUartOptions)
				{
					*pUartOptions = sbgTargetToHost16(handle->targetOutputMode, *(uint16*)(frame+sizeof(uint8)+sizeof(uint32)));
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
 *	Send to the external device attached a specific configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	pCommand			Command sent to the external device
 *  \param[in]	commandSize			Size of the command to send
 *  \param[out]	pAnswer				Answer of the external device
 *  \param[out]	pAnswerSize			Size of the device answer
 *  \param[in]	ansMaxSize			Maximum allowed size of the answer
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtDeviceConfig(SbgProtocolHandle handle, const uint8 *pCommand, uint16 commandSize, uint8 *pAnswer, uint16 *pAnswerSize, uint16 ansMaxSize)
{
	SbgErrorCode error;
	uint8 cmd;

	//
	// Check pointers and command size and send directly the command through the protocol
	//
	if ( (pCommand) && (pAnswer) && (pAnswerSize) )
	{
		if (commandSize < SBG_MAX_DATA_LENGTH)
		{
			//
			// Send the command used to send an external device command
			//
			error = sbgProtocolSend(handle, SBG_SET_EXTERNAL_DEVICE_CONF, pCommand, commandSize);

			if (error == SBG_NO_ERROR)
			{
				//
				// Try to read the answer and directly feed user output pointers
				//
				error = sbgProtocolReceiveTimeOut(handle, &cmd, pAnswer, pAnswerSize, ansMaxSize);

				//
				// Discriminate any answer which is not the good frame
				//
				if ((error == SBG_NO_ERROR) && (cmd != SBG_RET_EXTERNAL_DEVICE_CONF))
				{
					error = SBG_INVALID_FRAME;
				}
			}
		}
		else
		{
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}
	
	return error;
}
