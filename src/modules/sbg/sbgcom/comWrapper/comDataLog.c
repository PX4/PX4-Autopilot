#include "comWrapper.h"
#include <stdio.h>


/*!
 * Open the specified device at a specified baud and create a new device handle
 * \param[in]	deviceName			Device name / address (COM21 , /dev/ttys0, depending on platform)
 * \param[in]	baudRate			Baudrate to be used
 * \param[out]	pHandle				Device handle returned
 * \return							SBG_NO_ERROR if the device could be oppened properly
 */
SbgErrorCode sbgDeviceOpen(const char *deviceName, uint32 baudRate, SbgDeviceHandle *pHandle)
{
	SbgErrorCode error;
	FILE *fp;

	
	// Avoid warnings
	baudRate;


	//
	// First, check input pointers
	//
	if ((deviceName) && (pHandle))
	{
	
		//
		// Then try to open the Log file
		//
		fp = fopen(deviceName,"rb");

		//
		// Check file open result
		//
		if (fp)
		{
			// File opened. We have the file handle
			error = SBG_NO_ERROR;
			*pHandle = (void*)fp;
		}
		else
		{
			// File not found
			error = SBG_DEVICE_NOT_FOUND;
			*pHandle = NULL;
		}
	}
	else
	{
		// Invalid parameter / null pointers
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 * Close the device
 * \param[in]	handle				Device handle to be closed
 * \return							SBG_NO_ERROR if the device could be closed properly
 */
SbgErrorCode sbgDeviceClose(SbgDeviceHandle handle)
{
	SbgErrorCode error;
	//
	// Check parameter
	//
	if (handle)
	{
		//
		// Try to Close file
		//
		if (fclose((FILE*)handle) == 0)
		{
			error = SBG_NO_ERROR;
		}
		else
		{
			error = SBG_ERROR;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 * Change the baud rate out the opened device
 * \param[in]	handle				Device handle returned
 * \param[in]	baudRate			New baudrate to apply on COM port
 * \return							SBG_NO_ERROR if baudrate could be changed properly
 */
SbgErrorCode sbgDeviceChangeBaud(SbgDeviceHandle handle, uint32 baudRate)
{
	// Avoid warnings
	baudRate;
	handle;

	//
	// Dummy function
	//
	return SBG_ERROR;
}

/*!
 * Write some bytes to the tx queue
 * \param[in]	handle				Device handle returned
 * \param[in]	pBuffer				Buffer to be sent through interface
 * \param[in]	numBytesToWrite		Size of the buffer in bytes
 * \return							SBG_NO_ERROR if write could be done
 */
SbgErrorCode sbgDeviceWrite(SbgDeviceHandle handle, const void *pBuffer, uint32 numBytesToWrite)
{
	pBuffer;
	numBytesToWrite;
	handle;
	//
	// Dummy function
	//
	return SBG_ERROR;
}

/*!
 * Read some bytes from the rx queue
 * \param[in]	handle				Device handle returned
 * \param[in]	pBuffer				Buffer to be sent through interface
 * \param[in]	numBytesToRead		Max number of bytes to be read
 * \param[out]	pNumBytesRead		Actual number read by the function
 * \return							SBG_NO_ERROR if one or more bytes read
 */
SbgErrorCode sbgDeviceRead(SbgDeviceHandle handle, void *pBuffer, uint32 numBytesToRead, uint32 *pNumBytesRead)
{
	SbgErrorCode error;

	//
	// Check input parameters
	//
	if ((handle) && (pBuffer) && (pNumBytesRead))
	{
		//
		// Then try to read the file
		//
		*pNumBytesRead = fread(pBuffer,sizeof(uint8),numBytesToRead,(FILE*)handle);

		//
		// if no byte were read and the user wanted 1 or more byte to be read, return appropriate error code
		//
		if (!(*pNumBytesRead) && (numBytesToRead))
		{
			// No more unread byte in the file
			error = SBG_READ_ERROR;
		}
		else
		{
			// At least 1 byte could be read (or user didn't want any byte to be read)
			error = SBG_NO_ERROR;
		}
	}
	else
	{
		// Invalid param
		error = SBG_NULL_POINTER;
	}
	return error;
}

/*!
 * Flush the RX and TX buffers (remove all old data)
 * \param[in]	handle				Device handle returned
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgDeviceFlush(SbgDeviceHandle handle)
{
	// Avoid warnings
	handle;

	//
	// Dummy function
	//
	return SBG_ERROR;
}

/*!
 *	Defines the serial DTR and RTS pins states.
 *	\param[in]	handle				The serial communication handle.
 *	\param[in]	function			One of the SbgEscapeComm enum function.
 *	\return							SBG_NO_ERROR if the pin state has been defined.
 */
SbgErrorCode sbgSetEscapeComm(SbgDeviceHandle handle, SbgEscapeComm function)
{
	// Avoid warnings
	handle;
	function;

	//
	// Dummy function
	//
	return SBG_ERROR;
}