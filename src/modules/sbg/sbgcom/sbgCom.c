#include "sbgCom.h"
#include "sbgComVersion.h"
#include <string.h>
#include <stdio.h>

//----------------------------------------------------------------------//
//- sbgCom library main operations                                     -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the sbgCom library.<br>
 *	Open the COM port and try to get the default output mask and output mode from the device.<br>
 *	\param[in]	deviceName						Communication port to open ("COM1" on Windows, "/dev/ttysX" on UNIX platforms).
 *	\param[in]	baudRate						Baud rate used to communicate with the device.<br>
 *												Possible values are:<br>
 *												- 9600<br>
 *												- 19200<br>
 *												- 38400<br>
 *												- 57600<br>
 *												- 115200<br>
 *												- 230400<br>
 *												- 460800<br>
 *												- 921600<br>
 *	\param[out]	pHandle							Used to returns the created SbgProtocolHandle struct.
 *	\return										SBG_NO_ERROR if a connection with the device has been established.
 */
SbgErrorCode sbgComInit(const char *deviceName, uint32 baudRate, SbgProtocolHandle *pHandle)
{
	SbgErrorCode errorCode;
	uint32 i;

	//
	// Try to open COM port
	//
	errorCode = sbgProtocolInit(deviceName, baudRate, pHandle);

	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Try to retrive output mode of the product
		//
		for (i=0; i<5; i++)
		{
			errorCode = sbgGetOutputMode(*pHandle, NULL);

			if (errorCode == SBG_NO_ERROR)
			{
				break;
			}

			//
			// Wait 100 ms before retrying
			//
			sbgSleep(100);
			sbgProtocolFlush(*pHandle);
		}

		//
		// Check if we were able to get output mode
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to retrive default output mask
			//
			for (i=0; i<5; i++)
			{
				errorCode = sbgGetDefaultOutputMask(*pHandle, NULL);

				if (errorCode == SBG_NO_ERROR)
				{
					break;
				}

				//
				// Wait 100 ms before retrying
				//
				sbgSleep(100);
				sbgProtocolFlush(*pHandle);
			}
		}

		//
		// If we have an error, close the serial com port
		//
		if (errorCode != SBG_NO_ERROR)
		{
			sbgComClose(*pHandle);
		}
	}

	return errorCode;
}

/*!
 *	Close the COM port and release memory for the sbgCom library.
 *	\param[in]	handle							The sbgCom library handle to release.
 *	\return										SBG_NO_ERROR if we have released the handle.
 */
SbgErrorCode sbgComClose(SbgProtocolHandle handle)
{
	return sbgProtocolClose(handle);
}

/*!
 *	Returns an integer representing the version of the sbgCom library.
 *	\return										An integer representing the version of the sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgComGetVersion(void)
{
	return SBG_COM_VERSION_U;
}

/*!
 *	Retreive the sbgCom library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCom library version.
 */
const char *sbgComGetVersionAsString(void)
{
	return SBG_COM_VERSION;
}

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgComErrorToString(SbgErrorCode errorCode, char errorMsg[256])
{
	if (errorMsg)
	{
		//
		// For each error code, copy the error msg
		//
		switch (errorCode)
		{
		case SBG_NO_ERROR:
			strcpy(errorMsg, "SBG_NO_ERROR: No error."); 
			break;
		case SBG_ERROR:
			strcpy(errorMsg, "SBG_ERROR: Generic error."); 
			break;
		case SBG_NULL_POINTER:
			strcpy(errorMsg, "SBG_NULL_POINTER: A pointer is null."); 
			break;
		case SBG_INVALID_CRC:
			strcpy(errorMsg, "SBG_INVALID_CRC: The received frame has an invalid CRC.");
			break;
		case SBG_INVALID_FRAME:
			strcpy(errorMsg, "SBG_INVALID_FRAME: The received frame is invalid.");
			break;
		case SBG_TIME_OUT:
			strcpy(errorMsg, "SBG_TIME_OUT: We have a time out during frame reception.");
			break;
		case SBG_WRITE_ERROR:
			strcpy(errorMsg, "SBG_WRITE_ERROR: All bytes hasn't been written.");
			break;
		case SBG_READ_ERROR:
			strcpy(errorMsg, "SBG_READ_ERROR: All bytes hasn't been read.");
			break;
		case SBG_BUFFER_OVERFLOW:
			strcpy(errorMsg, "SBG_BUFFER_OVERFLOW: A buffer is too small to contain so much data.");
			break;
		case SBG_INVALID_PARAMETER:
			strcpy(errorMsg, "SBG_INVALID_PARAMETER: An invalid parameter has been founded.");
			break;
		case SBG_NOT_READY:
			strcpy(errorMsg, "SBG_NOT_READY: A device isn't ready (Rx isn't ready for example).");
			break;
		case SBG_MALLOC_FAILED:
			strcpy(errorMsg, "SBG_MALLOC_FAILED: Failed to allocate a buffer.");
			break;
		case SGB_CALIB_MAG_NOT_ENOUGH_POINTS:
			strcpy(errorMsg, "SGB_CALIB_MAG_NOT_ENOUGH_POINTS: Not enough points were available to perform magnetometers calibration.");
			break;
		case SBG_CALIB_MAG_INVALID_TAKE:
			strcpy(errorMsg, "SBG_CALIB_MAG_INVALID_TAKE: The calibration procedure could not be properly executed due to insufficient precision.");
			break;
		case SBG_CALIB_MAG_SATURATION:
			strcpy(errorMsg, "SBG_CALIB_MAG_SATURATION: Saturation were detected when attempt to calibrate magnetos.");
			break;
		case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE:
			strcpy(errorMsg, "SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE: 2D calibration procedure could not be performed.");
			break;
		case SBG_DEVICE_NOT_FOUND:
			strcpy(errorMsg, "SBG_DEVICE_NOT_FOUND: A device couldn't be founded or opened.");
			break;
		case SBG_OPERATION_CANCELLED:
			strcpy(errorMsg, "SBG_OPERATION_CANCELLED: An operation has been cancelled by a user.");
			break;
		case SBG_NOT_CONTINUOUS_FRAME:
			strcpy(errorMsg, "SBG_NOT_CONTINUOUS_FRAME: We have received a frame that isn't a continuous one.");
			break;
		case SBG_INCOMPATIBLE_HARDWARE:
			strcpy(errorMsg, "SBG_INCOMPATIBLE_HARDWARE: Hence valid, the configuration cannot be executed because of incompatible hardware.");
			break;
		default:
			sprintf(errorMsg, "Undefined error code: %u", errorCode);
			break;
		}
	}
}
