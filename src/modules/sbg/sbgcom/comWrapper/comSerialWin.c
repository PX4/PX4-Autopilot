#include "comWrapper.h"
#include "../time/sbgTime.h"
#include <stdio.h>
#include <windows.h>

//------------------------------------------------------------------------------//
//- SBG Device operations                                                      -//
//------------------------------------------------------------------------------//

// Returns the last error message for windows api calls
uint32 sbgGetWindowsErrorMsg(char outErrorMsg[256])
{
	DWORD dw = GetLastError(); 
	LPVOID lpMsgBuf;

	//
	// Get the error message
	//
	FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR)&lpMsgBuf, 0, NULL);

	//
	// Copy the error message
	//
	strcpy_s(outErrorMsg, 256, lpMsgBuf);

	//
	// Release the buffer
	//
	LocalFree(lpMsgBuf);

	return dw;
}

/*!
 * Open the specified device at a specified baud and create a new device handle
 * \param[in]	deviceName			Device name / address (COM21 , /dev/ttys0, depending on platform)
 * \param[in]	baudRate			Baudrate to be used
 * \param[out]	pHandle				Device handle returned
 * \return							SBG_NO_ERROR if the device could be oppened properly
 */
SbgErrorCode sbgDeviceOpen(const char *deviceName, uint32 baudRate, SbgDeviceHandle *pHandle)
{
	char errorMsg[256];
	char comPortPath[32];
	COMMTIMEOUTS comTimeOut;
	DCB comState;
	uint32 deviceNum;
	HANDLE hSerialDevice;

	//
	// First check if we have a valid pHandle
	//
	if (pHandle)
	{
		//
		// Extract device number
		//
		if (sscanf_s(deviceName, "COM%i", &deviceNum) == 1)
		{
			//
			// Build the com port path
			//
			sprintf_s(comPortPath, 32, "\\\\.\\COM%i", deviceNum);

			//
			// Init the com port
			//
			hSerialDevice = CreateFile(comPortPath, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

			if (hSerialDevice != INVALID_HANDLE_VALUE)
			{
				//
				// Define the device handle
				//
				*pHandle = hSerialDevice;

				//
				// Purge the com port
				//
				if (PurgeComm(hSerialDevice, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
				{
					//
					// Retreives current com state and com timeout
					//
					if ( (GetCommState(hSerialDevice, &comState)) && (GetCommTimeouts(hSerialDevice, &comTimeOut)) )
					{
						//
						// Define common attributes
						//
						comState.BaudRate= baudRate;
						comState.Parity= NOPARITY;
						comState.ByteSize= 8;
						comState.StopBits= ONESTOPBIT;

						//
						// Disable flow control
						//
						comState.fDsrSensitivity = FALSE;
						comState.fOutxCtsFlow = FALSE;
						comState.fOutxDsrFlow = FALSE;
						comState.fOutX = FALSE;
						comState.fInX = FALSE;

						//
						// Define timeout attributes (0 ms read timeout)
						//
						comTimeOut.ReadIntervalTimeout = MAXDWORD;
						comTimeOut.ReadTotalTimeoutMultiplier = 0;
						comTimeOut.ReadTotalTimeoutConstant = 0;

						comTimeOut.WriteTotalTimeoutConstant = 0;
						comTimeOut.WriteTotalTimeoutMultiplier = 0;
						

						//
						// Configure the com port
						//
						if ( (SetCommState(hSerialDevice, &comState)) && (SetCommTimeouts(hSerialDevice, &comTimeOut)) )
						{
							//
							// Wait until the com port has been configured by windows
							//
							sbgSleep(60);

							//
							// Define the COM port buffer size
							//
							if (SetupComm(hSerialDevice, SBG_SERIAL_RX_BUFFER_SIZE, SBG_SERIAL_TX_BUFFER_SIZE))
							{
								//
								// Purge the communication
								//
								return sbgDeviceFlush(hSerialDevice);
							}
							else
							{
								sbgGetWindowsErrorMsg(errorMsg);
								fprintf(stderr, "sbgDeviceOpen: Unable to define buffer size: %s.\n", errorMsg);
							}
						}
						else
						{
							sbgGetWindowsErrorMsg(errorMsg);
							fprintf(stderr, "sbgDeviceOpen: Unable to set com state and/or timeout: %s.\n", errorMsg);
						}
					}
					else
					{
						sbgGetWindowsErrorMsg(errorMsg);
						fprintf(stderr, "sbgDeviceOpen: Unable to retreive com state and/or timeout: %s.\n", errorMsg);
					}
				}
				else
				{
					sbgGetWindowsErrorMsg(errorMsg);
					fprintf(stderr, "sbgDeviceOpen: Unable to purge com port %i: %s.\n", deviceNum, errorMsg);
				}

				//
				// Close the device if only some part has been initialised
				//
				sbgDeviceClose(hSerialDevice);
			}
			else
			{
				//
				// We have an invalid device handle
				//
				*pHandle = SBG_INVALID_DEVICE_HANDLE;
				
				sbgGetWindowsErrorMsg(errorMsg);
				fprintf(stderr, "sbgDeviceOpen: Unable to open com port %i: %s\n", deviceNum, errorMsg);
			}

			return SBG_ERROR;
		}
		else
		{
			fprintf(stderr, "sbgDeviceOpen: Invalid deviceName: %s\n", deviceName);
			return SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		fprintf(stderr, "sbgDeviceOpen: pHandle == NULL for deviceName: %s\n", deviceName);
		return SBG_NULL_POINTER;
	}
}

/*!
 * Close the device
 * \param[in]	handle				Device handle to be closed
 * \return							SBG_NO_ERROR if the device could be closed properly
 */
SbgErrorCode sbgDeviceClose(SbgDeviceHandle handle)
{
	HANDLE pSerialDevice = (HANDLE)handle;

	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Close the port com
		//
		CloseHandle(pSerialDevice);
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 * Change the baud rate out the opened device
 * \param[in]	handle				Device handle returned
 * \param[in]	baudRate			New baudrate to apply on COM port
 * \return							SBG_NO_ERROR if baudrate could be changed properly
 */
SbgErrorCode sbgDeviceChangeBaud(SbgDeviceHandle handle, uint32 baudRate)
{
	DCB comState;
	SbgErrorCode error = SBG_ERROR;
	HANDLE pSerialDevice = (HANDLE)handle;
	char errorMsg[256];

	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Try to retreive current com state
		//
		if (GetCommState(pSerialDevice, &comState))
		{
			//
			// Change the baud rate
			//
			comState.BaudRate = baudRate;

			//
			// Configure the com state
			//
			if (SetCommState(pSerialDevice, &comState))
			{
				//
				// Wait until the com port has been configured by windows
				//
				sbgSleep(60);

				error = SBG_NO_ERROR;
			}
			else
			{
				sbgGetWindowsErrorMsg(errorMsg);
				fprintf(stderr, "sbgDeviceChangeBaud: Unable to set com state: %s.\n", errorMsg);
			}
		}
		else
		{
			sbgGetWindowsErrorMsg(errorMsg);
			fprintf(stderr, "sbgDeviceChangeBaud: Unable to retreive com state: %s.\n", errorMsg);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
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
	uint32 numBytesWritten;
	uint32 numBytesLeftToWrite = numBytesToWrite;
	uint8 *pCurrentBuffer = (uint8*)pBuffer;
	HANDLE pSerialDevice = (HANDLE)handle;
	char errorMsg[256];

	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		if (pBuffer)
		{
			//
			// Write all the buffer
			//
			while (numBytesLeftToWrite>0)
			{
				if (!WriteFile(pSerialDevice, pCurrentBuffer, numBytesLeftToWrite, (LPDWORD)&numBytesWritten, NULL))
				{
					sbgGetWindowsErrorMsg(errorMsg);
					fprintf(stderr, "sbgDeviceWrite: Write failed error: %s.\n", errorMsg);
					return SBG_WRITE_ERROR;
				}

				//
				// Update the buffer pointer and the number of bytes to write
				//
				numBytesLeftToWrite -= numBytesWritten;
				pCurrentBuffer += numBytesWritten;
			}

			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_NULL_POINTER;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
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
	HANDLE pSerialDevice = (HANDLE)handle;
	char errorMsg[256];

	if ( (handle != SBG_INVALID_DEVICE_HANDLE) && (pNumBytesRead) )
	{
		if (ReadFile(pSerialDevice, pBuffer, numBytesToRead, (LPDWORD)pNumBytesRead, NULL))
		{
			return SBG_NO_ERROR;
		}
		else
		{
			sbgGetWindowsErrorMsg(errorMsg);
			fprintf(stderr, "sbgDeviceRead: Read failed: %s.\n", errorMsg);
			return SBG_READ_ERROR;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 * Flush the RX and TX buffers (remove all old data)
 * \param[in]	handle				Device handle returned
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgDeviceFlush(SbgDeviceHandle handle)
{
	HANDLE pSerialDevice = (HANDLE)handle;
	char errorMsg[256];

	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		if (PurgeComm(pSerialDevice, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
		{
			return SBG_NO_ERROR;
		}
		else
		{
			sbgGetWindowsErrorMsg(errorMsg);
			fprintf(stderr, "sbgDeviceFlush: PurgeComm failed: %s.\n", errorMsg);
			return SBG_ERROR;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Defines the serial DTR and RTS pins states.
 *	\param[in]	handle				The serial communication handle.
 *	\param[in]	function			One of the SbgEscapeComm enum function.
 *	\return							SBG_NO_ERROR if the pin state has been defined.
 */
SbgErrorCode sbgSetEscapeComm(SbgDeviceHandle handle, SbgEscapeComm function)
{
	HANDLE pSerialDevice = (HANDLE)handle;
	uint32 escapeCommFunction;

	//
	// Check if we have a valid serial connexion
	//
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Translate the sbgCom espace function into the windows one
		//
		switch (function)
		{
		case SBG_SET_DTR:
			escapeCommFunction = SETDTR;
			break;
		case SBG_CLR_DTR:
			escapeCommFunction = CLRDTR;
			break;
		case SBG_SET_RTS:
			escapeCommFunction = SETRTS;
			break;
		case SBG_CLR_RTS:
		default:
			escapeCommFunction = CLRRTS;
			break;
		}

		//
		// Define the espace comm function
		//
		if (EscapeCommFunction(pSerialDevice, escapeCommFunction))
		{
			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_ERROR;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}
