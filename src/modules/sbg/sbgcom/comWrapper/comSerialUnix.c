#include "comWrapper.h"
#include "../time/sbgTime.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
uint32 sbgDeviceGetBaudRateConst(uint32 baudRate);
SbgErrorCode sbgDeviceOpen(const char *deviceName, uint32 baudRate, SbgDeviceHandle *pHandle);
SbgErrorCode sbgDeviceClose(SbgDeviceHandle handle);
SbgErrorCode sbgDeviceChangeBaud(SbgDeviceHandle handle, uint32 baudRate);
SbgErrorCode sbgDeviceWriteByte(SbgDeviceHandle handle, uint8 value);
SbgErrorCode sbgDeviceReadByte(SbgDeviceHandle handle, uint8 *pValue);
SbgErrorCode sbgDeviceWrite(SbgDeviceHandle handle, const void *pBuffer, uint32 numBytesToWrite);
SbgErrorCode sbgDeviceRead(SbgDeviceHandle handle, void *pBuffer, uint32 numBytesToRead, uint32 *pNumBytesRead);
SbgErrorCode sbgDeviceFlush(SbgDeviceHandle handle);
//------------------------------------------------------------------------------//
//- SBG Device operations                                                      -//
//------------------------------------------------------------------------------//

/*!
 *	Returns the right unix baud rate const according to a baud rate value.
 *	\param[in] baudRate		Our baud rate value (ie 115200).
 *	\return					Our Unix baud rate constante.
 */
uint32 sbgDeviceGetBaudRateConst(uint32 baudRate)
{
	uint32 baudRateConst;
	
	//
	// Create our right baud rate value for unix platforms
	//
	switch (baudRate)
	{
		case 9600:
			baudRateConst = B9600;
			break;
		case 19200:
			baudRateConst = B19200;
			break;
#ifdef B38400
		case 38400:
			baudRateConst = B38400;
			break;
#endif
#ifdef B57600
		case 57600:
			baudRateConst = B57600;
			break;
#endif
#ifdef B115200
		case 115200:
			baudRateConst = B115200;
			break;
#endif
#ifdef B230400
		case 230400:
			baudRateConst = B230400;
			break;
#endif
#ifdef B460800
		case 460800:
			baudRateConst = B460800;
			break;
#endif
#ifdef B921600
		case 921600:
			baudRateConst = B921600;
			break;
#endif
		default:
			baudRateConst = baudRate;
	}

	return baudRateConst;
}

/// Open the specified device at a specified baud and create a new device handle
SbgErrorCode sbgDeviceOpen(const char *deviceName, uint32 baudRate, SbgDeviceHandle *pHandle)
{
	SbgErrorCode errorCode = SBG_ERROR;
	struct termios options;
	uint32 baudRateConst;
	int32 fileId;
	
	//
	// Check if we have a valid handle
	//
	if (pHandle)
	{
		//
		// Check if we have a valid deviceName
		//
		if (deviceName)
		{
			//
			// Get our baud rate const for our Unix platform
			//
			baudRateConst = sbgDeviceGetBaudRateConst(baudRate);
			
			//
			// Open our com port
			//
			fileId = open(deviceName, O_RDWR | O_NOCTTY | O_NDELAY);
			
			if (fileId != -1)
			{
				//
				// Sotre our file id into our pHandle
				//
				*((int32*)pHandle) = fileId;
				
				//
				// Don't block on read call if no data are available
				//
				if (fcntl(fileId, F_SETFL, O_NONBLOCK) != -1)
				{
					//
					// Retreive current options
					//
					if (tcgetattr(fileId, &options) != -1)
					{
						//
						// Define com port options
						//
						options.c_cflag |=  (CLOCAL | CREAD);		// Enable the receiver and set local mode...
						options.c_cflag &= ~(PARENB|CSTOPB|CSIZE);	// No parity, 1 stop bit, mask character size bits
						options.c_cflag |= CS8;						// Select 8 data bits
						options.c_cflag &= ~CRTSCTS;				// Disable Hardware flow control
						
						//
						// Disable software flow control
						//
						options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
						
						//
						// We would like raw input
						//
						options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG /*| IEXTEN | ECHONL*/);
						options.c_oflag &= ~OPOST;

						//
						// Set our timeout to 0
						//
						options.c_cc[VMIN]     = 0;
						options.c_cc[VTIME]    = 1;
						
						//
						// Set both input and output baud
						//
						if ( (cfsetispeed(&options, baudRateConst) != -1)  && (cfsetospeed(&options, baudRateConst) != -1) )
						{
							//
							// Define options
							//
							if (tcsetattr(fileId, TCSANOW, &options) != -1)
							{								
								//
								// Flush our port com
								//
								return sbgDeviceFlush(pHandle);
							}
							else
							{
								fprintf(stderr, "sbgDeviceOpen: tcsetattr fails.\n");
							}
						}
						else
						{
							fprintf(stderr, "sbgDeviceOpen: Unable to set speed.\n");
						}
					}
					else
					{
						fprintf(stderr, "sbgDeviceOpen: tcgetattr fails.\n");
					}
				}
				else
				{
					fprintf(stderr, "sbgDeviceOpen: fcntl fails\n");
				}
				
				//
				// Close our device if only some part has been initialised
				//
				sbgDeviceClose(pHandle);
			}
			else
			{
				fprintf(stderr, "sbgDeviceOpen: Unable to open com port: %s\n", deviceName);
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
		else
		{
			fprintf(stderr, "sbgDeviceOpen: Invalid device name.\n");
			errorCode = SBG_NULL_POINTER;
		}
		
		//
		// We have an invalid device handle
		//
		*pHandle = SBG_INVALID_DEVICE_HANDLE;
	}
	else
	{
		fprintf(stderr, "sbgDeviceOpen: pHandle == NULL.\n");
		errorCode = SBG_NULL_POINTER;
	}
	
	return errorCode;
}

/// Close the device
SbgErrorCode sbgDeviceClose(SbgDeviceHandle handle)
{
	int32 fileId = *((int32*)&handle);
	
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Close our port com
		//
		close(fileId);

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/// Change the baud rate out our opened device
SbgErrorCode sbgDeviceChangeBaud(SbgDeviceHandle handle, uint32 baudRate)
{
	int32 fileId = *((int32*)&handle);
	struct termios options;
	uint32 baudRateConst;

	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Get our baud rate const for our Unix platform
		//
		baudRateConst = sbgDeviceGetBaudRateConst(baudRate);
		
		//
		// Retreive current options
		//
		if (tcgetattr(fileId, &options) != -1)
		{	
			//
			// Set both input and output baud
			//
			if ( (cfsetispeed(&options, baudRateConst) == -1)  || (cfsetospeed(&options, baudRateConst) == -1) )
			{
				fprintf(stderr, "sbgDeviceChangeBaud: Unable to set speed.\n");
				return SBG_ERROR;
			}
			
			//
			// Define options
			//
			if (tcsetattr(fileId, TCSADRAIN, &options) != -1)
			{
				return SBG_NO_ERROR;
			}
			else
			{
				fprintf(stderr, "sbgDeviceChangeBaud: tcsetattr fails.\n");
				return SBG_ERROR;
			}
		}
		else
		{
			fprintf(stderr, "sbgDeviceChangeBaud: tcgetattr fails.\n");
			return SBG_ERROR;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/// Write one byte to our tx queue
SbgErrorCode sbgDeviceWriteByte(SbgDeviceHandle handle, uint8 value)
{
	return sbgDeviceWrite(handle, &value, 1);
}

/// Read one byte from our rx queue
SbgErrorCode sbgDeviceReadByte(SbgDeviceHandle handle, uint8 *pValue)
{
	int32 fileId = *((int32*)&handle);
	
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		if (read(fileId, pValue, sizeof(uint8)) == sizeof(uint8))
		{
			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_NOT_READY;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/// Write some bytes to our tx queue
SbgErrorCode sbgDeviceWrite(SbgDeviceHandle handle, const void *pBuffer, uint32 numBytesToWrite)
{
	int32 fileId = *((int32*)&handle);
	uint32 numBytesWritten;
	uint32 numBytesLeftToWrite = numBytesToWrite;
	uint8 *pCurrentBuffer = (uint8*)pBuffer;
	
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		if (pBuffer)
		{
			//
			// Write all our buffer
			//
			while (numBytesLeftToWrite>0)
			{
				numBytesWritten = write(fileId, pCurrentBuffer, numBytesToWrite);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"

				if (numBytesWritten == -1)
				{
					fprintf(stderr, "sbgDeviceWrite: Unable to write to our device: %s\n", strerror(errno));
					return SBG_WRITE_ERROR;
				}
#pragma GCC diagnostic pop
				
				//
				// Update our buffer pointer and the number of bytes to write
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

/// Read some bytes from our rx queue
SbgErrorCode sbgDeviceRead(SbgDeviceHandle handle, void *pBuffer, uint32 numBytesToRead, uint32 *pNumBytesRead)
{
	SbgErrorCode errorCode = SBG_ERROR;
	int32 fileId = *((int32*)&handle);
	int32 numBytesRead;
	
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		if (pBuffer)
		{
			//
			// Read our buffer
			//
			numBytesRead = read(fileId, pBuffer, numBytesToRead);
			
			//
			// Check if we have read at least one byte
			//
			if (numBytesRead>0)
			{
				errorCode = SBG_NO_ERROR;
			}
			else
			{
				errorCode = SBG_READ_ERROR;
				numBytesRead = 0;
			}
			
			//
			// If we can, returns the number of read bytes
			//
			if (pNumBytesRead)
			{
				*pNumBytesRead = numBytesRead;
			}			
		}
		else
		{
			errorCode = SBG_NULL_POINTER;
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}
	
	return errorCode;
}

/// Flush our RX and TX buffers (remove all old data)
SbgErrorCode sbgDeviceFlush(SbgDeviceHandle handle)
{
	int32 fileId = *((int32*)&handle);
	
	if (handle != SBG_INVALID_DEVICE_HANDLE)
	{
		//
		// Flush our port
		//
		tcflush(fileId, TCIOFLUSH);
		
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}
