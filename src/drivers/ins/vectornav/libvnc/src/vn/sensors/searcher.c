#include "vn/bool.h"
#include "vn/sensors/searcher.h"
#include "vn/xplat/serialport.h"
#include "vn/xplat/thread.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#ifdef __linux__
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#elif defined _WIN32
#include <tchar.h>
#endif

uint32_t SubBD[9] = {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};

uint32_t VnSearcher_matchBaud(char inputData[])
{

	uint32_t inputBaud = 0;
	char* start = strrchr(inputData, ',') + 1;
	char* end =  strrchr(start, '*');
	size_t index = 0;
	bool integerDigit = true;

	/* Isolate the baud in the message. */

	/* Ensure the entire baud string was recieved. */
	/* If end is null then this is an incomplete packet. */
	if (NULL != end)
	{
		/* Ensure this is an integer digit. */
		while ((start + index) < end)
		{
			char character = start[index];
			if (0 == isdigit((int)character))
			{
				integerDigit = false;
				break;
			}
			++index;
		}

		/* Convert the string to an integer for comparing. */
		if (integerDigit)
		{
			char stringBaud[20] = { 0 };
			uint32_t length = end - start;

			memcpy(stringBaud, start, length);
			inputBaud = atoi(stringBaud);
		}
	}

	/* Return the found baud */
	return inputBaud;
}

void VnSearcher_discardData(char inputData[], size_t dataLength, size_t discardLength)
{
	/* This function will move the pertinent data to be begining of the array. */
	/* If discard length is the full size of the array then all data is discarded. */
	if (dataLength <= discardLength)
	{
		/* Just zero out everything */
		memset(inputData, 0, dataLength);
	}
	else
	{
		/* Move the data to the front of the array and zero out everything else */
		/* Size of the data to move */
		size_t tmpSize = dataLength - discardLength;
		/* Temporary swap buffer */
		char* tmp = (char*)malloc(sizeof(char) * tmpSize);
		/* And the swap operations */
		/*memcpy_s(tmp, tmpSize, inputData + discardLength, tmpSize);*/
		memcpy(tmp, inputData + discardLength, tmpSize);
		memset(inputData, 0, dataLength);
		/*memcpy_s(inputData, tmpSize, tmp, tmpSize);*/
		memcpy(inputData, tmp, tmpSize);
	}
}

bool VnSearcher_findString(char* inputData, size_t* inputSize, char* toFind, size_t findSize)
{
	/* Assume failure */
	bool success = false;
	char* end = inputData + *inputSize;
	char* start = inputData;
	size_t size = *inputSize;
	
	/* Loop through the data until we run out */
	while ((NULL != start) && (!success) && (findSize <= size) && (NULL != end))
	{
		/* find the first instance of the toFind array */
		start = (char*)memchr(start, '$', size);

		/* It doesn't exist so just zero out everything; the data is useless */
		if (NULL == start)
		{
			VnSearcher_discardData(inputData, *inputSize, *inputSize);
			*inputSize = 0;
		}
		/* We found it, now to see if this is the beginning of the input array */
		else
		{
			end = (char*)memchr(start, '*', size);

			/* We found an end sentinal */
			if (NULL != end)
			{
				/* We have both a start and end sentinal.  Let's see if */
				/* we have a proper packet. */
				
				if(((end - start) >= (int)findSize) &&
				   (strncmp(start, toFind, findSize) == 0))
				{
					/* This is what we're looking for */
					success = true;

					/* Readjust the size of the data. */
					VnSearcher_discardData(inputData, *inputSize, start - inputData);
					*inputSize = end - start + 1;
				}
				else
				{
					/* Wrong type of packet or no packet.  Remove the sentinal character */
					/* and continue the search. */
					start[0] = '\0';
				}
			}

		}
	}

	return success;
}

bool VnSearcher_validateData(char inputData[], size_t* dataLength)
{
	/* Assume failure */
	bool validated = false;
	bool result1 = false;
	bool result2 = false;

	char search1[256] = { 0 };
	char search2[256] = { 0 };

	size_t size1 = *dataLength;
	size_t size2 = *dataLength;
	
	memcpy(search1, inputData, *dataLength);
	memcpy(search2, inputData, *dataLength);

	/* search for a $VNRRG,5 or %VNRRG,05 */

	result1 = VnSearcher_findString(search1, &size1, "$VNRRG,5", 8);
	result2 = VnSearcher_findString(search2, &size2, "$VNRRG,05", 9);

	/*printf("validateData::search1-1 %s\n", search1);*/
	if (result1)
	{
		validated = true;
		VnSearcher_discardData(inputData, *dataLength, *dataLength);
		memcpy(inputData, search1, size1);
		*dataLength = size1;
	}
	else if (result2)
	{
		validated = true;
		VnSearcher_discardData(inputData, *dataLength, *dataLength);
		memcpy(inputData, search2, size2);
		*dataLength = size2;
	}
	else if (size1 > size2)
	{
		*dataLength = size1;
	}
	else
	{
		*dataLength = size2;
	}

	/* Return the result. */
	return validated;
}

void VnSearcher_dataReceivedHandler(void* portInfo)
{
	/* Cast the input void pointer */
	VnPortInfo* port = (VnPortInfo*)portInfo;

	/* Prepare for handling incoming data */
	char readBuffer[255] = {0};

	/* Data validation flag.  Assume failure. */
	bool validated = false;

	size_t bytesToRead = sizeof(port->data) - port->dataSize;
	size_t bytesRead = 0;
	
	/* Keep looping until a valid packet is found or the thread shuts down. */
	/* Read the data from the port. */
	VnError error = VnSerialPort_read(port->port, readBuffer, bytesToRead, &bytesRead);

	/*printf("read %d %s\n", bytesRead, readBuffer);*/

	memcpy(port->data + port->dataSize, readBuffer, bytesRead);
	port->dataSize += bytesRead;
	port->data[254] = '\n';


	/* Check if this is valid data from the sensor. */
	/*validated = VnSearcher_validateData(readBuffer);*/
	validated = VnSearcher_validateData(port->data, &(port->dataSize));

	/* If an error occurs place a value of -2 into the baud variable to
		* indicate this and end the thread.
		*/
	if (E_NONE != error)
	{
		port->baud = -2;
		/*break;*/
	}

	/* If this is valid data then extract the baud rate and put the value into
	 * the port object.
	 */
	if(validated)
	{
		uint32_t bufferBaud = VnSearcher_matchBaud(port->data);
		port->baud = bufferBaud;
		validated = false;
	}
}

void VnSearcher_testPort(void* portInfo)
{
	VnError error = E_NONE;
	/* Cast the input void pointer */
	VnPortInfo* port = (VnPortInfo*)portInfo;

	size_t index = 0;
	size_t numBaudrates = sizeof(SubBD) / sizeof(SubBD[0]);

	/* Loop through all of the possible baud rates. */
	while(index < numBaudrates)
	{
		/* Set up a serial port object to communicate with. */
		VnSerialPort serialPort;
		port->port = &serialPort;
		VnSerialPort_initialize(&serialPort);
		VnSerialPort_registerDataReceivedHandler(&serialPort, VnSearcher_dataReceivedHandler, port);

		/* Open the port for communications. */
		error = VnSerialPort_open(&serialPort, port->portName, SubBD[index]);

		/* ensure the port opened. */
		if(VnSerialPort_isOpen(&serialPort) && (E_NONE == error))
		{
			/* Request the baud rate from the sensor. */
			VnSerialPort_write(&serialPort, "$VNRRG,05*XX\r\n", 14);

			/* Now wait 50 miliseconds to see if the sensor answers. */
			VnThread_sleepMs(50);

			/* Close the port to tell the data handling thread to terminate. */
			VnSerialPort_close(&serialPort);

			/* Check if we have a baud.
			 * -2 - Unknown error occured.
			 * -1 - No baud rate was found.
			 *  0 - Initial state.
			 * >0 - A valid baud rate was found. */
			if(port->baud != 0)
			{
				/* Either a sensor was found or an error occurred.
				 * End the while loop. */
				break;
			}
		}

		/* We don't want an infinite loop. */
		index++;
	}

	/* If there was no change in the baud variable indicate no sensor found
	 * using a value of -1. */
	if(port->baud == 0)
	{
		port->baud = -1;
	}
}

/* Public Functions */

char* VnSearcher_getPortName(VnPortInfo* portInfo)
{
	return portInfo->portName;
}

int32_t VnSearcher_getPortBaud(VnPortInfo* portInfo)
{
	return portInfo->baud;
}

#ifdef __linux__
void VnSearcher_findPorts_LINUX(char*** portNamesOut, int32_t* numPortsFound)
{
	char portName[15] = {0};
	char portNames[16 * 255] = {0};
	char* portNamePtr = portNames;
	const size_t MAX_PORTS = 255;
	int portFd = -1;
	size_t index = 0;
	size_t nameLen = 0;

	*numPortsFound = 0;

	while(index < MAX_PORTS)
	{
		/* Create the port name for reading. */
		sprintf(portName, "/dev/ttyUSB%u", (unsigned int) index);

		/* Attempt to open the serial port */
		portFd = open(portName,
					  #if __linux__ || __CYGWIN__ || __QNXNTO__
					  O_RDWR | O_NOCTTY );
					  #elif __APPLE__
					  O_RDWR | O_NOCTTY | O_NONBLOCK);
					  #else
					  #error "Unknown System"
					  #endif

		/* Check to see if the port opened */
		if(portFd != -1)
		{
			/* We have a usable port */
			/* Record the port name */
			nameLen = strlen(portName);
			sprintf(portNamePtr, "%s,", portName);
			portNamePtr += nameLen + 1;
			++(*numPortsFound);

			/* Close the port */
			close(portFd);
		}
		/* If not we ignore it */
		++index;
	}

	if (*numPortsFound > 0)
	{
		char* start = NULL;
		char* end = NULL;

		uint32_t length = 0;

		*portNamesOut = (char**)malloc(index * sizeof(char*));
		index = 0;

		start = portNames;
		end = portNames;

		while (index < *numPortsFound)
		{
			start = strstr(end, "/dev");

			end = strchr(start, ',');
			length = end - start;
			(*portNamesOut)[index] = (char*)malloc(length + 1);
			memcpy((*portNamesOut)[index], start, length);
			(*portNamesOut)[index][length] = '\0';
			end += 1;

			++index;
		}
	}}
#elif defined _WIN32
void VnSearcher_findPorts_WIN32(char*** portNamesOut, int32_t* numPortsFound)
{
	uint32_t MAX_PORTS = 255;

	char portName[12] = {0};
	char portNames[13 * 255] = {0};
	char* portNamePtr = portNames;

	HANDLE portHandle;

	uint32_t index = 0;
	uint32_t portNameLength = 0;

	#if defined(_MSC_VER)
	/* Disable warnings regarding using strcpy_s since this
	 * function's signature does not provide us with information
	 * about the length of 'out'. */
	#pragma warning(push)
	#pragma warning(disable:4996)
	#endif

	while (index < MAX_PORTS)
	{
		/* Create the port name for reading. */
		sprintf(portName, "\\\\.\\COM%u", index);

		/* Create the port file. */
		portHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

		/* Check if this is a valid handle. */
		if (INVALID_HANDLE_VALUE != portHandle)
		{
			/* We have a valid handle. */
			/* Record the port name and increment the number of ports counter. */
			portNameLength = strlen(portName);
			sprintf(portNamePtr, "%s,", portName);
			portNamePtr += portNameLength + 1;
			++(*numPortsFound);

			/* Clean up.  Close the handle. */
			if(!CloseHandle(portHandle))
			{
				printf("ERROR\n");
			}


		}

		index++;
	}

	#if defined(_MSC_VER)
	#pragma warning(pop)
	#endif

	if (*numPortsFound > 0)
	{
		char* start = portNames;
		char* end = portNames;

		*portNamesOut = (char**) malloc(index * sizeof(char*));
		index = 0;

		while ((int32_t)index < *numPortsFound)
		{
			uint32_t length;
			
			start = strstr(end, "COM");

			end = strchr(start, ',');
			length = end - start;
			(*portNamesOut)[index] = (char*)malloc(length + 1);
			memcpy((*portNamesOut)[index], start, length);
			(*portNamesOut)[index][length] = '\0';
			end += 1;
			
			index++;
		}
	}
}
#endif

void VnSearcher_findPortBaud(char* portName, int32_t* foundBaudrate)
{
	/* These will handle the return data. */
	VnPortInfo** portInfo;

	/* These will handle the input data. */
	char** portNames = &portName;
	int32_t numPorts = 1;
	
	/* This function will search the port and, if found, return a VnPortInfo */
	/* containing the baud rate. */
	VnSearcher_searchInputPorts(&portNames, numPorts, &portInfo);

	/* The number in baud will indicate whether the function succeeded
	 * -2 - error
	 * -1 - no sensor found
	 * >0 - sensor's baud rate
	 */
	*foundBaudrate = portInfo[0]->baud;
}

void VnSearcher_findAllSensors(VnPortInfo*** returnSensors, int32_t* numSensors)
{
	int32_t numSystemPorts = 0;
	char** systemPorts = NULL;

	/* First, find all of the ports on the system */
	VnSearcher_findPorts(&systemPorts, &numSystemPorts);

	/* Check if a sensor is accessible on the port */
	/* Only do this if the port search succeeded and found available system ports */
	if(numSystemPorts > 0)
	{
		VnSearcher_searchInputPorts(&systemPorts, numSystemPorts, returnSensors);
		*numSensors = numSystemPorts;
	}
}

void VnSearcher_searchInputPorts(char*** portsToCheck, int32_t numPortsToCheck, VnPortInfo*** sensorsFound)
{
	int32_t linkCount = 0;

	/* Initialize the sensorsFound array with space for any future pointers. */
	*sensorsFound = (VnPortInfo**) malloc(sizeof(VnPortInfo*) * numPortsToCheck);

	while(linkCount < numPortsToCheck)
	{
        /*VnPortInfo* newPort = NULL;*/
        size_t nameSize = 0;

        /*newPort = (*sensorsFound)[linkCount];*/
		nameSize = strlen((*portsToCheck)[linkCount]);

		/* Create the port info */
		(*sensorsFound)[linkCount] = (VnPortInfo*)malloc(sizeof(VnPortInfo));
		(*sensorsFound)[linkCount]->dataSize = 0;

		/* Give it the port name */
		(*sensorsFound)[linkCount]->portName = (char*)malloc(sizeof(char) * nameSize + 1);
		memcpy((*sensorsFound)[linkCount]->portName, (*portsToCheck)[linkCount], nameSize + 1);

		/* Make sure the data field is zeroed out. */
		memset(&(*sensorsFound)[linkCount]->data, 0, sizeof((*sensorsFound)[linkCount]->data));

		/* Set baud to 0.  A baud of 0 will indicate that the test is not finished
		 * running just yet.  A baud of -1 will indicate that no sensor has been
		 * found.  A baud of -2 will indicate an error.  Any baud over 0 will be
		 * the actual baud rate. */
		(*sensorsFound)[linkCount]->baud = 0;

		/* Set up the thread */

		/* Now, start the thread to check the port.*/
		VnThread_startNew(&((*sensorsFound)[linkCount]->thread), VnSearcher_testPort, (*sensorsFound)[linkCount]);

		/* And increment the link counter */
		++linkCount;
	}

	/* Reset the link counter. */
	linkCount = 0;

	/* Loop until all of the threads have completed. */
	while(linkCount < numPortsToCheck)
	{
		/* Sleep until the baud rate changes */
		while((*sensorsFound)[linkCount]->baud == 0)
		{
			VnThread_sleepMs(1);
		}

		/* The thread has finished.  The thread handles setting any pertinent
		 * data.  Simply increment the linkCount variable
		 */
		++linkCount;
	}
}
