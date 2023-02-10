#include "vn/protocol/upackf.h"

#include <string.h>
/*#include <stdlib.h>
#include <stdio.h>*/

#include "vn/util.h"

/*#define MAXIMUM_REGISTER_ID		255
#define ASCII_START_CHAR		'$'

#define BINARY_START_CHAR		0xFA*/
#define ASCII_END_CHAR1			'\r'
#define ASCII_END_CHAR2			'\n'
#define MAX_BINARY_PACKET_SIZE	600

/* Function declarations */
void VnUartPacketFinder_dispatchPacket(VnUartPacketFinder* finder, VnUartPacket* packet, size_t running);
void VnUartPacketFinder_processPacket(VnUartPacketFinder* finder, uint8_t* start, size_t len, size_t running);
void VnUartPacketFinder_resetAllTrackingStatus(VnUartPacketFinder* finder);
void VnUartPacketFinder_resetAsciiStatus(VnUartPacketFinder* finder);
void VnUartPacketFinder_resetBinaryStatus(VnUartPacketFinder* finder);


/** \brief Dispatch packet.
* \param[in] running Running Index. */
void vn_uart_packet_finder_packet_found(VnUartPacketFinder* finder, uint8_t* start, size_t len, size_t running);

char* vnstrtok(char* str, size_t* startIndex);

void VnUartPacketFinder_initialize(VnUartPacketFinder* toInitialize)
{
	toInitialize->packetFoundHandler = NULL;
	toInitialize->packetFoundHandlerUserData = NULL;
	toInitialize->runningDataIndex = 0;
	toInitialize->asciiCurrentlyBuildingPacket = false;
	toInitialize->asciiPossibleStartOfPacketIndex = 0;
	toInitialize->asciiRunningDataIndexOfStart = 0;
	toInitialize->asciiEndChar1Found = false;
	toInitialize->binaryCurrentlyBuildingBinaryPacket = false;
	toInitialize->binaryRunningDataIndexOfStart = 0;
	toInitialize->bufferSize = VNUART_PROTOCOL_BUFFER_SIZE;
	toInitialize->binaryGroupsPresentFound = false;
	toInitialize->binaryGroupsPresent = 0;
	toInitialize->binaryNumOfBytesRemainingToHaveAllGroupFields = 0;
	toInitialize->binaryPossibleStartIndex = 0;
	toInitialize->binaryNumberOfBytesRemainingForCompletePacket = 0;
	toInitialize->bufferAppendLocation = 0;
}

VnError VnUartPacketFinder_processData(VnUartPacketFinder* finder, uint8_t* data, size_t len)
{
	return VnUartPacketFinder_processData_ex(finder, data, len, false);
}


VnError VnUartPacketFinder_processData_ex(VnUartPacketFinder* finder, uint8_t* data, size_t len, bool bootloaderFilter)
{
	bool asciiStartFoundInProvidedBuffer = false;
	bool binaryStartFoundInProvidedDataBuffer = false;
	size_t i, dataIndexToStartCopyingFrom, binaryDataMoveOverIndexAdjustment;
	bool binaryDataToCopyOver;

	/* Assume that since the _runningDataIndex is unsigned, any overflows
	* will naturally go to zero, which is the behavior that we want. */
	for (i = 0; i < len; i++, finder->runningDataIndex++)
	{
		bool justFinishedBinaryPacket = false;

		if ((data[i] == VN_ASCII_START_CHAR) || (bootloaderFilter && (!finder->asciiCurrentlyBuildingPacket && data[i] == VN_BOOTLOAD_START_CHAR)))
		{
			VnUartPacketFinder_resetAsciiStatus(finder);
			finder->asciiCurrentlyBuildingPacket = true;
			finder->asciiPossibleStartOfPacketIndex = i;
			finder->asciiRunningDataIndexOfStart = finder->runningDataIndex;

			asciiStartFoundInProvidedBuffer = true;
		}
		else if (finder->asciiCurrentlyBuildingPacket && data[i] == ASCII_END_CHAR1)
		{
			finder->asciiEndChar1Found = true;
		}
		else if ((bootloaderFilter || (!bootloaderFilter && finder->asciiEndChar1Found)) && (finder->asciiCurrentlyBuildingPacket && data[i] == ASCII_END_CHAR2))
		{
			if (asciiStartFoundInProvidedBuffer)
			{
				uint8_t* startOfAsciiPacket;
				size_t packetLength;

				/* All the packet was in this data buffer so we don't
				* need to do any copying. */

				startOfAsciiPacket = data + finder->asciiPossibleStartOfPacketIndex;
				packetLength = i - finder->asciiPossibleStartOfPacketIndex + 1;

				VnUartPacketFinder_processPacket(finder, startOfAsciiPacket, packetLength, finder->asciiRunningDataIndexOfStart);

				asciiStartFoundInProvidedBuffer = false;
				VnUartPacketFinder_resetAsciiStatus(finder);
			}
			else
			{
				/* The packet was split between the running data buffer
				* the current data buffer. We need to copy the data
				* over before further processing. */

				if (finder->bufferAppendLocation + i < finder->bufferSize)
				{
					uint8_t* startOfAsciiPacket;
					size_t packetLength;

					memcpy(finder->receiveBuffer + finder->bufferAppendLocation, data, i + 1);

					startOfAsciiPacket = finder->receiveBuffer + finder->asciiPossibleStartOfPacketIndex;
					packetLength = finder->bufferAppendLocation + i + 1 - finder->asciiPossibleStartOfPacketIndex;

					VnUartPacketFinder_processPacket(finder, startOfAsciiPacket, packetLength, finder->asciiRunningDataIndexOfStart);

					asciiStartFoundInProvidedBuffer = false;
					VnUartPacketFinder_resetAsciiStatus(finder);
				}
				else
				{
					/* We are about to overflow our buffer. */
					VnUartPacketFinder_resetAllTrackingStatus(finder);
					asciiStartFoundInProvidedBuffer = false;
					binaryStartFoundInProvidedDataBuffer = false;
				}
			}
		}
		else if (finder->asciiEndChar1Found)
		{
			/* Must not be a valid ASCII packet. */
			VnUartPacketFinder_resetAsciiStatus(finder);
			asciiStartFoundInProvidedBuffer = false;
			if (!finder->binaryCurrentlyBuildingBinaryPacket)
				finder->bufferAppendLocation = 0;
		}

		/* Update our binary packet in processing. */
		if (finder->binaryCurrentlyBuildingBinaryPacket)
		{
			if (!finder->binaryGroupsPresentFound)
			{
				/* This byte must be the groups present. */
				finder->binaryGroupsPresentFound = true;
				finder->binaryGroupsPresent = (uint8_t) data[i];
				finder->binaryNumOfBytesRemainingToHaveAllGroupFields = (uint8_t) (2 * VnUtil_countSetBitsUint8((uint8_t) data[i]));
			}
			else if (finder->binaryNumOfBytesRemainingToHaveAllGroupFields != 0)
			{
				/* We found another byte belonging to this possible binary packet. */
				finder->binaryNumOfBytesRemainingToHaveAllGroupFields--;

				if (finder->binaryNumOfBytesRemainingToHaveAllGroupFields == 0)
				{
					/* We have all of the group fields now. */

					size_t remainingBytesForCompletePacket = 0;
					if (binaryStartFoundInProvidedDataBuffer)
					{
						size_t headerLength = i - finder->binaryPossibleStartIndex + 1;
						remainingBytesForCompletePacket = VnUartPacket_computeBinaryPacketLength(data + finder->binaryPossibleStartIndex) - headerLength;
					}
					else
					{
						/* Not all of the packet's group is inside the caller's provided buffer. */

						/* Temporarily copy the rest of the packet to the receive buffer
						 * for computing the size of the packet.
						 */

						size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;
						size_t headerLength = finder->bufferAppendLocation - finder->binaryPossibleStartIndex + numOfBytesToCopyIntoReceiveBuffer;

						if (finder->bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < finder->bufferSize)
						{
							memcpy(
								finder->receiveBuffer + finder->bufferAppendLocation,
								data,
								numOfBytesToCopyIntoReceiveBuffer);

							remainingBytesForCompletePacket = VnUartPacket_computeBinaryPacketLength(finder->receiveBuffer + finder->binaryPossibleStartIndex) - headerLength;
						}
						else
						{
							/* About to overrun our receive buffer! */
							VnUartPacketFinder_resetAllTrackingStatus(finder);
							binaryStartFoundInProvidedDataBuffer = false;
							asciiStartFoundInProvidedBuffer = false;
						}
					}

					if (finder->binaryCurrentlyBuildingBinaryPacket)
					{
						if (remainingBytesForCompletePacket > MAX_BINARY_PACKET_SIZE)
						{
							/* Must be a bad possible binary packet. */
							VnUartPacketFinder_resetBinaryStatus(finder);
							binaryStartFoundInProvidedDataBuffer = false;

							if (!finder->asciiCurrentlyBuildingPacket)
								finder->bufferAppendLocation = 0;
						}
						else
						{
							finder->binaryNumberOfBytesRemainingForCompletePacket = remainingBytesForCompletePacket;
						}
					}
				}
			}
			else
			{
				/* We are currently collecting data for our packet. */

				finder->binaryNumberOfBytesRemainingForCompletePacket--;
				
				if (finder->binaryNumberOfBytesRemainingForCompletePacket == 0)
				{
					/* We have a possible binary packet! */

					if (binaryStartFoundInProvidedDataBuffer)
					{
						uint8_t* packetStart;
						size_t packetLength;

						/* The binary packet exists completely in the user's provided buffer. */
						packetStart = data + finder->binaryPossibleStartIndex;
						packetLength = i - finder->binaryPossibleStartIndex + 1;
						
						VnUartPacketFinder_processPacket(finder, packetStart, packetLength, finder->binaryRunningDataIndexOfStart);

						justFinishedBinaryPacket = true;
					}
					else
					{
						/* The packet is split between our receive buffer and the user's buffer. */
						size_t numOfBytesToCopyIntoReceiveBuffer = i + 1;

						if (finder->bufferAppendLocation + numOfBytesToCopyIntoReceiveBuffer < finder->bufferSize)
						{
							uint8_t* packetStart;
							size_t packetLength;

							memcpy(
								finder->receiveBuffer + finder->bufferAppendLocation,
								data,
								numOfBytesToCopyIntoReceiveBuffer);

							packetStart = finder->receiveBuffer + finder->binaryPossibleStartIndex;
							packetLength = finder->bufferAppendLocation - finder->binaryPossibleStartIndex + i + 1;

							VnUartPacketFinder_processPacket(finder, packetStart, packetLength, finder->binaryRunningDataIndexOfStart);

							justFinishedBinaryPacket = true;
						}
						else
						{
							/* About to overrun our receive buffer! */
							VnUartPacketFinder_resetAllTrackingStatus(finder);
							binaryStartFoundInProvidedDataBuffer = false;
							asciiStartFoundInProvidedBuffer = false;
						}
					}
				}
			}
		}

		if (data[i] == VN_BINARY_START_CHAR && !justFinishedBinaryPacket)
		{
			if (!finder->binaryCurrentlyBuildingBinaryPacket)
			{
				/* Possible start of a binary packet. */
				binaryStartFoundInProvidedDataBuffer = true;
				finder->binaryCurrentlyBuildingBinaryPacket = true;
				finder->binaryPossibleStartIndex = i;
				finder->binaryGroupsPresentFound = false;
				finder->binaryNumOfBytesRemainingToHaveAllGroupFields = 0;
				finder->binaryNumberOfBytesRemainingForCompletePacket = 0;
				finder->binaryRunningDataIndexOfStart = finder->runningDataIndex;
			}
		}
	}

	/* Perform any data copying to our receive buffer. */

	dataIndexToStartCopyingFrom = 0;
	binaryDataToCopyOver = false;
	binaryDataMoveOverIndexAdjustment = 0;

	if (finder->binaryCurrentlyBuildingBinaryPacket)
	{
		binaryDataToCopyOver = true;

		if (binaryStartFoundInProvidedDataBuffer)
		{
			dataIndexToStartCopyingFrom = finder->binaryPossibleStartIndex;
			binaryDataMoveOverIndexAdjustment = dataIndexToStartCopyingFrom;
		}
	}

	if (finder->asciiCurrentlyBuildingPacket && asciiStartFoundInProvidedBuffer)
	{
		if (finder->asciiPossibleStartOfPacketIndex < dataIndexToStartCopyingFrom)
		{
			binaryDataMoveOverIndexAdjustment -= binaryDataMoveOverIndexAdjustment - finder->asciiPossibleStartOfPacketIndex;
			dataIndexToStartCopyingFrom = finder->asciiPossibleStartOfPacketIndex;
		}
		else if (!binaryDataToCopyOver)
		{
			dataIndexToStartCopyingFrom = finder->asciiPossibleStartOfPacketIndex;
		}

		/* Adjust our ASCII index to be based on the receive buffer. */
		finder->asciiPossibleStartOfPacketIndex = finder->bufferAppendLocation + finder->asciiPossibleStartOfPacketIndex - dataIndexToStartCopyingFrom;
	}

	/* Adjust any binary packet indexes we are currently building. */
	if (finder->binaryCurrentlyBuildingBinaryPacket)
	{
		if (binaryStartFoundInProvidedDataBuffer)
		{
			finder->binaryPossibleStartIndex = finder->binaryPossibleStartIndex - binaryDataMoveOverIndexAdjustment + finder->bufferAppendLocation;
		}
	}

	if (finder->bufferAppendLocation + len - dataIndexToStartCopyingFrom < finder->bufferSize)
	{
		/* Safe to copy over the data. */
		size_t numOfBytesToCopyOver = len - dataIndexToStartCopyingFrom;
		uint8_t *copyFromStart = data + dataIndexToStartCopyingFrom;

		memcpy(
			finder->receiveBuffer + finder->bufferAppendLocation,
			copyFromStart,
			numOfBytesToCopyOver);

		finder->bufferAppendLocation += numOfBytesToCopyOver;
	}
	else
	{
		/* We are about to overflow our buffer. */
		VnUartPacketFinder_resetAsciiStatus(finder);
		VnUartPacketFinder_resetBinaryStatus(finder);
		finder->bufferAppendLocation = 0;
	}

	return E_NONE;
}

void VnUartPacketFinder_processPacket(VnUartPacketFinder* finder, uint8_t* start, size_t len, size_t running)
{
	VnUartPacket p;
	PacketType t;

	VnUartPacket_initialize(&p, start, len);

	t = VnUartPacket_type(&p);

	if (VnUartPacket_isValid(&p))
	{
		VnUartPacketFinder_dispatchPacket(finder, &p, running);

		/* Reset tracking for both packet types since this packet was valid. */
		VnUartPacketFinder_resetAsciiStatus(finder);
		VnUartPacketFinder_resetBinaryStatus(finder);

		finder->bufferAppendLocation = 0;
	}
	else
	{
		/* Invalid packet! Reset tracking. */

		if (t == PACKETTYPE_ASCII)
		{
			VnUartPacketFinder_resetAsciiStatus(finder);

			if (!finder->binaryCurrentlyBuildingBinaryPacket)
				finder->bufferAppendLocation = 0;
		}
		else
		{
			VnUartPacketFinder_resetBinaryStatus(finder);

			if (!finder->asciiCurrentlyBuildingPacket)
				finder->bufferAppendLocation = 0;
		}
	}
}

void VnUartPacketFinder_resetAsciiStatus(VnUartPacketFinder* finder)
{
	finder->asciiCurrentlyBuildingPacket = false;
	finder->asciiPossibleStartOfPacketIndex = 0;
	finder->asciiEndChar1Found = false;
	finder->asciiRunningDataIndexOfStart = 0;
}

void VnUartPacketFinder_resetBinaryStatus(VnUartPacketFinder* finder)
{
	finder->binaryCurrentlyBuildingBinaryPacket = false;
	finder->binaryGroupsPresentFound = false;
	finder->binaryGroupsPresent = 0;
	finder->binaryNumOfBytesRemainingToHaveAllGroupFields = 0;
	finder->binaryPossibleStartIndex = 0;
	finder->binaryNumberOfBytesRemainingForCompletePacket = 0;
	finder->binaryRunningDataIndexOfStart = 0;
}

void VnUartPacketFinder_resetAllTrackingStatus(VnUartPacketFinder *finder)
{
	if (finder->asciiCurrentlyBuildingPacket)
		VnUartPacketFinder_resetAsciiStatus(finder);

	if (finder->binaryCurrentlyBuildingBinaryPacket)
		VnUartPacketFinder_resetBinaryStatus(finder);

	finder->bufferAppendLocation = 0;
}

void VnUartPacketFinder_dispatchPacket(VnUartPacketFinder* finder, VnUartPacket* packet, size_t running)
{
	if (finder->packetFoundHandler == NULL)
		return;

	finder->packetFoundHandler(finder->packetFoundHandlerUserData, packet, running);
}

VnError VnUartPacketFinder_registerPacketFoundHandler(VnUartPacketFinder* finder, VnUartPacketFinder_PacketFoundHandler handler, void *userData)
{
	if (finder->packetFoundHandler != NULL)
		return E_INVALID_OPERATION;

	finder->packetFoundHandler = handler;
	finder->packetFoundHandlerUserData = userData;

	return E_NONE;
}
