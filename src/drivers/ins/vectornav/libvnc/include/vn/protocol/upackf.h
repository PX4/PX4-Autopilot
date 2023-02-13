#ifndef VNUPACKF_H_INCLUDED
#define VNUPACKF_H_INCLUDED

#include "vn/protocol/upack.h"

/*#include "vnint.h"*/
/*#include "vnbool.h"*/
/*#include "vntypes.h"
#include "vnerror.h"*/


#ifndef VNUART_PROTOCOL_BUFFER_SIZE
	/** Default internal buffers size for handling received UART data. */
	#define VNUART_PROTOCOL_BUFFER_SIZE 256
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Defines signature of functions that can handle callback
*  notifications of packets successfully received and validated from a
*  VectorNav sensor. */
typedef void(*VnUartPacketFinder_PacketFoundHandler)(void *userData, VnUartPacket* packet, size_t runningIndexOfPacketStart);

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4820)
#endif

/** \brief Data structure holding current parsing status of data received from
 *  a VectorNav sensor.
 *
 * This structure contains a buffer which will hold bytes that are currently
 * being processed. The size of this buffer can be adjusted by defining the
 * size using the preprocesser. For example, the size can be adjusted to use
 * 1024 bytes by defining VNUART_PROTOCOL_BUFFER_SIZE=1024. */
typedef struct
{
	/** \brief Callback for when a packet has been found and validated. */
	VnUartPacketFinder_PacketFoundHandler packetFoundHandler;

	/** \brief User data for callbacks on the packetFoundHandler. */
	void *packetFoundHandlerUserData;

	/** \brief Used for correlating the position in the received raw data
	*   stream where packets are found.	*/
	size_t runningDataIndex;

	/** \brief Indicates if an ASCII packet is currently being built. */
	bool asciiCurrentlyBuildingPacket;

	/** \brief Indicates a suspected start of an ASCII packet. */
	size_t asciiPossibleStartOfPacketIndex;

	/** \brief Index of start of ASCII packet in total running index. */
	size_t asciiRunningDataIndexOfStart;

	/** \brief Indicates if the first ending character has been found. */
	bool asciiEndChar1Found;

	/** \brief Indicates if we are currently building a binary packet. */
	bool binaryCurrentlyBuildingBinaryPacket;

	/** \brief Index of start of binary packet in total running index. */
	size_t binaryRunningDataIndexOfStart;

	/** \brief Holds the size of the receive buffer. */
	size_t bufferSize;

	/** \brief The current location to append data into the buffer. */
	size_t bufferAppendLocation;

	/** \brief Indicates if we have found the groups present data field for a
	 *  binary packet we are building. */
	bool binaryGroupsPresentFound;

	/** \brief The groups present found from a binary packet. */
	uint8_t binaryGroupsPresent;

	/** \brief Indicates the number of bytes remaining to have all group fields
	 *  for a binary data packet we are processing. */
	uint8_t binaryNumOfBytesRemainingToHaveAllGroupFields;

	/** \brief Start index of a possible binary packet. */
	size_t binaryPossibleStartIndex;

	/** \brief Keeps track of the number of bytes remaining for a complete
	 *  binary packet. */
	size_t binaryNumberOfBytesRemainingForCompletePacket;

	/** \brief The receive buffer. */
	uint8_t receiveBuffer[VNUART_PROTOCOL_BUFFER_SIZE];

} VnUartPacketFinder;

#ifdef _WIN32
#pragma warning(pop)
#endif

/** \brief Initializes the VnUartPacketFinder data structure.
*
* \param[in] pf VnUartPacketFinder to init. */
void VnUartPacketFinder_initialize(VnUartPacketFinder* pf);

/** \brief Processes received data from the UART.
*
* \param[in] finder The associated VnUartPacketFinder containing the data
*     processing state.
* \param[in] data The received data.
* \param[in] len The number of bytes to process.
* \param[in] bootloaderFilter Enable filtering for bootloader messages. */
VnError VnUartPacketFinder_processData_ex(VnUartPacketFinder* finder, uint8_t* data, size_t len, bool bootloaderFilter);
VnError VnUartPacketFinder_processData(VnUartPacketFinder* finder, uint8_t* data, size_t len);


/** \brief Allows registering for notification of when valid packets are found.
 *
 * \param[in] handler The callback function for receiving notifications.
 * \param[in] userData Pointer to user supplied data which will be sent on all callback notifications.
 * \return Any errors encountered. */
VnError VnUartPacketFinder_registerPacketFoundHandler(VnUartPacketFinder* finder, VnUartPacketFinder_PacketFoundHandler handler, void *userData);


#ifdef __cplusplus
}
#endif

#endif
