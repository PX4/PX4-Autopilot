#ifndef VN_SERIALPORT_H
#define VN_SERIALPORT_H

/** Cross-platform access to serial ports. */

#include "vn/int.h"
#include "vn/error.h"
#include "vn/bool.h"
#include "vn/xplat/thread.h"
#include "vn/xplat/criticalsection.h"

#if defined(_WIN32)

	/* Disable some warnings for Visual Studio with -Wall. */
	#if defined(_MSC_VER)
		#pragma warning(push)
		#pragma warning(disable:4668)
		#pragma warning(disable:4820)
		#pragma warning(disable:4255)
	#endif

	#include <Windows.h>

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
#endif

#ifdef __linux__
	#include <linux/serial.h>
#elif defined __APPLE__
	#include <dirent.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Type for listening to data received events from the VnSerialPort. */
typedef void (*VnSerialPort_DataReceivedHandler)(void *userData);

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4820)
#endif

/** \brief Provides access to a serial port. */
typedef struct
{
	#ifdef _WIN32
	HANDLE handle;
	/* Windows appears to need single-thread access to read/write API functions. */
	VnCriticalSection readWriteCS;
	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)
	int handle;
	#else
	#error "Unknown System"
	#endif

	bool purgeFirstDataBytesWhenSerialPortIsFirstOpened;

	VnSerialPort_DataReceivedHandler dataReceivedHandler;

	void *dataReceivedHandlerUserData;

	VnThread serialPortNotificationsThread;

	bool continueHandlingSerialPortEvents;

	size_t numberOfReceiveDataDroppedSections;

	VnCriticalSection dataReceivedHandlerCriticalSection;

	char portName[50];

} VnSerialPort;

#ifdef _WIN32
#pragma warning(pop)
#endif

/** \brief Initializes a VnSerialPort structure.
 *
 * \param[in] serialport The VnSerialPort structure to initialize.
 * \return Any errors encountered. */
VnError VnSerialPort_initialize(VnSerialPort *serialport);

/** \brief Opens a serial port.
 *
 * \param[in] serialport The associated VnSerialPort structure.
 * \param[in] portName The name of the serial port to open.
 * \param[in] baudrate The baudrate to open the serial port at.
 * \return Any errors encountered. */
VnError VnSerialPort_open(VnSerialPort *serialport, char const *portName, uint32_t baudrate);

/** \brief Closes the serial port.
 *
 * \param[in] serialport The associated VnSerialPort structure.
 * \return Any errors encountered. */
VnError VnSerialPort_close(VnSerialPort *serialport);

/** \brief Indicates if the serial port is open or not.
 *
 * \param[in] serialport The associated VnSerialPort structure.
 * \return <c>true</c> if the serial port is open; otherwise <c>false</c>. */
bool VnSerialPort_isOpen(VnSerialPort *serialport);

/** \brief Reads data from a serial port.
 *
 * \param[in] serialport The associated VnSerialPort structure.
 * \param[in] buffer Buffer to place the read bytes.
 * \param[in] numOfBytesToRead The number of bytes to read from the serial port.
 * \param[out] numOfBytesActuallyRead The number of bytes actually read from the serial port.
 * \return Any errors encountered. */
VnError VnSerialPort_read(VnSerialPort *serialport, char *buffer, size_t numOfBytesToRead, size_t *numOfBytesActuallyRead);

/** \brief Writes data out of a serial port.
 *
 * \param[in] serialport The associated VnSerialPort.
 * \param[in] data The data to write out.
 * \param[in] numOfBytesToWrite The number of bytes to write out of the serial port.
 * \return Any errors encountered. */
VnError VnSerialPort_write(VnSerialPort *serialport, char const *data, size_t numOfBytesToWrite);

/** \brief Changes the baudrate the port is connected at.
*
* \param[in] serialport The associated VnSerialPort.
* \param[in] baudrate The new baudrate.
* \return Any errors encountered. */
VnError VnSerialPort_changeBaudrate(VnSerialPort *serialport, uint32_t baudrate);

/** \brief Allows registering for notification of data received events.
 *
 * \param[in] serialPort The associated VnSerialPort.
 * \param[in] handler The callback method to receive notifications.
 * \param[in] userData User supplied data that will be sent to the handler on callbacks.
 * \return Any errors encountered. */
VnError VnSerialPort_registerDataReceivedHandler(VnSerialPort *serialPort, VnSerialPort_DataReceivedHandler handler, void *userData);

/** \brief Allows unregistering for notification of data received events.
*
* \param[in] serialPort The associated VnSerialPort. */
VnError VnSerialPort_unregisterDataReceivedHandler(VnSerialPort *serialPort);

#ifdef __cplusplus
}
#endif

#endif
