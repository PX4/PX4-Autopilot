#ifndef _SEARCHER_H_
#define _SEARCHER_H_

#include "vn/bool.h"
#include "vn/int.h"
#include "vn/xplat/serialport.h"
#include "vn/xplat/thread.h"

/* These defines are used to enable a single function name while implementing */
/* different solutions given the OS. */
#if defined __linux__ || defined __CYGWIN__
	#define VnSearcher_findPorts VnSearcher_findPorts_LINUX
#elif defined _WIN32
	#define VnSearcher_findPorts VnSearcher_findPorts_WIN32
#else
	#error ERROR: System not yet supported in VnSearcher
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4820)
#endif

/** \brief containing information about the port to be searched. */
typedef struct
{
	/** \brief Contains the name of the port. */
	char* portName;

	/** \brief VnThread object for callback purposes. */
	VnThread thread;

	/** \brief Baud of the attached senosr, -1 for no sensor, -2 for error. */
	int32_t baud;

	/** \brief size of the data array */
	size_t dataSize;

	/** \brief VnSerialPort object to handle communication to the sensor. */
	VnSerialPort* port;

	/** \brief Array to store partial/completed communication data from the sensor. */
	char data[255];
} VnPortInfo;

#ifdef _WIN32
#pragma warning(pop)
#endif

/** \brief Takes a VnPortInfo and returns the name of the port.
 * \parm[in] portInfo Pointer to a VnPortInfo struct
 * \return The name of the port
 */
char* VnSearcher_getPortName(VnPortInfo* portInfo);

/** \brief Takes a VnPortInfo and returns the baud rate or error code.
 * \parm[in] portInfo Pointer to a VnPortInfo struct
 * \return The baud rate of the port or an error code.
 * -1 indicates no sensor on the port.
 * -2 indicates an error trying to find a sensor.
 */
int32_t VnSearcher_getPortBaud(VnPortInfo* portInfo);

/** \brief Function to find the names of all of the system's serial ports.
 * \parm[out] portNames Array containing the names of all of the serial ports.
 * \parm[out] numPortsFound Number of serial ports found.
 */
void VnSearcher_findPorts(char*** portNames, int32_t* numPortsFound);

/** \brief Function to search an input port and return either the baud rate of the connected
 * sensor or an error code.
 * \parm[in] portName The system name of the port to be searched.
 * \parm[out] foundBaudrate Baud rate of the attacked sensor, -1 for no sensor, -2 for error
 */
void VnSearcher_findPortBaud(char* portName, int32_t* foundBaudrate);

/** \brief Convenience function to find all available sensors currently attached
 * to the system.
 * \parm[out] sensorsFound Pointer to an array of sensors attached to the system.
 * \parm[out] numSensors The number of sensors found.
 */
void VnSearcher_findAllSensors(VnPortInfo*** sensorsFound, int32_t* numSensors);

/** \brief Function that will search all of the input ports to see if an available sensor is attached to it.
 * NOTE: The OS will not allow the detection of sensor that is in use.
 * \parm[in] portsToCheck Array containing the names of all the ports to check.
 * \parm[in] Number of ports to check.
 * \parm[out] An array of VnPortInfo structs the same size as numPortsToCheck.  The baud rate will indicate
 * if a sensor is attached.  baud > 0 is the sensor baudrate, baud = -1 no sensor attached, and baud = -2
 * an error occured while detecting the sensor.
 */
void VnSearcher_searchInputPorts(char*** portsToCheck, int32_t numPortsToCheck, VnPortInfo*** sensorsFound);

#endif
