/*!
 *	\file		sbgCom.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		18/07/07
 *
 *	\brief		Main header file for sbgCom library.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */

/*!
 *	\mainpage sbgCom library documentation
 *	Welcome to the sbgCom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgCom library.
 */

#ifndef __SBG_COM_H__
#define __SBG_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgCommon.h"
#include "time/sbgTime.h"
#include "protocol/protocol.h"
#include "protocol/protocolOutputMode.h"
#include "protocol/protocolOutput.h"
#include "protocol/commands.h"
#include "protocol/commandsCalib.h"
#include "protocol/commandsFilter.h"
#include "protocol/commandsOrientation.h"
#include "protocol/commandsOutput.h"
#include "protocol/commandsNav.h"
#include "protocol/commandsIg30.h"
#include "protocol/commandsExt.h"
#include "protocol/commandsSync.h"
#include "protocol/commandsOdo.h"
#include "protocol/extDevices/extIg.h"
#include "protocol/extDevices/extNmea.h"

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
SbgErrorCode sbgComInit(const char *deviceName, uint32 baudRate, SbgProtocolHandle *pHandle);

/*!
 *	Close the COM port and release memory for the sbgCom library.
 *	\param[in]	handle							The sbgCom library handle to release.
 *	\return										SBG_NO_ERROR if we have released the handle.
 */
SbgErrorCode sbgComClose(SbgProtocolHandle handle);

/*!
 *	Returns an integer representing the version of the sbgCom library.
 *	\return										An integer representing the version of the sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgComGetVersion(void);

/*!
 *	Retreive the sbgCom library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCom library version.
 */
const char *sbgComGetVersionAsString(void);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgComErrorToString(SbgErrorCode errorCode, char errorMsg[256]);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	// __SBG_COM_H__
