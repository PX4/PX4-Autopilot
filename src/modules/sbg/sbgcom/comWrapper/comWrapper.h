/*!
 *	\file		comWrapper.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		18/07/07
 *
 *	\brief		Wrapper declaration for low-level serial commuication functions.<br>
 *				You can provide your own implementation adapated to your platform.<br>
 *				Copyright 2007-2008 SBG Systems. All rights reserved.
 */

#ifndef __COM_WRAPPER_H__
#define __COM_WRAPPER_H__

#include "../sbgCommon.h"

//------------------------------------------------------------------------------//
//- SBG Device definitions                                                     -//
//------------------------------------------------------------------------------//

/// The device handle is different on each os
typedef void* SbgDeviceHandle;

#define SBG_INVALID_DEVICE_HANDLE	((const SbgDeviceHandle)-1)		/*!< Identify an invalid device handle. */
#define SBG_SERIAL_TX_BUFFER_SIZE			(2048)					/*!< Define the transmission buffer size for the serial port. */
#define SBG_SERIAL_RX_BUFFER_SIZE			(2048)					/*!< Define the reception buffer size for the serial port. */

/*!
 *	Used with sbgSetEscapeComm to define the DTR and RTS pins states.
 */
typedef enum _SbgEscapeComm
{
	SBG_SET_DTR,				/*!< Set DTR pin to high. */
	SBG_CLR_DTR,				/*!< Set DTR pin to low. */
	SBG_SET_RTS,				/*!< Set RTS pin to high. */
	SBG_CLR_RTS					/*!< Set RTS pin to low. */
} SbgEscapeComm;

//------------------------------------------------------------------------------//
//- SBG Device operations                                                      -//
//------------------------------------------------------------------------------//

/*!
 * Open the specified device at a specified baud and create a new device handle
 * \param[in]	deviceName			Device name / address (COM21 , /dev/ttys0, depending on platform)
 * \param[in]	baudRate			Baudrate to be used
 * \param[out]	pHandle				Device handle returned
 * \return							SBG_NO_ERROR if the device could be oppened properly
 */
SbgErrorCode sbgDeviceOpen(const char *deviceName, uint32 baudRate, SbgDeviceHandle *pHandle);

/*!
 * Close the device
 * \param[in]	handle				Device handle to be closed
 * \return							SBG_NO_ERROR if the device could be closed properly
 */
SbgErrorCode sbgDeviceClose(SbgDeviceHandle handle);

/*!
 * Change the baud rate out the opened device
 * \param[in]	handle				Device handle returned
 * \param[in]	baudRate			New baudrate to apply on COM port
 * \return							SBG_NO_ERROR if baudrate could be changed properly
 */
SbgErrorCode sbgDeviceChangeBaud(SbgDeviceHandle handle, uint32 baudRate);

/*!
 * Write some bytes to the tx queue
 * \param[in]	handle				Device handle returned
 * \param[in]	pBuffer				Buffer to be sent through interface
 * \param[in]	numBytesToWrite		Size of the buffer in bytes
 * \return							SBG_NO_ERROR if write could be done
 */
SbgErrorCode sbgDeviceWrite(SbgDeviceHandle handle, const void *pBuffer, uint32 numBytesToWrite);

/*!
 * Read some bytes from the rx queue
 * \param[in]	handle				Device handle returned
 * \param[in]	pBuffer				Buffer to be sent through interface
 * \param[in]	numBytesToRead		Max number of bytes to be read
 * \param[out]	pNumBytesRead		Actual number read by the function
 * \return							SBG_NO_ERROR if one or more bytes read
 */
SbgErrorCode sbgDeviceRead(SbgDeviceHandle handle, void *pBuffer, uint32 numBytesToRead, uint32 *pNumBytesRead);

/*!
 * Flush the RX and TX buffers (remove all old data)
 * \param[in]	handle				Device handle returned
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgDeviceFlush(SbgDeviceHandle handle);

/*!
 *	Defines the serial DTR and RTS pins states.
 *	\param[in]	handle				The serial communication handle.
 *	\param[in]	function			One of the SbgEscapeComm enum function.
 *	\return							SBG_NO_ERROR if the pin state has been defined.
 */
SbgErrorCode sbgSetEscapeComm(SbgDeviceHandle handle, SbgEscapeComm function);

#endif
