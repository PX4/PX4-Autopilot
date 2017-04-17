/*!
 *	\file		protocol.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		02/04/07
 *
 *	\brief		Implementation of the IG devices communication protocol.<br>
 *				You can access low-level communication with the device.<br>
 *				Copyright 2007-20011 SBG Systems. All rights reserved.
 *
 *	You will find below, the frame definition used by IG devices.<br>
 *	<br>
 *	<table>
 *	<caption>Frame structure</caption>
 *		<tr align="center"><td>Fields</td>			<td>SYNC</td>	<td>STX</td>	<td>CMD</td>	<td>SIZE MSB</td>	<td>SIZE LSB</td>	<td>DATA</td>		<td>CRC MSB</td>	<td>CRC LSB</td>	<td>ETX</td></tr>
 *		<tr align="center"><td>Size in bytes</td>	<td>1</td>		<td>1</td>		<td>1</td>		<td>1</td>			<td>1</td>			<td>(0-504)</td>	<td>1</td>			<td>1</td>			<td>1</td></tr>
 *		<tr align="center"><td>Value</td>			<td>0xFF</td>	<td>0x02</td>	<td>?</td>		<td>?</td>			<td>?</td>			<td>?</td>			<td>?</td>			<td>?</td>			<td>0x03</td></tr>
 *	</table>
 *	<br>
 *	Size in bytes indicates the size of the data field.<br>
 *	The minimum frame size is 8 bytes and the maximum is 512 bytes.<br>
 *	<br>
 *	The CRC is calculated on the whole frame without:<br>
 *	SYNC STX CRC and ETX fields.<br>
 */
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "../sbgCommon.h"
#include "../comWrapper/comWrapper.h"
#include "protocolOutput.h"

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#define SBG_RX_BUFFER_SIZE					(SBG_SERIAL_RX_BUFFER_SIZE)		/*!< Reception buffer size. */
#define SBG_FRAME_RECEPTION_TIME_OUT		(450)							/*!< Default time out for new frame reception. */
#define SBG_MAX_DATA_LENGTH					(1016)							/*!< Maximum size of the data part of the frame. */

#define SBG_SYNC							(0xFF)							/*!< First SYNC char of the frame. */
#define SBG_STX								(0x02)							/*!< Second STX char of the frame. */
#define SBG_ETC								(0x03)							/*!< Last ETC char of the frame. */

#define SBG_INVALID_PROTOCOL_HANDLE			(NULL)							/*!< Identify an invalid protocol handle. */

//----------------------------------------------------------------------//
//- Communication protocol structs and definitions                     -//
//----------------------------------------------------------------------//

/*!
 *	Struct containing all protocol related data.
 */
typedef struct _SbgProtocolHandleInt
{
	uint8 serialBuffer[SBG_RX_BUFFER_SIZE];				/*!< The reception buffer */
	uint16 serialBufferSize;							/*!< Size of the reception buffer */
	SbgDeviceHandle serialHandle;						/*!< Handle to the device */

	uint8 targetOutputMode;								/*!< Define target settings (big/little endian and float/fixed) */
	uint32 targetDefaultOutputMask;						/*!< Define default output mask for SBG_GET_DEFAULT_OUTPUT_MASK command */

	void (*pUserHandlerContinuousError)(struct _SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg);					/*!< Function pointer that should be called when we have an error on a continuous/triggered operation */
	void (*pUserHandlerTriggeredOutput)(struct _SbgProtocolHandleInt *pHandler, uint32 triggerMask, SbgOutput *pOutput, void *pUsrArg);	/*!< Function pointer that should be called when we receive a new triggered frame */
	void (*pUserHandlerDefaultOutput)(struct _SbgProtocolHandleInt *pHandler, SbgOutput *pOutput, void *pUsrArg);						/*!< Function pointer that should be called when we receive a new continous frame */
	
	void *pUserArgContinuousError;						/*!< User defined data passed to the continuous error callback function. */
	void *pUserArgDefaultOutput;						/*!< User defined data passed to the continuous callback function */
	void *pUserArgTriggeredOutput;						/*!< User defined data passed to the Triggered output callback function */

} SbgProtocolHandleInt;

/*!
 *	Function pointer definition for continuous error callback.<br>
 *	This callback is called each time we have an error on a continuous frame.
 *	\param[in]	pHandler								The associated protocol handle.
 *	\param[in]	errorCode								Error code that occured on a continuous mode operation.
 *	\param[in]	pUsrArg									Pointer to the user defined argument.
 */
typedef void (*ContinuousErrorCallback)(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg);

/*!
 *	Function pointer definition for continuous callback.<br>
 *	This callback is called each time we read a new valid continuous frame.
 *	\param[in]	pHandler								The associated protocol handle.
 *	\param[in]	pOutput									Pointer to the filled SbgOutput struct.<br>
 *														Don't delete output as you don't have the ownership.
 *	\param[in]	pUsrArg									Pointer to the user defined argument.
 */
typedef void (*ContinuousModeCallback)(SbgProtocolHandleInt *pHandler, SbgOutput *pOutput, void *pUsrArg);

/*!
 *	Function pointer definition for triggered output callback.<br>
 *	This callback is called each time we read a new valid triggered frame.
 *	\param[in]	pHandler								The associated protocol handle.
 *  \param[in]	triggerMask								Trigger bit mask indicating which data have generated the triggered output
 *	\param[in]	pOutput									Pointer to the filled SbgOutput struct.<br>
 *														Don't delete output as you don't have the ownership.
 *	\param[in]	pUsrArg									Pointer to the user defined argument.
 */
typedef void (*TriggeredModeCallback)(SbgProtocolHandleInt *pHandler,uint32 triggerMask, SbgOutput *pOutput, void *pUsrArg);

/*!
 *	Handle type used by the protocol system.
 */
typedef SbgProtocolHandleInt* SbgProtocolHandle;

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Init the protocol system used to communicate with the product and return the created handle.
 *	\param[in]	deviceName				Device location to open such as COM1 on windows platforms.
 *	\param[in]	baudRate				Baud rate to use to communication with the device such as 115200.
 *	\param[out]	pHandle					Pointer used to handle the created sbgCom library handle.
 *	\return								SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgProtocolInit(const char *deviceName, uint32 baudRate, SbgProtocolHandle *pHandle);

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	handle					A valid sbgCom library handle to close.
 *	\return								SBG_NO_ERROR if we have closed and released the protocol system.
 */
SbgErrorCode sbgProtocolClose(SbgProtocolHandle handle);

/*!
 *	Compute a CRC 16 for a specified buffer using a polynomial 0x8408 
 *	\param[in]	pFrame						Buffer to compute the CRC on.
 *	\param[in]	bufferSize					Buffer size in bytes.
 *	\return									CRC 16 computed for the buffer.
 */
uint16 sbgProtocolCalcCRC(const void *pFrame, uint16 bufferSize);

/*!
 *	Flush all data and the serial com port.
 *	\param[in]	handle		A valid sbgCom library handle.
 *	\return					SBG_NO_ERROR if the data has been flused.
 */
SbgErrorCode sbgProtocolFlush(SbgProtocolHandle handle);

/*!
 *	Change the baud rate used to communication with the device.<br>
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	baudRate				New baudrate to use.
 *	\return								SBG_NO_ERROR if we have changed the serial baud rate.
 */
SbgErrorCode sbgProtocolChangeBaud(SbgProtocolHandle handle, uint32 baudRate);

/*!
 *	Send a frame to the device (size should be less than 504 bytes).
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[in]	cmd						Command number to send.
 *	\param[in]	pData					Pointer to the data field to send.
 *	\param[in]	size					Size of the data field to send.
 *	\return								SBG_NO_ERROR if the frame has been sent.
 */
SbgErrorCode sbgProtocolSend(SbgProtocolHandle handle, uint8 cmd, const void *pData, uint16 size);

/*!
 *	Try to receive a frame from the device and returns the cmd, data and size of data field (maxSize less than 504 bytes)
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 */
SbgErrorCode sbgProtocolReceive(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize);

/*!
 *	Try to receive a frame during a time out.
 *	This function also handle continuous frame present in the serial buffer.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolReceiveTimeOut(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize);

/*!
 *	Try to receive a frame during a time out.
 *	This function also handle continuous frame present in the serial buffer.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pCmd					Pointer to hold the returned command.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOutMs				Time in millisecond during which a frame can be received.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolReceiveTimeOutMs(SbgProtocolHandle handle, uint8 *pCmd, void *pData, uint16 *pSize, uint16 maxSize, uint32 timeOutMs);

/*!
 *	Handles a received continuous frame and call the user continuous mode callback if the received frame is valid.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pData					Received data field buffer.
 *	\param[out]	size					Size of the received data field buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolManageContinuousFrame(SbgProtocolHandle handle, uint8 *pData, uint16 size);

/*!
 *	Handles a received triggered frame and call the user triggered mode callback if the received frame is valid.
 *	\param[in]	handle					A valid sbgCom library handle.
 *	\param[out]	pFullFrame				Received data field buffer.
 *	\param[out]	size					Size of the received data field buffer.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgProtocolManageTriggeredFrame(SbgProtocolHandle handle, uint8 *pFullFrame, uint16 size);

/*!
 *	Scan the serial port for continuous frames.<br>
 *	For each received frame, call the continuous user callback function.<br>
 *	For each error on a frame reception, call the continuous error callback function.<br>
 *	This function returns only when no more frames are available in the serial port.
 *	\param[in]	handle		A valid sbgCom library handle.
 *	\return					SBG_NO_ERROR if all the frames has been read sucessfully.
 */
SbgErrorCode sbgProtocolContinuousModeHandle(SbgProtocolHandle handle);

/*!
 *	Defines the handle function to call when we have received a valid triggered frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the Triggered frame handler function.
 *	\param[in]	pUserArg					User argument to pass to the triggered frame handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetTriggeredModeCallback(SbgProtocolHandle handle, TriggeredModeCallback callback, void *pUserArg);

/*!
 *	Defines the handle function to call when we have an error on a continuous frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the error handler function.
 *	\param[in]	pUserArg					User argument to pass to the error handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetContinuousErrorCallback(SbgProtocolHandle handle, ContinuousErrorCallback callback, void *pUserArg);

/*!
 *	Defines the handle function to call when we have received a valid continuous frame.
 *	\param[in]	handle						A valid sbgCom library handle.
 *	\param[in]	callback					Pointer to the Continuous frame handler function.
 *	\param[in]	pUserArg					User argument to pass to the continuous frame handler function.
 *	\return									SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgSetContinuousModeCallback(SbgProtocolHandle handle, ContinuousModeCallback callback, void *pUserArg);

#endif
