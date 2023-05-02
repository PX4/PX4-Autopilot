#include "vn/sensors.h"
#include "vn/error.h"
#include "vn/util.h"

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "vn/xplat/time.h"

//#define UNUSED(x) (void)(sizeof(x))

#define DEFAULT_RESPONSE_TIMEOUT_MS	500
#define DEFAULT_RETRANSMIT_DELAY_MS	200
#define DEFAULT_READ_BUFFER_SIZE	256
#define COMMAND_MAX_LENGTH 0x100

void VnSensor_dataReceivedHandler(void *userData);
void VnSensor_packetFoundHandler(void *userData, VnUartPacket *packet, size_t runningIndex);

VnError VnSensor_transactionNoFinalize(
	VnSensor *s,
	char *toSend,
	size_t toSendLength,
	bool waitForReply,
	char *responseBuffer,
	size_t *responseSize);

VnError VnSensor_transactionNoFinalizeWithTiming(
	VnSensor *s,
	char *toSend,
	size_t toSendLength,
	bool waitForReply,
	char *responseBuffer,
	size_t* responseBufferSize,
	uint16_t responseTimeoutMs,
	uint16_t retransmitDelayMs);



void strFromSensorError(char *out, SensorError val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case ERR_HARD_FAULT:
		strcpy(out, "HardFault");
		break;
	case ERR_SERIAL_BUFFER_OVERFLOW:
		strcpy(out, "SerialBufferOverflow");
		break;
	case ERR_INVALID_CHECKSUM:
		strcpy(out, "InvalidChecksum");
		break;
	case ERR_INVALID_COMMAND:
		strcpy(out, "InvalidCommand");
		break;
	case ERR_NOT_ENOUGH_PARAMETERS:
		strcpy(out, "NotEnoughParameters");
		break;
	case ERR_TOO_MANY_PARAMETERS:
		strcpy(out, "TooManyParameters");
		break;
	case ERR_INVALID_PARAMETER:
		strcpy(out, "InvalidParameter");
		break;
	case ERR_INVALID_REGISTER:
		strcpy(out, "InvalidRegister");
		break;
	case ERR_UNAUTHORIZED_ACCESS:
		strcpy(out, "UnauthorizedAccess");
		break;
	case ERR_WATCHDOG_RESET:
		strcpy(out, "WatchdogReset");
		break;
	case ERR_OUTPUT_BUFFER_OVERFLOW:
		strcpy(out, "OutputBufferOverflow");
		break;
	case ERR_INSUFFICIENT_BAUD_RATE:
		strcpy(out, "InsufficientBaudRate");
		break;
	case ERR_ERROR_BUFFER_OVERFLOW:
		strcpy(out, "ErrorBufferOverflow");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void BinaryOutputRegister_initialize(
	BinaryOutputRegister *reg,
	AsyncMode asyncMode,
	uint32_t rateDivisor,
	CommonGroup commonField,
	TimeGroup timeField,
	ImuGroup imuField,
	GpsGroup gpsField,
	AttitudeGroup attitudeField,
	InsGroup insField,
  GpsGroup gps2Field)
{
	reg->asyncMode = asyncMode;
	reg->rateDivisor = (uint16_t)rateDivisor;
	reg->commonField = commonField;
	reg->timeField = timeField;
	reg->imuField = imuField;
	reg->gpsField = gpsField;
	reg->attitudeField = attitudeField;
	reg->insField = insField;
  reg->gps2Field = gps2Field;
}

VnError VnSensor_initialize(VnSensor* s)
{
	VnSerialPort_initialize(&s->serialPort);
	s->sendErrorDetectionMode = VNERRORDETECTIONMODE_CHECKSUM;
	s->responseTimeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS;
	s->retransmitDelayMs = DEFAULT_RETRANSMIT_DELAY_MS;
	VnCriticalSection_initialize(&s->transactionCS);
	s->responseWaitingForProcessing = false;
	s->waitingForResponse = false;
	s->bootloaderFilter = false;
	VnEvent_initialize(&s->newResponsesEvent);
	s->responseLength = 0;
	s->runningDataIndex = 0;
	VnUartPacketFinder_initialize(&s->packetFinder);
	s->asyncPacketFoundHandler = NULL;
	s->asyncPacketFoundHandlerUserData = NULL;
	s->errorMessageReceivedHandler = NULL;
	s->errorMessageReceivedHandlerUserData = NULL;

	VnUartPacketFinder_registerPacketFoundHandler(&s->packetFinder, VnSensor_packetFoundHandler, s);

	return E_NONE;
}

VnError VnSensor_connect(VnSensor *s, const char *portName, uint32_t baudrate)
{
	VnError error;

	if (VnSensor_isConnected(s))
		return E_INVALID_OPERATION;

	if ((error = VnSerialPort_open(&s->serialPort, portName, baudrate)) != E_NONE)
		return error;

	VnSerialPort_registerDataReceivedHandler(&s->serialPort, VnSensor_dataReceivedHandler, s);

	return E_NONE;
}

VnError VnSensor_disconnect(VnSensor* s)
{
	VnError error;

	if (!VnSensor_isConnected(s))
		return E_INVALID_OPERATION;

	VnSerialPort_unregisterDataReceivedHandler(&s->serialPort);

	if ((error = VnSerialPort_close(&s->serialPort)) != E_NONE)
		return error;

	return E_NONE;
}

VnError VnSensor_changeBaudrate(VnSensor* s, uint32_t baudrate)
{
	VnError error;

	if ((error = VnSensor_writeSerialBaudRate(s, baudrate, true)) != E_NONE)
		return error;

	return VnSerialPort_changeBaudrate(&s->serialPort, baudrate);
}

VnError VnSensor_transaction(VnSensor* s, char* toSend, size_t toSendLength, char* response, size_t* responseLength)
{
	char buffer[COMMAND_MAX_LENGTH];
	size_t finalLength = toSendLength;

	/* Copy over what was provided. */
	memcpy(buffer, toSend, toSendLength);

	/* Add null termination for string operations below. */
	buffer[toSendLength] = '\0';

	/* First see if an '*' is present. */
	if (strchr(buffer, '*') == NULL)
	{
		VnUartPacket_finalizeCommand(s->sendErrorDetectionMode, (uint8_t*)buffer, &finalLength);
	}
	else if (buffer[finalLength - 2] != '\r' && buffer[finalLength - 1] != '\n')
	{
		buffer[finalLength++] = '\r';
		buffer[finalLength++] = '\n';
	}

	return VnSensor_transactionNoFinalize(s, buffer, finalLength, true, response, responseLength);
}

bool VnSensor_isConnected(VnSensor* s)
{
	return VnSerialPort_isOpen(&s->serialPort);
}

VnError VnSensor_writeSettings(VnSensor *s, bool waitForReply)
{
	VnError error;
	char toSend[37];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdWriteSettings(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);
}

VnError VnSensor_restoreFactorySettings(VnSensor *s, bool waitForReply)
{
	VnError error;
	char toSend[37];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdRestoreFactorySettings(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);
}

VnError VnSensor_reset(VnSensor *s, bool waitForReply)
{
	VnError error;
	char toSend[37];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdReset(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000);

}

VnError VnSensor_firmwareUpdateMode(VnSensor* s, bool waitForReply)
{
	VnError error = E_NONE;
	char toSend[37];
	size_t toSendLength = sizeof(toSend);
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	memset(toSend, 0, toSendLength);
	memset(responseBuffer, 0, responseLength);

	if ((error = VnUartPacket_genCmdFirmwareUpdate((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &toSendLength)) != E_NONE)	return error;

	if ((error = VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength, 2500, 1000)) != E_NONE)	return error;

	return error;

}

VnError VnSensor_tare(VnSensor *s, bool waitForReply)
{
	VnError error;
	char toSend[14];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdTare(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_setGyroBias(VnSensor *s, bool waitForReply)
{
	VnError error;
	char toSend[14];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdSetGyroBias(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_magneticDisturbancePresent(VnSensor *s, bool disturbancePresent, bool waitForReply)
{
	VnError error;
	char toSend[14];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdKnownMagneticDisturbance(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		disturbancePresent,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_accelerationDisturbancePresent(VnSensor *s, bool disturbancePresent, bool waitForReply)
{
	VnError error;
	char toSend[14];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genCmdKnownAccelerationDisturbance(
		(uint8_t*)toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		disturbancePresent,
		&toSendLength)) != E_NONE)
	{
		return error;
	}

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

bool VnSensor_verifySensorConnectivity(VnSensor* sensor)
{
	char temp[50];

	return VnSensor_readModelNumber(sensor, temp, sizeof(temp)) == E_NONE;
}

uint16_t VnSensor_getResponseTimeoutMs(VnSensor* sensor)
{
	return sensor->responseTimeoutMs;
}

VnError VnSensor_setResponseTimeoutMs(VnSensor* sensor, uint16_t reponseTimeoutMs)
{
	sensor->responseTimeoutMs = reponseTimeoutMs;

	return E_NONE;
}

uint16_t VnSensor_getRetransmitDelayMs(VnSensor* sensor)
{
	return sensor->retransmitDelayMs;
}

VnError VnSensor_setRetransmitDelayMs(VnSensor* sensor, uint16_t retransmitDelayMs)
{
	sensor->retransmitDelayMs = retransmitDelayMs;

	return E_NONE;
}

VnError VnSensor_registerAsyncPacketReceivedHandler(VnSensor *s, VnSensor_PacketFoundHandler handler, void *userData)
{
	if (s->asyncPacketFoundHandler != NULL)
		return E_INVALID_OPERATION;

	s->asyncPacketFoundHandler = handler;
	s->asyncPacketFoundHandlerUserData = userData;

	return E_NONE;
}

VnError VnSensor_unregisterAsyncPacketReceivedHandler(VnSensor *s)
{
	if (s->asyncPacketFoundHandler == NULL)
		return E_INVALID_OPERATION;

	s->asyncPacketFoundHandler = NULL;
	s->asyncPacketFoundHandlerUserData = NULL;

	return E_NONE;
}

VnError VnSensor_registerErrorPacketReceivedHandler(VnSensor *s, VnSensor_PacketFoundHandler handler, void *userData)
{
	if (s->errorMessageReceivedHandler != NULL)
		return E_INVALID_OPERATION;

	s->errorMessageReceivedHandler = handler;
	s->errorMessageReceivedHandlerUserData = userData;

	return E_NONE;
}

VnError VnSensor_unregisterErrorPacketReceivedHandler(VnSensor *s)
{
	if (s->errorMessageReceivedHandler == NULL)
		return E_INVALID_OPERATION;

	s->errorMessageReceivedHandler = NULL;
	s->errorMessageReceivedHandlerUserData = NULL;

	return E_NONE;
}

void VnSensor_dataReceivedHandler(void *userData)
{
	char readBuffer[DEFAULT_READ_BUFFER_SIZE];
	size_t numOfBytesRead = 0;
	memset(readBuffer, 0, sizeof(readBuffer));

	VnSensor *s = (VnSensor*) userData;

	VnSerialPort_read(&s->serialPort, readBuffer, sizeof(readBuffer), &numOfBytesRead);

	if (numOfBytesRead == 0)	return;

	VnUartPacketFinder_processData_ex(&s->packetFinder, (uint8_t*)readBuffer, numOfBytesRead, s->bootloaderFilter );

	s->runningDataIndex += numOfBytesRead;
}

void VnSensor_onErrorPacketReceived(VnSensor *s, VnUartPacket *packet, size_t runningIndex)
{
	if (s->errorMessageReceivedHandler != NULL)
		s->errorMessageReceivedHandler(s->errorMessageReceivedHandlerUserData, packet, runningIndex);
}

void VnSensor_onAsyncPacketReceived(VnSensor *s, VnUartPacket *packet, size_t runningIndex)
{
	if (s->asyncPacketFoundHandler != NULL)
		s->asyncPacketFoundHandler(s->asyncPacketFoundHandlerUserData, packet, runningIndex);
}

void VnSensor_packetFoundHandler(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	VnSensor *s = (VnSensor*) userData;

	/* Packet should be valid at this point so no need to check its validity. */

	if (!VnUartPacket_isBootloader(packet))
	{
		s->bootloaderFilter = false;
	}

	if (VnUartPacket_isError(packet))
	{
		if (s->waitingForResponse)
		{
			VnCriticalSection_enter(&s->transactionCS);
			memcpy(s->response, packet->data, packet->length);
			s->responseLength = packet->length;
			VnEvent_signal(&s->newResponsesEvent);
			VnCriticalSection_leave(&s->transactionCS);
		}

		VnSensor_onErrorPacketReceived(s, packet, runningIndex);

		return;
	}

	if (VnUartPacket_isResponse(packet))
	{
		if (s->waitingForResponse)
		{
			VnCriticalSection_enter(&s->transactionCS);
			memcpy(s->response, packet->data, packet->length);
			s->responseLength = packet->length;
			VnEvent_signal(&s->newResponsesEvent);
			VnCriticalSection_leave(&s->transactionCS);
		}

		return;
	}

	/* This wasn't anything else. We assume it is an async packet. */
	VnSensor_onAsyncPacketReceived(s, packet, runningIndex);
}

VnError transactionWithWait(VnSensor *s, char* toSend, size_t length, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs, char *responseBuffer, size_t *responseLength)
{
	VnStopwatch timeoutSw;
	float curElapsedTime;
	VnError waitResult;
	bool loopVariable = true;

	/* Make sure we don't have any existing responses. */
	VnCriticalSection_enter(&s->transactionCS);
	s->responseWaitingForProcessing = false;
	s->waitingForResponse = true;
	VnCriticalSection_leave(&s->transactionCS);

	/* Send the command and continue sending if retransmits are enabled until
	 * we receive the response or timeout. */
	VnStopwatch_initializeAndStart(&timeoutSw);

	VnSerialPort_write(&s->serialPort, toSend, length);
	VnStopwatch_elapsedMs(&timeoutSw, &curElapsedTime);

	/* loopVariable is used to avoid a compile variable */
	while (loopVariable)
	{
		bool shouldRetransmit = false;

		/* Compute how long we should wait for a response before taking more
		 * action. */
		float responseWaitTime = responseTimeoutMs - curElapsedTime;
		if (responseWaitTime > retransmitDelayMs)
		{
			responseWaitTime = retransmitDelayMs;
			shouldRetransmit = true;
		}

		/* See if we have time left. */
		if (responseWaitTime < 0)
		{
			s->waitingForResponse = false;
			return E_TIMEOUT;
		}

		/* Wait for any new responses that come in or until it is time to send
		 * a new retransmit. */
		waitResult = VnEvent_waitMs(&s->newResponsesEvent, (uint32_t) responseWaitTime);

		if (waitResult == E_TIMEOUT)
		{
			if (!shouldRetransmit)
			{
				s->waitingForResponse = false;
				return E_TIMEOUT;
			}
		}

		if (waitResult == E_SIGNALED)
		{
			/* Get the current collection of responses. */
			VnCriticalSection_enter(&s->transactionCS);
			memcpy(responseBuffer, s->response, s->responseLength);
			*responseLength = s->responseLength;
			VnCriticalSection_leave(&s->transactionCS);

			/* Process the collection of responses we have. */
			if (VnUartPacket_isErrorRaw((uint8_t*)responseBuffer))
			{
				uint8_t sensorError;

				s->waitingForResponse = false;
				VnUartPacket_parseErrorRaw((uint8_t*)responseBuffer, &sensorError);

				return (VnError) (sensorError + E_SENSOR_HARD_FAULT - 1);
			}

			/* We must have a response packet. */
			s->waitingForResponse = false;

			return E_NONE;
		}

		/* Retransmit. */
		VnSerialPort_write(&s->serialPort, toSend, length);
		VnStopwatch_elapsedMs(&timeoutSw, &curElapsedTime);
	}

	return E_UNKNOWN;
}

VnError VnSensor_transactionNoFinalizeWithTiming(VnSensor *s, char *toSend, size_t toSendLength, bool waitForReply, char *responseBuffer, size_t* responseBufferSize, uint16_t responseTimeoutMs, uint16_t retransmitDelayMs)
{
	/* This line is used to avoid a compiler warning */
	UNUSED(responseBufferSize);

	if (!VnSensor_isConnected(s))
		return E_INVALID_OPERATION;

	if (waitForReply)
	{
		size_t responseLength;

		return transactionWithWait(s, toSend, toSendLength, responseTimeoutMs, retransmitDelayMs, responseBuffer, &responseLength);
	}
	else
	{
		VnSerialPort_write(&s->serialPort, toSend, toSendLength);
	}

	return E_NONE;
}

VnError VnSensor_transactionNoFinalize(VnSensor *s, char *toSend, size_t toSendLength, bool waitForReply, char *responseBuffer, size_t *responseSize)
{
	return VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, waitForReply, responseBuffer, responseSize, s->responseTimeoutMs, s->retransmitDelayMs);
}

VnError VnSensor_readBinaryOutput1(VnSensor *s, BinaryOutputRegister *fields)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField, gps2Field;

	if ((error = VnUartPacket_genReadBinaryOutput1((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField, &gps2Field);

	fields->asyncMode = (AsyncMode) asyncMode;
	fields->commonField = (CommonGroup) commonField;
	fields->timeField = (TimeGroup) timeField;
	fields->imuField = (ImuGroup) imuField;
	fields->gpsField = (GpsGroup) gpsField;
	fields->attitudeField = (AttitudeGroup) attitudeField;
	fields->insField = (InsGroup) insField;
  fields->gps2Field = (GpsGroup)gps2Field;

	return E_NONE;
}

VnError VnSensor_writeBinaryOutput1(VnSensor *s, BinaryOutputRegister *fields, bool waitForReply)
{
	char toSend[256];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput1((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField, fields->gps2Field)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readBinaryOutput2(VnSensor *s, BinaryOutputRegister *fields)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField, gps2Field;

	if ((error = VnUartPacket_genReadBinaryOutput2((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField, &gps2Field);

	fields->asyncMode = (AsyncMode) asyncMode;
	fields->commonField = (CommonGroup) commonField;
	fields->timeField = (TimeGroup) timeField;
	fields->imuField = (ImuGroup) imuField;
	fields->gpsField = (GpsGroup) gpsField;
	fields->attitudeField = (AttitudeGroup) attitudeField;
	fields->insField = (InsGroup) insField;
  fields->gps2Field = (GpsGroup)gps2Field;

	return E_NONE;
}

VnError VnSensor_writeBinaryOutput2(VnSensor *s, BinaryOutputRegister *fields, bool waitForReply)
{
	char toSend[256];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput2((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField, fields->gps2Field)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readBinaryOutput3(VnSensor *s, BinaryOutputRegister *fields)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField, gps2Field;

	if ((error = VnUartPacket_genReadBinaryOutput3((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField, &gps2Field);

	fields->asyncMode = (AsyncMode) asyncMode;
	fields->commonField = (CommonGroup) commonField;
	fields->timeField = (TimeGroup) timeField;
	fields->imuField = (ImuGroup) imuField;
	fields->gpsField = (GpsGroup) gpsField;
	fields->attitudeField = (AttitudeGroup) attitudeField;
	fields->insField = (InsGroup) insField;
  fields->gps2Field = (GpsGroup)gps2Field;

	return E_NONE;
}

VnError VnSensor_writeBinaryOutput3(VnSensor *s, BinaryOutputRegister *fields, bool waitForReply)
{
	char toSend[256];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput3((uint8_t*)toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField, fields->gps2Field)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

#ifdef EXTRA

VnError VnSensor_readBinaryOutput4(VnSensor *s, BinaryOutputRegister *fields)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField, gps2Field;

	if ((error = VnUartPacket_genReadBinaryOutput4(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField, &gps2Field);

	fields->asyncMode = (AsyncMode) asyncMode;
	fields->commonField = (CommonGroup) commonField;
	fields->timeField = (TimeGroup) timeField;
	fields->imuField = (ImuGroup) imuField;
	fields->gpsField = (GpsGroup) gpsField;
	fields->attitudeField = (AttitudeGroup) attitudeField;
	fields->insField = (InsGroup) insField;
  fields->gps2Field = (GpsGroup)gps2Field;

	return E_NONE;
}

VnError VnSensor_writeBinaryOutput4(VnSensor *s, BinaryOutputRegister *fields, bool waitForReply)
{
	char toSend[256];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput4(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField, fields->gps2Field)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readBinaryOutput5(VnSensor *s, BinaryOutputRegister *fields)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
  uint16_t asyncMode, outputGroup, commonField, timeField, imuField, gpsField, attitudeField, insField, gps2Field;

	if ((error = VnUartPacket_genReadBinaryOutput5(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

  VnUartPacket_parseBinaryOutputRaw((uint8_t*)responseBuffer, &asyncMode, &fields->rateDivisor, &outputGroup, &commonField, &timeField, &imuField, &gpsField, &attitudeField, &insField, &gps2Field);

	fields->asyncMode = (AsyncMode) asyncMode;
	fields->commonField = (CommonGroup) commonField;
	fields->timeField = (TimeGroup) timeField;
	fields->imuField = (ImuGroup) imuField;
	fields->gpsField = (GpsGroup) gpsField;
	fields->attitudeField = (AttitudeGroup) attitudeField;
	fields->insField = (InsGroup) insField;
  fields->gps2Field = (GpsGroup)gps2Field;

	return E_NONE;
}

VnError VnSensor_writeBinaryOutput5(VnSensor *s, BinaryOutputRegister *fields, bool waitForReply)
{
	char toSend[256];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

  if ((error = VnUartPacket_genWriteBinaryOutput5(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length, fields->asyncMode, fields->rateDivisor, fields->commonField, fields->timeField, fields->imuField, fields->gpsField, fields->attitudeField, fields->insField, fields->gps2Field)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, length, waitForReply, responseBuffer, &responseLength);
}

#endif

VnError VnSensor_readUserTag(VnSensor *s, char *tagBuffer, size_t tagBufferLength)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	/*Put this here to avoid compiler warnings.*/
	UNUSED(tagBufferLength);

	if ((error = VnUartPacket_genReadUserTag(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseUserTagRaw(responseBuffer, tagBuffer);

	return E_NONE;
}

VnError VnSensor_writeUserTag(VnSensor *s, char* tag, bool waitForReply)
{
	VnError error;
	char toSend[37];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteUserTag(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		tag)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readModelNumber(VnSensor *s, char *productNameBuffer, size_t productNameBufferLength)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	/*Put this here to avoid compiler warnings.*/
	UNUSED(productNameBufferLength);

	if ((error = VnUartPacket_genReadModelNumber(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseModelNumberRaw(responseBuffer, productNameBuffer);

	return E_NONE;
}

VnError VnSensor_readHardwareRevision(VnSensor *s, uint32_t *revision)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadHardwareRevision(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseHardwareRevisionRaw(responseBuffer, revision);

	return E_NONE;
}

VnError VnSensor_readSerialNumber(VnSensor *s, uint32_t *serialNum)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadSerialNumber(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseSerialNumberRaw(responseBuffer, serialNum);

	return E_NONE;
}

VnError VnSensor_readFirmwareVersion(VnSensor *s, char *firmwareVersionBuffer, size_t firmwareVersionBufferLength)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	/*Put this here to avoid compiler warnings.*/
	UNUSED(firmwareVersionBufferLength);

	if ((error = VnUartPacket_genReadFirmwareVersion(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseFirmwareVersionRaw(responseBuffer, firmwareVersionBuffer);

	return E_NONE;
}

VnError VnSensor_readSerialBaudRate(VnSensor *s, uint32_t *baudrate)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadSerialBaudRate(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseSerialBaudRateRaw(responseBuffer, baudrate);

	return E_NONE;
}

VnError VnSensor_writeSerialBaudRate(VnSensor *s, uint32_t baudrate, bool waitForReply)
{
	VnError error;
	char toSend[25];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteSerialBaudRate(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		baudrate)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readAsyncDataOutputType(VnSensor *s, VnAsciiAsync *ador)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint32_t adorPlaceholder;

	if ((error = VnUartPacket_genReadAsyncDataOutputType(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAsyncDataOutputTypeRaw(responseBuffer, &adorPlaceholder);

	*ador = (VnAsciiAsync) adorPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeAsyncDataOutputType(VnSensor *s, VnAsciiAsync ador, bool waitForReply)
{
	VnError error;
	char toSend[19];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteAsyncDataOutputType(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		ador)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readAsyncDataOutputFrequency(VnSensor *s, uint32_t *adof)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadAsyncDataOutputFrequency(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAsyncDataOutputFrequencyRaw(responseBuffer, adof);

	return E_NONE;
}

VnError VnSensor_writeAsyncDataOutputFrequency(VnSensor *s, uint32_t adof, bool waitForReply)
{
	VnError error;
	char toSend[26];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteAsyncDataOutputFrequency(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		adof)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readYawPitchRoll(VnSensor *s, vec3f *yawPitchRoll)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadYawPitchRoll(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseYawPitchRollRaw(responseBuffer, yawPitchRoll);

	return E_NONE;
}

VnError VnSensor_readAttitudeQuaternion(VnSensor *s, vec4f *quat)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadAttitudeQuaternion(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAttitudeQuaternionRaw(responseBuffer, quat);

	return E_NONE;
}

VnError VnSensor_readQuaternionMagneticAccelerationAndAngularRates(VnSensor *s, QuaternionMagneticAccelerationAndAngularRatesRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadQuaternionMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseQuaternionMagneticAccelerationAndAngularRatesRaw(responseBuffer, &reg->quat, &reg->mag, &reg->accel, &reg->gyro);


	return E_NONE;
}

VnError VnSensor_readMagneticMeasurements(VnSensor *s, vec3f *mag)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadMagneticMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseMagneticMeasurementsRaw(responseBuffer, mag);

	return E_NONE;
}

VnError VnSensor_readAccelerationMeasurements(VnSensor *s, vec3f *accel)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadAccelerationMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAccelerationMeasurementsRaw(responseBuffer, accel);

	return E_NONE;
}

VnError VnSensor_readAngularRateMeasurements(VnSensor *s, vec3f *gyro)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadAngularRateMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAngularRateMeasurementsRaw(responseBuffer, gyro);

	return E_NONE;
}

VnError VnSensor_readMagneticAccelerationAndAngularRates(VnSensor *s, MagneticAccelerationAndAngularRatesRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseMagneticAccelerationAndAngularRatesRaw(responseBuffer, &reg->mag, &reg->accel, &reg->gyro);


	return E_NONE;
}

VnError VnSensor_readMagneticAndGravityReferenceVectors(VnSensor *s, MagneticAndGravityReferenceVectorsRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadMagneticAndGravityReferenceVectors(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseMagneticAndGravityReferenceVectorsRaw(responseBuffer, &reg->magRef, &reg->accRef);


	return E_NONE;
}

VnError VnSensor_writeMagneticAndGravityReferenceVectors(VnSensor *s, MagneticAndGravityReferenceVectorsRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteMagneticAndGravityReferenceVectors(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.magRef,
		fields.accRef)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readMagnetometerCompensation(VnSensor *s, MagnetometerCompensationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadMagnetometerCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseMagnetometerCompensationRaw(responseBuffer, &reg->c, &reg->b);


	return E_NONE;
}

VnError VnSensor_writeMagnetometerCompensation(VnSensor *s, MagnetometerCompensationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteMagnetometerCompensation(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.c,
		fields.b)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readAccelerationCompensation(VnSensor *s, AccelerationCompensationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadAccelerationCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseAccelerationCompensationRaw(responseBuffer, &reg->c, &reg->b);


	return E_NONE;
}

VnError VnSensor_writeAccelerationCompensation(VnSensor *s, AccelerationCompensationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteAccelerationCompensation(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.c,
		fields.b)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readReferenceFrameRotation(VnSensor *s, mat3f *c)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadReferenceFrameRotation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseReferenceFrameRotationRaw(responseBuffer, c);

	return E_NONE;
}

VnError VnSensor_writeReferenceFrameRotation(VnSensor *s, mat3f c, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteReferenceFrameRotation(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		c)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readYawPitchRollMagneticAccelerationAndAngularRates(VnSensor *s, YawPitchRollMagneticAccelerationAndAngularRatesRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadYawPitchRollMagneticAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseYawPitchRollMagneticAccelerationAndAngularRatesRaw(responseBuffer, &reg->yawPitchRoll, &reg->mag, &reg->accel, &reg->gyro);


	return E_NONE;
}

VnError VnSensor_readCommunicationProtocolControl(VnSensor *s, CommunicationProtocolControlRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t serialCountPlaceholder;
	uint8_t serialStatusPlaceholder;
	uint8_t spiCountPlaceholder;
	uint8_t spiStatusPlaceholder;
	uint8_t serialChecksumPlaceholder;
	uint8_t spiChecksumPlaceholder;
	uint8_t errorModePlaceholder;

	if ((error = VnUartPacket_genReadCommunicationProtocolControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseCommunicationProtocolControlRaw(responseBuffer, &serialCountPlaceholder, &serialStatusPlaceholder, &spiCountPlaceholder, &spiStatusPlaceholder, &serialChecksumPlaceholder, &spiChecksumPlaceholder, &errorModePlaceholder);

	reg->serialCount = (VnCountMode) serialCountPlaceholder;
	reg->serialStatus = (VnStatusMode) serialStatusPlaceholder;
	reg->spiCount = (VnCountMode) spiCountPlaceholder;
	reg->spiStatus = (VnStatusMode) spiStatusPlaceholder;
	reg->serialChecksum = (VnChecksumMode) serialChecksumPlaceholder;
	reg->spiChecksum = (VnChecksumMode) spiChecksumPlaceholder;
	reg->errorMode = (VnErrorMode) errorModePlaceholder;

	return E_NONE;
}

VnError VnSensor_writeCommunicationProtocolControl(VnSensor *s, CommunicationProtocolControlRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteCommunicationProtocolControl(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.serialCount,
		fields.serialStatus,
		fields.spiCount,
		fields.spiStatus,
		fields.serialChecksum,
		fields.spiChecksum,
		fields.errorMode)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readSynchronizationControl(VnSensor *s, SynchronizationControlRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t syncInModePlaceholder;
	uint8_t syncInEdgePlaceholder;
	uint32_t reserved1Placeholder;
	uint8_t syncOutModePlaceholder;
	uint8_t syncOutPolarityPlaceholder;
	uint32_t reserved2Placeholder;

	if ((error = VnUartPacket_genReadSynchronizationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseSynchronizationControlRaw(responseBuffer, &syncInModePlaceholder, &syncInEdgePlaceholder, &reg->syncInSkipFactor, &reserved1Placeholder, &syncOutModePlaceholder, &syncOutPolarityPlaceholder, &reg->syncOutSkipFactor, &reg->syncOutPulseWidth, &reserved2Placeholder);

	reg->syncInMode = (VnSyncInMode) syncInModePlaceholder;
	reg->syncInEdge = (VnSyncInEdge) syncInEdgePlaceholder;
	reg->syncOutMode = (VnSyncOutMode) syncOutModePlaceholder;
	reg->syncOutPolarity = (VnSyncOutPolarity) syncOutPolarityPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeSynchronizationControl(VnSensor *s, SynchronizationControlRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteSynchronizationControl(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.syncInMode,
		fields.syncInEdge,
		fields.syncInSkipFactor,
		0,
		fields.syncOutMode,
		fields.syncOutPolarity,
		fields.syncOutSkipFactor,
		fields.syncOutPulseWidth,
		0)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readSynchronizationStatus(VnSensor *s, SynchronizationStatusRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadSynchronizationStatus(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseSynchronizationStatusRaw(responseBuffer, &reg->syncInCount, &reg->syncInTime, &reg->syncOutCount);


	return E_NONE;
}

VnError VnSensor_writeSynchronizationStatus(VnSensor *s, SynchronizationStatusRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteSynchronizationStatus(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.syncInCount,
		fields.syncInTime,
		fields.syncOutCount)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readVpeBasicControl(VnSensor *s, VpeBasicControlRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t enablePlaceholder;
	uint8_t headingModePlaceholder;
	uint8_t filteringModePlaceholder;
	uint8_t tuningModePlaceholder;

	if ((error = VnUartPacket_genReadVpeBasicControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseVpeBasicControlRaw(responseBuffer, &enablePlaceholder, &headingModePlaceholder, &filteringModePlaceholder, &tuningModePlaceholder);

	reg->enable = (VnVpeEnable) enablePlaceholder;
	reg->headingMode = (VnHeadingMode) headingModePlaceholder;
	reg->filteringMode = (VnVpeMode) filteringModePlaceholder;
	reg->tuningMode = (VnVpeMode) tuningModePlaceholder;

	return E_NONE;
}

VnError VnSensor_writeVpeBasicControl(VnSensor *s, VpeBasicControlRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteVpeBasicControl(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.enable,
		fields.headingMode,
		fields.filteringMode,
		fields.tuningMode)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readVpeMagnetometerBasicTuning(VnSensor *s, VpeMagnetometerBasicTuningRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadVpeMagnetometerBasicTuning(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseVpeMagnetometerBasicTuningRaw(responseBuffer, &reg->baseTuning, &reg->adaptiveTuning, &reg->adaptiveFiltering);


	return E_NONE;
}

VnError VnSensor_writeVpeMagnetometerBasicTuning(VnSensor *s, VpeMagnetometerBasicTuningRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteVpeMagnetometerBasicTuning(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.baseTuning,
		fields.adaptiveTuning,
		fields.adaptiveFiltering)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readVpeAccelerometerBasicTuning(VnSensor *s, VpeAccelerometerBasicTuningRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadVpeAccelerometerBasicTuning(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseVpeAccelerometerBasicTuningRaw(responseBuffer, &reg->baseTuning, &reg->adaptiveTuning, &reg->adaptiveFiltering);


	return E_NONE;
}

VnError VnSensor_writeVpeAccelerometerBasicTuning(VnSensor *s, VpeAccelerometerBasicTuningRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteVpeAccelerometerBasicTuning(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.baseTuning,
		fields.adaptiveTuning,
		fields.adaptiveFiltering)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readMagnetometerCalibrationControl(VnSensor *s, MagnetometerCalibrationControlRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t hsiModePlaceholder;
	uint8_t hsiOutputPlaceholder;

	if ((error = VnUartPacket_genReadMagnetometerCalibrationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseMagnetometerCalibrationControlRaw(responseBuffer, &hsiModePlaceholder, &hsiOutputPlaceholder, &reg->convergeRate);

	reg->hsiMode = (VnHsiMode) hsiModePlaceholder;
	reg->hsiOutput = (VnHsiOutput) hsiOutputPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeMagnetometerCalibrationControl(VnSensor *s, MagnetometerCalibrationControlRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteMagnetometerCalibrationControl(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.hsiMode,
		fields.hsiOutput,
		fields.convergeRate)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readCalculatedMagnetometerCalibration(VnSensor *s, CalculatedMagnetometerCalibrationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadCalculatedMagnetometerCalibration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseCalculatedMagnetometerCalibrationRaw(responseBuffer, &reg->c, &reg->b);


	return E_NONE;
}

VnError VnSensor_readVelocityCompensationMeasurement(VnSensor *s, vec3f *velocity)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadVelocityCompensationMeasurement(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseVelocityCompensationMeasurementRaw(responseBuffer, velocity);

	return E_NONE;
}

VnError VnSensor_writeVelocityCompensationMeasurement(VnSensor *s, vec3f velocity, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteVelocityCompensationMeasurement(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		velocity)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readVelocityCompensationControl(VnSensor *s, VelocityCompensationControlRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t modePlaceholder;

	if ((error = VnUartPacket_genReadVelocityCompensationControl(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseVelocityCompensationControlRaw(responseBuffer, &modePlaceholder, &reg->velocityTuning, &reg->rateTuning);

	reg->mode = (VnVelocityCompensationMode) modePlaceholder;

	return E_NONE;
}

VnError VnSensor_writeVelocityCompensationControl(VnSensor *s, VelocityCompensationControlRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteVelocityCompensationControl(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.mode,
		fields.velocityTuning,
		fields.rateTuning)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readImuMeasurements(VnSensor *s, ImuMeasurementsRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadImuMeasurements(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseImuMeasurementsRaw(responseBuffer, &reg->mag, &reg->accel, &reg->gyro, &reg->temp, &reg->pressure);


	return E_NONE;
}

VnError VnSensor_readGpsConfiguration(VnSensor *s, GpsConfigurationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t modePlaceholder;
	uint8_t ppsSourcePlaceholder;
	uint8_t reserved1Placeholder;
	uint8_t reserved2Placeholder;
	uint8_t reserved3Placeholder;

	if ((error = VnUartPacket_genReadGpsConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsConfigurationRaw(responseBuffer, &modePlaceholder, &ppsSourcePlaceholder, &reserved1Placeholder, &reserved2Placeholder, &reserved3Placeholder);

	reg->mode = (VnGpsMode) modePlaceholder;
	reg->ppsSource = (VnPpsSource) ppsSourcePlaceholder;

	return E_NONE;
}

VnError VnSensor_writeGpsConfiguration(VnSensor *s, GpsConfigurationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteGpsConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.mode,
		fields.ppsSource,
		5,
		0,
		0)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readGpsAntennaOffset(VnSensor *s, vec3f *position)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadGpsAntennaOffset(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsAntennaOffsetRaw(responseBuffer, position);

	return E_NONE;
}

VnError VnSensor_writeGpsAntennaOffset(VnSensor *s, vec3f position, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteGpsAntennaOffset(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		position)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readGpsSolutionLla(VnSensor *s, GpsSolutionLlaRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t gpsFixPlaceholder;

	if ((error = VnUartPacket_genReadGpsSolutionLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsSolutionLlaRaw(responseBuffer, &reg->time, &reg->week, &gpsFixPlaceholder, &reg->numSats, &reg->lla, &reg->nedVel, &reg->nedAcc, &reg->speedAcc, &reg->timeAcc);

	reg->gpsFix = (VnGpsFix) gpsFixPlaceholder;

	return E_NONE;
}

VnError VnSensor_readGpsSolutionEcef(VnSensor *s, GpsSolutionEcefRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t gpsFixPlaceholder;

	if ((error = VnUartPacket_genReadGpsSolutionEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsSolutionEcefRaw(responseBuffer, &reg->tow, &reg->week, &gpsFixPlaceholder, &reg->numSats, &reg->position, &reg->velocity, &reg->posAcc, &reg->speedAcc, &reg->timeAcc);

	reg->gpsFix = (VnGpsFix) gpsFixPlaceholder;

	return E_NONE;
}

VnError VnSensor_readInsSolutionLla(VnSensor *s, InsSolutionLlaRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadInsSolutionLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsSolutionLlaRaw(responseBuffer, &reg->time, &reg->week, &reg->status, &reg->yawPitchRoll, &reg->position, &reg->nedVel, &reg->attUncertainty, &reg->posUncertainty, &reg->velUncertainty);


	return E_NONE;
}

VnError VnSensor_readInsSolutionEcef(VnSensor *s, InsSolutionEcefRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadInsSolutionEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsSolutionEcefRaw(responseBuffer, &reg->time, &reg->week, &reg->status, &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->attUncertainty, &reg->posUncertainty, &reg->velUncertainty);


	return E_NONE;
}

VnError VnSensor_readInsBasicConfigurationVn200(VnSensor *s, InsBasicConfigurationRegisterVn200 *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t scenarioPlaceholder;
	uint8_t resv1Placeholder;
	uint8_t resv2Placeholder;

	if ((error = VnUartPacket_genReadInsBasicConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsBasicConfigurationRaw(responseBuffer, &scenarioPlaceholder, &reg->ahrsAiding, &resv1Placeholder, &resv2Placeholder);

	reg->scenario = (VnScenario) scenarioPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeInsBasicConfigurationVn200(VnSensor *s, InsBasicConfigurationRegisterVn200 fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteInsBasicConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.scenario,
		fields.ahrsAiding,
		0,
		0)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readInsBasicConfigurationVn300(VnSensor *s, InsBasicConfigurationRegisterVn300 *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t scenarioPlaceholder;
	uint8_t resv2Placeholder;

	if ((error = VnUartPacket_genReadInsBasicConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsBasicConfigurationRaw(responseBuffer, &scenarioPlaceholder, &reg->ahrsAiding, &reg->estBaseline, &resv2Placeholder);

	reg->scenario = (VnScenario) scenarioPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeInsBasicConfigurationVn300(VnSensor *s, InsBasicConfigurationRegisterVn300 fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteInsBasicConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.scenario,
		fields.ahrsAiding,
		fields.estBaseline,
		0)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readInsStateLla(VnSensor *s, InsStateLlaRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadInsStateLla(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsStateLlaRaw(responseBuffer, &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->accel, &reg->angularRate);


	return E_NONE;
}

VnError VnSensor_readInsStateEcef(VnSensor *s, InsStateEcefRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadInsStateEcef(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseInsStateEcefRaw(responseBuffer, &reg->yawPitchRoll, &reg->position, &reg->velocity, &reg->accel, &reg->angularRate);


	return E_NONE;
}

VnError VnSensor_readStartupFilterBiasEstimate(VnSensor *s, StartupFilterBiasEstimateRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadStartupFilterBiasEstimate(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseStartupFilterBiasEstimateRaw(responseBuffer, &reg->gyroBias, &reg->accelBias, &reg->pressureBias);


	return E_NONE;
}

VnError VnSensor_writeStartupFilterBiasEstimate(VnSensor *s, StartupFilterBiasEstimateRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteStartupFilterBiasEstimate(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.gyroBias,
		fields.accelBias,
		fields.pressureBias)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readDeltaThetaAndDeltaVelocity(VnSensor *s, DeltaThetaAndDeltaVelocityRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadDeltaThetaAndDeltaVelocity(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseDeltaThetaAndDeltaVelocityRaw(responseBuffer, &reg->deltaTime, &reg->deltaTheta, &reg->deltaVelocity);


	return E_NONE;
}

VnError VnSensor_readDeltaThetaAndDeltaVelocityConfiguration(VnSensor *s, DeltaThetaAndDeltaVelocityConfigurationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t integrationFramePlaceholder;
	uint8_t gyroCompensationPlaceholder;
	uint8_t accelCompensationPlaceholder;
	uint8_t reserved1Placeholder;
	uint16_t reserved2Placeholder;

	if ((error = VnUartPacket_genReadDeltaThetaAndDeltaVelocityConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseDeltaThetaAndDeltaVelocityConfigurationRaw(responseBuffer, &integrationFramePlaceholder, &gyroCompensationPlaceholder, &accelCompensationPlaceholder, &reserved1Placeholder, &reserved2Placeholder);

	reg->integrationFrame = (VnIntegrationFrame) integrationFramePlaceholder;
	reg->gyroCompensation = (VnCompensationMode) gyroCompensationPlaceholder;
	reg->accelCompensation = (VnCompensationMode) accelCompensationPlaceholder;

	return E_NONE;
}

VnError VnSensor_writeDeltaThetaAndDeltaVelocityConfiguration(VnSensor *s, DeltaThetaAndDeltaVelocityConfigurationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteDeltaThetaAndDeltaVelocityConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.integrationFrame,
		fields.gyroCompensation,
		fields.accelCompensation,
		0,
		0)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readReferenceVectorConfiguration(VnSensor *s, ReferenceVectorConfigurationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t resv1Placeholder;
	uint8_t resv2Placeholder;

	if ((error = VnUartPacket_genReadReferenceVectorConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseReferenceVectorConfigurationRaw(responseBuffer, &reg->useMagModel, &reg->useGravityModel, &resv1Placeholder, &resv2Placeholder, &reg->recalcThreshold, &reg->year, &reg->position);


	return E_NONE;
}

VnError VnSensor_writeReferenceVectorConfiguration(VnSensor *s, ReferenceVectorConfigurationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteReferenceVectorConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.useMagModel,
		fields.useGravityModel,
		0,
		0,
		fields.recalcThreshold,
		fields.year,
		fields.position)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readGyroCompensation(VnSensor *s, GyroCompensationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadGyroCompensation(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGyroCompensationRaw(responseBuffer, &reg->c, &reg->b);


	return E_NONE;
}

VnError VnSensor_writeGyroCompensation(VnSensor *s, GyroCompensationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteGyroCompensation(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.c,
		fields.b)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readImuFilteringConfiguration(VnSensor *s, ImuFilteringConfigurationRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t magFilterModePlaceholder;
	uint8_t accelFilterModePlaceholder;
	uint8_t gyroFilterModePlaceholder;
	uint8_t tempFilterModePlaceholder;
	uint8_t presFilterModePlaceholder;

	if ((error = VnUartPacket_genReadImuFilteringConfiguration(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseImuFilteringConfigurationRaw(responseBuffer, &reg->magWindowSize, &reg->accelWindowSize, &reg->gyroWindowSize, &reg->tempWindowSize, &reg->presWindowSize, &magFilterModePlaceholder, &accelFilterModePlaceholder, &gyroFilterModePlaceholder, &tempFilterModePlaceholder, &presFilterModePlaceholder);

	reg->magFilterMode = (VnFilterMode) magFilterModePlaceholder;
	reg->accelFilterMode = (VnFilterMode) accelFilterModePlaceholder;
	reg->gyroFilterMode = (VnFilterMode) gyroFilterModePlaceholder;
	reg->tempFilterMode = (VnFilterMode) tempFilterModePlaceholder;
	reg->presFilterMode = (VnFilterMode) presFilterModePlaceholder;

	return E_NONE;
}

VnError VnSensor_writeImuFilteringConfiguration(VnSensor *s, ImuFilteringConfigurationRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteImuFilteringConfiguration(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.magWindowSize,
		fields.accelWindowSize,
		fields.gyroWindowSize,
		fields.tempWindowSize,
		fields.presWindowSize,
		fields.magFilterMode,
		fields.accelFilterMode,
		fields.gyroFilterMode,
		fields.tempFilterMode,
		fields.presFilterMode)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readGpsCompassBaseline(VnSensor *s, GpsCompassBaselineRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadGpsCompassBaseline(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsCompassBaselineRaw(responseBuffer, &reg->position, &reg->uncertainty);


	return E_NONE;
}

VnError VnSensor_writeGpsCompassBaseline(VnSensor *s, GpsCompassBaselineRegister fields, bool waitForReply)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genWriteGpsCompassBaseline(
		toSend,
		sizeof(toSend),
		s->sendErrorDetectionMode,
		&toSendLength,
		fields.position,
		fields.uncertainty)) != E_NONE)
		return error;

	return VnSensor_transactionNoFinalize(s, toSend, toSendLength, waitForReply, responseBuffer, &responseLength);
}

VnError VnSensor_readGpsCompassEstimatedBaseline(VnSensor *s, GpsCompassEstimatedBaselineRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);
	uint8_t resvPlaceholder;

	if ((error = VnUartPacket_genReadGpsCompassEstimatedBaseline(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseGpsCompassEstimatedBaselineRaw(responseBuffer, &reg->estBaselineUsed, &resvPlaceholder, &reg->numMeas, &reg->position, &reg->uncertainty);


	return E_NONE;
}

VnError VnSensor_readYawPitchRollTrueBodyAccelerationAndAngularRates(VnSensor *s, YawPitchRollTrueBodyAccelerationAndAngularRatesRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadYawPitchRollTrueBodyAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseYawPitchRollTrueBodyAccelerationAndAngularRatesRaw(responseBuffer, &reg->yawPitchRoll, &reg->bodyAccel, &reg->gyro);


	return E_NONE;
}

VnError VnSensor_readYawPitchRollTrueInertialAccelerationAndAngularRates(VnSensor *s, YawPitchRollTrueInertialAccelerationAndAngularRatesRegister *reg)
{
	char toSend[17];
	size_t length;
	VnError error;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if ((error = VnUartPacket_genReadYawPitchRollTrueInertialAccelerationAndAngularRates(toSend, sizeof(toSend), s->sendErrorDetectionMode, &length)) != E_NONE)
		return error;

	if ((error = VnSensor_transactionNoFinalize(s, toSend, length, true, responseBuffer, &responseLength)) != E_NONE)
		return error;

	VnUartPacket_parseYawPitchRollTrueInertialAccelerationAndAngularRatesRaw(responseBuffer, &reg->yawPitchRoll, &reg->inertialAccel, &reg->gyro);


	return E_NONE;
}

VnError VnSensor_switchProcessors(VnSensor* s, VnProcessorType processor, char *model, char *firmware)
{
	VnError error;
	char toSend[256];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	if (strncmp("VN-300", model, 6) == 0)
	{
		switch (processor)
		{
		case VNPROCESSOR_GPS:
			if ((strstr(model, "SMD") != NULL) || (strstr(model, "DEV") != NULL))
			{
				#if VN_HAVE_SECURE_CRT
					toSendLength = sprintf_s(toSend, sizeof(toSend), "$VNSPS,1,1,115200");
				#else
					toSendLength = sprintf(toSend, "$VNSPS,1,1,115200");
				#endif
			}
			else if (strstr(model, "CR") != NULL)
			{
				#if VN_HAVE_SECURE_CRT
					toSendLength = sprintf_s(toSend, sizeof(toSend), "$VNDBS,3,1");
				#else
					toSendLength = sprintf(toSend, "$VNDBS,3,1");
				#endif
			}
			break;
		}
	}
	else if ((strncmp("VN-110", model, 6) == 0) || (strncmp("VN-210", model, 6) == 0) || (strncmp("VN-310", model, 6) == 0))
	{
		switch (processor)
		{
		case VNPROCESSOR_NAV:
			#if VN_HAVE_SECURE_CRT
				toSendLength = sprintf_s(toSend, sizeof(toSend), "$VNSBL,0");
			#else
				toSendLength = sprintf(toSend, "$VNSBL,0");
			#endif
			break;
		case VNPROCESSOR_GPS:
			if ((strncmp("VN-310", model, 6) == 0) || (strncmp("VN-210E", model, 7) == 0))
			{
			#if VN_HAVE_SECURE_CRT
				toSendLength = sprintf_s(toSend, sizeof(toSend), "$VNSBL,1");
			#else
				toSendLength = sprintf(toSend, "$VNSBL,1");
			#endif
			}
			break;
		case VNPROCESSOR_IMU:
			#if VN_HAVE_SECURE_CRT
				toSendLength = sprintf_s(toSend, sizeof(toSend), "$VNSBL,2");
			#else
				toSendLength = sprintf(toSend, "$VNSBL,2");
			#endif
			break;
		}
	}

	if (strlen(toSend) > 0)
	{
		VnUartPacket_finalizeCommand(s->sendErrorDetectionMode, (uint8_t*)toSend, &toSendLength);
		error = VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, true, responseBuffer, &responseLength, 6000, 6000);

		if (error == E_NONE)
		{
			/* Check the response buffer for errors */

			/* Wait 2 seconds to allow the processor to switch */
			VnThread_sleepMs(2000);

		}

		return error;
	}
	else
	{
		return E_NOT_SUPPORTED;
	}
}

FILE *VnSensor_openFirmwareUpdateFile(char* filename)
{
#if VN_HAVE_SECURE_CRT
	FILE *file;
	int error = fopen_s(&file, filename, "r");
	return file;
#else
	return fopen(filename, "r");
#endif

}

bool VnSensor_getNextFirmwareUpdateRecord(FILE* file, char* record, int MaxRecordSize)
{
	bool result = false;


#if VN_HAVE_SECURE_CRT
	int retval = fscanf_s(file, "%s\n", record, MaxRecordSize);
#else
	int retval = fscanf(file, "%s\n", record);
#endif

	if (retval != EOF)
	{
		result = true;
	}

	return result;
}

void VnSensor_closeFirmwareUpdateFile(FILE* file)
{
	if (file != NULL)
	{
		int error = fclose(file);
		file = NULL;
	}
}

void VnSensor_calibrateBootloader(VnSensor *s)
{
	char calibration[] = "                ";
	char bootloaderVersion[64];
	size_t bootloaderVersionSize = sizeof(bootloaderVersion);
	memset(bootloaderVersion, 0, 64);
	uint16_t responseTimeoutMs = 500;
	uint16_t retransmitDelayMs = responseTimeoutMs;

	s->bootloaderFilter = true;
	if (E_NONE == VnSensor_transactionNoFinalizeWithTiming(s, calibration, sizeof(calibration) - 2, true, bootloaderVersion, &bootloaderVersionSize, responseTimeoutMs, retransmitDelayMs))
	{
		/* printf("Bootloader: %s\n", bootloaderVersion); */
	}
}

VnError VnSensor_writeFirmwareUpdateRecord(VnSensor* s, char* record)
{
	VnError error = E_NONE;
	char toSend[MAXFIRMWAREUPDATERECORDSIZE +12];
	size_t toSendLength;
	char responseBuffer[0x100];
	size_t responseLength = sizeof(responseBuffer);

	memset(responseBuffer, 0, 0x100);

	/* VNERRORDETECTIONMODE_CRC - Must be 16-bit CRC for Firmware Updates, also, strip off the leading ':' from the record */
	if ((error = VnUartPacket_genWriteFirmwareUpdate(toSend, sizeof(toSend), VNERRORDETECTIONMODE_CRC, &toSendLength, &record[1])) != E_NONE)
		return error;

RESEND:
	s->bootloaderFilter = true;
	error = VnSensor_transactionNoFinalizeWithTiming(s, toSend, toSendLength, true, responseBuffer, &responseLength, 7000, 7000);
	if (error != E_NONE)
	{
		char buffer[128];
		memset(buffer, 0, sizeof(buffer));
		strFromVnError(buffer, error);
		/* printf("VnSensor_writeFirmwareUpdate: Error %d - %s", error, buffer); */
	}
	else
	{
		/* Check Firmware Update Response Code */
		int response = VnUtil_toUint8FromHexStr(&(responseBuffer[7])) + E_BOOTLOADER_NONE;
		error = (VnError)response;

		if (response == E_BOOTLOADER_COMM_ERROR) goto RESEND;
		else if (response != E_BOOTLOADER_NONE)
		{
			char buffer[128];
			memset(buffer, 0, sizeof(buffer));
			strFromVnError(buffer, response);
			/* printf("Error: %s\n", buffer); */
		}
	}
	return error;
}

VnError VnSensor_firmwareUpdate(VnSensor* s, int baudRate, char* filename)
{
	VnError error = E_NONE;
	int RecordSize = MAXFIRMWAREUPDATERECORDSIZE;
	char record[MAXFIRMWAREUPDATERECORDSIZE];
	bool stillGoing = true;

	/* Open Firmware Update File*/
	FILE* firmwareUpdateFile = VnSensor_openFirmwareUpdateFile(filename);

	/* Enter the bootloader */
	error = VnSensor_firmwareUpdateMode(s, true);

	/* Change the baud rate for updating the firmware */
	error = VnSerialPort_changeBaudrate(&(s->serialPort), baudRate);

	/* Give the processor some time to reboot */
	VnThread_sleepMs(1000);

	/* Calibrate the Bootloader before sending records */
	VnSensor_calibrateBootloader(s);

	/* Write the firmware update file to the sensor, one record at a time */
	while (stillGoing)
	{
		stillGoing = VnSensor_getNextFirmwareUpdateRecord(firmwareUpdateFile, record, RecordSize);
		if (stillGoing)
		{
			error = VnSensor_writeFirmwareUpdateRecord(s, record);
			if (error != E_BOOTLOADER_NONE)
			{
				stillGoing = false;
			}
		}
	}

	/* Switch the baud rate back to the original setting */
	error = VnSerialPort_changeBaudrate(&(s->serialPort), baudRate);

	/* Close the firmware update file */
	VnSensor_closeFirmwareUpdateFile(firmwareUpdateFile);

	/* Exit Bootloader mode. Just sleep for 10 seconds */
	VnThread_sleepMs(10000);

	/* Do a reset */
	VnSensor_reset(s, true);

	/* Wait 2 seconds for the reset to finish */
	VnThread_sleepMs(2000);

	return error;
}

void strFromSyncInMode(char *out, VnSyncInMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	#ifdef EXTRA
	case VNSYNCINMODE_COUNT2:
		strcpy(out, "Count2");
		break;
	case VNSYNCINMODE_ADC2:
		strcpy(out, "Adc2");
		break;
	case VNSYNCINMODE_ASYNC2:
		strcpy(out, "Async2");
		break;
	#endif
	case VNSYNCINMODE_COUNT:
		strcpy(out, "Count");
		break;
	case VNSYNCINMODE_IMU:
		strcpy(out, "Imu");
		break;
	case VNSYNCINMODE_ASYNC:
		strcpy(out, "Async");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromSyncInEdge(char *out, VnSyncInEdge val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNSYNCINEDGE_RISING:
		strcpy(out, "Rising");
		break;
	case VNSYNCINEDGE_FALLING:
		strcpy(out, "Falling");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromSyncOutMode(char *out, VnSyncOutMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNSYNCOUTMODE_NONE:
		strcpy(out, "None");
		break;
	case VNSYNCOUTMODE_ITEMSTART:
		strcpy(out, "ItemStart");
		break;
	case VNSYNCOUTMODE_IMUREADY:
		strcpy(out, "ImuReady");
		break;
	case VNSYNCOUTMODE_INS:
		strcpy(out, "Ins");
		break;
	case VNSYNCOUTMODE_GPSPPS:
		strcpy(out, "GpsPps");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromSyncOutPolarity(char *out, VnSyncOutPolarity val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNSYNCOUTPOLARITY_NEGATIVE:
		strcpy(out, "Negative");
		break;
	case VNSYNCOUTPOLARITY_POSITIVE:
		strcpy(out, "Positive");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromCountMode(char *out, VnCountMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNCOUNTMODE_NONE:
		strcpy(out, "None");
		break;
	case VNCOUNTMODE_SYNCINCOUNT:
		strcpy(out, "SyncInCount");
		break;
	case VNCOUNTMODE_SYNCINTIME:
		strcpy(out, "SyncInTime");
		break;
	case VNCOUNTMODE_SYNCOUTCOUNTER:
		strcpy(out, "SyncOutCounter");
		break;
	case VNCOUNTMODE_GPSPPS:
		strcpy(out, "GpsPps");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromStatusMode(char *out, VnStatusMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNSTATUSMODE_OFF:
		strcpy(out, "Off");
		break;
	case VNSTATUSMODE_VPESTATUS:
		strcpy(out, "VpeStatus");
		break;
	case VNSTATUSMODE_INSSTATUS:
		strcpy(out, "InsStatus");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromChecksumMode(char *out, VnChecksumMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNCHECKSUMMODE_OFF:
		strcpy(out, "Off");
		break;
	case VNCHECKSUMMODE_CHECKSUM:
		strcpy(out, "Checksum");
		break;
	case VNCHECKSUMMODE_CRC:
		strcpy(out, "Crc");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromErrorMode(char *out, VnErrorMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNERRORMODE_IGNORE:
		strcpy(out, "Ignore");
		break;
	case VNERRORMODE_SEND:
		strcpy(out, "Send");
		break;
	case VNERRORMODE_SENDANDOFF:
		strcpy(out, "SendAndOff");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromFilterMode(char *out, VnFilterMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNFILTERMODE_NOFILTERING:
		strcpy(out, "NoFiltering");
		break;
	case VNFILTERMODE_ONLYRAW:
		strcpy(out, "OnlyRaw");
		break;
	case VNFILTERMODE_ONLYCOMPENSATED:
		strcpy(out, "OnlyCompensated");
		break;
	case VNFILTERMODE_BOTH:
		strcpy(out, "Both");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromIntegrationFrame(char *out, VnIntegrationFrame val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNINTEGRATIONFRAME_BODY:
		strcpy(out, "Body");
		break;
	case VNINTEGRATIONFRAME_NED:
		strcpy(out, "Ned");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromCompensationMode(char *out, VnCompensationMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNCOMPENSATIONMODE_NONE:
		strcpy(out, "None");
		break;
	case VNCOMPENSATIONMODE_BIAS:
		strcpy(out, "Bias");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromGpsFix(char *out, VnGpsFix val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNGPSFIX_NOFIX:
		strcpy(out, "NoFix");
		break;
	case VNGPSFIX_TIMEONLY:
		strcpy(out, "TimeOnly");
		break;
	case VNGPSFIX_2D:
		strcpy(out, "2D");
		break;
	case VNGPSFIX_3D:
		strcpy(out, "3D");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromGpsMode(char *out, VnGpsMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNGPSMODE_ONBOARDGPS:
		strcpy(out, "OnBoardGps");
		break;
	case VNGPSMODE_EXTERNALGPS:
		strcpy(out, "ExternalGps");
		break;
	case VNGPSMODE_EXTERNALVN200GPS:
		strcpy(out, "ExternalVn200Gps");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromPpsSource(char *out, VnPpsSource val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNPPSSOURCE_GPSPPSRISING:
		strcpy(out, "GpsPpsRising");
		break;
	case VNPPSSOURCE_GPSPPSFALLING:
		strcpy(out, "GpsPpsFalling");
		break;
	case VNPPSSOURCE_SYNCINRISING:
		strcpy(out, "SyncInRising");
		break;
	case VNPPSSOURCE_SYNCINFALLING:
		strcpy(out, "SyncInFalling");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromVpeEnable(char *out, VnVpeEnable val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNVPEENABLE_DISABLE:
		strcpy(out, "Disable");
		break;
	case VNVPEENABLE_ENABLE:
		strcpy(out, "Enable");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromHeadingMode(char *out, VnHeadingMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNHEADINGMODE_ABSOLUTE:
		strcpy(out, "Absolute");
		break;
	case VNHEADINGMODE_RELATIVE:
		strcpy(out, "Relative");
		break;
	case VNHEADINGMODE_INDOOR:
		strcpy(out, "Indoor");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromVpeMode(char *out, VnVpeMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNVPEMODE_OFF:
		strcpy(out, "Off");
		break;
	case VNVPEMODE_MODE1:
		strcpy(out, "Mode1");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromScenario(char *out, VnScenario val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNSCENARIO_AHRS:
		strcpy(out, "Ahrs");
		break;
	case VNSCENARIO_INSWITHPRESSURE:
		strcpy(out, "InsWithPressure");
		break;
	case VNSCENARIO_INSWITHOUTPRESSURE:
		strcpy(out, "InsWithoutPressure");
		break;
	case VNSCENARIO_GPSMOVINGBASELINEDYNAMIC:
		strcpy(out, "GpsMovingBaselineDynamic");
		break;
	case VNSCENARIO_GPSMOVINGBASELINESTATIC:
		strcpy(out, "GpsMovingBaselineStatic");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromHsiMode(char *out, VnHsiMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNHSIMODE_OFF:
		strcpy(out, "Off");
		break;
	case VNHSIMODE_RUN:
		strcpy(out, "Run");
		break;
	case VNHSIMODE_RESET:
		strcpy(out, "Reset");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromHsiOutput(char *out, VnHsiOutput val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNHSIOUTPUT_NOONBOARD:
		strcpy(out, "NoOnboard");
		break;
	case VNHSIOUTPUT_USEONBOARD:
		strcpy(out, "UseOnboard");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromVelocityCompensationMode(char *out, VnVelocityCompensationMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNVELOCITYCOMPENSATIONMODE_DISABLED:
		strcpy(out, "Disabled");
		break;
	case VNVELOCITYCOMPENSATIONMODE_BODYMEASUREMENT:
		strcpy(out, "BodyMeasurement");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromMagneticMode(char *out, VnMagneticMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNMAGNETICMODE_2D:
		strcpy(out, "2D");
		break;
	case VNMAGNETICMODE_3D:
		strcpy(out, "3D");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromExternalSensorMode(char *out, VnExternalSensorMode val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNEXTERNALSENSORMODE_INTERNAL:
		strcpy(out, "Internal");
		break;
	case VNEXTERNALSENSORMODE_EXTERNAL200HZ:
		strcpy(out, "External200Hz");
		break;
	case VNEXTERNALSENSORMODE_EXTERNALONUPDATE:
		strcpy(out, "ExternalOnUpdate");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}

void strFromFoamInit(char *out, VnFoamInit val)
{
	#if defined(_MSC_VER)
		/* Disable warnings regarding using strcpy_s since this
		 * function's signature does not provide us with information
		 * about the length of 'out'. */
		#pragma warning(push)
		#pragma warning(disable:4996)
	#endif

	switch (val)
	{
	case VNFOAMINIT_NOFOAMINIT:
		strcpy(out, "NoFoamInit");
		break;
	case VNFOAMINIT_FOAMINITPITCHROLL:
		strcpy(out, "FoamInitPitchRoll");
		break;
	case VNFOAMINIT_FOAMINITHEADINGPITCHROLL:
		strcpy(out, "FoamInitHeadingPitchRoll");
		break;
	case VNFOAMINIT_FOAMINITPITCHROLLCOVARIANCE:
		strcpy(out, "FoamInitPitchRollCovariance");
		break;
	case VNFOAMINIT_FOAMINITHEADINGPITCHROLLCOVARIANCE:
		strcpy(out, "FoamInitHeadingPitchRollCovariance");
		break;
	default:
		strcpy(out, "Unknown");
		break;
	}

	#if defined(_MSC_VER)
		#pragma warning(pop)
	#endif
}
