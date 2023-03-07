#include "vn/xplat/serialport.h"

#if _WIN32
	/* Nothing to do. */
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__
	#include <fcntl.h>
	#include <errno.h>
	#include <termios.h>
	#include <string.h>
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/select.h>
#else
	#error "Unknown System"
#endif

#define NUMBER_OF_BYTES_TO_PURGE_ON_OPENING_SERIAL_PORT 100
#define WAIT_TIME_FOR_SERIAL_PORT_READS_MS 100

/* Private declarations. */
void VnSerialPort_purgeFirstDataBytesFromSerialPort(VnSerialPort *serialport);
void VnSerialPort_startSerialPortNotificationsThread(VnSerialPort *serialport);
void VnSerialPort_stopSerialPortNotificationsThread(VnSerialPort *serialport);
void VnSerialPort_handleSerialPortNotifications(void* data);
void VnSerialPort_onDataReceived(VnSerialPort *serialport);
VnError VnSerialPort_open_internal(VnSerialPort *serialport, char const *portName, uint32_t baudrate, bool checkAndToggleIsOpenFlag);
VnError VnSerialPort_close_internal(VnSerialPort *serialport, bool checkAndToggleIsOpenFlag);

VnError VnSerialPort_initialize(VnSerialPort *sp)
{
	#if _WIN32
	sp->handle = NULL;
	VnCriticalSection_initialize(&sp->readWriteCS);
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__
	sp->handle = -1;
	#else
	#error "Unknown System"
	#endif

	sp->purgeFirstDataBytesWhenSerialPortIsFirstOpened = true;
	sp->dataReceivedHandler = NULL;
	sp->dataReceivedHandlerUserData = NULL;
	sp->continueHandlingSerialPortEvents = false;
	sp->numberOfReceiveDataDroppedSections = 0;

	VnCriticalSection_initialize(&sp->dataReceivedHandlerCriticalSection);

	return E_NONE;
}

VnError VnSerialPort_open_internal(VnSerialPort *serialport, char const *portName, uint32_t baudrate, bool checkAndToggleIsOpenFlag)
{
	#if _WIN32
	DCB config;
	const char *preName = "\\\\.\\";
	char *fullName;
	size_t fullNameLen;
	COMMTIMEOUTS comTimeOut;
	#ifdef UNICODE
	WCHAR wFullName[0x100];
	#endif

	#elif (defined __linux__ || defined __APPLE__ || defined __CYGWIN__ || defined __QNXNTO__ || defined __NUTTX__)

	struct termios portSettings;
	int portFd = -1;
	tcflag_t baudrateFlag;

	#endif

	#if VN_HAVE_SECURE_CRT
	strcpy_s(serialport->portName, sizeof(serialport->portName), portName);
	#else
	strcpy(serialport->portName, portName);
	#endif

	if (checkAndToggleIsOpenFlag && VnSerialPort_isOpen(serialport))
		return E_INVALID_OPERATION;

	#if _WIN32

	fullNameLen = strlen(preName) + strlen(portName) + 1;
	fullName = (char*) malloc(fullNameLen);
	#if VN_HAVE_SECURE_CRT
	strcpy_s(fullName, fullNameLen, preName);
	strcat_s(fullName, fullNameLen, portName);
	#else
	strcpy(fullName, preName);
	strcat(fullName, portName);
	#endif

	#ifdef UNICODE
	mbstowcs(wFullName, fullName, strlen(fullName) + 1);
	#endif

	serialport->handle = CreateFile(
		#ifdef UNICODE
		wFullName,
		#else
		fullName,
		#endif
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		NULL);

	free(fullName);

	if (serialport->handle == INVALID_HANDLE_VALUE)
	{
		DWORD error = GetLastError();

		if (error == ERROR_ACCESS_DENIED)
			/* Port is probably already open. */
			return E_INVALID_OPERATION;

		if (error == ERROR_FILE_NOT_FOUND)
			/* Port probably does not exist. */
			return E_NOT_FOUND;

		return E_UNKNOWN;
	}

	/* Set the state of the COM port. */
	if (!GetCommState(serialport->handle, &config))
	{
		DWORD error = GetLastError();

		if (error != ERROR_OPERATION_ABORTED)
			return E_UNKNOWN;

		/* Try clearing this error. */
		if (!ClearCommError(serialport->handle, &error, NULL))
			return E_UNKNOWN;

		/* Retry the operation. */
		if (!GetCommState(serialport->handle, &config))
			return E_UNKNOWN;
	}

	config.BaudRate = baudrate;
	config.StopBits = ONESTOPBIT;
	config.Parity = NOPARITY;
	config.ByteSize = 8;
	config.fAbortOnError = 0;

	if (!SetCommState(serialport->handle, &config))
	{
		DWORD error = GetLastError();

		if (error == ERROR_INVALID_PARAMETER)
		{
			if (!CloseHandle(serialport->handle))
				return E_UNKNOWN;

			return E_INVALID_VALUE;
		}

		if (error != ERROR_OPERATION_ABORTED)
			return E_UNKNOWN;

		/* Try clearing this error. */
		if (!ClearCommError(serialport->handle, &error, NULL))
			return E_UNKNOWN;

		/* Retry this operation. */
		if (!SetCommState(serialport->handle, &config))
			return E_UNKNOWN;
	}

	comTimeOut.ReadIntervalTimeout = 0;
	comTimeOut.ReadTotalTimeoutMultiplier = 0;
	comTimeOut.ReadTotalTimeoutConstant = 1;
	comTimeOut.WriteTotalTimeoutMultiplier = 3;
	comTimeOut.WriteTotalTimeoutConstant = 2;

	if (!SetCommTimeouts(serialport->handle, &comTimeOut))
	{
		DWORD error = GetLastError();

		if (error != ERROR_OPERATION_ABORTED)
			return E_UNKNOWN;

		/* Try clearing this error. */
		if (!ClearCommError(serialport->handle, &error, NULL))
			return E_UNKNOWN;

		/* Retry the operation. */
		if (!SetCommTimeouts(serialport->handle, &comTimeOut))
			return E_UNKNOWN;
	}

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	portFd = open(
		portName,
		#if __linux__ || __CYGWIN__ || __QNXNTO__
		O_RDWR | O_NOCTTY);
		#elif __APPLE__ || __NUTTX__
		O_RDWR | O_NOCTTY | O_NONBLOCK);
		#else
		#error "Unknown System"
		#endif

	if (portFd == -1)
	{
		switch (errno)
		{
		case EACCES:
			return E_PERMISSION_DENIED;
		case ENXIO:
		case ENOTDIR:
		case ENOENT:
			return E_NOT_FOUND;
		default:
			return E_UNKNOWN;
		}
	}

	memset(
		&portSettings,
		0,
		sizeof(portSettings));

	switch (baudrate)
	{
		case 9600:
			baudrateFlag = B9600;
			break;
		case 19200:
			baudrateFlag = B19200;
			break;
		case 38400:
			baudrateFlag = B38400;
			break;
		case 57600:
			baudrateFlag = B57600;
			break;
		case 115200:
			baudrateFlag = B115200;
			break;

		/* QNX does not have higher baudrates defined. */
		#if !defined(__QNXNTO__)

		case 230400:
			baudrateFlag = B230400;
			break;

		/* Not available on Mac OS X??? */
		#if !defined(__APPLE__)

		case 460800:
			baudrateFlag = B460800;
			break;
		case 921600:
			baudrateFlag = B921600;
			break;

		#endif

		#endif

		default:
			return E_INVALID_VALUE;
	}

	/* Set baudrate, 8n1, no modem control, enable receiving characters. */
	#if __linux__ || __QNXNTO__ || __CYGWIN__
	portSettings.c_cflag = baudrateFlag;
	#elif defined(__APPLE__) || __NUTTX__
	cfsetspeed(&portSettings, baudrateFlag);
	#endif
	portSettings.c_cflag |= CS8 | CLOCAL | CREAD;

	portSettings.c_iflag = IGNPAR;		/* Ignore bytes with parity errors. */
	portSettings.c_oflag = 0;			/* Enable raw data output. */
	portSettings.c_cc[VTIME] = 0;		/* Do not use inter-character timer. */
	portSettings.c_cc[VMIN] = 0;		/* Block on reads until 0 character is received. */

	/* Clear the COM port buffers. */
	if (tcflush(portFd, TCIFLUSH) != 0)
		return E_UNKNOWN;

	if (tcsetattr(portFd, TCSANOW, &portSettings) != 0)
		return E_UNKNOWN;

	serialport->handle = portFd;

	#else
	#error "Unknown System"
	#endif

	if (serialport->purgeFirstDataBytesWhenSerialPortIsFirstOpened)
		VnSerialPort_purgeFirstDataBytesFromSerialPort(serialport);

	VnSerialPort_startSerialPortNotificationsThread(serialport);

	return E_NONE;
}

VnError VnSerialPort_close_internal(VnSerialPort *serialport, bool checkAndToggleIsOpenFlag)
{
	if (checkAndToggleIsOpenFlag && !VnSerialPort_isOpen(serialport))
		return E_INVALID_OPERATION;

	VnSerialPort_stopSerialPortNotificationsThread(serialport);

	#if _WIN32

	if (!CloseHandle(serialport->handle))
		return E_UNKNOWN;

	serialport->handle = NULL;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	if (close(serialport->handle) == -1)
		return E_UNKNOWN;

	serialport->handle = -1;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnSerialPort_closeAfterUserUnpluggedSerialPort(VnSerialPort *serialport)
{
	#if _WIN32

	if (!CloseHandle(serialport->handle))
		return E_UNKNOWN;

	serialport->handle = NULL;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	if (close(serialport->handle) == -1)
		return E_UNKNOWN;

	serialport->handle = -1;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnSerialPort_open(VnSerialPort *serialport, char const *portName, uint32_t baudrate)
{
	return VnSerialPort_open_internal(serialport, portName, baudrate, true);
}

bool VnSerialPort_isOpen(VnSerialPort *serialport)
{
	#if defined(_WIN32)
	return serialport->handle != NULL;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__
	return serialport->handle != -1;
	#else
	#error "Unknown System"
	#endif
}

VnError VnSerialPort_changeBaudrate(VnSerialPort *serialport, uint32_t baudrate)
{
	VnError error;

	if (!VnSerialPort_isOpen(serialport))
		return E_INVALID_OPERATION;

	if ((error = VnSerialPort_close_internal(serialport, false)) != E_NONE)
		return error;

	return VnSerialPort_open_internal(serialport, serialport->portName, baudrate, false);
}

void VnSerialPort_purgeFirstDataBytesFromSerialPort(VnSerialPort *serialport)
{
	char buffer[NUMBER_OF_BYTES_TO_PURGE_ON_OPENING_SERIAL_PORT];
	size_t numOfBytesRead;

	VnSerialPort_read(serialport, buffer, NUMBER_OF_BYTES_TO_PURGE_ON_OPENING_SERIAL_PORT, &numOfBytesRead);
}

VnError VnSerialPort_read(VnSerialPort *serialport, char *buffer, size_t numOfBytesToRead, size_t *numOfBytesActuallyRead)
{
	#if defined(_WIN32)
	OVERLAPPED overlapped;
	BOOL result;
	DWORD numOfBytesTransferred;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__
	int result;
	#else
	#error "Unknown System"
	#endif

	if (!VnSerialPort_isOpen(serialport))
		return E_INVALID_OPERATION;

	#if _WIN32

	memset(&overlapped, 0, sizeof(OVERLAPPED));

	VnCriticalSection_enter(&serialport->readWriteCS);

	result = ReadFile(
		serialport->handle,
		buffer,
		numOfBytesToRead,
		NULL,
		&overlapped);

	if (!result && GetLastError() != ERROR_IO_PENDING)
	{
		VnCriticalSection_leave(&serialport->readWriteCS);
		return E_UNKNOWN;
	}

	result = GetOverlappedResult(
		serialport->handle,
		&overlapped,
		&numOfBytesTransferred,
		TRUE);

	VnCriticalSection_leave(&serialport->readWriteCS);

	*numOfBytesActuallyRead = numOfBytesTransferred;

	if (!result)
		return E_UNKNOWN;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	result = read(
		serialport->handle,
		buffer,
		numOfBytesToRead);

	if (result == -1)
		return E_UNKNOWN;

	*numOfBytesActuallyRead = result;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnSerialPort_write(VnSerialPort *sp, char const *data, size_t length)
{
	#if defined(_WIN32)
	DWORD numOfBytesWritten;
	BOOL result;
	OVERLAPPED overlapped;
	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__
	size_t numOfBytesWritten;
	#else
	#error "Unknown System"
	#endif

	if (!VnSerialPort_isOpen(sp))
		return E_INVALID_OPERATION;

	#if defined(_WIN32)

	memset(&overlapped, 0, sizeof(OVERLAPPED));

	VnCriticalSection_enter(&sp->readWriteCS);

	result = WriteFile(
		sp->handle,
		data,
		length,
		NULL,
		&overlapped);

	if (!result && GetLastError() != ERROR_IO_PENDING)
	{
		VnCriticalSection_leave(&sp->readWriteCS);
		return E_UNKNOWN;
	}

	result = GetOverlappedResult(
		sp->handle,
		&overlapped,
		&numOfBytesWritten,
		TRUE);

	if (!result)
	{
		VnCriticalSection_leave(&sp->readWriteCS);
		return E_UNKNOWN;
	}

	result = FlushFileBuffers(sp->handle);

	VnCriticalSection_leave(&sp->readWriteCS);

	if (!result)
		return E_UNKNOWN;

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	numOfBytesWritten = write(
		sp->handle,
		data,
		length);

	if (numOfBytesWritten == -1)
		return E_UNKNOWN;

	#else
	#error "Unknown System"
	#endif

	return E_NONE;
}

VnError VnSerialPort_close(VnSerialPort *serialport)
{
	return VnSerialPort_close_internal(serialport, true);
}

void VnSerialPort_startSerialPortNotificationsThread(VnSerialPort *serialport)
{
	serialport->continueHandlingSerialPortEvents = true;

	VnThread_startNew(&serialport->serialPortNotificationsThread, VnSerialPort_handleSerialPortNotifications, serialport);
}

void VnSerialPort_stopSerialPortNotificationsThread(VnSerialPort *serialport)
{
	serialport->continueHandlingSerialPortEvents = false;

	VnThread_join(&serialport->serialPortNotificationsThread);
}

void VnSerialPort_handleSerialPortNotifications(void* routineData)
{
	bool userUnpluggedUsbCable = false;
	VnSerialPort *sp = (VnSerialPort*) routineData;

	#if _WIN32

	OVERLAPPED overlapped;

	memset(&overlapped, 0, sizeof(OVERLAPPED));

	overlapped.hEvent = CreateEvent(
		NULL,
		false,
		false,
		NULL);

	SetCommMask(
		sp->handle,
		EV_RXCHAR | EV_ERR | EV_RX80FULL);

	#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

	fd_set readfs;
	int error;
	struct timeval readWaitTime;

	#else
	#error "Unknown System"
	#endif

	while (sp->continueHandlingSerialPortEvents)
	{
		#if _WIN32

		DWORD mask = 0;
		DWORD temp = 0;
		DWORD waitResult;
		BOOL result;

		result = WaitCommEvent(
			sp->handle,
			&mask,
			&overlapped);

		if (result)
		{
			VnSerialPort_onDataReceived(sp);

			continue;
		}

		if (GetLastError() != ERROR_IO_PENDING)
			/* Something unexpected happened. */
			break;

		KeepWaiting:

		/* We need to wait for the event to occur. */
		waitResult = WaitForSingleObject(
			overlapped.hEvent,
			WAIT_TIME_FOR_SERIAL_PORT_READS_MS);

		if (!sp->continueHandlingSerialPortEvents)
			break;

		if (waitResult == WAIT_TIMEOUT)
			goto KeepWaiting;

		if (waitResult != WAIT_OBJECT_0)
			/* Something unexpected happened. */
			break;

		if (!GetOverlappedResult(
			sp->handle,
			&overlapped,
			&temp,
			TRUE))
		{
			if (GetLastError() == ERROR_OPERATION_ABORTED)
			{
				/* User probably uplugged UART-to-USB cable. */
				sp->continueHandlingSerialPortEvents = false;
				userUnpluggedUsbCable = true;
			}

			/* Something unexpected happened. */
			break;
		}

		if (mask & EV_RXCHAR)
		{
			VnSerialPort_onDataReceived(sp);

			continue;
		}

		if (mask & EV_RX80FULL)
		{
			/* We assume the RX buffer was overrun. */
			sp->numberOfReceiveDataDroppedSections++;

			continue;
		}

		if (mask & EV_ERR)
		{
			DWORD spErrors;
			COMSTAT comStat;

			if (!ClearCommError(
				sp->handle,
				&spErrors,
				&comStat))
			{
				/* Something unexpected happened. */
				break;
			}

			if ((spErrors & CE_OVERRUN) || (spErrors & CE_RXOVER))
			{
				/* The serial buffer RX buffer was overrun. */
				sp->numberOfReceiveDataDroppedSections++;
			}

			continue;
		}

		#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__ || __NUTTX__

		FD_ZERO(&readfs);
		FD_SET(sp->handle, &readfs);

		/* Select sets the values in readWaitTime. */
		readWaitTime.tv_sec = 0;
		readWaitTime.tv_usec = WAIT_TIME_FOR_SERIAL_PORT_READS_MS * 1000;

		error = select(
			sp->handle + 1,
			&readfs,
			NULL,
			NULL,
			&readWaitTime);

		if (error == -1)
			/* Something unexpected happened. */
			break;

		if (!FD_ISSET(sp->handle, &readfs))
			continue;

		VnSerialPort_onDataReceived(sp);

		#else
		#error "Unknown System"
		#endif

	}

	if (sp->continueHandlingSerialPortEvents)
		; /* An error must have occurred. Do nothing for now. */

	#if _WIN32

	if (!userUnpluggedUsbCable)
	{
		SetCommMask(
			sp->handle,
			0);
	}

	#endif

	if (userUnpluggedUsbCable)
		VnSerialPort_closeAfterUserUnpluggedSerialPort(sp);
}

void VnSerialPort_onDataReceived(VnSerialPort *sp)
{
	VnCriticalSection_enter(&sp->dataReceivedHandlerCriticalSection);

	if (sp->dataReceivedHandler != NULL)
		sp->dataReceivedHandler(sp->dataReceivedHandlerUserData);

	VnCriticalSection_leave(&sp->dataReceivedHandlerCriticalSection);
}

VnError VnSerialPort_registerDataReceivedHandler(VnSerialPort *sp, VnSerialPort_DataReceivedHandler handler, void *userData)
{
	if (sp->dataReceivedHandler != NULL)
		return E_INVALID_OPERATION;

	VnCriticalSection_enter(&sp->dataReceivedHandlerCriticalSection);

	sp->dataReceivedHandler = handler;
	sp->dataReceivedHandlerUserData = userData;

	VnCriticalSection_leave(&sp->dataReceivedHandlerCriticalSection);

	return E_NONE;
}

VnError VnSerialPort_unregisterDataReceivedHandler(VnSerialPort *sp)
{
	if (sp->dataReceivedHandler == NULL)
		return E_INVALID_OPERATION;

	VnCriticalSection_enter(&sp->dataReceivedHandlerCriticalSection);

	sp->dataReceivedHandler = NULL;
	sp->dataReceivedHandlerUserData = NULL;

	VnCriticalSection_leave(&sp->dataReceivedHandlerCriticalSection);

	return E_NONE;
}
