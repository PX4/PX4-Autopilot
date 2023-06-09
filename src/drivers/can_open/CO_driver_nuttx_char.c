/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// NuttX CAN character device driver wrapper for CanOpenNode

#include "301/CO_driver.h"

#include <fcntl.h>
#include <poll.h>

#include "stm32_can.h"

#include <px4_platform_common/log.h>

// For some reason, this doesn't seem to be declared in a header file.  Not going
// to worry much about this until we have to bring this back to mainline.
extern int can_devinit(void);

#ifndef CO_SINGLE_THREAD
static pthread_mutex_t CO_CAN_SEND_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t CO_EMCY_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t CO_OD_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

/******************************************************************************/
static int canlib_setbaud(int fd, int bauds)
{
  int ret;
  struct canioc_bittiming_s timings;

  ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)&timings);
  if (ret != OK)
	{
	  PX4_ERR("CANIOC_GET_BITTIMING failed, errno=%d\n", errno);
	  return ret;
	}

  timings.bt_baud = bauds;

  ret = ioctl(fd, CANIOC_SET_BITTIMING, (unsigned long)&timings);
  if (ret != OK)
	{
	  PX4_ERR("CANIOC_SET_BITTIMING failed, errno=%d\n", errno);
	}

  return ret;
}

int canlib_getbaud(int fd, FAR int *bauds)
{
  int ret;
  struct canioc_bittiming_s timings;

  ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)&timings);
  if (ret != OK)
	{
	  PX4_ERR("CANIOC_GET_BITTIMING failed, errno=%d\n", errno);
	  return 0;
	}

  *bauds = timings.bt_baud;

  return ret;
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
	/* Put CAN module in configuration mode */
	if (can_devinit() != 0) {
		PX4_ERR("can_devinit() returned an error");
		return;
	}
}

int CO_port_set_baud_rate(CO_CANmodule_t *CANmodule, int bitrate)
{
	if(canlib_setbaud(CANmodule->fd, bitrate) < 0) {
		PX4_ERR("Couldn't set bit rate %d", bitrate);
		return -1;
	}
	if(canlib_getbaud(CANmodule->fd, &bitrate) < 0) {
		PX4_ERR("Couldn't get bit rate");
		return -1;
	}
	PX4_INFO("CAN1 baud rate %d", bitrate);
	return 0;
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
	/* Put CAN module in normal mode */
	if(CANmodule != NULL) {
		if(CANmodule->fd == -1) {
			CANmodule->fd = open("/dev/can0", O_RDWR | O_NONBLOCK);
			if(CANmodule->fd < 0) {
				PX4_ERR("Can't open CAN FD");
				return;
			}
		}

		/* Put CAN module in normal mode */
		CANmodule->CANnormal = true;
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
		CO_CANmodule_t         *CANmodule,
		void                   *CANptr,
		CO_CANrx_t              rxArray[],
		uint16_t                rxSize,
		CO_CANtx_t              txArray[],
		uint16_t                txSize,
		uint16_t                CANbitRate)
{
	uint16_t i;

	/* verify arguments */
	if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Configure object variables */
	CANmodule->CANptr = CANptr;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false;/* microcontroller dependent */
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;
	CANmodule->fd = -1;

#ifndef CO_SINGLE_THREAD
	CANmodule->CO_CAN_SEND_mutex_ptr = &CO_CAN_SEND_mutex;
	CANmodule->CO_EMCY_mutex_ptr = &CO_EMCY_mutex;
	CANmodule->CO_OD_mutex_ptr = &CO_OD_mutex;
#endif

	for(i=0U; i<rxSize; i++){
		rxArray[i].canHdr.ch_id = 0U;
		rxArray[i].mask = 0xFFFFU;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for(i=0U; i<txSize; i++){
		txArray[i].bufferFull = false;
	}

	return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
	if (CANmodule == NULL) {
		return;
	}
	PX4_INFO("CO_CANmodule_disable");
	CANmodule->CANnormal = false;
	if(CANmodule->fd >= 0) {
		close(CANmodule->fd);
		CANmodule->fd = -1;
	}
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
		CO_CANmodule_t         *CANmodule,
		uint16_t                index,
		uint16_t                ident,
		uint16_t                mask,
		bool_t                  rtr,
		void                   *object,
		void                  (*CANrx_callback)(void *object, void *message))
{
	CO_ReturnError_t ret = CO_ERROR_NO;

	if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
		/* buffer, which will be configured */
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
		buffer->canHdr.ch_id = ident;
		buffer->canHdr.ch_rtr = rtr;
		buffer->mask = mask;
	}
	else{
		PX4_ERR("illegal arguement");
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
		CO_CANmodule_t         *CANmodule,
		uint16_t                index,
		uint16_t                ident,
		bool_t                  rtr,
		uint8_t                 noOfBytes,
		bool_t                  syncFlag)
{
	CO_CANtx_t *buffer = NULL;

	if((CANmodule != NULL) && (index < CANmodule->txSize) && (noOfBytes < 16)) {
		/* get specific buffer */
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
		 * Microcontroller specific. */
		buffer->canHdr.ch_id = ((uint32_t)ident);
		buffer->canHdr.ch_rtr = rtr;
		buffer->canHdr.ch_dlc = noOfBytes;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
	} else {
		PX4_ERR("CO_CANtxBufferInit issues: CANmodule 0x%x, index %d, noOfBytes %d",
				(unsigned int)CANmodule, index, noOfBytes);
	}

	return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
	struct pollfd fds;
	struct can_msg_s transmit_msg;

	if (CANmodule==NULL || buffer==NULL) {
		PX4_ERR("CO_CANsend - invalid argument");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	if(CANmodule->fd < 0) {
		PX4_ERR("CO_CANsend - no fd");
		return CO_ERROR_INVALID_STATE;
	}

	memset(&fds, 0, sizeof(struct pollfd));
	fds.fd = CANmodule->fd;
	fds.events = POLLOUT;

	const int poll_result = poll(&fds, 1, 0);

	//PX4_DEBUG("CO_CANsend: buffer 0x%x, poll_result 0x%x, revents 0x%lx", (unsigned int)buffer, poll_result, fds.revents);

	if (poll_result < 0) {
		PX4_ERR("CO_CANsend = poll return 0x%x", poll_result);
		return CO_ERROR_INVALID_STATE;
	}
	if (poll_result == 0) {
		/*
		   Note, sends will fail once the 32 entry fifo is filled, which will
		   happen if the CAN bus isn't connect to any other nodes (something as to
		   acknowledge the transmit) */
	//	PX4_DEBUG("CO_CANsend = revents 0x%lx, poll return 0x%x, fd 0x%x", fds.revents, poll_result, fds.fd);

		if(!buffer->bufferFull) {
			buffer->bufferFull = true;
			CANmodule->CANtxCount++;
		}
		// TODO: shall we do something more if we run out of space in the TX fifo?
		return CO_ERROR_TX_BUSY;
	}
	if ((fds.revents & POLLOUT) == 0) {
		PX4_ERR("CO_CANsend = revents 0x%x, poll return 0x%x, fd %d", (unsigned int)fds.revents, poll_result, fds.fd);
		return CO_ERROR_INVALID_STATE;
	}

	memcpy(&transmit_msg.cm_hdr, &buffer->canHdr, sizeof(struct can_hdr_s));
	memcpy(transmit_msg.cm_data, buffer->data, transmit_msg.cm_hdr.ch_dlc);
	const size_t msg_len = CAN_MSGLEN(transmit_msg.cm_hdr.ch_dlc);
	//PX4_DEBUG("CO_CANsend: writing %d bytes to fd: 0x%x", msg_len, CANmodule->fd);
	//PX4_DEBUG("CO_CANsend: id 0x%x, dlc 0x%x, rtr 0x%x, unused 0x%x",
	//		 transmit_msg.cm_hdr.ch_id, transmit_msg.cm_hdr.ch_dlc, transmit_msg.cm_hdr.ch_rtr, transmit_msg.cm_hdr.ch_unused);
	const ssize_t nbytes = write(CANmodule->fd, &transmit_msg, msg_len);

	if (nbytes < 0 || (size_t)nbytes != msg_len) {
		/* Send failed, message will be re-sent by CO_CANmodule_process() */
		PX4_ERR("CO_CANsend - send failed. Wanted to send %d bytes.  Sent %d", msg_len, nbytes);
		if(!buffer->bufferFull) {
			buffer->bufferFull = true;
			CANmodule->CANtxCount++;
		}
		return CO_ERROR_SYSCALL;
	}

	if(buffer->bufferFull) {
		buffer->bufferFull = false;
		CANmodule->CANtxCount--;
	}
	return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
	(void)CANmodule;
	// TODO - not sure how important this is.
	PX4_DEBUG("CO_CANclearPendingSyncPDOs called.  Not implemented yet");
}

void CO_driver_receive(CO_CANmodule_t *CANmodule) {
	/* Process messages received */
	uint32_t i;
	// File desriptor for CAN.
	struct pollfd fds;

	do {
		memset(&fds, 0, sizeof(struct pollfd));
		fds.fd = CANmodule->fd;
		fds.events = POLLIN;

		// Any received CAN messages will cause the poll statement to unblock and run
		// This way CAN read runs with minimal latency.
		// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
		poll(&fds, 1, 0);

		// Only execute this part if can0 is changed.
		if (fds.revents & POLLIN) {
			// Try to read.
			struct can_msg_s receive_msg;
			CO_CANrx_t *rx_buffer;
			bool message_found = false;

//          PX4_DEBUG("CO_CANmodule_process POLLIN, fd = 0x%x", fds.fd);

			const ssize_t nbytes = read(fds.fd, &receive_msg, sizeof(receive_msg));

			if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
				// error - todo - could this happen if we read in the middle of a packet receive?
				PX4_ERR("CO_process: error on receive. Bytes read: %d", nbytes);
				CANmodule->CANerrorStatus |= CO_CAN_ERRRX_WARNING;
			} else {
				/*
				PX4_DEBUG("CO_process received: ch_id 0x%x, ch_dlc: %d, ch_rtr: %d",
						  receive_msg.cm_hdr.ch_id, receive_msg.cm_hdr.ch_dlc, receive_msg.cm_hdr.ch_rtr);
				PX4_DEBUG("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						  receive_msg.cm_data[0],
						  receive_msg.cm_data[1],
						  receive_msg.cm_data[2],
						  receive_msg.cm_data[3],
						  receive_msg.cm_data[4],
						  receive_msg.cm_data[5],
						  receive_msg.cm_data[6],
						  receive_msg.cm_data[7]);
			*/
				for(i = 0; i < CANmodule->rxSize; i++) {
					rx_buffer = &CANmodule->rxArray[i];
					if(((receive_msg.cm_hdr.ch_id ^ rx_buffer->canHdr.ch_id) & rx_buffer->mask) == 0U) {
						if(receive_msg.cm_hdr.ch_rtr == rx_buffer->canHdr.ch_rtr) {
							message_found = true;
							break;
						} else {
							PX4_ERR("Message found but rtrs didn't match - shouldn't happen");
						}
					} else {
						/*
						 * Useful to figure out why messages are being ignored by CanOpen
						PX4_DEBUG("received ch_id: 0x%x, looking for ch_id 0x%x, mask 0x%x",
								  receive_msg.cm_hdr.ch_id, rx_buffer->canHdr.ch_id, rx_buffer->mask);
						*/

					}
				}
			}
			/* Call specific function, which will process the message */
			if(message_found && rx_buffer != NULL && rx_buffer->CANrx_callback != NULL) {
				// PX4_DEBUG("CO_process: Message found.  Calling callback");
				rx_buffer->CANrx_callback(rx_buffer->object, (void*) &receive_msg);
			} else {
				/*
				PX4_DEBUG("CO_process: Got message, throwing it away - ID: 0x%x",
						  receive_msg.cm_hdr.ch_id);
				*/
			}
		}
	} while(fds.revents & POLLIN);
}


/******************************************************************************/
void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
	uint32_t i;

	if (CANmodule == NULL || CANmodule->fd < 0) {
		PX4_ERR("CO_CANmodule_process error");
		return;
	}

	/* recall CO_CANsend(), if message was unsent before */
	if (CANmodule->CANtxCount > 0) {
		bool_t found = false;

		//PX4_DEBUG("CO_CANmodule_process: CANtxCount %d", CANmodule->CANtxCount);
		for (i = 0; i < CANmodule->txSize; i++) {
			CO_CANtx_t *tx_buffer = &CANmodule->txArray[i];

			if (tx_buffer->bufferFull) {
				found = true;
				tx_buffer->bufferFull = false;
				CANmodule->CANtxCount--;
				CO_CANsend(CANmodule, tx_buffer);
				break;
			}
		}

		if (!found) {
			CANmodule->CANtxCount = 0;
		}
	}
}
