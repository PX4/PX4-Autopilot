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

// NuttX CAN Socket driver wrapper for CanOpenNode

#include "301/CO_driver.h"

#include <string.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>

#include <nuttx/can.h>
#include <netpacket/can.h>
#include <netutils/netlib.h>

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
void CO_CANsetConfigurationMode(void *CANptr){
	/* Put CAN module in configuration mode */
	if (can_devinit() != 0) {
		PX4_ERR("can_devinit() returned an error");
		return;
	}
	netlib_ifup("can0");
}

int CO_port_set_baud_rate(CO_CANmodule_t *CANmodule, int bitrate)
{
	struct ifreq ifr;

	/* set the device name */

	strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';

	ifr.ifr_ifru.ifru_can_data.arbi_bitrate = bitrate / 1000; /* Convert bit/s to kbit/s */
	ifr.ifr_ifru.ifru_can_data.arbi_samplep = 80;

	if (ioctl(CANmodule->fd, SIOCSCANBITRATE, &ifr) < 0) {
		PX4_ERR("set speed %d failed", bitrate);
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
			struct ifreq ifr;
			struct sockaddr_can addr;

			/* open socket */
			CANmodule->fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
			if (CANmodule->fd < 0) {
				PX4_ERR("Can't open CAN socket");
				return;
			}
			strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
			ifr.ifr_name[IFNAMSIZ - 1] = '\0';
			ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
			if (!ifr.ifr_ifindex) {
				PX4_ERR("if_nametoindex");
				return;
			}

			memset(&addr, 0, sizeof(addr));
			addr.can_family = AF_CAN;
			addr.can_ifindex = ifr.ifr_ifindex;

			setsockopt(CANmodule->fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

			if(bind(CANmodule->fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				PX4_ERR("bind");
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
	CANmodule->useCANrxFilters = false;/* microcontroller dependent */
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
		rxArray[i].canHdr.can_id = 0U;
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
	netlib_ifdown("can0");
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
		buffer->canHdr.can_id = ident;
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
		buffer->canHdr.can_id = ident;
		buffer->canHdr.can_dlc = noOfBytes;
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
	struct can_frame transmit_msg;

	if (CANmodule==NULL || buffer==NULL) {
		PX4_ERR("CO_CANsend - invalid argument");
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	if(CANmodule->fd < 0) {
		PX4_ERR("CO_CANsend - no fd");
		return CO_ERROR_INVALID_STATE;
	}

	// PX4_DEBUG("CO_CANsend: buffer 0x%x, poll_result 0x%x, revents 0x%lx", (unsigned int)buffer, poll_result, fds.revents);

	transmit_msg.can_id = buffer->canHdr.can_id;
	transmit_msg.can_dlc = buffer->canHdr.can_dlc;
	memcpy(transmit_msg.data, buffer->data, transmit_msg.can_dlc);
	// PX4_DEBUG("CO_CANsend: writing %d bytes to fd: 0x%x", CAN_MTU, CANmodule->fd);
	// PX4_DEBUG("CO_CANsend: id 0x%lx, dlc 0x%x", transmit_msg.can_id, transmit_msg.can_dlc);
	const ssize_t nbytes = write(CANmodule->fd, &transmit_msg, CAN_MTU);

	if (nbytes < 0 || (size_t)nbytes != CAN_MTU) {
		/* Send failed, message will be re-sent by CO_CANmodule_process() */
		PX4_ERR("CO_CANsend - send failed. Wanted to send %d bytes.  Sent %d, errno %d", CAN_MTU, nbytes, errno);
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
	struct can_frame receive_msg;
	ssize_t nbytes;

	do {
		uint16_t i;
		CO_CANrx_t *rx_buffer;
		bool message_found = false;
		nbytes = recv(CANmodule->fd, &receive_msg, sizeof(receive_msg), 0);

		if(nbytes < 0) {
			if(errno != EAGAIN) {
				PX4_ERR("error on receive. returned %d, errno: 0x%x\n", nbytes, errno);
				CANmodule->CANerrorStatus |= CO_CAN_ERRRX_WARNING;
			}
		} else if((size_t)nbytes > sizeof(receive_msg)) {
			// error - todo - could this happen if we read in the middle of a packet receive?
			PX4_ERR("error on receive. Bytes read: %d\n", nbytes);
			CANmodule->CANerrorStatus |= CO_CAN_ERRRX_WARNING;
		} else {
			/*
			PX4_DEBUG("CO_process received: can_id 0x%lx, can_dlc: %d",
						receive_msg.can_id, receive_msg.can_dlc);
			PX4_DEBUG("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						receive_msg.data[0],
						receive_msg.data[1],
						receive_msg.data[2],
						receive_msg.data[3],
						receive_msg.data[4],
						receive_msg.data[5],
						receive_msg.data[6],
						receive_msg.data[7]);
			*/
			for(i = 0; i < CANmodule->rxSize; i++) {
				rx_buffer = &CANmodule->rxArray[i];
				if(((receive_msg.can_id ^ rx_buffer->canHdr.can_id) & rx_buffer->mask) == 0U) {
					message_found = true;
					break;
				} else {
					/*
						* Useful to figure out why messages are being ignored by CanOpen
					PX4_DEBUG("received can_id: 0x%lx, looking for can_id 0x%lx, mask 0x%lx",
								receive_msg.can_id, rx_buffer->canHdr.can_id, rx_buffer->mask);
					*/
				}
			}
			/* Call specific function, which will process the message */
			if(message_found && rx_buffer != NULL && rx_buffer->CANrx_callback != NULL) {
				// PX4_DEBUG("CO_process: Message found.  Calling callback");
				rx_buffer->CANrx_callback(rx_buffer->object, (void*) &receive_msg);
			} else {
				/*
				PX4_DEBUG("CO_process: Got message, throwing it away - ID: 0x%lx",
							receive_msg.can_id);
				*/
			}
		}
	} while(nbytes == sizeof(receive_msg));
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
