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


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include <px4_platform_common/posix.h>
#if defined(CONFIG_NET_CAN)
#include <nuttx/can.h>
#else
#include <nuttx/can/can.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE CO_CONFIG_STORAGE_ENABLE
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */
#ifndef CO_CONFIG_CRC16
#define CO_CONFIG_CRC16 (CO_CONFIG_CRC16_ENABLE)
#endif

#ifndef CO_CONFIG_SDO_SRV
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | \
			CO_CONFIG_SDO_SRV_BLOCK | \
			CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
			CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | \
			CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

/* Gateway command interface, SDO client and LSS master are disabled. */
#define CO_CONFIG_SDO_CLI (0)

#ifndef CO_CONFIG_SDO_SRV_BUFFER_SIZE
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 900
#endif

#ifndef CO_CONFIG_EM
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | \
		CO_CONFIG_EM_PROD_CONFIGURABLE | \
		CO_CONFIG_EM_PROD_INHIBIT | \
		CO_CONFIG_EM_HISTORY | \
		CO_CONFIG_EM_STATUS_BITS | \
		CO_CONFIG_EM_CONSUMER | \
		CO_CONFIG_FLAG_TIMERNEXT)
#endif

#ifndef CO_CONFIG_TIME
#define CO_CONFIG_TIME (CO_CONFIG_TIME_ENABLE | \
			CO_CONFIG_TIME_PRODUCER | \
			CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
			CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#endif

#ifndef CO_TPDO_DEFAULT_CANID_COUNT
#define CO_TPDO_DEFAULT_CANID_COUNT 15
#endif

#ifndef CO_RPDO_DEFAULT_CANID_COUNT
#define CO_RPDO_DEFAULT_CANID_COUNT 0
#endif

// No LEDS, LSS, gateway, fifo
#define CO_CONFIG_LEDS (0)
#define CO_CONFIG_LSS (0)
#define CO_CONFIG_GTW (0)
#define CO_CONFIG_FIFO (0)


// right now, everything runs in a single thread.
#define CO_SINGLE_THREAD

#ifdef CO_SINGLE_THREAD
#define VOLATILE
#else
#define VOLATILE volatile
#endif

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;

#if defined(CONFIG_NET_CAN)
/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)((struct can_frame *)(msg))->can_id)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)((struct can_frame *)(msg))->can_dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)((struct can_frame *)(msg))->data)

/* Received message object */
struct can_hdr_s
{
	canid_t     can_id;         /* 11-bit standard ID */
	uint8_t     can_dlc;        /* number of bytes of data */
};

typedef struct {
	struct can_hdr_s canHdr;
	canid_t mask;
	void *object;
	void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;
#else
/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)((struct can_msg_s *)(msg))->cm_hdr.ch_id)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)((struct can_msg_s *)(msg))->cm_hdr.ch_dlc)
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)((struct can_msg_s *)(msg))->cm_data)

/* Received message object */
typedef struct {
    struct can_hdr_s canHdr;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;
#endif

/* Transmit message object */
typedef struct {
	struct can_hdr_s canHdr;
	uint8_t data[8];
	VOLATILE bool_t bufferFull;
	VOLATILE bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
	void *CANptr;
	CO_CANrx_t *rxArray;
	uint16_t rxSize;
	CO_CANtx_t *txArray;
	uint16_t txSize;
	uint16_t CANerrorStatus;
	VOLATILE bool_t CANnormal;
	VOLATILE bool_t useCANrxFilters;
	VOLATILE bool_t bufferInhibitFlag;
	VOLATILE bool_t firstCANtxMessage;
	VOLATILE uint16_t CANtxCount;
	uint32_t errOld;
	int fd;     // file descriptor for CAN device
#ifndef CO_SINGLE_THREAD
	pthread_mutex_t *CO_CAN_SEND_mutex_ptr;
	pthread_mutex_t *CO_EMCY_mutex_ptr;
	pthread_mutex_t *CO_OD_mutex_ptr;
#endif
} CO_CANmodule_t;


/* Data storage: Maximum file name length including path */
#ifndef CO_STORAGE_PATH_MAX
#define CO_STORAGE_PATH_MAX 100
#endif

/* Data storage object for one entry */
typedef struct {
	void *addr;
	size_t len;
	uint8_t subIndexOD;
	uint8_t attr;
	/* Name of the file, where data block is stored */
	char filename[CO_STORAGE_PATH_MAX];
	/* CRC checksum of the data stored previously, for auto storage */
	uint16_t crc;
	/* Pointer to opened file, for auto storage */
	FILE *fp;
} CO_storage_entry_t;

#ifdef CO_SINGLE_THREAD
/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew)             ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)              {rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew)            {rxNew = NULL;}
#else
/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)    (pthread_mutex_lock((CAN_MODULE)->CO_CAN_SEND_mutex_ptr))
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)  (pthread_mutex_unlock((CAN_MODULE)->CO_CAN_SEND_mutex_ptr))

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)        (pthread_mutex_lock((CAN_MODULE)->CO_EMCY_mutex_ptr))
#define CO_UNLOCK_EMCY(CAN_MODULE)      (pthread_mutex_unlock((CAN_MODULE)->CO_EMCY_mutex_ptr))

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)          (pthread_mutex_lock((CAN_MODULE)->CO_OD_mutex_ptr))
#define CO_UNLOCK_OD(CAN_MODULE)        (pthread_mutex_unlock((CAN_MODULE)->CO_OD_mutex_ptr))

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()              {__sync_synchronize();}
#define CO_FLAG_READ(rxNew)             ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)              {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew)            {CO_MemoryBarrier(); rxNew = NULL;}
#endif

void CO_driver_receive(CO_CANmodule_t *CANmodule);
int CO_port_initialize(int port);
int CO_port_set_baud_rate(CO_CANmodule_t *CANmodule, int bitrate);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
