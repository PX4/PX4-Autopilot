
#pragma once

#include <stdint.h>

#include "dsm.h"
#include "sbus.h"
#include "st24.h"
#include "sumd.h"
#include "dsm.h"

#pragma pack(push, 1)
typedef   uint8_t dsm_frame_t[DSM_BUFFER_SIZE]; /**< DSM dsm frame receive buffer */
typedef   uint8_t dsm_buf_t[DSM_FRAME_SIZE * 2]; // Define working buffer

typedef  struct dsm_decode_t {
	dsm_frame_t frame;
	dsm_buf_t buf;
} dsm_decode_t;

typedef   uint8_t sbus_frame_t[SBUS_FRAME_SIZE + (SBUS_FRAME_SIZE / 2)];

typedef  struct rc_decode_buf_ {
	union {
		dsm_decode_t dsm;
		sbus_frame_t sbus_frame;
		ReceiverFcPacket _strxpacket;
		ReceiverFcPacketHoTT _hottrxpacket;
	};
} rc_decode_buf_t;
#pragma pack(pop)

extern rc_decode_buf_t rc_decode_buf;
