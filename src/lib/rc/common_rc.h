
#pragma once

#include <stdint.h>

#include "crsf.h"
#include "dsm.h"
#include "sbus.h"
#include "st24.h"
#include "sumd.h"

#pragma pack(push, 1)
typedef  struct rc_decode_buf_ {
	union {
		crsf_frame_t crsf_frame;
		dsm_decode_t dsm;
		sbus_frame_t sbus_frame;
		ReceiverFcPacket _strxpacket;
		ReceiverFcPacketHoTT _hottrxpacket;
	};
} rc_decode_buf_t;
#pragma pack(pop)

extern rc_decode_buf_t rc_decode_buf;
